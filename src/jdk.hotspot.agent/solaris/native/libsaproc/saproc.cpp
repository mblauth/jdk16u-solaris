/*
 * Copyright (c) 2002, 2020, Oracle and/or its affiliates. All rights reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Oracle, 500 Oracle Parkway, Redwood Shores, CA 94065 USA
 * or visit www.oracle.com if you need additional information or have any
 * questions.
 *
 */

#include "salibproc.h"
#include "sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal.h"
#include <thread_db.h>
#include <strings.h>
#include <limits.h>
#include <cxxabi.h>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include "cds.h"

#define CHECK_EXCEPTION_(value) if(env->ExceptionOccurred()) { return value; }
#define CHECK_EXCEPTION if(env->ExceptionOccurred()) { return;}
#define THROW_NEW_DEBUGGER_EXCEPTION_(str, value) { throwNewDebuggerException(env, str); return value; }
#define THROW_NEW_DEBUGGER_EXCEPTION(str) { throwNewDebuggerException(env, str); return;}

#define SYMBOL_BUF_SIZE  256
#define ERR_MSG_SIZE     (PATH_MAX + 256)

// debug modes
static int _libsaproc_debug = 0;

static void print_debug(const char* format,...) {
  if (_libsaproc_debug) {
    va_list alist;

    va_start(alist, format);
    fputs("libsaproc DEBUG: ", stderr);
    vfprintf(stderr, format, alist);
    va_end(alist);
  }
}

struct Debugger {
    JNIEnv* env;
    jobject this_obj;
};

struct DebuggerWithObject : Debugger {
    jobject obj;
};

struct DebuggerWith2Objects : DebuggerWithObject {
    jobject obj2;
};

/*
* Portions of user thread level detail gathering code is from pstack source
* code. See pstack.c in Solaris 2.8 user commands source code.
*/

static void throwNewDebuggerException(JNIEnv* env, const char* errMsg) {
  jclass clazz = env->FindClass("sun/jvm/hotspot/debugger/DebuggerException");
  CHECK_EXCEPTION;
  env->ThrowNew(clazz, errMsg);
}

// JNI ids for some fields, methods

// libproc handler pointer
static jfieldID p_ps_prochandle_ID = 0;

// libthread.so dlopen handle, thread agent ptr and function pointers
static jfieldID libthread_db_handle_ID   = 0;
static jfieldID p_td_thragent_t_ID       = 0;
static jfieldID p_td_init_ID             = 0;
static jfieldID p_td_ta_new_ID           = 0;
static jfieldID p_td_ta_delete_ID        = 0;
static jfieldID p_td_ta_thr_iter_ID      = 0;
static jfieldID p_td_thr_get_info_ID     = 0;
static jfieldID p_td_ta_map_id2thr_ID    = 0;
static jfieldID p_td_thr_getgregs_ID     = 0;

// reg index fields
static jfieldID pcRegIndex_ID            = 0;
static jfieldID fpRegIndex_ID            = 0;

// part of the class sharing workaround
static jfieldID classes_jsa_fd_ID        = 0;
static jfieldID p_file_map_header_ID     = 0;

// method ids

static jmethodID getThreadForThreadId_ID = 0;
static jmethodID createSenderFrame_ID    = 0;
static jmethodID createClosestSymbol_ID  = 0;
static jmethodID listAdd_ID              = 0;

/*
 * Functions we need from libthread_db
 */
typedef td_err_e
        (*p_td_init_t)(void);
typedef td_err_e
        (*p_td_ta_new_t)(void *, td_thragent_t **);
typedef td_err_e
        (*p_td_ta_delete_t)(td_thragent_t *);
typedef td_err_e
        (*p_td_ta_thr_iter_t)(const td_thragent_t *, td_thr_iter_f *, void *,
                td_thr_state_e, int, sigset_t *, unsigned);
typedef td_err_e
        (*p_td_thr_get_info_t)(const td_thrhandle_t *, td_thrinfo_t *);
typedef td_err_e
        (*p_td_ta_map_id2thr_t)(const td_thragent_t *, thread_t,  td_thrhandle_t *);
typedef td_err_e
        (*p_td_thr_getgregs_t)(const td_thrhandle_t *, prgregset_t);

static void
clear_libthread_db_ptrs(JNIEnv* env, jobject this_obj) {
  // release libthread_db agent, if we had created
  p_td_ta_delete_t p_td_ta_delete = 0;
  p_td_ta_delete = (p_td_ta_delete_t) env->GetLongField(this_obj, p_td_ta_delete_ID);

  td_thragent_t *p_td_thragent_t = 0;
  p_td_thragent_t = (td_thragent_t*) env->GetLongField(this_obj, p_td_thragent_t_ID);
  if (p_td_thragent_t != 0 && p_td_ta_delete != 0) {
     p_td_ta_delete(p_td_thragent_t);
  }

  // dlclose libthread_db.so
  void* libthread_db_handle = (void*) env->GetLongField(this_obj, libthread_db_handle_ID);
  if (libthread_db_handle != 0) {
    dlclose(libthread_db_handle);
  }

  env->SetLongField(this_obj, libthread_db_handle_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_init_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_ta_new_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_ta_delete_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_ta_thr_iter_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_thr_get_info_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_ta_map_id2thr_ID, (jlong)0);
  env->SetLongField(this_obj, p_td_thr_getgregs_ID, (jlong)0);
}


static void detach_internal(JNIEnv* env, jobject this_obj) {
  // clear libthread_db stuff
  clear_libthread_db_ptrs(env, this_obj);

  // release ptr to ps_prochandle
  jlong p_ps_prochandle;
  p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
  if (p_ps_prochandle != 0L) {
    Prelease((struct ps_prochandle*) p_ps_prochandle, PRELEASE_CLEAR);
  }

  // part of the class sharing workaround
  int classes_jsa_fd = env->GetIntField(this_obj, classes_jsa_fd_ID);
  if (classes_jsa_fd != -1) {
    close(classes_jsa_fd);
    CDSFileMapHeaderBase* pheader = (CDSFileMapHeaderBase*) env->GetLongField(this_obj, p_file_map_header_ID);
    if (pheader != NULL) {
      free(pheader);
    }
  }
}

// Is it okay to ignore libthread_db failure? Set env var to ignore
// libthread_db failure. You can still debug, but will miss threads
// related functionality.
static bool sa_ignore_threaddb = (getenv("SA_IGNORE_THREADDB") != 0);

#define HANDLE_THREADDB_FAILURE(msg)          \
  if (sa_ignore_threaddb) {                   \
     printf("libsaproc WARNING: %s\n", msg);  \
     return;                                  \
  } else {                                    \
     THROW_NEW_DEBUGGER_EXCEPTION(msg);       \
  }

#define HANDLE_THREADDB_FAILURE_(msg, ret)    \
  if (sa_ignore_threaddb) {                   \
     printf("libsaproc WARNING: %s\n", msg);  \
     return ret;                              \
  } else {                                    \
     THROW_NEW_DEBUGGER_EXCEPTION_(msg, ret); \
  }

static const char * alt_root = NULL;
static int alt_root_len = -1;

#define SA_ALTROOT "SA_ALTROOT"

static void init_alt_root() {
  if (alt_root_len == -1) {
    alt_root = getenv(SA_ALTROOT);
    if (alt_root)
      alt_root_len = strlen(alt_root);
    else
      alt_root_len = 0;
  }
}

// This function is a complete substitute for the open system call
// since it's also used to override open calls from libproc to
// implement as a pathmap style facility for the SA.  If libproc
// starts using other interfaces then this might have to extended to
// cover other calls.
extern "C" JNIEXPORT int JNICALL
libsaproc_open(const char * name, int oflag, ...) {
  if (oflag == O_RDONLY) {
    init_alt_root();

    if (_libsaproc_debug) {
      printf("libsaproc DEBUG: libsaproc_open %s\n", name);
    }

    if (alt_root_len > 0) {
      int fd = -1;
      char alt_path[PATH_MAX+1];

      strcpy(alt_path, alt_root);
      strcat(alt_path, name);
      fd = open(alt_path, O_RDONLY);
      if (fd >= 0) {
        if (_libsaproc_debug) {
          printf("libsaproc DEBUG: libsaproc_open substituted %s\n", alt_path);
        }
        return fd;
      }

      if (strrchr(name, '/')) {
        strcpy(alt_path, alt_root);
        strcat(alt_path, strrchr(name, '/'));
        fd = open(alt_path, O_RDONLY);
        if (fd >= 0) {
          if (_libsaproc_debug) {
            printf("libsaproc DEBUG: libsaproc_open substituted %s\n", alt_path);
          }
          return fd;
        }
      }
    }
  }

  {
    mode_t mode;
    va_list ap;
    va_start(ap, oflag);
    mode = va_arg(ap, mode_t);
    va_end(ap);

    return open(name, oflag, mode);
  }
}


static void * pathmap_dlopen(const char * name, int mode) {
  init_alt_root();

  if (_libsaproc_debug) {
    printf("libsaproc DEBUG: pathmap_dlopen %s\n", name);
  }

  void * handle = NULL;
  if (alt_root_len > 0) {
    char alt_path[PATH_MAX+1];
    strcpy(alt_path, alt_root);
    strcat(alt_path, name);
    handle = dlopen(alt_path, mode);
    if (_libsaproc_debug && handle) {
      printf("libsaproc DEBUG: pathmap_dlopen substituted %s\n", alt_path);
    }

    if (handle == NULL && strrchr(name, '/')) {
      strcpy(alt_path, alt_root);
      strcat(alt_path, strrchr(name, '/'));
      handle = dlopen(alt_path, mode);
      if (_libsaproc_debug && handle) {
        printf("libsaproc DEBUG: pathmap_dlopen substituted %s\n", alt_path);
      }
    }
  }
  if (handle == NULL) {
    handle = dlopen(name, mode);
  }
  if (_libsaproc_debug) {
    printf("libsaproc DEBUG: pathmap_dlopen %s return 0x%lx\n", name, (unsigned long) handle);
  }
  return handle;
}

// libproc and libthread_db callback functions

extern "C" {

static int
init_libthread_db_ptrs(void *cd, const prmap_t *pmp, const char *object_name) {
  Debugger* dbg = (Debugger*) cd;
  JNIEnv* env = dbg->env;
  jobject this_obj = dbg->this_obj;
  struct ps_prochandle* ph = (struct ps_prochandle*) env->GetLongField(this_obj, p_ps_prochandle_ID);

  char *s1 = 0, *s2 = 0;
  char libthread_db[PATH_MAX];

  if (strstr(object_name, "/libthread.so.") == NULL)
     return (0);

  /*
   * We found a libthread.
   * dlopen() the matching libthread_db and get the thread agent handle.
   */
  if (Pstatus(ph)->pr_dmodel == PR_MODEL_NATIVE) {
     (void) strcpy(libthread_db, object_name);
     s1 = (char*) strstr(object_name, ".so.");
     s2 = (char*) strstr(libthread_db, ".so.");
     (void) strcpy(s2, "_db");
     s2 += 3;
     (void) strcpy(s2, s1);
  } else {
#ifdef _LP64
     /*
      * The victim process is 32-bit, we are 64-bit.
      * We have to find the 64-bit version of libthread_db
      * that matches the victim's 32-bit version of libthread.
      */
     (void) strcpy(libthread_db, object_name);
     s1 = (char*) strstr(object_name, "/libthread.so.");
     s2 = (char*) strstr(libthread_db, "/libthread.so.");
     (void) strcpy(s2, "/64");
     s2 += 3;
     (void) strcpy(s2, s1);
     s1 = (char*) strstr(s1, ".so.");
     s2 = (char*) strstr(s2, ".so.");
     (void) strcpy(s2, "_db");
     s2 += 3;
     (void) strcpy(s2, s1);
#else
     return (0);
#endif  /* _LP64 */
  }

  void* libthread_db_handle = 0;
  if ((libthread_db_handle = pathmap_dlopen(libthread_db, RTLD_LAZY|RTLD_LOCAL)) == NULL) {
     char errMsg[PATH_MAX + 256];
     sprintf(errMsg, "Can't load %s!", libthread_db);
     HANDLE_THREADDB_FAILURE_(errMsg, 0);
  }
  env->SetLongField(this_obj, libthread_db_handle_ID, (jlong)(uintptr_t)libthread_db_handle);

  void* tmpPtr = 0;
  tmpPtr = dlsym(libthread_db_handle, "td_init");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_init!", 0);
  }
  env->SetLongField(this_obj, p_td_init_ID, (jlong)(uintptr_t) tmpPtr);

  tmpPtr =dlsym(libthread_db_handle, "td_ta_new");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_ta_new!", 0);
  }
  env->SetLongField(this_obj, p_td_ta_new_ID, (jlong)(uintptr_t) tmpPtr);

  tmpPtr = dlsym(libthread_db_handle, "td_ta_delete");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_ta_delete!", 0);
  }
  env->SetLongField(this_obj, p_td_ta_delete_ID, (jlong)(uintptr_t) tmpPtr);

  tmpPtr = dlsym(libthread_db_handle, "td_ta_thr_iter");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_ta_thr_iter!", 0);
  }
  env->SetLongField(this_obj, p_td_ta_thr_iter_ID, (jlong)(uintptr_t) tmpPtr);

  tmpPtr = dlsym(libthread_db_handle, "td_thr_get_info");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_thr_get_info!", 0);
  }
  env->SetLongField(this_obj, p_td_thr_get_info_ID, (jlong)(uintptr_t) tmpPtr);

  tmpPtr = dlsym(libthread_db_handle, "td_ta_map_id2thr");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_ta_map_id2thr!", 0);
  }
  env->SetLongField(this_obj, p_td_ta_map_id2thr_ID, (jlong)(uintptr_t) tmpPtr);

  tmpPtr = dlsym(libthread_db_handle, "td_thr_getgregs");
  if (tmpPtr == 0) {
     HANDLE_THREADDB_FAILURE_("dlsym failed on td_thr_getgregs!", 0);
  }
  env->SetLongField(this_obj, p_td_thr_getgregs_ID, (jlong)(uintptr_t) tmpPtr);

  return 1;
}

static int
fill_thread_list(const td_thrhandle_t *p_td_thragent_t, void* cd) {
  DebuggerWithObject* dbgo = (DebuggerWithObject*) cd;
  JNIEnv* env = dbgo->env;
  jobject this_obj = dbgo->this_obj;
  jobject list = dbgo->obj;

  td_thrinfo_t thrinfo;
  p_td_thr_get_info_t p_td_thr_get_info = (p_td_thr_get_info_t) env->GetLongField(this_obj, p_td_thr_get_info_ID);

  if (p_td_thr_get_info(p_td_thragent_t, &thrinfo) != TD_OK)
    return (0);

  jobject threadProxy = env->CallObjectMethod(this_obj, getThreadForThreadId_ID, (jlong)(uintptr_t) thrinfo.ti_tid);
  CHECK_EXCEPTION_(1);
  env->CallBooleanMethod(list, listAdd_ID, threadProxy);
  CHECK_EXCEPTION_(1);
  return 0;
}

// Pstack_iter() proc_stack_f callback prior to Nevada-B159
static int
fill_cframe_list(void *cd, const prgregset_t regs, uint_t argc, const long *argv) {
  DebuggerWith2Objects* dbgo2 = (DebuggerWith2Objects*) cd;
  JNIEnv* env = dbgo2->env;
  jobject this_obj = dbgo2->this_obj;
  jobject curFrame = dbgo2->obj2;

  jint pcRegIndex = env->GetIntField(this_obj, pcRegIndex_ID);
  jint fpRegIndex = env->GetIntField(this_obj, fpRegIndex_ID);

  jlong pc = (jlong) (uintptr_t) regs[pcRegIndex];
  jlong fp = (jlong) (uintptr_t) regs[fpRegIndex];

  dbgo2->obj2 = env->CallObjectMethod(this_obj, createSenderFrame_ID,
                                    curFrame, pc, fp);
  CHECK_EXCEPTION_(1);
  if (dbgo2->obj == 0) {
     dbgo2->obj = dbgo2->obj2;
  }
  return 0;
}

// Pstack_iter() proc_stack_f callback in Nevada-B159 or later
/*ARGSUSED*/
static int
wrapper_fill_cframe_list(void *cd, const prgregset_t regs, uint_t argc,
                         const long *argv, int frame_flags, int sig) {
  return(fill_cframe_list(cd, regs, argc, argv));
}

//---------------------------------------------------------------
// Part of the class sharing workaround:
//
// With class sharing, pages are mapped from classes.jsa file.
// The read-only class sharing pages are mapped as MAP_SHARED,
// PROT_READ pages. These pages are not dumped into core dump.
// With this workaround, these pages are read from classes.jsa.

static bool
read_jboolean(struct ps_prochandle* ph, psaddr_t addr, jboolean* pvalue) {
  jboolean i;
  if (ps_pread(ph, addr, &i, sizeof(i)) == PS_OK) {
    *pvalue = i;
    return true;
  } else {
    return false;
  }
}

static bool
read_pointer(struct ps_prochandle* ph, psaddr_t addr, uintptr_t* pvalue) {
  uintptr_t uip;
  if (ps_pread(ph, addr, &uip, sizeof(uip)) == PS_OK) {
    *pvalue = uip;
    return true;
  } else {
    return false;
  }
}

static bool
read_string(struct ps_prochandle* ph, psaddr_t addr, char* buf, size_t size) {
  char ch = ' ';
  size_t i = 0;

  while (ch != '\0') {
    if (ps_pread(ph, addr, &ch, sizeof(ch)) != PS_OK)
      return false;

    if (i < size - 1) {
      buf[i] = ch;
    } else { // smaller buffer
      return false;
    }

    i++; addr++;
  }

  buf[i] = '\0';
  return true;
}

#define USE_SHARED_SPACES_SYM   "UseSharedSpaces"
#define SHARED_BASE_ADDRESS_SYM "SharedBaseAddress"
// mangled symbol name for Arguments::SharedArchivePath
#define SHARED_ARCHIVE_PATH_SYM "__1cJArgumentsRSharedArchivePath_"

static uintptr_t sharedBaseAddress = 0;
static int
init_classsharing_workaround(void *cd, const prmap_t* pmap, const char* obj_name) {
  Debugger* dbg = (Debugger*) cd;
  JNIEnv*   env = dbg->env;
  jobject this_obj = dbg->this_obj;
  const char* jvm_name = 0;
  if ((jvm_name = strstr(obj_name, "libjvm.so")) != NULL) {
    jvm_name = obj_name;
  } else {
    return 0;
  }

  struct ps_prochandle* ph = (struct ps_prochandle*) env->GetLongField(this_obj, p_ps_prochandle_ID);

  // initialize classes.jsa file descriptor field.
  dbg->env->SetIntField(this_obj, classes_jsa_fd_ID, -1);

  // check whether class sharing is on by reading variable "UseSharedSpaces"
  psaddr_t useSharedSpacesAddr = 0;
  ps_pglobal_lookup(ph, jvm_name, USE_SHARED_SPACES_SYM, &useSharedSpacesAddr);
  if (useSharedSpacesAddr == 0) {
    THROW_NEW_DEBUGGER_EXCEPTION_("can't find 'UseSharedSpaces' flag\n", 1);
  }

  // read the value of the flag "UseSharedSpaces"
  // Since hotspot types are not available to build this library. So
  // equivalent type "jboolean" is used to read the value of "UseSharedSpaces"
  // which is same as hotspot type "bool".
  jboolean value = 0;
  if (read_jboolean(ph, useSharedSpacesAddr, &value) != true) {
    THROW_NEW_DEBUGGER_EXCEPTION_("can't read 'UseSharedSpaces' flag", 1);
  } else if ((int)value == 0) {
    print_debug("UseSharedSpaces is false, assuming -Xshare:off!\n");
    return 1;
  }

  psaddr_t sharedBaseAddressAddr = 0;
  ps_pglobal_lookup(ph, jvm_name, SHARED_ARCHIVE_PATH_SYM, &sharedBaseAddressAddr);
  if (sharedBaseAddressAddr == 0) {
    print_debug("can't find symbol 'SharedBaseAddress'\n");
    THROW_NEW_DEBUGGER_EXCEPTION_("can't find 'SharedBaseAddress' flag\n", 1);
  }

  sharedBaseAddress = 0;
  if (read_pointer(ph, sharedBaseAddressAddr, &sharedBaseAddress) != true) {
    print_debug("can't read the value of 'SharedBaseAddress' flag\n");
    THROW_NEW_DEBUGGER_EXCEPTION_("can't get SharedBaseAddress from debuggee", 1);
  }

  char classes_jsa[PATH_MAX];
  psaddr_t sharedArchivePathAddrAddr = 0;
  ps_pglobal_lookup(ph, jvm_name, SHARED_ARCHIVE_PATH_SYM, &sharedArchivePathAddrAddr);
  if (sharedArchivePathAddrAddr == 0) {
    print_debug("can't find symbol 'Arguments::SharedArchivePath'\n");
    THROW_NEW_DEBUGGER_EXCEPTION_("can't get shared archive path from debuggee", 1);
  }

  uintptr_t sharedArchivePathAddr = 0;
  if (read_pointer(ph, sharedArchivePathAddrAddr, &sharedArchivePathAddr) != true) {
    print_debug("can't find read pointer 'Arguments::SharedArchivePath'\n");
    THROW_NEW_DEBUGGER_EXCEPTION_("can't get shared archive path from debuggee", 1);
  }

  if (read_string(ph, (psaddr_t)sharedArchivePathAddr, classes_jsa, sizeof(classes_jsa)) != true) {
    print_debug("can't find read 'Arguments::SharedArchivePath' value\n");
    THROW_NEW_DEBUGGER_EXCEPTION_("can't get shared archive path from debuggee", 1);
  }

  print_debug("looking for %s\n", classes_jsa);

  // open the classes.jsa
  int fd = libsaproc_open(classes_jsa, O_RDONLY);
  if (fd < 0) {
    char errMsg[ERR_MSG_SIZE];
    sprintf(errMsg, "can't open shared archive file %s", classes_jsa);
    THROW_NEW_DEBUGGER_EXCEPTION_(errMsg, 1);
  } else {
    print_debug("opened shared archive file %s\n", classes_jsa);
  }

  // parse classes.jsa
  CDSFileMapHeaderBase* pheader = (CDSFileMapHeaderBase*) malloc(sizeof(CDSFileMapHeaderBase));
  if (pheader == NULL) {
    close(fd);
    THROW_NEW_DEBUGGER_EXCEPTION_("can't allocate memory for shared file map header", 1);
  }

  memset(pheader, 0, sizeof(CDSFileMapHeaderBase));
  // read CDSFileMapHeaderBase
  size_t n = read(fd, pheader, sizeof(CDSFileMapHeaderBase));
  if (n != sizeof(CDSFileMapHeaderBase)) {
    char errMsg[ERR_MSG_SIZE];
    sprintf(errMsg, "unable to read shared archive file map header from %s", classes_jsa);
    close(fd);
    free(pheader);
    THROW_NEW_DEBUGGER_EXCEPTION_(errMsg, 1);
  }

  // check file magic
  if (pheader->_magic != CDS_ARCHIVE_MAGIC) {
    char errMsg[ERR_MSG_SIZE];
    sprintf(errMsg, "%s has bad shared archive magic 0x%x, expecting 0x%x",
            classes_jsa, pheader->_magic, CDS_ARCHIVE_MAGIC);
    close(fd);
    free(pheader);
    THROW_NEW_DEBUGGER_EXCEPTION_(errMsg, 1);
  }

  // check version
  if (pheader->_version != CURRENT_CDS_ARCHIVE_VERSION) {
    char errMsg[ERR_MSG_SIZE];
    sprintf(errMsg, "%s has wrong shared archive version %d, expecting %d",
                   classes_jsa, pheader->_version, CURRENT_CDS_ARCHIVE_VERSION);
    close(fd);
    free(pheader);
    THROW_NEW_DEBUGGER_EXCEPTION_(errMsg, 1);
  }

  if (_libsaproc_debug) {
    for (int m = 0; m < NUM_CDS_REGIONS; m++) {
      if (!pheader->_space[m]._is_heap_region &&
          !pheader->_space[m]._is_bitmap_region) {
        jlong mapping_offset = pheader->_space[m]._mapping_offset;
        jlong baseAddress = mapping_offset + (jlong)sharedBaseAddress;
        print_debug("shared file offset %d mapped at 0x%lx, size = %ld, read only? = %d\n",
                    pheader->_space[m]._file_offset, baseAddress,
                    pheader->_space[m]._used, pheader->_space[m]._read_only);
      }
    }
  }

  // FIXME: For now, omitting other checks such as VM version etc.

  // store class archive file fd and map header in debugger object fields
  dbg->env->SetIntField(this_obj, classes_jsa_fd_ID, fd);
  dbg->env->SetLongField(this_obj, p_file_map_header_ID, (jlong)(uintptr_t) pheader);
  return 1;
}

} // extern "C"

// error messages for proc_arg_grab failure codes. The messages are
// modified versions of comments against corresponding #defines in
// libproc.h.
static const char* proc_arg_grab_errmsgs[] = {
                      "",
 /* G_NOPROC */       "No such process",
 /* G_NOCORE */       "No such core file",
 /* G_NOPROCORCORE */ "No such process or core",
 /* G_NOEXEC */       "Cannot locate executable file",
 /* G_ZOMB   */       "Zombie processs",
 /* G_PERM   */       "No permission to attach",
 /* G_BUSY   */       "Another process has already attached",
 /* G_SYS    */       "System process - can not attach",
 /* G_SELF   */       "Process is self - can't debug myself!",
 /* G_INTR   */       "Interrupt received while grabbing",
 /* G_LP64   */       "debuggee is 64 bit, use java -d64 for debugger",
 /* G_FORMAT */       "File is not an ELF format core file - corrupted core?",
 /* G_ELF    */       "Libelf error while parsing an ELF file",
 /* G_NOTE   */       "Required PT_NOTE Phdr not present - corrupted core?",
};

static void attach_internal(JNIEnv* env, jobject this_obj, jstring cmdLine, jboolean isProcess) {
  jboolean isCopy;
  int gcode;
  const char* cmdLine_cstr = env->GetStringUTFChars(cmdLine, &isCopy);
  char errMsg[ERR_MSG_SIZE];
  td_err_e te;
  CHECK_EXCEPTION;
  if (cmdLine_cstr == NULL) {
    return;
  }

  // some older versions of libproc.so crash when trying to attach 32 bit
  // debugger to 64 bit core file. check and throw error.
#ifndef _LP64
  errno = 0;
  strtol(cmdLine_cstr, NULL, 10);
  if (errno) {
     // core file
     int core_fd;
     if ((core_fd = open64(cmdLine_cstr, O_RDONLY)) >= 0) {
        Elf32_Ehdr e32;
        if (pread64(core_fd, &e32, sizeof (e32), 0) == sizeof (e32) &&
            memcmp(&e32.e_ident[EI_MAG0], ELFMAG, SELFMAG) == 0 &&
            e32.e_type == ET_CORE && e32.e_ident[EI_CLASS] == ELFCLASS64) {
              close(core_fd);
              env->ReleaseStringUTFChars(cmdLine, cmdLine_cstr);
              THROW_NEW_DEBUGGER_EXCEPTION("debuggee is 64 bit, use java -d64 for debugger");
        }
        close(core_fd);
     }
     // all other conditions are handled by libproc.so.
  }
#endif

  // connect to process/core
  ps_prochandle_t* ph = proc_arg_grab(cmdLine_cstr, (isProcess? PR_ARG_PIDS : PR_ARG_CORES), PGRAB_FORCE, &gcode, NULL);

  env->ReleaseStringUTFChars(cmdLine, cmdLine_cstr);

  if (! ph) {
     if (gcode > 0 && gcode < sizeof(proc_arg_grab_errmsgs)/sizeof(const char*)) {
        snprintf(errMsg, ERR_MSG_SIZE, "Attach failed : %s", proc_arg_grab_errmsgs[gcode]);
        THROW_NEW_DEBUGGER_EXCEPTION(errMsg);
    } else {
        if (_libsaproc_debug && gcode == G_STRANGE) {
           perror("libsaproc DEBUG: ");
        }
        if (isProcess) {
           THROW_NEW_DEBUGGER_EXCEPTION("Not able to attach to process!");
        } else {
           THROW_NEW_DEBUGGER_EXCEPTION("Not able to attach to core file!");
        }
     }
  }

  // even though libproc.so supports 64 bit debugger and 32 bit debuggee, we don't
  // support such cross-bit-debugging. check for that combination and throw error.
#ifdef _LP64
  int data_model;
  if (ps_pdmodel(ph, &data_model) != PS_OK) {
     Prelease(ph, PRELEASE_CLEAR);
     THROW_NEW_DEBUGGER_EXCEPTION("can't determine debuggee data model (ILP32? or LP64?)");
  }
  if (data_model == PR_MODEL_ILP32) {
     Prelease(ph, PRELEASE_CLEAR);
     THROW_NEW_DEBUGGER_EXCEPTION("debuggee is 32 bit, use 32 bit java for debugger");
  }
#endif

  env->SetLongField(this_obj, p_ps_prochandle_ID, (jlong)(uintptr_t)ph);

  Debugger dbg;
  dbg.env = env;
  dbg.this_obj = this_obj;
  jthrowable exception = 0;
  if (! isProcess) {
    /*
     * With class sharing, shared perm. gen heap is allocated in with MAP_SHARED|PROT_READ.
     * These pages are mapped from the file "classes.jsa". MAP_SHARED pages are not dumped
     * in Solaris core.To read shared heap pages, we have to read classes.jsa file.
     */
    Pobject_iter(ph, init_classsharing_workaround, &dbg);
    exception = env->ExceptionOccurred();
    if (exception) {
      env->ExceptionClear();
      detach_internal(env, this_obj);
      env->Throw(exception);
      return;
    }
  }

  /*
   * Iterate over the process mappings looking
   * for libthread and then dlopen the appropriate
   * libthread_db and get function pointers.
   */
  Pobject_iter(ph, init_libthread_db_ptrs, &dbg);
  exception = env->ExceptionOccurred();
  if (exception) {
    env->ExceptionClear();
    if (!sa_ignore_threaddb) {
      detach_internal(env, this_obj);
      env->Throw(exception);
    }
    return;
  }

  // init libthread_db and create thread_db agent
  p_td_init_t p_td_init = (p_td_init_t) env->GetLongField(this_obj, p_td_init_ID);
  if (p_td_init == 0) {
    if (!sa_ignore_threaddb) {
      detach_internal(env, this_obj);
    }
    HANDLE_THREADDB_FAILURE("Did not find libthread in target process/core!");
  }

  te = p_td_init();
  if (te != TD_OK) {
    if (!sa_ignore_threaddb) {
      detach_internal(env, this_obj);
    }
    snprintf(errMsg, ERR_MSG_SIZE, "Can't initialize thread_db! td_init failed: %d", te);
    HANDLE_THREADDB_FAILURE(errMsg);
  }

  p_td_ta_new_t p_td_ta_new = (p_td_ta_new_t) env->GetLongField(this_obj, p_td_ta_new_ID);

  td_thragent_t *p_td_thragent_t = 0;
  te = p_td_ta_new(ph, &p_td_thragent_t);
  if (te != TD_OK) {
    if (!sa_ignore_threaddb) {
      detach_internal(env, this_obj);
    }
    snprintf(errMsg, ERR_MSG_SIZE, "Can't create thread_db agent! td_ta_new failed: %d", te);
    HANDLE_THREADDB_FAILURE(errMsg);
  }
  env->SetLongField(this_obj, p_td_thragent_t_ID, (jlong)(uintptr_t) p_td_thragent_t);

}

/*
 * Class:     sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:    attach0
 * Signature: (Ljava/lang/String;)V
 * Description: process detach
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_attach0__Ljava_lang_String_2
  (JNIEnv *env, jobject this_obj, jstring pid) {
  attach_internal(env, this_obj, pid, JNI_TRUE);
}

/*
 * Class:     sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:    attach0
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 * Description: core file detach
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_attach0__Ljava_lang_String_2Ljava_lang_String_2
  (JNIEnv *env, jobject this_obj, jstring executable, jstring corefile) {
  // ignore executable file name, libproc.so can detect a.out name anyway.
  attach_internal(env, this_obj, corefile, JNI_FALSE);
}


/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      detach0
 * Signature:   ()V
 * Description: process/core file detach
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_detach0
  (JNIEnv *env, jobject this_obj) {
  detach_internal(env, this_obj);
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      getRemoteProcessAddressSize0
 * Signature:   ()I
 * Description: get process/core address size
 */
JNIEXPORT jint JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_getRemoteProcessAddressSize0
  (JNIEnv *env, jobject this_obj) {
  jlong p_ps_prochandle;
  p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
  int data_model = PR_MODEL_ILP32;
  ps_pdmodel((struct ps_prochandle*) p_ps_prochandle, &data_model);
  print_debug("debuggee is %d bit\n", data_model == PR_MODEL_ILP32? 32 : 64);
  return (jint) data_model == PR_MODEL_ILP32? 32 : 64;
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      getPageSize0
 * Signature:   ()I
 * Description: get process/core page size
 */
JNIEXPORT jint JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_getPageSize0
  (JNIEnv *env, jobject this_obj) {

/*
  We are not yet attached to a java process or core file. getPageSize is called from
  the constructor of ProcDebuggerLocal. The following won't work!

    jlong p_ps_prochandle;
    p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
    CHECK_EXCEPTION_(-1);
    struct ps_prochandle* prochandle = (struct ps_prochandle*) p_ps_prochandle;
    return (Pstate(prochandle) == PS_DEAD) ? Pgetauxval(prochandle, AT_PAGESZ)
                                           : getpagesize();

  So even though core may have been generated with a different page size settings, for now
  call getpagesize.
*/

  return getpagesize();
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      getThreadIntegerRegisterSet0
 * Signature:   (J)[J
 * Description: get gregset for a given thread specified by thread id
 */
JNIEXPORT jlongArray JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_getThreadIntegerRegisterSet0
  (JNIEnv *env, jobject this_obj, jlong tid) {
  char errMsg[ERR_MSG_SIZE];
  td_err_e te;
  // map the thread id to thread handle
  p_td_ta_map_id2thr_t p_td_ta_map_id2thr = (p_td_ta_map_id2thr_t) env->GetLongField(this_obj, p_td_ta_map_id2thr_ID);

  td_thragent_t* p_td_thragent_t = (td_thragent_t*) env->GetLongField(this_obj, p_td_thragent_t_ID);
  if (p_td_thragent_t == 0) {
     return 0;
  }

  td_thrhandle_t thr_handle;
  te = p_td_ta_map_id2thr(p_td_thragent_t, (thread_t) tid, &thr_handle);
  if (te != TD_OK) {
     snprintf(errMsg, ERR_MSG_SIZE, "can't map thread id to thread handle! td_ta_map_id2thr failed: %d", te);
     THROW_NEW_DEBUGGER_EXCEPTION_(errMsg, 0);
  }

  p_td_thr_getgregs_t p_td_thr_getgregs = (p_td_thr_getgregs_t) env->GetLongField(this_obj, p_td_thr_getgregs_ID);
  prgregset_t gregs;
  p_td_thr_getgregs(&thr_handle, gregs);

  jlongArray res = env->NewLongArray(NPRGREG);
  CHECK_EXCEPTION_(0);
  jboolean isCopy;
  jlong* ptr = env->GetLongArrayElements(res, &isCopy);
  CHECK_EXCEPTION_(NULL);
  for (int i = 0; i < NPRGREG; i++) {
    ptr[i] = (jlong) (uintptr_t) gregs[i];
  }
  env->ReleaseLongArrayElements(res, ptr, JNI_COMMIT);
  return res;
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      fillThreadList0
 * Signature:   (Ljava/util/List;)V
 * Description: fills thread list of the debuggee process/core
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_fillThreadList0
  (JNIEnv *env, jobject this_obj, jobject list) {

  td_thragent_t* p_td_thragent_t = (td_thragent_t*) env->GetLongField(this_obj, p_td_thragent_t_ID);
  if (p_td_thragent_t == 0) {
     return;
  }

  p_td_ta_thr_iter_t p_td_ta_thr_iter = (p_td_ta_thr_iter_t) env->GetLongField(this_obj, p_td_ta_thr_iter_ID);

  DebuggerWithObject dbgo;
  dbgo.env = env;
  dbgo.this_obj = this_obj;
  dbgo.obj = list;

  p_td_ta_thr_iter(p_td_thragent_t, fill_thread_list, &dbgo,
                   TD_THR_ANY_STATE, TD_THR_LOWEST_PRIORITY, TD_SIGNO_MASK, TD_THR_ANY_USER_FLAGS);
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      fillCFrameList0
 * Signature:   ([J)Lsun/jvm/hotspot/debugger/proc/ProcCFrame;
 * Description: fills CFrame list for a given thread
 */
JNIEXPORT jobject JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_fillCFrameList0
  (JNIEnv *env, jobject this_obj, jlongArray regsArray) {
  jlong p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);

  DebuggerWith2Objects dbgo2;
  dbgo2.env  = env;
  dbgo2.this_obj = this_obj;
  dbgo2.obj  = NULL;
  dbgo2.obj2 = NULL;

  jboolean isCopy;
  jlong* ptr = env->GetLongArrayElements(regsArray, &isCopy);
  CHECK_EXCEPTION_(0);

  prgregset_t gregs;
  for (int i = 0; i < NPRGREG; i++) {
     gregs[i] = (uintptr_t) ptr[i];
  }

  env->ReleaseLongArrayElements(regsArray, ptr, JNI_ABORT);
  CHECK_EXCEPTION_(0);

  Pstack_iter((struct ps_prochandle*) p_ps_prochandle, gregs,
              wrapper_fill_cframe_list, &dbgo2);
  return dbgo2.obj;
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      readBytesFromProcess0
 * Signature:   (JJ)[B
 * Description: read bytes from debuggee process/core
 */
JNIEXPORT jbyteArray JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_readBytesFromProcess0
  (JNIEnv *env, jobject this_obj, jlong address, jlong numBytes) {

  jbyteArray array = env->NewByteArray(numBytes);
  CHECK_EXCEPTION_(0);
  jboolean isCopy;
  jbyte* bufPtr = env->GetByteArrayElements(array, &isCopy);
  CHECK_EXCEPTION_(0);

  jlong p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
  ps_err_e ret = ps_pread((struct ps_prochandle*) p_ps_prochandle,
                       (psaddr_t)address, bufPtr, (size_t)numBytes);

  if (ret != PS_OK) {
    // part of the class sharing workaround. try shared heap area
    int classes_jsa_fd = env->GetIntField(this_obj, classes_jsa_fd_ID);
    if (classes_jsa_fd != -1 && address != (jlong)0) {
      print_debug("read failed at 0x%lx, attempting shared heap area\n", (long) address);

      CDSFileMapHeaderBase* pheader = (CDSFileMapHeaderBase*) env->GetLongField(this_obj, p_file_map_header_ID);
      // walk through the shared mappings -- we just have 9 of them.
      // so, linear walking is okay.
      for (int m = 0; m < NUM_CDS_REGIONS; m++) {

        // We can skip the non-read-only maps. These are mapped as MAP_PRIVATE
        // and hence will be read by libproc. Besides, the file copy may be
        // stale because the process might have modified those pages.
        if (pheader->_space[m]._read_only &&
            !pheader->_space[m]._is_heap_region &&
            !pheader->_space[m]._is_bitmap_region) {
         jlong mapping_offset = (jlong) (uintptr_t) pheader->_space[m]._mapping_offset;
         jlong baseAddress = mapping_offset + (jlong)sharedBaseAddress;
         size_t usedSize = pheader->_space[m]._used;
         if (address >= baseAddress && address < (baseAddress + usedSize)) {
            // the given address falls in this shared metadata area
            print_debug("found shared map at 0x%lx\n", (long) baseAddress);


            // If more data is asked than actually mapped from file, we need to zero fill
            // till the end-of-page boundary. But, java array new does that for us. we just
            // need to read as much as data available.

#define MIN2(x, y) (((x) < (y))? (x) : (y))

            jlong diff = address - baseAddress;
            jlong bytesToRead = MIN2(numBytes, usedSize - diff);
            off_t offset = pheader->_space[m]._file_offset  + off_t(diff);
            ssize_t bytesRead = pread(classes_jsa_fd, bufPtr, bytesToRead, offset);
            if (bytesRead != bytesToRead) {
              env->ReleaseByteArrayElements(array, bufPtr, JNI_ABORT);
              print_debug("shared map read failed\n");
              return jbyteArray(0);
            } else {
              print_debug("shared map read succeeded\n");
              env->ReleaseByteArrayElements(array, bufPtr, 0);
              return array;
            }
          } // is in current map
        } // is read only map
      } // for shared maps
    } // classes_jsa_fd != -1
    env->ReleaseByteArrayElements(array, bufPtr, JNI_ABORT);
    return jbyteArray(0);
  } else {
    env->ReleaseByteArrayElements(array, bufPtr, 0);
    return array;
  }
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      writeBytesToProcess0
 * Signature:   (JJ[B)V
 * Description: write bytes into debugger process
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_writeBytesToProcess0
  (JNIEnv *env, jobject this_obj, jlong address, jlong numBytes, jbyteArray data) {
  char errMsg[ERR_MSG_SIZE];
  ps_err_e pe;
  jlong p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
  jboolean isCopy;
  jbyte* ptr = env->GetByteArrayElements(data, &isCopy);
  CHECK_EXCEPTION;

  pe = ps_pwrite((struct ps_prochandle*) p_ps_prochandle, address, ptr, numBytes);
  if (pe != PS_OK) {
     snprintf(errMsg, ERR_MSG_SIZE, "Process write failed! ps_pwrite failed: %d", pe);
     env->ReleaseByteArrayElements(data, ptr, JNI_ABORT);
     THROW_NEW_DEBUGGER_EXCEPTION(errMsg);
  }

  env->ReleaseByteArrayElements(data, ptr, JNI_ABORT);
}

/*
 * Class:     sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:    suspend0
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_suspend0
  (JNIEnv *env, jobject this_obj) {
  jlong p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
  // for now don't check return value. revisit this again.
  Pstop((struct ps_prochandle*) p_ps_prochandle, 1000);
}

/*
 * Class:     sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:    resume0
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_resume0
  (JNIEnv *env, jobject this_obj) {
  jlong p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);
  // for now don't check return value. revisit this again.
  Psetrun((struct ps_prochandle*) p_ps_prochandle, 0, PRCFAULT|PRSTOP);
}

/*
  * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
  * Method:      lookupByName0
  * Signature:   (Ljava/lang/String;Ljava/lang/String;)J
  * Description: symbol lookup by name
*/
JNIEXPORT jlong JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_lookupByName0
   (JNIEnv *env, jobject this_obj, jstring objectName, jstring symbolName) {
   jlong p_ps_prochandle;
   p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);

   jboolean isCopy;
   const char* objectName_cstr = NULL;
   if (objectName != NULL) {
     objectName_cstr = env->GetStringUTFChars(objectName, &isCopy);
     CHECK_EXCEPTION_(0);
   } else {
     objectName_cstr = PR_OBJ_EVERY;
   }

   const char* symbolName_cstr = env->GetStringUTFChars(symbolName, &isCopy);
   if (env->ExceptionOccurred()) {
     if (objectName_cstr != PR_OBJ_EVERY) {
       env->ReleaseStringUTFChars(objectName, objectName_cstr);
     }
     return 0;
   }

   psaddr_t symbol_addr = (psaddr_t) 0;
   ps_pglobal_lookup((struct ps_prochandle*) p_ps_prochandle,  objectName_cstr,
                    symbolName_cstr, &symbol_addr);

   if (symbol_addr == 0) {
      print_debug("lookup for %s in %s failed\n", symbolName_cstr, objectName_cstr);
   }

   if (objectName_cstr != PR_OBJ_EVERY) {
     env->ReleaseStringUTFChars(objectName, objectName_cstr);
   }
   env->ReleaseStringUTFChars(symbolName, symbolName_cstr);
   return (jlong) (uintptr_t) symbol_addr;
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      lookupByAddress0
 * Signature:   (J)Lsun/jvm/hotspot/debugger/cdbg/ClosestSymbol;
 * Description: lookup symbol name for a given address
 */
JNIEXPORT jobject JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_lookupByAddress0
   (JNIEnv *env, jobject this_obj, jlong address) {
   jlong p_ps_prochandle;
   p_ps_prochandle = env->GetLongField(this_obj, p_ps_prochandle_ID);

   char nameBuf[SYMBOL_BUF_SIZE + 1];
   GElf_Sym sym;
   int res = Plookup_by_addr((struct ps_prochandle*) p_ps_prochandle, (uintptr_t) address,
                             nameBuf, sizeof(nameBuf), &sym, NULL);

   if (res != 0) { // failed
      return 0;
   }

   jstring resSym = env->NewStringUTF(nameBuf);
   CHECK_EXCEPTION_(0);

   return env->CallObjectMethod(this_obj, createClosestSymbol_ID, resSym, (address - sym.st_value));
}

/*
 * Class:     sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:    demangle0
 * Signature: (Ljava/lang/String;)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_demangle0
  (JNIEnv *env, jobject this_object, jstring name) {
  jboolean isCopy;
  const char* ptr = env->GetStringUTFChars(name, &isCopy);
  CHECK_EXCEPTION_(NULL);
  char  buf[2*SYMBOL_BUF_SIZE + 1];
  jstring res = 0;
  int   status;
  char* result;
  if ((result = abi::__cxa_demangle(ptr, NULL, NULL, &status)) != NULL) {
    snprintf(buf, 2*SYMBOL_BUF_SIZE+1, "%s", result);
    res = env->NewStringUTF(buf);
    // call c library's free
    ::free(result);
  } else {
    res = name;
  }
  env->ReleaseStringUTFChars(name, ptr);
  return res;
}

/*
 * Class:       sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal
 * Method:      initIDs
 * Signature:   ()V
 * Description: get JNI ids for fields and methods of ProcDebuggerLocal class
 */
JNIEXPORT void JNICALL Java_sun_jvm_hotspot_debugger_proc_ProcDebuggerLocal_initIDs
  (JNIEnv *env, jclass clazz) {
  _libsaproc_debug = getenv("LIBSAPROC_DEBUG") != NULL;
  if (_libsaproc_debug) {
     // propagate debug mode to libproc.so
     static const char* var = "LIBPROC_DEBUG=1";
     putenv((char*)var);
  }

  void* libproc_handle = dlopen("libproc.so", RTLD_LAZY | RTLD_GLOBAL);
  if (libproc_handle == 0)
     THROW_NEW_DEBUGGER_EXCEPTION("can't load libproc.so, if you are using Solaris 5.7 or below, copy libproc.so from 5.8!");

  p_ps_prochandle_ID = env->GetFieldID(clazz, "p_ps_prochandle", "J");
  CHECK_EXCEPTION;

  libthread_db_handle_ID = env->GetFieldID(clazz, "libthread_db_handle", "J");
  CHECK_EXCEPTION;

  p_td_thragent_t_ID = env->GetFieldID(clazz, "p_td_thragent_t", "J");
  CHECK_EXCEPTION;

  p_td_init_ID = env->GetFieldID(clazz, "p_td_init", "J");
  CHECK_EXCEPTION;

  p_td_ta_new_ID = env->GetFieldID(clazz, "p_td_ta_new", "J");
  CHECK_EXCEPTION;

  p_td_ta_delete_ID = env->GetFieldID(clazz, "p_td_ta_delete", "J");
  CHECK_EXCEPTION;

  p_td_ta_thr_iter_ID = env->GetFieldID(clazz, "p_td_ta_thr_iter", "J");
  CHECK_EXCEPTION;

  p_td_thr_get_info_ID = env->GetFieldID(clazz, "p_td_thr_get_info", "J");
  CHECK_EXCEPTION;

  p_td_ta_map_id2thr_ID = env->GetFieldID(clazz, "p_td_ta_map_id2thr", "J");
  CHECK_EXCEPTION;

  p_td_thr_getgregs_ID = env->GetFieldID(clazz, "p_td_thr_getgregs", "J");
  CHECK_EXCEPTION;

  getThreadForThreadId_ID = env->GetMethodID(clazz,
                            "getThreadForThreadId", "(J)Lsun/jvm/hotspot/debugger/ThreadProxy;");
  CHECK_EXCEPTION;

  pcRegIndex_ID = env->GetFieldID(clazz, "pcRegIndex", "I");
  CHECK_EXCEPTION;

  fpRegIndex_ID = env->GetFieldID(clazz, "fpRegIndex", "I");
  CHECK_EXCEPTION;

  createSenderFrame_ID = env->GetMethodID(clazz,
                            "createSenderFrame", "(Lsun/jvm/hotspot/debugger/proc/ProcCFrame;JJ)Lsun/jvm/hotspot/debugger/proc/ProcCFrame;");
  CHECK_EXCEPTION;

  createClosestSymbol_ID = env->GetMethodID(clazz,
                            "createClosestSymbol", "(Ljava/lang/String;J)Lsun/jvm/hotspot/debugger/cdbg/ClosestSymbol;");
  CHECK_EXCEPTION;

  jclass list_clazz = env->FindClass("java/util/List");
  CHECK_EXCEPTION;
  listAdd_ID = env->GetMethodID(list_clazz, "add", "(Ljava/lang/Object;)Z");
  CHECK_EXCEPTION;

  // part of the class sharing workaround
  classes_jsa_fd_ID = env->GetFieldID(clazz, "classes_jsa_fd", "I");
  CHECK_EXCEPTION;
  p_file_map_header_ID = env->GetFieldID(clazz, "p_file_map_header", "J");
  CHECK_EXCEPTION;
}
