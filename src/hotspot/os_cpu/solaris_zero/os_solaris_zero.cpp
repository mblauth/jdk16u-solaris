/*
 * Copyright (c) 2003, 2020, Oracle and/or its affiliates. All rights reserved.
 * Copyright 2007, 2008, 2009, 2010 Red Hat, Inc.
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

#include <pthread.h> /* For pthread_attr_get_np */

// no precompiled headers
#include "jvm.h"
#include "assembler_zero.inline.hpp"
#include "classfile/classLoader.hpp"
#include "classfile/systemDictionary.hpp"
#include "classfile/vmSymbols.hpp"
#include "code/icBuffer.hpp"
#include "code/vtableStubs.hpp"
#include "interpreter/interpreter.hpp"
#include "memory/allocation.inline.hpp"
#include "nativeInst_zero.hpp"
#include "os_share_solaris.hpp"
#include "prims/jniFastGetField.hpp"
#include "prims/jvm_misc.hpp"
#include "runtime/arguments.hpp"
#include "runtime/frame.inline.hpp"
#include "runtime/interfaceSupport.inline.hpp"
#include "runtime/java.hpp"
#include "runtime/javaCalls.hpp"
#include "runtime/mutexLocker.hpp"
#include "runtime/osThread.hpp"
#include "runtime/sharedRuntime.hpp"
#include "runtime/stubRoutines.hpp"
#include "runtime/thread.inline.hpp"
#include "runtime/timer.hpp"
#include "signals_posix.hpp"
#include "utilities/events.hpp"
#include "utilities/vmError.hpp"

// See stubGenerator_zero.cpp
#include <setjmp.h>
extern sigjmp_buf* get_jmp_buf_for_continuation();

address os::current_stack_pointer() {
  // return the address of the current function
  return (address)__builtin_frame_address(0);
}

frame os::get_sender_for_C_frame(frame* fr) {
  ShouldNotCallThis();
  return frame();
}

frame os::current_frame() {
  // The only thing that calls this is the stack printing code in
  // VMError::report:
  //   - Step 110 (printing stack bounds) uses the sp in the frame
  //     to determine the amount of free space on the stack.  We
  //     set the sp to a close approximation of the real value in
  //     order to allow this step to complete.
  //   - Step 120 (printing native stack) tries to walk the stack.
  //     The frame we create has a NULL pc, which is ignored as an
  //     invalid frame.
  frame dummy = frame();
  dummy.set_sp((intptr_t *) current_stack_pointer());
  return dummy;
}

char* os::non_memory_address_word() {
  // Must never look like an address returned by reserve_memory,
  // even in its subfields (as defined by the CPU immediate fields,
  // if the CPU splits constants across multiple instructions).
  // This is the value for x86; works pretty well for PPC too.
  return (char *) -1;
}

address os::Solaris::ucontext_get_pc(const ucontext_t* uc) {
  ShouldNotCallThis();
  return NULL;
}

void os::Solaris::ucontext_set_pc(ucontext_t * uc, address pc) {
  ShouldNotCallThis();
}

address os::fetch_frame_from_context(const void* ucVoid,
                                     intptr_t** ret_sp,
                                     intptr_t** ret_fp) {
  ShouldNotCallThis();
  return NULL;
}

frame os::fetch_frame_from_context(const void* ucVoid) {
  ShouldNotCallThis();
  return frame();
}

extern "C" JNIEXPORT int
JVM_handle_solaris_signal(int sig, siginfo_t* info, void* ucVoid,
                          int abort_if_unrecognized) {
  ucontext_t* uc = (ucontext_t*) ucVoid;

#ifndef AMD64
  if (sig == SIGILL && info->si_addr == (caddr_t)sse_check) {
    // the SSE instruction faulted. supports_sse() need return false.
    uc->uc_mcontext.gregs[EIP] = (greg_t)sse_unavailable;
    return true;
  }
#endif // !AMD64

  Thread* t = Thread::current_or_null_safe();

  // If crash protection is installed we may longjmp away and no destructors
  // for objects in this scope will be run.
  // So don't use any RAII utilities before crash protection is checked.
  os::ThreadCrashProtection::check_crash_protection(sig, t);

  if(sig == SIGPIPE || sig == SIGXFSZ) {
    if (PosixSignals::chained_handler(sig, info, ucVoid)) {
      return true;
    } else {
      // Ignoring SIGPIPE/SIGXFSZ - see bugs 4229104 or 6499219
      return true;
    }
  }

  JavaThread* thread = NULL;
  VMThread* vmthread = NULL;

  if (PosixSignals::are_signal_handlers_installed()) {
    if (t != NULL ){
      if(t->is_Java_thread()) {
        thread = (JavaThread*)t;
      }
      else if(t->is_VM_thread()){
        vmthread = (VMThread *)t;
      }
    }
  }

  if (sig == ASYNC_SIGNAL) {
    if(thread || vmthread){
      return true;
    } else if (PosixSignals::chained_handler(sig, info, ucVoid)) {
      return true;
    } else {
      // If ASYNC_SIGNAL not chained, and this is a non-vm and
      // non-java thread
      return true;
    }
  }

  if (info == NULL || info->si_code <= 0 || info->si_code == SI_NOINFO) {
    // can't decode this kind of signal
    info = NULL;
  } else {
    assert(sig == info->si_signo, "bad siginfo");
  }

  // decide if this trap can be handled by a stub
  address stub = NULL;

  address pc          = NULL;

  //%note os_trap_1
  if (info != NULL && uc != NULL && thread != NULL) {
    // factor me: getPCfromContext
    pc = (address) uc->uc_mcontext.gregs[REG_PC];

    if (StubRoutines::is_safefetch_fault(pc)) {
      os::Solaris::ucontext_set_pc(uc, StubRoutines::continuation_for_safefetch_fault(pc));
      return true;
    }

    // Handle ALL stack overflow variations here
    if (sig == SIGSEGV && info->si_code == SEGV_ACCERR) {
      address addr = (address) info->si_addr;

      // check if fault address is within thread stack
      if (thread->is_in_full_stack(addr)) {
        StackOverflow* overflow_state = thread->stack_overflow_state();
        // stack overflow
        if (overflow_state->in_stack_yellow_reserved_zone(addr)) {
          overflow_state->disable_stack_yellow_reserved_zone();
          ShouldNotCallThis();
        }
        else if (overflow_state->in_stack_red_zone(addr)) {
          overflow_state->disable_stack_red_zone();
          ShouldNotCallThis();
        }
      }
    }

    if (thread->thread_state() == _thread_in_vm ||
         thread->thread_state() == _thread_in_native) {
      if (sig == SIGBUS && info->si_code == BUS_OBJERR && thread->doing_unsafe_access()) {
	ShouldNotCallThis();
      }
    }

  }

  // signal-chaining
  if (PosixSignals::chained_handler(sig, info, ucVoid)) {
    return true;
  }

  if (!abort_if_unrecognized) {
    // caller wants another chance, so give it to him
    return false;
  }

  if (pc == NULL && uc != NULL) {
    pc = (address) uc->uc_mcontext.gregs[REG_PC];
  }

  // unmask current signal
  sigset_t newset;
  sigemptyset(&newset);
  sigaddset(&newset, sig);
  sigprocmask(SIG_UNBLOCK, &newset, NULL);

  // Determine which sort of error to throw.  Out of swap may signal
  // on the thread stack, which could get a mapping error when touched.
  address addr = (address) info->si_addr;
  if (sig == SIGBUS && info->si_code == BUS_OBJERR && info->si_errno == ENOMEM) {
    vm_exit_out_of_memory(0, OOM_MMAP_ERROR, "Out of swap space to map in thread stack.");
  }

  VMError::report_and_die(t, sig, pc, info, ucVoid);

  ShouldNotReachHere();
  return false;
}

void os::Solaris::init_thread_fpu_state(void) {
  // Nothing to do
}

bool os::is_allocatable(size_t bytes) {
#ifdef _LP64
  return true;
#else
  if (bytes < 2 * G) {
    return true;
  }

  char* addr = reserve_memory(bytes, NULL);

  if (addr != NULL) {
    release_memory(addr, bytes);
  }

  return addr != NULL;
#endif // _LP64
}

// Minimum usable stack sizes required to get to user code. Space for
// HotSpot guard pages is added later.
#ifdef _LP64
// The adlc generated method 'State::MachNodeGenerator(int)' used by the C2 compiler
// threads requires a large stack with the Solaris Studio C++ compiler version 5.13
// and product VM builds (debug builds require significantly less stack space).
size_t os::Posix::_compiler_thread_min_stack_allowed = 325 * K;
size_t os::Posix::_java_thread_min_stack_allowed = 48 * K;
size_t os::Posix::_vm_internal_thread_min_stack_allowed = 224 * K;
#else
size_t os::Posix::_compiler_thread_min_stack_allowed = 32 * K;
size_t os::Posix::_java_thread_min_stack_allowed = 32 * K;
size_t os::Posix::_vm_internal_thread_min_stack_allowed = 64 * K;
#endif // _LP64


static void current_stack_region(address *bottom, size_t *size) {
  address stack_bottom;
  address stack_top;
  size_t stack_bytes;

  pthread_attr_t attr;

  int rslt = pthread_attr_init(&attr);

  // JVM needs to know exact stack location, abort if it fails
  if (rslt != 0)
    fatal("pthread_attr_init failed with error = " INT32_FORMAT, rslt);

  rslt = pthread_attr_get_np(pthread_self(), &attr);

  if (rslt != 0)
    fatal("pthread_attr_get_np failed with error = " INT32_FORMAT, rslt);

  if (pthread_attr_getstackaddr(&attr, (void **) &stack_bottom) != 0 ||
      pthread_attr_getstacksize(&attr, &stack_bytes) != 0) {
    fatal("Can not locate current stack attributes!");
  }

  pthread_attr_destroy(&attr);

  stack_top = stack_bottom + stack_bytes;

  assert(os::current_stack_pointer() >= stack_bottom, "should do");
  assert(os::current_stack_pointer() < stack_top, "should do");

  *bottom = stack_bottom;
  *size = stack_top - stack_bottom;
}

/////////////////////////////////////////////////////////////////////////////
// helper functions for fatal error handler

void os::print_context(outputStream* st, const void* context) {
  ShouldNotCallThis();
}

void os::print_register_info(outputStream *st, const void *context) {
  ShouldNotCallThis();
}

/////////////////////////////////////////////////////////////////////////////
// Stubs for things that would be in bsd_zero.s if it existed.
// You probably want to disassemble these monkeys to check they're ok.

extern "C" {
  int SpinPause() {
    return 1;
  }

  void _Copy_conjoint_jshorts_atomic(const jshort* from, jshort* to, size_t count) {
    if (from > to) {
      const jshort *end = from + count;
      while (from < end)
        *(to++) = *(from++);
    }
    else if (from < to) {
      const jshort *end = from;
      from += count - 1;
      to   += count - 1;
      while (from >= end)
        *(to--) = *(from--);
    }
  }
  void _Copy_conjoint_jints_atomic(const jint* from, jint* to, size_t count) {
    if (from > to) {
      const jint *end = from + count;
      while (from < end)
        *(to++) = *(from++);
    }
    else if (from < to) {
      const jint *end = from;
      from += count - 1;
      to   += count - 1;
      while (from >= end)
        *(to--) = *(from--);
    }
  }
  void _Copy_conjoint_jlongs_atomic(const jlong* from, jlong* to, size_t count) {
    if (from > to) {
      const jlong *end = from + count;
      while (from < end)
        os::atomic_copy64(from++, to++);
    }
    else if (from < to) {
      const jlong *end = from;
      from += count - 1;
      to   += count - 1;
      while (from >= end)
        os::atomic_copy64(from--, to--);
    }
  }

  void _Copy_arrayof_conjoint_bytes(const HeapWord* from,
                                    HeapWord* to,
                                    size_t    count) {
    memmove(to, from, count);
  }
  void _Copy_arrayof_conjoint_jshorts(const HeapWord* from,
                                      HeapWord* to,
                                      size_t    count) {
    memmove(to, from, count * 2);
  }
  void _Copy_arrayof_conjoint_jints(const HeapWord* from,
                                    HeapWord* to,
                                    size_t    count) {
    memmove(to, from, count * 4);
  }
  void _Copy_arrayof_conjoint_jlongs(const HeapWord* from,
                                     HeapWord* to,
                                     size_t    count) {
    memmove(to, from, count * 8);
  }
};

/////////////////////////////////////////////////////////////////////////////
// Implementations of atomic operations not supported by processors.
//  -- http://gcc.gnu.org/onlinedocs/gcc-4.2.1/gcc/Atomic-Builtins.html

#ifndef _LP64
extern "C" {
  long long unsigned int __sync_val_compare_and_swap_8(
    volatile void *ptr,
    long long unsigned int oldval,
    long long unsigned int newval) {
    ShouldNotCallThis();
  }
};
#endif // !_LP64

#ifndef PRODUCT
void os::verify_stack_alignment() {
}
#endif

int os::extra_bang_size_in_bytes() {
  // Zero does not require an additional stack bang.
  return 0;
}
