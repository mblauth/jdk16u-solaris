/*
 * Copyright (c) 2002, 2020, Oracle and/or its affiliates. All rights reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.  Oracle designates this
 * particular file as subject to the "Classpath" exception as provided
 * by Oracle in the LICENSE file that accompanied this code.
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
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#ifdef __solaris__
#include <sys/systeminfo.h>
#endif

#include <string.h>

#include "jni.h"

#ifndef MAXDNAME
#define MAXDNAME                1025
#endif


/*
 * Class:     sun_net_dns_ResolverConfigurationImpl
 * Method:    localDomain0
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_sun_net_dns_ResolverConfigurationImpl_localDomain0(JNIEnv *env, jclass cls)
{
    /*
     * On Solaris the LOCALDOMAIN environment variable has absolute
     * priority.
     */
#ifdef __solaris__
    {
        char *cp = getenv("LOCALDOMAIN");
        if (cp != NULL) {
            jstring s = (*env)->NewStringUTF(env, cp);
            return s;
        }
    }
#endif
    return (jstring)NULL;
}

/*
 * Class:     sun_net_dns_ResolverConfigurationImpl
 * Method:    loadConfig0
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_sun_net_dns_ResolverConfigurationImpl_fallbackDomain0(JNIEnv *env, jclass cls)
{
    char buf[MAXDNAME];

    /*
     * On Solaris if domain or search directives aren't specified
     * in /etc/resolv.conf then sysinfo or gethostname is used to
     * determine the domain name.
     *
     * On Linux if domain or search directives aren't specified
     * then gethostname is used.
     */

#ifdef __solaris__
    {
        int ret = sysinfo(SI_SRPC_DOMAIN, buf, sizeof(buf));

        if ((ret > 0) && (ret<sizeof(buf))) {
            char *cp;
            jstring s;

            if (buf[0] == '+') {
                buf[0] = '.';
            }
            cp = strchr(buf, '.');
            if (cp == NULL) {
                s = (*env)->NewStringUTF(env, buf);
            } else {
                s = (*env)->NewStringUTF(env, cp+1);
            }
            return s;
        }
    }
#endif

    if (gethostname(buf, sizeof(buf)) == 0) {
        char *cp;

        /* gethostname doesn't null terminate if insufficient space */
        buf[sizeof(buf)-1] = '\0';

        cp = strchr(buf, '.');
        if (cp != NULL) {
            jstring s = (*env)->NewStringUTF(env, cp+1);
            return s;
        }
    }

    return (jstring)NULL;
}
