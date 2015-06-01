/*
Copyright (c) 2014, Gemalto. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

/** @file
  TODO: Brief Description of UEFI Driver SmartCardReader

  TODO: Detailed Description of UEFI Driver SmartCardReader

**/

#ifndef __EFI_DEBUGLOG_H__
#define __EFI_DEBUGLOG_H__

#include "config.h"
#include <Uefi.h>

//
// Libraries
//
#include <Library/DebugLib.h>



/* Debug macros */
#ifdef NO_LOG

#define Log0(priority) do { } while(0)
#define Log1(priority, fmt) do { } while(0)
#define Log2(priority, fmt, data) do { } while(0)
#define Log3(priority, fmt, data1, data2) do { } while(0)
#define Log4(priority, fmt, data1, data2, data3) do { } while(0)
#define Log5(priority, fmt, data1, data2, data3, data4) do { } while(0)
#define Log9(priority, fmt, data1, data2, data3, data4, data5, data6, data7, data8) do { } while(0)
#define LogXxd(priority, msg, buffer, size) do { } while(0)

#else

#define Log0(priority) DEBUG((priority, "%d:%a()\n", __LINE__, __FUNCTION__))
#define Log1(priority, fmt) DEBUG((priority, "%d:%a() " fmt "\n", __LINE__, __FUNCTION__))
#define Log2(priority, fmt, data) DEBUG((priority, "%d:%a() " fmt "\n", __LINE__, __FUNCTION__, data))
#define Log3(priority, fmt, data1, data2) DEBUG((priority, "%d:%a() " fmt "\n", __LINE__, __FUNCTION__, data1, data2))
#define Log4(priority, fmt, data1, data2, data3) DEBUG((priority, "%d:%a() " fmt "\n", __LINE__, __FUNCTION__, data1, data2, data3))
#define Log5(priority, fmt, data1, data2, data3, data4) DEBUG((priority, "%d:%a() " fmt "\n", __LINE__, __FUNCTION__, data1, data2, data3, data4))
#define Log9(priority, fmt, data1, data2, data3, data4, data5, data6, data7, data8) DEBUG((priority, "%d:%a() " fmt "\n", __LINE__, __FUNCTION__, data1, data2, data3, data4, data5, data6, data7, data8))
#define LogXxd(priority, msg, buffer, size) log_xxd((priority, msg, buffer, size))
#endif

#define PCSC_LOG_DEBUG EFI_D_WARN
#define PCSC_LOG_INFO EFI_D_INFO
#define PCSC_LOG_ERROR EFI_D_ERROR
#define PCSC_LOG_CRITICAL EFI_D_ERROR

void log_xxd(const int priority, const char *msg,
    const unsigned char *buffer, const int size);

#endif
