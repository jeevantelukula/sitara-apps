/*
 * Copyright (C) 2017-2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "app_log.h"

#define APP_LOG_BUF_MAX  (256u)

void appLogPrintf(const char *format, ...)
{
    va_list va_args_ptr;
    uint32_t str_len = 0;
    uint64_t cur_time;
    char buf[APP_LOG_BUF_MAX];    

    cur_time = appLogGetTimeInUsec();
    str_len  = (uint32_t)snprintf(buf, APP_LOG_BUF_MAX,
                "%6d.%06u s: ",
                (uint32_t)(cur_time / 1000000U),
                (uint32_t)(cur_time % 1000000U));

    /* if str_len is equal to APP_LOG_BUF_MAX, i.e string overflows buffer,
       then don't write string. */
    if (str_len < APP_LOG_BUF_MAX)
    {
        va_start(va_args_ptr, format);
        vsnprintf((char *)(buf + str_len),
                  APP_LOG_BUF_MAX - str_len,
                    format, va_args_ptr);
        va_end(va_args_ptr);

        printf(buf);
    }
}

