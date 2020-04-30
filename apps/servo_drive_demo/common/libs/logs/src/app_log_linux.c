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
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#if _POSIX_C_SOURCE >= 199309L
#include <time.h>   /* for nanosleep */
int nanosleep(const struct timespec *req, struct timespec *rem);
#else
#include <unistd.h> /* for usleep */
extern int usleep (__useconds_t __useconds);
#endif
#include <utils/logs/include/app_log.h>


uint64_t appLogGetTimeInUsec()
{
    uint64_t timeInUsecs = 0;
    struct timeval tv;
    static uint64_t g_start_time = 0;
    
    if (gettimeofday(&tv, NULL) < 0)
    {
        timeInUsecs = 0;
    }
    else
    {
        timeInUsecs = tv.tv_sec * 1000000ull + tv.tv_usec;
    }
    if(g_start_time==0)
    {
        g_start_time = timeInUsecs;
    }

    return (timeInUsecs-g_start_time);
}

void appLogWaitMsecs(uint32_t time_in_msecs)
{
#if _POSIX_C_SOURCE >= 199309L
    struct timespec delay_time, remain_time;
    int ret;

    delay_time.tv_sec  = time_in_msecs/1000;
    delay_time.tv_nsec = (time_in_msecs%1000)*1000000;

    do
    {
        ret = nanosleep(&delay_time, &remain_time);
        if(ret < 0 && remain_time.tv_sec > 0 && remain_time.tv_nsec > 0)
        {
            /* restart for remaining time */
            delay_time = remain_time;
        }
        else
        {
            break;
        }
    } while(1);
#else
    usleep(msec * 1000);
#endif
}

