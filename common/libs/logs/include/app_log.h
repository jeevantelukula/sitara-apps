/*
 * Copyright (C) 2018-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef APP_LOG_H_
#define APP_LOG_H_

#include <stdint.h>

/**
 * \defgroup group_apps_utils_log Logging APIs
 *
 * \brief This section contains APIs for logging from remote cores to a
 *        central host core which prints all the logs to a common console.
 *
 * \ingroup group_apps_utils
 *
 * @{
 */

/** \brief Max CPUs that are participating in the logging */
#define APP_LOG_MAX_CPUS        (16u)

/** \brief Write a string to shared memory
 *
 * \param format [in] string to log with variable number of arguments
 *
 * \return 0 on success, else failure.
 */
void appLogPrintf(const char *format, ...);

/** \brief Get current time in units of usecs
 *
 * \return current time in units of usecs
 */
uint64_t appLogGetTimeInUsec();

/** \brief Pending on 'n' msecs
 *
 * \param time_in_msecs [in] Time in units of msecs
 */
void appLogWaitMsecs(uint32_t time_in_msecs);

/* @} */

#endif

