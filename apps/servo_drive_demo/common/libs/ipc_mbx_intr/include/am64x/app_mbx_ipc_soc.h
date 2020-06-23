/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/


#ifndef APP_MBX_IPC_SOC_H_
#define APP_MBX_IPC_SOC_H_

/**
 * \defgroup group_apps_utils_ipc Inter-processor communication (IPC) APIs
 *
 * \brief This section contains APIs for Inter-processor communication (IPC)
 *
 * \ingroup group_apps_utils
 *
 * @{
 */
 
/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/** \brief Core definitions */
#define MAILBOX_IPC_CPUID_MPU1_0     (0U)    /**< ARM A53 - VM0 */
#define MAILBOX_IPC_CPUID_MCU1_0     (1U)    /**< ARM MAINSS1  R5F - core0 */
#define MAILBOX_IPC_CPUID_MCU1_1     (2U)    /**< ARM MAINSS1  R5F - core1 */
#define MAILBOX_IPC_CPUID_MCU2_0     (3U)    /**< ARM MAINSS2  R5F - core0 */
#define MAILBOX_IPC_CPUID_MCU2_1     (4U)    /**< ARM MAINSS2  R5F - core1 */
#define MAILBOX_IPC_CPUID_M4F_0      (5U)    /**< ARM MCUSS  M4F - core0 */
#define MAILBOX_IPC_MAX_PROCS        (6U)    /**< Maximum Processors */

/* @} */

#endif

