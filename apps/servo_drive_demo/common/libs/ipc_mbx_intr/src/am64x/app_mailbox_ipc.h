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


#ifndef APP_MAILBOX_IPC_H_
#define APP_MAILBOX_IPC_H_


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr_gtc.h>
#include <ti/drv/mailbox/mailbox.h>
#include <app_mbx_ipc.h>


/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/** \brief Invalid CPU ID */
#define APP_IPC_CPU_INVALID            (0xFFu)

#define LOCAL_DELAY_COUNT              (0x10)
#define MAILBOX_APP_SYNC_MESSAGE       (0xBABEFACE)
#define MAILBOX_APP_ACK_MESSAGE        (0xC00DC00D)

#define IS_CPU_ENABLED(x) (appMbxIpcIsCpuEnabled(x) && (appMbxIpcGetSelfCpuId()==x))

/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */
/* IPC global data object */
typedef struct {
    app_mbxipc_init_prm_t prm;
    app_mbxipc_notify_handler_f mbxipc_notify_handler;
    Mbox_Handle handle[MAILBOX_IPC_MAX_PROCS];
} app_mbxipc_obj_t;


#endif /* APP_MAILBOX_IPC_H_ */

