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


#ifndef MAILBOX_CONFIG_H_
#define MAILBOX_CONFIG_H_


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#include <ti/csl/soc.h>
#include <ti/drv/sciclient/sciclient.h>
#include <app_mbx_ipc.h>
#include <ti/csl/cslr_gtc.h>

/** \brief Invalid CPU ID */
#define APP_IPC_CPU_INVALID        (0xFFu)

/* In general , Get this from CSL, not available for AM65XX */
#define MAILBOX_MAX_CLUSTER_CNT    (12U)

#define MAILBOX_CLUSTER_INVALID    (0xFFU)
#define MAILBOX_USER_INVALID       (0xFFU)

#define MAILBOX_SCICLIENT_TIMEOUT  (0xffffffffu)

#define VIM_BASE_ADDR              (CSL_MCU_DOMAIN_VIM_BASE_ADDR)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */

typedef struct mailboxIpc_MailboxEntry_s
{
    uint32_t    cluster;
    uint32_t    user;
    uint32_t    fifo;
} mailboxIpc_MailboxEntry;

typedef struct mailboxIpc_MailboxInfo_s
{
    mailboxIpc_MailboxEntry    rx;
} mailboxIpc_MailboxInfo;

extern const mailboxIpc_MailboxInfo gMailboxIpc_MailboxInfo[MAILBOX_IPC_MAX_PROCS][MAILBOX_IPC_MAX_PROCS];
extern const uint32_t gMailboxIpc_MailboxBaseAddressArray[MAILBOX_MAX_CLUSTER_CNT];
extern const uint32_t gMailboxIpc_MailboxClusterIdArray[MAILBOX_MAX_CLUSTER_CNT];
extern uint32_t gMailboxIpc_MailboxInterruptInfo[MAILBOX_IPC_MAX_PROCS];


#endif /* MAILBOX_CONFIG_H_ */

