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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_mailbox.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/osal/osal.h>
#include <ti/drv/sciclient/sciclient.h>

#include <app_log.h>
#include <app_mbx_ipc.h>

#include "mailbox_config.h"

extern app_mbxipc_obj_t g_app_mbxipc_obj;
extern int appMbxIpcSync(void);
extern int32_t appMbxIpcInterruptInit(uint32_t intNum, uint16_t remoteId);

int32_t appMbxIpcInit(app_mbxipc_init_prm_t *prm)
{
    int32_t retVal = 0;
    uint16_t remoteId;
    uint16_t interruptOffset;
    uint32_t selfId;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;
    obj->prm = *prm;
    obj->mbxipc_notify_handler = NULL;

    appLogPrintf("MBX-IPC: Init ... !!!\n");

    selfId = appMbxIpcGetSelfCpuId();

    retVal = appMbxIpcSync();
    if ( retVal != 0)
    {
        appLogPrintf("MBX-IPC: appMbxIpcSync() failed \r\n");
        return -1;
    }

    if (retVal == 0)
    {
        interruptOffset = 0;
        for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
        {
            /* Skip self */
            if (remoteId == selfId)
            {
                continue;
            }
            if (!appMbxIpcIsCpuEnabled(remoteId))
            {
                continue;
            }

            /* Store the interrupt number */
            gMailboxIpc_MailboxInterruptInfo[remoteId] = MAILBOX_IPC_R5F_CLUSTER0_INT_NUM;

            /* Configure and initalize interrupt handler */
            retVal = appMbxIpcInterruptInit(MAILBOX_IPC_R5F_CLUSTER0_INT_NUM, remoteId);

            if (retVal == 0)
            {
                /* Enable Interrupt at the mailbox */
                MailboxEnableNewMsgInt(gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[selfId][remoteId].rx.cluster],
                                       gMailboxIpc_MailboxInfo[selfId][remoteId].rx.user,
                                       gMailboxIpc_MailboxInfo[selfId][remoteId].rx.fifo);

            }
            if ( retVal != 0)
            {
                break;
            }
            interruptOffset++;
        }
    }

    if (retVal)
    {
        appLogPrintf("MBX-IPC: Init ... Failed !!!\n");
    }
    else
    {
        appLogPrintf("MBX-IPC: Init ... Done !!!\n");
    }

    return retVal;
}

/********************************* End of file ******************************/
