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

#include <mailbox_config.h>
#include "app_mailbox_ipc.h"

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define LOCAL_DELAY_COUNT              (0x10)
#define MAILBOX_APP_SYNC_MESSAGE       (0xBABEFACE)
#define MAILBOX_APP_ACK_MESSAGE        (0xC00DC00D)
#define MAILBOX_INT_PRIORITY           (0x1U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

app_mbxipc_obj_t g_app_mbxipc_obj;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void appMailboxIsr(void *handle);

int appMbxIpcSync(void);

int32_t appMbxIpcInterruptInit(uint32_t intNum, uint16_t remoteId);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

__attribute__((interrupt("IRQ")))     void appMailboxIsr_0(void);
__attribute__((interrupt("IRQ")))     void appMailboxIsr_1(void);
__attribute__((interrupt("IRQ")))     void appMailboxIsr_2(void);

void appMailboxIsr_0(void)
{
    appMailboxIsr((void *)0u);
}
void appMailboxIsr_1(void)
{
    appMailboxIsr((void *)1u);
}
void appMailboxIsr_2(void)
{
    appMailboxIsr((void *)2u);
}

void *mailboxIsrArray[3] =
{
    (void *)&appMailboxIsr_0,
    (void *)&appMailboxIsr_1,
    (void *)&appMailboxIsr_2,
};


void appMbxIpcInitPrmSetDefault(app_mbxipc_init_prm_t *prm)
{
    uint32_t cpu_id = 0;

    prm->num_cpus = 0;

    for(cpu_id=0; cpu_id<MAILBOX_IPC_MAX_PROCS; cpu_id++)
    {
        prm->enabled_cpu_id_list[cpu_id] = APP_IPC_CPU_INVALID;
    }

    prm->self_cpu_id = APP_IPC_CPU_INVALID;
    prm->master_cpu_id = APP_IPC_CPU_INVALID;
}

int32_t appMbxIpcRegisterNotifyHandler(app_mbxipc_notify_handler_f handler)
{
    int32_t status = -1;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    if(handler)
    {
        obj->mbxipc_notify_handler = handler;
        status = 0;
    }

    return status;
}

uint32_t appMbxIpcGetMasterCpuId(void)
{
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    return obj->prm.master_cpu_id;
}

uint32_t appMbxIpcGetSelfCpuId(void)
{
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    return obj->prm.self_cpu_id;
}

uint32_t appMbxIpcIsCpuEnabled(uint32_t cpu_id)
{
    uint32_t is_enabled = 0, cur_cpu_id;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    if(cpu_id>=MAILBOX_IPC_MAX_PROCS)
    {
        is_enabled = 0;
    }
    for(cur_cpu_id=0; cur_cpu_id<obj->prm.num_cpus; cur_cpu_id++)
    {
        if(cpu_id==obj->prm.enabled_cpu_id_list[cur_cpu_id])
        {
            is_enabled = 1;
            break;
        }
    }
    return is_enabled;
}

int32_t appMbxIpcInterruptInit(uint32_t intNum, uint16_t remoteId)
{
    int32_t retVal = 0;

    /* Register Mailbox interrupt using VIM direct INT registration */
    if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_1))
    {
        CSL_vimCfgIntr((CSL_vimRegs *)(uintptr_t)VIM_BASE_ADDR, intNum,
                   MAILBOX_INT_PRIORITY,
                   (CSL_VimIntrMap)CSL_VIM_INTR_MAP_IRQ,
                   CSL_VIM_INTR_TYPE_LEVEL,
                   (uint32_t)mailboxIsrArray[remoteId] );
        CSL_vimSetIntrEnable((CSL_vimRegs *)(uintptr_t)VIM_BASE_ADDR,
                         intNum, true );   /* Enable interrupt in vim */
        Intc_SystemEnable();
    }
    else
    {
        /* Register Mailbox interrupt using OSAL APIs */
        OsalRegisterIntrParams_t intrPrms;
        OsalInterruptRetCode_e osalRetVal;
        HwiP_Handle hwiHandle;

        Osal_RegisterInterrupt_initParams(&intrPrms);
        intrPrms.corepacConfig.arg          = (uintptr_t)remoteId;
        intrPrms.corepacConfig.priority     = MAILBOX_INT_PRIORITY;
        intrPrms.corepacConfig.corepacEventNum = CSL_VIM_INTR_MAP_IRQ; /* NOT USED */
        intrPrms.corepacConfig.isrRoutine   = (Osal_IsrRoutine) &appMailboxIsr;
        intrPrms.corepacConfig.intVecNum    = intNum;

        osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);
        if(OSAL_INT_SUCCESS != osalRetVal)
        {
           appLogPrintf (" MBX IPC : Error Could not register ISR !!!\n");
           retVal = -1;
        }
    }
    return retVal;
}

int32_t appMbxIpcDeInit(void)
{
    int32_t retVal = 0;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    appLogPrintf("MBX-IPC: DeInit ... !!!\n");

	appMbxIpcInitPrmSetDefault(&obj->prm);
    obj->mbxipc_notify_handler = NULL;

    appLogPrintf("MBX-IPC: DeInit... Done !!!\n");

    return retVal;
}

void appMailboxIsr(void *handle)
{
    int16_t remoteId;
    uint32_t baseAddr;
    uint32_t fifo;
    uint32_t user;
    uint32_t payload;
    uint32_t selfId = appMbxIpcGetSelfCpuId();
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    remoteId = (uint16_t)((uintptr_t)handle);

    baseAddr = gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[selfId][remoteId].rx.cluster];
    fifo = gMailboxIpc_MailboxInfo[selfId][remoteId].rx.fifo;
    user = gMailboxIpc_MailboxInfo[selfId][remoteId].rx.user;

    MailboxReadMessage(baseAddr, fifo, &payload);

    if(obj->mbxipc_notify_handler)
    {
        obj->mbxipc_notify_handler(remoteId, payload);
    }

    /* clear mlb intr */
    MailboxClrNewMsgStatus(baseAddr, fifo, user);
    if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_1))
    {
        CSL_vimClrIntrPending((CSL_vimRegs *)(uintptr_t)VIM_BASE_ADDR,
                    gMailboxIpc_MailboxInterruptInfo[remoteId]);
        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr((CSL_vimRegs *)(uintptr_t)VIM_BASE_ADDR, \
                    (CSL_VimIntrMap)CSL_VIM_INTR_MAP_IRQ );
    }
}

int appMbxIpcSync(void)
{
    uint16_t remoteId;
    int retVal = 0;
	uint32_t msgStatus = MESSAGE_INVALID;
	uint32_t selfId = appMbxIpcGetSelfCpuId();
	uint32_t payload;

	if (selfId == appMbxIpcGetMasterCpuId())
	{
		/* Send sync message to all cores */
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
			/* Send initial Sync message */
			MailboxSendMessage(gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[remoteId][selfId].rx.cluster],
							   gMailboxIpc_MailboxInfo[remoteId][selfId].rx.fifo,
							   (uint32_t) MAILBOX_APP_SYNC_MESSAGE);
		}

		/* Receive Ack message back for all cores */
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
			do
			{
				msgStatus = MailboxGetMessage(gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[selfId][remoteId].rx.cluster],
											   gMailboxIpc_MailboxInfo[selfId][remoteId].rx.fifo,
											   &payload);
			} while (msgStatus == MESSAGE_INVALID);
			if (payload != MAILBOX_APP_ACK_MESSAGE)
			{
				retVal = -1;
			}
		}
	}
	else
	{
		remoteId = appMbxIpcGetMasterCpuId();
		do
		{
			msgStatus = MailboxGetMessage(gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[selfId][remoteId].rx.cluster],
										   gMailboxIpc_MailboxInfo[selfId][remoteId].rx.fifo,
										   &payload);
		} while (msgStatus == MESSAGE_INVALID);
		if (payload != MAILBOX_APP_SYNC_MESSAGE)
		{
			retVal = -1;
		}
		if ( retVal == 0)
		{
			/* Send initial Sync message */
			MailboxSendMessage(gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[remoteId][selfId].rx.cluster],
							   gMailboxIpc_MailboxInfo[remoteId][selfId].rx.fifo,
							   (uint32_t) MAILBOX_APP_ACK_MESSAGE);
		}
	}

    return retVal;
}

void appMbxIpcSendNotify(uint32_t remoteId, uint32_t payload)
{
    uint32_t remoteBaseAddr, remoteFifo;
    uint32_t selfId = appMbxIpcGetSelfCpuId();

    remoteBaseAddr = gMailboxIpc_MailboxBaseAddressArray[gMailboxIpc_MailboxInfo[remoteId][selfId].rx.cluster];
    remoteFifo = gMailboxIpc_MailboxInfo[remoteId][selfId].rx.fifo;

    MailboxWriteMessage(remoteBaseAddr,
                        remoteFifo,
                        (uint32_t) payload);
    return;
}


/********************************* End of file ******************************/
