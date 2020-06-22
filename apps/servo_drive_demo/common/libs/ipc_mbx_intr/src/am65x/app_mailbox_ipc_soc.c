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
#include "app_mailbox_ipc.h"

extern app_mbxipc_obj_t g_app_mbxipc_obj;
extern int appMbxIpcSync(void);
extern int32_t appMbxIpcInterruptInit(uint32_t intNum, uint16_t remoteId);

int32_t appMbxIpcInit(app_mbxipc_init_prm_t *prm)
{
    int32_t retVal = 0;
    uint16_t remoteId;
    uint16_t interruptOffset;
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    /* Enable interrupt router settings to connect interrupt event */
    struct tisci_msg_rm_get_resource_range_resp res;
    struct tisci_msg_rm_get_resource_range_req  req;
    uint16_t intStartNum, intRangeNum;
    uint16_t intNum, dst_input;
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

    if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_0))
    {
        req.type           = TISCI_DEV_MAIN2MCU_LVL_INTRTR0;
        req.subtype        = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
        req.secondary_host = (uint8_t)TISCI_HOST_ID_R5_0;
    }

    if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_1))
    {
        req.type           = TISCI_DEV_MAIN2MCU_LVL_INTRTR0;
        req.subtype        = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
        req.secondary_host = (uint8_t)TISCI_HOST_ID_R5_2;
    }

    /* Get interrupt number range */
    retVal =  Sciclient_rmGetResourceRange(
              &req,
              &res,
              MAILBOX_SCICLIENT_TIMEOUT);
    if (CSL_PASS != retVal || res.range_num == 0)
    {
        /* Try with HOST_ID_ALL */
        req.secondary_host = TISCI_HOST_ID_ALL;

        retVal = Sciclient_rmGetResourceRange(
                 &req,
                 &res,
                 MAILBOX_SCICLIENT_TIMEOUT);
    }
    /* Add an offset of 24 to avoid interrupt number resource
       conflict with EtherCAT slave which uses the first 17 */
    res.range_start += 24;
    res.range_num = res.range_num - 24;

    if (CSL_PASS == retVal)
    {

        intStartNum = res.range_start;
        intRangeNum = res.range_num;
        if (intRangeNum == 0)
        {
            retVal = -1;
        }
    }

    if (retVal == 0)
    {
		if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_0))
		{
			rmIrqReq.dst_id         = TISCI_DEV_MCU_ARMSS0_CPU0;
			rmIrqReq.secondary_host = TISCI_HOST_ID_R5_0;
		}

		if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_1))
		{
			rmIrqReq.dst_id         = TISCI_DEV_MCU_ARMSS0_CPU1;
			rmIrqReq.secondary_host = TISCI_HOST_ID_R5_2;
		}

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

            intNum = intStartNum + interruptOffset;

            if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_0))
            {
                retVal = Sciclient_rmIrqTranslateIrOutput(
                            TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
                            intNum, TISCI_DEV_MCU_ARMSS0_CPU0,&dst_input);
            }

            if (IS_CPU_ENABLED(MAILBOX_IPC_CPUID_MCU1_1))
            {
                retVal = Sciclient_rmIrqTranslateIrOutput(
                            TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
                            intNum, TISCI_DEV_MCU_ARMSS0_CPU1,&dst_input);
            }

            /* Store the interrupt number */
            gMailboxIpc_MailboxInterruptInfo[remoteId] = dst_input;

            if (retVal == 0)
            {

                rmIrqReq.ia_id                  = 0U;
                rmIrqReq.vint                   = 0U;
                rmIrqReq.global_event           = 0U;
                rmIrqReq.vint_status_bit_index  = 0U;

                rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID
                                          | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID
                                          | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
                rmIrqReq.src_id         = gMailboxIpc_MailboxClusterIdArray[gMailboxIpc_MailboxInfo[selfId][remoteId].rx.cluster];
                rmIrqReq.src_index      = gMailboxIpc_MailboxInfo[selfId][remoteId].rx.user;
                rmIrqReq.dst_host_irq   = dst_input;

                /* Config event */
                retVal = Sciclient_rmIrqSet(
                             &rmIrqReq, &rmIrqResp, MAILBOX_SCICLIENT_TIMEOUT);
            }

            if (retVal == 0)
            {
                /* Configure and initalize interrupt handler */
                retVal = appMbxIpcInterruptInit(dst_input, remoteId);
            }

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
