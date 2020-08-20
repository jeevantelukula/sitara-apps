/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <ti/csl/tistdtypes.h>
#include <ti/osal/CacheP.h>
#include "app_pslctrl_mbxipc.h"

/* Global data MSG objects used in IPC communication */
appPslCtrlReceiveMsgObj_t   gAppPslCtrlRxMsgAxes[MAX_NUM_AXES]  /* receive CtrlVars parameters per axis */
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;
appPslCtrlSendMsgObj_t      gAppPslCtrlTxMsgAxes[MAX_NUM_AXES]  /* send CtrlVars parameters per axis */
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;

/* MBX IPC Rx message ISR count */
uint32_t gTotMbxIpcRxMsgCnt = 0;
/* Per-axis MBX IPC Rx message ISR count */
uint32_t gMbxIpcRxMsgCnt[MAX_NUM_AXES] = {0};
/* MBX IPC Rx message ISR error count */
uint32_t gMbxIpcRxMsgErrCnt = 0;

/* Initialize PSL Control MBX IPC */
int32_t appPslCtrlMbxIpcInit(
    appPslCtrlMbxIpcCfg_t *pPslCtrlMbxIpcCfg
)
{
    app_mbxipc_init_prm_t mbxipc_init_prm;
    uint16_t i;
    int32_t status;
    
    /* Initialize Mailbox IPC */
    /* IPC cpu sync check works only when appMbxIpcInit() called from both R5Fs */
    appMbxIpcInitPrmSetDefault(&mbxipc_init_prm);
    mbxipc_init_prm.master_cpu_id = pPslCtrlMbxIpcCfg->appMbxIpcInitPrm.master_cpu_id;
    mbxipc_init_prm.self_cpu_id = pPslCtrlMbxIpcCfg->appMbxIpcInitPrm.self_cpu_id;
    mbxipc_init_prm.num_cpus = pPslCtrlMbxIpcCfg->appMbxIpcInitPrm.num_cpus;
    for (i = 0; i < pPslCtrlMbxIpcCfg->appMbxIpcInitPrm.num_cpus; i++)
    {
        mbxipc_init_prm.enabled_cpu_id_list[i] = pPslCtrlMbxIpcCfg->appMbxIpcInitPrm.enabled_cpu_id_list[i];
    }
    status = appMbxIpcInit(&mbxipc_init_prm);
    if (status != 0)
    {
        return APP_PSLCTRL_MBXIPC_SERR_MBXINIT;
    }
    
    /* Register Mailbox IPC ISR */
    status = appMbxIpcRegisterNotifyHandler((app_mbxipc_notify_handler_f) pPslCtrlMbxIpcCfg->appMbxIpcMsgHandler);
    if (status != 0)
    {
        return APP_PSLCTRL_MBXIPC_SERR_REGISR;
    }
    
    for (i = 0; i < MAX_NUM_AXES; i++)
    {
        gAppPslCtrlRxMsgAxes[i].isMsgReceived = 0;
    }

    return APP_PSLCTRL_MBXIPC_SOK;
}

/* Mailbox IPC Rx message handler */
void appMbxIpcMsgHandler(uint32_t src_cpu_id, uint32_t payload)
{
    mc2ecat_msg_obj_t *rxobj;
    mc2ecat_msg_obj_t *payload_ptr;
    uint16_t axisIdx;
    
    gTotMbxIpcRxMsgCnt++;

    if (src_cpu_id == IPC_PSL_MC_CPU_ID)
    {
        payload_ptr = (mc2ecat_msg_obj_t *)payload;
        axisIdx = payload_ptr->u16AxisIndex;

        if (axisIdx < MAX_NUM_AXES)
        {
            gMbxIpcRxMsgCnt[axisIdx]++;

            rxobj = &gAppPslCtrlRxMsgAxes[axisIdx].receiveObj;
            *rxobj = *payload_ptr;
            gAppPslCtrlRxMsgAxes[axisIdx].isMsgReceived = 1;
        }
        else
        {
            gMbxIpcRxMsgErrCnt++;
        }
    }
}
