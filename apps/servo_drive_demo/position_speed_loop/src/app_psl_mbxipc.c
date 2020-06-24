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
#include <ti/osal/osal.h>
#include "hw_types.h"
#include "fcl_enum.h"
#include "cia402appl.h"
#include "cfg_mcu_intr_soc.h"
#include "app_cfg_soc.h"
#include "app_psl_mbxipc.h"

/* Translate Rx MC parameters, write to control variables for node */
static int32_t xlateRxMcParams(
    ecat2mc_msg_obj_t *rxobj, 
    CTRL_Vars_t *pCtrl
);
/* Translate MC feedback variables for node, write to control Tx MC parameters */
static inline int32_t xlateTxMcParams(
    mc2ecat_msg_obj_t *txobj, 
    CTRL_Vars_t *pCtrl
);

/* Global data MSG objects used in IPC communication */
appPslReceiveMsgObj_t   gAppPslRxMsgAxes[MAX_NUM_AXES]  /* receive CtrlVars parameters per axis */
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;
appPslSendMsgObj_t      gAppPslTxMsgAxes[MAX_NUM_AXES]  /* send CtrlVars parameters per axis */
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;

/* MBX IPC Rx message ISR count */
uint32_t gTotMbxIpcRxMsgCnt = 0;
/* Per-axis MBX IPC Rx message ISR count */
uint32_t gMbxIpcRxMsgCnt[MAX_NUM_AXES] = {0};
/* MBX IPC Rx message ISR error count */
uint32_t gMbxIpcRxMsgErrCnt = 0;

/* Initialize PSL MBX IPC */
int32_t appPslMbxIpcInit(
    appPslMbxIpcCfg_t *pPslMbxIpcCfg
)
{
    app_mbxipc_init_prm_t mbxipc_init_prm;
    uint16_t i;
    int32_t status;
    
    /* Initialize Mailbox IPC */
    /* IPC cpu sync check works only when appMbxIpcInit() called from both R5Fs */
    appMbxIpcInitPrmSetDefault(&mbxipc_init_prm);
    mbxipc_init_prm.master_cpu_id = pPslMbxIpcCfg->appMbxIpcInitPrm.master_cpu_id;
    mbxipc_init_prm.self_cpu_id = pPslMbxIpcCfg->appMbxIpcInitPrm.self_cpu_id;
    mbxipc_init_prm.num_cpus = pPslMbxIpcCfg->appMbxIpcInitPrm.num_cpus;
    for (i = 0; i < pPslMbxIpcCfg->appMbxIpcInitPrm.num_cpus; i++)
    {
        mbxipc_init_prm.enabled_cpu_id_list[i] = pPslMbxIpcCfg->appMbxIpcInitPrm.enabled_cpu_id_list[i];
    }
    status = appMbxIpcInit(&mbxipc_init_prm);
    if (status != 0)
    {
        return APP_PSL_MBXIPC_SERR_MBXINIT;
    }
    
    /* Register Mailbox IPC ISR */
    status = appMbxIpcRegisterNotifyHandler((app_mbxipc_notify_handler_f) pPslMbxIpcCfg->appMbxIpcMsgHandler);
    if (status != 0)
    {
        return APP_PSL_MBXIPC_SERR_REGISR;
    }
    
    for (i = 0; i < MAX_NUM_AXES; i++)
    {
        gAppPslRxMsgAxes[i].isMsgReceived = 0;
    }

    return APP_PSL_MBXIPC_SOK;
}

/* Translate Rx MC parameters, write to control variables for node */
static int32_t xlateRxMcParams(
    ecat2mc_msg_obj_t *rxobj, 
    CTRL_Vars_t *pCtrl
)
{
    /* Update target speed & position */
    pCtrl->speedSet = ECAT_puToFloat(rxobj->i32TargetVelocity);
    pCtrl->positionSet = ECAT_puToFloat(rxobj->i32TargetPosition);

    if (rxobj->i16ModesOfOperation == CYCLIC_SYNC_VELOCITY_MODE) {
        /* CSV mode */
        
        if (rxobj->i16State == STATE_OPERATION_ENABLED) {
            /* Motor enabled */
            pCtrl->ctrlStateSet = CTRL_RUN;
        }
        else if (rxobj->i16State == STATE_READY_TO_SWITCH_ON) {
            /* Motor disabled */
            pCtrl->ctrlStateSet = CTRL_STOP;
        }
        else {
            /* Unsupported state, disable motor */
            pCtrl->ctrlStateSet = CTRL_STOP;
        }
    }
    else if (rxobj->i16ModesOfOperation == CYCLIC_SYNC_POSITION_MODE) {
        /* CSP mode currently unsupported, disable motor */
        pCtrl->ctrlStateSet = CTRL_STOP;
    }
    else {
        /* Unsupported mode, disable motor */
        pCtrl->ctrlStateSet = CTRL_STOP;
    }
    
    return APP_PSL_MBXIPC_SOK;
}

/* Mailbox IPC, receive message for MC node (axis) */
int32_t appPslMbxIpcRxMsg(
    SysNode_e nodeIdx
)
{
    uintptr_t key;
    ecat2mc_msg_obj_t *rxobj;    
    uint16_t mcAxisIdx;

    /* Translate slave node index to MBX IPC mailbox index */
    mcAxisIdx = (uint16_t)(nodeIdx - 1);
    
    /* Get latest target values from EtherCAT */
    if (mcAxisIdx < MAX_NUM_AXES) {
        /* Enter critical section (Rx mailbox message receive flag), disable interrupts */
        key = HwiP_disable();
        if (gAppPslRxMsgAxes[mcAxisIdx].isMsgReceived == 1) {
            gAppPslRxMsgAxes[mcAxisIdx].isMsgReceived = 0;        
            /* Exit critical section (Rx mailbox message receive flag), restore interrupt setting */
            HwiP_restore(key);
    
            /* Get receive object */
            rxobj = &gAppPslRxMsgAxes[mcAxisIdx].receiveObj;            
            
            /* Translate Rx MC parameters, write translated parameters to control variables for node */
            xlateRxMcParams(rxobj, &ctrlVars[nodeIdx]);
        }
        else {
            /* Exit critical section (Rx mailbox message receive flag), restore interrupt setting */
            HwiP_restore(key);
        }        
    }

    return APP_PSL_MBXIPC_SOK;
}

/* Translate MC feedback variables for node, write to control Tx MC parameters */
static inline int32_t xlateTxMcParams(
    mc2ecat_msg_obj_t *txobj, 
    CTRL_Vars_t *pCtrl
)
{
    /* Update actual speed & position */
    txobj->i32VelocityActual = ECAT_floatToPU(pCtrl->speedWe);
    txobj->i32PositionActual = ECAT_floatToPU(pCtrl->posMechTheta);

    return APP_PSL_MBXIPC_SOK;
}

/* Mailbox IPC, transmit message for MC node (axis) */
int32_t appPslMbxIpcTxMsg(
    SysNode_e nodeIdx
)
{
    uint32_t payload;
    mc2ecat_msg_obj_t *txobj;
    uint16_t mcAxisIdx;

    /* Translate slave node index to MBX IPC mailbox index */
    mcAxisIdx = (uint16_t)(nodeIdx - 1);
    
    if ((mcAxisIdx < MAX_NUM_AXES) && 
        (gAppPslTxMsgAxes[mcAxisIdx].isMsgSend == 1))
    {
        /* Enter critical section, disable Timer interrupt. */
        // TO DO: replace w/ appropriate Time sync event when this is integrated
        Osal_DisableInterrupt(0, SIM_ECAT_TIMER_INTNUM);
        gAppPslTxMsgAxes[mcAxisIdx].isMsgSend = 0;
        /* Leave critical section, enable Timer interrupt */
        Osal_EnableInterrupt(0, SIM_ECAT_TIMER_INTNUM);

        /* Get transmit object */
        txobj = &gAppPslTxMsgAxes[mcAxisIdx].sendObj;

        /* Disable FSI Rx interrupts for critical section. */
        /* Velocity & Position updated in FSI ISR. */
        McuIntc_enableIntr(MCU_INTR_IDX(mcAxisIdx), false);  
 
        /* Translate MC feedback variables for node, write to control Tx MC parameters */
        xlateTxMcParams(txobj, &ctrlVars[nodeIdx]);

        /* Re-enable FSI Rx interrupts for critical section */
        McuIntc_enableIntr(MCU_INTR_IDX(mcAxisIdx), true);
        
        txobj->u16AxisIndex = mcAxisIdx;
        CacheP_wb(txobj, sizeof(mc2ecat_msg_obj_t));
        
        /* Translate the ATCM local view addr to SoC view addr */
        payload = (uint32_t)txobj;
        payload = CPU1_ATCM_SOCVIEW(payload);
        /* Tx address of payload */
        appMbxIpcSendNotify(IPC_ETHERCAT_CPU_ID, payload);
    }

    return APP_PSL_MBXIPC_SOK;
}

/* Mailbox IPC Rx message handler */
void appMbxIpcMsgHandler(uint32_t src_cpu_id, uint32_t payload)
{
    ecat2mc_msg_obj_t *payload_ptr;
    ecat2mc_msg_obj_t *rxobj;
    uint16_t axisIdx;
    
    gTotMbxIpcRxMsgCnt++;

    if (src_cpu_id == IPC_ETHERCAT_CPU_ID)
    {
        payload_ptr = (ecat2mc_msg_obj_t *)payload;
        CacheP_Inv(payload_ptr, sizeof(ecat2mc_msg_obj_t));
        axisIdx = payload_ptr->u16AxisIndex;

        if (axisIdx < MAX_NUM_AXES)
        {
            gMbxIpcRxMsgCnt[axisIdx]++;

            rxobj = &gAppPslRxMsgAxes[axisIdx].receiveObj;
            *rxobj = *payload_ptr;
            gAppPslRxMsgAxes[axisIdx].isMsgReceived = 1;
        }
        else
        {
            gMbxIpcRxMsgErrCnt++;
        }
    }
}
