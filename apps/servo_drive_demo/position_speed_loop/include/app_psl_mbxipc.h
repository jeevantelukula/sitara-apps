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

#ifndef APP_PSL_MBX_IPC_H_
#define APP_PSL_MBX_IPC_H_

#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include "ipc_motorcontrol_if.h"
#include "hw_types.h"
#include "multi_axis_master_ctrl.h"
#include "app_cfg_soc.h"

/* Status return codes */
#define APP_PSL_MBXIPC_SOK              (  0 )
#define APP_PSL_MBXIPC_SERR_MBXINIT     ( -1 )
#define APP_PSL_MBXIPC_SERR_REGISR      ( -2 )

/* Axis indices */
#define ECAT_MC_AXIS_IDX0               ( SYS_NODE1 - 1 )
#define ECAT_MC_AXIS_IDX1               ( SYS_NODE2 - 1 )
#define ECAT_MC_AXIS_IDX2               ( SYS_NODE3 - 1 )

/* IPC message object to send motor control parameters */
typedef struct {
    volatile int32_t        isMsgSend;
    mc2ecat_msg_obj_t       sendObj;
} appPslSendMsgObj_t;

/* IPC message object to receive motor control parameters */
typedef struct {
    volatile int32_t        isMsgReceived;
    ecat2mc_msg_obj_t       receiveObj;
} appPslReceiveMsgObj_t;

/* PSL MBX IPC configuration */
typedef struct {
    app_mbxipc_init_prm_t appMbxIpcInitPrm;
    app_mbxipc_notify_handler_f appMbxIpcMsgHandler;
} appPslMbxIpcCfg_t;

/* Initialize PSL MBX IPC */
int32_t appPslMbxIpcInit(
    appPslMbxIpcCfg_t *pPslMbxIpcCfg
);

/* Mailbox IPC, receive message for MC axis (node) */
int32_t appPslMbxIpcRxMsg(
    uint16_t mcAxisIdx, 
    SysNode_e sysNodeIdx
);

/* Mailbox IPC, transmit message for MC axis (node) */
int32_t appPslMbxIpcTxMsg(
    uint16_t mcAxisIdx, 
    SysNode_e sysNodeIdx
);

/* Mailbox IPC Rx message handler */
void appMbxIpcMsgHandler(
    uint32_t src_cpu_id, 
    uint32_t payload
);

/* Global data MSG objects used in IPC communication */
extern appPslReceiveMsgObj_t   gAppPslRxMsgAxes[MAX_NUM_AXES];  /* receive CtrlVars parameters per axis */
extern appPslSendMsgObj_t      gAppPslTxMsgAxes[MAX_NUM_AXES];  /* send CtrlVars parameters per axis */

/* May need to adjust these based on CiA402 or TwinCAT requirements */
#define ECAT_FLOAT2PU_SF                ( 10000L )
#define ECAT_PU2FLOAT_SF                ( 0.0001F )

/* Translate 32-bit integer into SP 32-bit float for Rx from EthCAT */
static inline float32_t ECAT_puToFloat(int32_t valuePu)
{
    int32_t valueTemp;

    valueTemp = ((valuePu > ECAT_FLOAT2PU_SF) ?
            ECAT_FLOAT2PU_SF : (valuePu < -ECAT_FLOAT2PU_SF) ?
                    -ECAT_FLOAT2PU_SF : valuePu);

    float32_t resultFloat = valueTemp * ECAT_PU2FLOAT_SF;

    return(resultFloat);
}

/* Translate SP 32-bit float into 32-bit integer for Tx to EthCAT */
static inline int32_t ECAT_floatToPU(float32_t valueFloat)
{
    int32_t resultPU = ECAT_FLOAT2PU_SF * valueFloat;

    return(resultPU);
}

#endif /* APP_PSL_MBX_IPC_H_ */
