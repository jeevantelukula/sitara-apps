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

#ifndef APP_PSL_CTRL_MBX_IPC_H_
#define APP_PSL_CTRL_MBX_IPC_H_

#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include "ipc_motorcontrol_if.h"
#include "app_pslctrl_cfg.h"

/* Status return codes */
#define APP_PSLCTRL_MBXIPC_SOK              (  0 )
#define APP_PSLCTRL_MBXIPC_SERR_MBXINIT     ( -1 )
#define APP_PSLCTRL_MBXIPC_SERR_REGISR      ( -2 )

/* IPC message object to receive motor control parameters */
typedef struct {
    volatile int32_t         isMsgReceived;  /* remove volatile qualifier once this moved to TCM & enable write through */
    mc2ecat_msg_obj_t        receiveObj;
} appPslCtrlReceiveMsgObj_t;

/* IPC message object to send motor control parameters */
typedef struct {
    volatile int32_t        isMsgSend;
    ecat2mc_msg_obj_t       sendObj;
} appPslCtrlSendMsgObj_t;

/* PSL Control MBX IPC configuration */
typedef struct {
    app_mbxipc_init_prm_t appMbxIpcInitPrm;
    app_mbxipc_notify_handler_f appMbxIpcMsgHandler;
} appPslCtrlMbxIpcCfg_t;

/* Initialize PSL Control MBX IPC */
int32_t appPslCtrlMbxIpcInit(
    appPslCtrlMbxIpcCfg_t *pPslCtrlMbxIpcCfg
);

/* Mailbox IPC Rx message handler */
void appMbxIpcMsgHandler(
    uint32_t src_cpu_id, 
    uint32_t payload
);

/* Global data MSG objects used in IPC communication */
extern appPslCtrlReceiveMsgObj_t    gAppPslCtrlRxMsgAxes[MAX_NUM_AXES]; /* receive CtrlVars parameters per axis */
extern appPslCtrlSendMsgObj_t       gAppPslCtrlTxMsgAxes[MAX_NUM_AXES]; /* send CtrlVars parameters per axis */

#endif /* APP_PSL_CTRL_MBX_IPC_H_ */
