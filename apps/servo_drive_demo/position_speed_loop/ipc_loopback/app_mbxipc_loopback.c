/*
 * Copyright (C) 2018-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdint.h>
#include <stdio.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_mailbox.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/osal/CacheP.h>
#include <ti/osal/HwiP.h>

#include <app_log.h>
#include <app_mbx_ipc.h>
#include <app_sciclient.h>
#include <app_init.h>
#include <app_mbxipc_loopback.h>

/* Global data MSG object used in IPC communication */
static app_ipc_mc_obj_t gAppIpcMsgObj
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;


/* ISR callback that is invoke when current CPU receives a MSG */
void appMbxIpcMsgHandler (uint32_t src_cpu_id, uint32_t payload)
{
    uint16_t axisIndex;
    receive_msg_obj_t *rxobj;
    receive_msg_obj_t *payload_ptr = (receive_msg_obj_t*)payload;

    CacheP_Inv(payload_ptr, sizeof(receive_msg_obj_t));
    axisIndex = payload_ptr->axisIndex;
    if (axisIndex < MAX_NUM_AXIS)
    {
        rxobj = &gAppIpcMsgObj.axisObj[axisIndex].receiveObj;
        if (src_cpu_id==IPC_ETHERCAT_CPU_ID)
        {
            *rxobj= *payload_ptr;
            gAppIpcMsgObj.axisObj[axisIndex].isMsgReceived = 1;
        }
    }
}

/* Added for debug purpose when load and run via SBL.
 * set enableDebug = 1 and build for debug.
 * Once started running connect CCS and reset enableDebug=0
 * to proceed with single-step from the beginning
 */
void StartupEmulatorWaitFxn (void)
{
    volatile uint32_t enableDebug = 0;
    do
    {
    }while (enableDebug);
}

int main(void)
{
    int32_t i;
    uintptr_t key;
    int32_t bRunState = 1;
    uint32_t payload;
    app_mbxipc_init_prm_t mbxipc_init_prm;
    send_msg_obj_t *txobj;
    receive_msg_obj_t *rxobj;
    
    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    appLogPrintf("MCU-SS core0 is up !!!! \n");

    appSciclientInit();

    /* initialize CSL Mbx IPC */
    appMbxIpcInitPrmSetDefault(&mbxipc_init_prm);
    mbxipc_init_prm.master_cpu_id = IPC_ETHERCAT_CPU_ID;
    mbxipc_init_prm.self_cpu_id = IPC_PSL_MC_CPU_ID;
    mbxipc_init_prm.num_cpus = 0;
    mbxipc_init_prm.enabled_cpu_id_list[mbxipc_init_prm.num_cpus] = IPC_ETHERCAT_CPU_ID;
    mbxipc_init_prm.num_cpus++;
    mbxipc_init_prm.enabled_cpu_id_list[mbxipc_init_prm.num_cpus] = IPC_PSL_MC_CPU_ID;
    mbxipc_init_prm.num_cpus++;
    /* IPC CPU sync check works only when appMbxIpcInit() called from both R5Fs */
    appMbxIpcInit(&mbxipc_init_prm);
    /* Register Application callback to invoke on receiving a notify message */
    appMbxIpcRegisterNotifyHandler((app_mbxipc_notify_handler_f) appMbxIpcMsgHandler);
    for (i=0; i< MAX_NUM_AXIS; i++){
        gAppIpcMsgObj.axisObj[i].isMsgReceived = 0;
    }

    do
    {
        int32_t isMsgFound =0;
        uint16_t axisIndex = 0xFF;

        /* Wait for IPC MSG from ECAT R5F with target MC parameters */
        do
        {
            for (i=0; i< MAX_NUM_AXIS; i++)
            {
                key = HwiP_disable();
                if (1==gAppIpcMsgObj.axisObj[i].isMsgReceived)
                {
                    gAppIpcMsgObj.axisObj[i].isMsgReceived = 0;
                    axisIndex = i;
                    isMsgFound = 1;
                }
                HwiP_restore(key);
                if (isMsgFound)
                {
                    break;
                }
            }
        }while (!isMsgFound);

        txobj = &gAppIpcMsgObj.axisObj[axisIndex].sendObj;
        rxobj = &gAppIpcMsgObj.axisObj[axisIndex].receiveObj;
        payload = (uint32_t)txobj;

        /* In the Demo loopback application, sending Actual values calculated */
        /* by CiA402_DummyMotionControl() to Motor Control R5F and receiving  */
        /* back on EtherCAT R5F.  This will be modified to make use of the    */
        /* Position-speed MC algo on MC R5F to calculate the Actual Values.   */
        if (axisIndex==rxobj->axisIndex)
        {
            txobj->i32PositionActualValue = rxobj->i32TargetPosition;
            txobj->i32VelocityActualValue = rxobj->i32TargetVelocity;
            txobj->axisIndex = rxobj->axisIndex;
            bRunState = rxobj->i16State;
            CacheP_wb(txobj, sizeof(send_msg_obj_t));

            /* Send back MC parameters to ECAT R5F */
            if (appMbxIpcGetSelfCpuId()==IPC_PSL_MC_CPU_ID)
            {
                appMbxIpcSendNotify(IPC_ETHERCAT_CPU_ID, CPU1_ATCM_SOCVIEW(payload));
            }
        }
        else
        {
            APP_ASSERT_SUCCESS(1);
        }
    }while (bRunState);

    appMbxIpcDeInit();

    appSciclientDeInit();

    return 0;
}


