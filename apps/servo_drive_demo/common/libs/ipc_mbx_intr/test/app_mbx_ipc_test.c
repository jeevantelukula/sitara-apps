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
#include <string.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_mailbox.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/osal/CacheP.h>
#include <ti/osal/HwiP.h>

#include <app_log.h>
#include <app_mbx_ipc.h>
#include <app_sciclient.h>
#include <app_mbx_ipc_test.h>


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* MSG object used in IPC communication */
static app_mbxipc_test_obj_t gTestMsgObj
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Please note that the test application works only between a pair of CPus
 * Not more than 2 CPUs are supported, as each CPU can receive message
 * from one remote CPU only.
 */

void mbxIpcMsgTestHandler (uint32_t src_cpu_id, uint32_t payload)
{
    test_msg_obj_t *rxobj = &gTestMsgObj.receiveObj;
    test_msg_obj_t *payload_ptr = (test_msg_obj_t*)payload;

    *rxobj = *payload_ptr;
    gTestMsgObj.srcCpuId = src_cpu_id;
    gTestMsgObj.isMsgReceived = 1;
}

int checkStatus (void)
{
    uintptr_t key;
    int32_t status = -1;
    test_msg_obj_t *rxobj = &gTestMsgObj.receiveObj;
    uint16_t remoteId;
    uint32_t iterationCnt;

    while (1!=gTestMsgObj.isMsgReceived);

    key = HwiP_disable();
    gTestMsgObj.isMsgReceived = 0;
    HwiP_restore(key);
    remoteId = gTestMsgObj.srcCpuId;
    iterationCnt = rxobj->u32IterationCnt;

    if (gTestMsgObj.srcCpuId < MAILBOX_IPC_MAX_PROCS)
    {
        if ((rxobj->i32Velocity == VELOCITY+remoteId+iterationCnt) &&
            (rxobj->i32Position == POSITION+remoteId+iterationCnt) &&
            (rxobj->i16State == STATE+remoteId+iterationCnt))
        {
            status = 0;
        }
    }
    gTestMsgObj.srcCpuId = 0xFF;
    memset(rxobj, 0, sizeof(test_msg_obj_t));

    return status;
}

int32_t ipcTestRun(int32_t iterationCnt)
{
    int32_t status = 0;
    uint16_t remoteId;
    uint32_t selfId = appMbxIpcGetSelfCpuId();
    test_msg_obj_t *txobj = &gTestMsgObj.sendObj;
    uint32_t payload = (uint32_t)txobj;

    /* Communicate with each of the other cores */
    for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
    {
        if (remoteId == selfId)
        {
            continue;
        }
        if (!appMbxIpcIsCpuEnabled(remoteId))
        {
            continue;
        }

        txobj->i32Velocity = VELOCITY+selfId+iterationCnt;
        txobj->i32Position = POSITION+selfId+iterationCnt;
        txobj->i16State = STATE+selfId+iterationCnt;
        txobj->u32IterationCnt = iterationCnt;
		
        /* Translate the ATCM local view addr to SoC view addr */
        if (MBXIPC_TEST_CPU_1 == appMbxIpcGetSelfCpuId())
            payload = CPU0_BTCM_SOCVIEW(payload);
        if (MBXIPC_TEST_CPU_2 == appMbxIpcGetSelfCpuId())
            payload = CPU1_BTCM_SOCVIEW(payload);

        appMbxIpcSendNotify(remoteId, payload);
    }

    if (MAX_ITERATION_COUNT > iterationCnt)
    {
        status = checkStatus();
    }

    return status;
}

/********************************* End of file ******************************/

