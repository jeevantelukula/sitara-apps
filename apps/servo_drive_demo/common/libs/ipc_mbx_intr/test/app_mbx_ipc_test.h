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


#ifndef APP_MBX_IPC_TEST_H_
#define APP_MBX_IPC_TEST_H_

#include <stdint.h>
#include <app_mbx_ipc.h>
#include <app_mbx_ipc_test_soc.h>


/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* Sample MC parameter values */
#define VELOCITY  (50)
#define POSITION  (200)
#define STATE     (2)

/* IPC test message object to mimic Motor control scenarios */
typedef struct {
    int32_t i32Velocity;
    int32_t i32Position;
    int16_t i16State;
} test_msg_obj_t;

typedef struct {
    /* Remove volatile qualifier once this moved to TCM & enable write through */
    volatile int32_t isMsgReceived;
    uint32_t srcCpuId;
    test_msg_obj_t sendObj;
    test_msg_obj_t receiveObj;
} app_mbxipc_test_obj_t;


/* ========================================================================== */
/*                 Function Declarations                             */
/* ========================================================================== */

void mbxIpcMsgTestHandler (uint32_t src_cpu_id, uint32_t payload);

int32_t ipcTestRun(int32_t iterationCnt);

#endif

