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

#ifndef APP_MBXIPC_LOOPBACK_H_
#define APP_MBXIPC_LOOPBACK_H_

#include <stdint.h>
#include <stdio.h>
#include <app_mbx_ipc.h>

/* IPC CPU ID should match with EtherCAT CPU configuration */
#define IPC_ETHERCAT_CPU_ID    (MAILBOX_IPC_CPUID_MCU1_0)
#define IPC_PSL_MC_CPU_ID      (MAILBOX_IPC_CPUID_MCU1_1)

/* MAX number of independent axis supported */
#define MAX_NUM_AXIS           (3)

/* IPC message objects to send/receive motor control parameters   */
/* Below send and receive data structures should be in align with */ 
/* the receive and send data structures of ethercat_loop          */
typedef struct {
    int32_t i32TargetVelocity;
    int32_t i32TargetPosition;
    int16_t i16ModesOfOperation;
    int16_t i16State;
    uint16_t axisIndex;
} receive_msg_obj_t;

typedef struct {
    int32_t i32VelocityActualValue;
    int32_t i32PositionActualValue;
    uint16_t axisIndex;
} send_msg_obj_t;

typedef struct {
    /* Remove volatile qualifier once this moved to TCM & enable write through */
    volatile int32_t isMsgReceived;
    send_msg_obj_t sendObj;
    receive_msg_obj_t receiveObj;
} app_ipc_axis_obj_t;

typedef struct {
    app_ipc_axis_obj_t axisObj[MAX_NUM_AXIS];
} app_ipc_mc_obj_t;


#endif /* APP_MBXIPC_LOOPBACK_H_ */
