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
 
#ifndef _IPC_MOTORCONTROL_IF_H_
#define _IPC_MOTORCONTROL_IF_H_

#include <ti/csl/soc.h>
#include <app_mbx_ipc.h>

#define APP_ASSERT_SUCCESS(x)  { if((x)!=0) while(1); }

/* IPC CPU ID should match with PSL MC CPU configuration */
#define IPC_ETHERCAT_CPU_ID    (MAILBOX_IPC_CPUID_MCU1_0)
#define IPC_PSL_MC_CPU_ID      (MAILBOX_IPC_CPUID_MCU1_1)

/* Translate the ATCM local view addr to SoC view addr */
#define CPU0_ATCM_SOCVIEW(x)   (CSL_MCU_ARMSS0_CORE0_ATCM_BASE+x)
#define CPU1_ATCM_SOCVIEW(x)   (CSL_MCU_ARMSS0_CORE1_ATCM_BASE+x)

/* MAX number of independent axis supported */
#define MAX_NUM_AXES           (3)

/* IPC message objects to send/receive motor control parameters   */
/* Below send and receive data structures should be in align with */
/* the receive and send data structures of its counterpart        */
/* IPC message object to send motor control parameters */
/* from EtherCAT slave to Motor control loop           */
typedef struct {
    int32_t i32TargetVelocity;
    int32_t i32TargetPosition;
    int16_t i16ModesOfOperation;
    int16_t i16State;
    uint16_t u16AxisIndex;
} ecat2mc_msg_obj_t;

/* IPC message object to send motor control parameters */
/* from Motor control loop to EtherCAT slave           */
typedef struct {
    int32_t i32VelocityActual;
    int32_t i32PositionActual;
    uint16_t u16AxisIndex;
} mc2ecat_msg_obj_t;


#endif /* _IPC_MOTORCONTROL_IF_H_ */
