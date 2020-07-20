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

#ifndef APP_PSL_CTRL_CFG_H_
#define APP_PSL_CTRL_CFG_H_

#include <ti/csl/tistdtypes.h>
#include <ti/board/src/am64x_evm/AM64xx_pinmux.h>

#define ENABLE_BOARD

#define TASK_PSL_CTRL_PRI       ( 1 )           /* Task PSL Control priority */
#define TASK_PSL_CTRL_SZ        ( 2048 )        /* Task PSL Control stack size */

/* Time Sync PRU firmware, ICSSG instance ID */
/* TODO: move to PRUICCSS_INSTANCE_TWO once EtherCAT has been moved to ICSSG1 */
#define TS_ICSSG_INST_ID                    ( PRUICCSS_INSTANCE_ONE )
/* Time Sync PRU firmware, PRU instance ID */
#define TS_PRU_INST_ID                      ( PRUICCSS_RTU0 )

/* Time Sync IEP0 Period (nsec.) */
#define TS_IEP_PRD_NSEC                     ( 5 )           /* 5 nsec = 1/200MHz */
/* Time Sync CMP Periods & Offsets */
#define TS_PRD_COUNT1                       ( TS_IEP_PRD_NSEC*200000000u/8000u )    /* 125us (8Khz) (CMP7) */
/* Time Sync CMP Offsets */
#define TS_PRD_OFFSET1                      ( -TS_IEP_PRD_NSEC*2000 )               /* 10 usec. pre-trigger */

/* ICSSG_0_IEP0_CMP_TIMER7_INT (23)/ICSSG_1_IEP0_CMP_TIMER7_INT (55) */
#define TS_CMPEVT_INTRTR_IN0                ( CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_7 )
#define TS_CMPEVT_INTRTR_OUT0               ( 24 )  /* CMP_EVT_RTR_OUT_24 -> R5F2_0/R5F2_1_IN_48  */

/* Simulated SYNC0 pulse, Time Sync CMP1 Period */
#define SIM_SYNC0_TS_PRD_COUNT0             ( TS_IEP_PRD_NSEC*200000000u/1000u )    /* 1ms (1KHz) (CMP1) sim Sync0 */

/* Simulated SYNC0 pulse, Compare Event Router input & output */
/* ICSSG_0_IEP0_CMP_TIMER1_INT (17)/ICSSG_1_IEP0_CMP_TIMER1_INT (49) */
#define SIM_SYNC0_CMPEVT_INTRTR_IN          ( CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_1 )
#define SIM_SYNC0_CMPEVT_INTRTR_OUT         ( 16 )  /* CMP_EVT_RTR_OUT_16 -> R5F1_0/R5F1_1_IN_48 */
/* Simulated SYNC0 pulse, MCU interrupt number */
/* CMP_EVT_RTR_OUT_16 -> R5FSS0_0_IN_48 */
#define SIM_SYNC0_INTR_NUM                  ( CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16 )

/* Compare Event Router configuration base address */
#define CMPEVTRTR_INTRTR0_CFG_BASE_ADDRESS  ( CSL_CMP_EVENT_INTROUTER0_CFG_BASE )

extern pinmuxPerCfg_t gFsiPinCfg;

#endif /* APP_PSL_CTRL_CFG_H_ */
