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
#include <ti/drv/pruss/pruicss.h>
#include <ti/board/src/am65xx_evm/am65xx_evm_pinmux.h>

#define ENABLE_BOARD

#define TASK_PSL_CTRL_PRI       ( 1 )           /* Task PSL Control priority */
#define TASK_PSL_CTRL_SZ        ( 2048 )        /* Task PSL Control stack size */

/* Time Sync PRU firmware, ICSSG instance ID */
#define TS_ICSSG_INST_ID        ( PRUICCSS_INSTANCE_ONE )
/* Time Sync PRU firmware, PRU instance ID */
#define TS_PRU_INST_ID          ( PRUICCSS_RTU0 )

/* Time Sync IEP0 Period (nsec.) */
#define TS_IEP_PRD_NSEC         ( 5 )           /* 5 nsec = 1/200MHz */
/* Time Sync CMP Periods & Offsets */
#define TS_PRD_COUNT1           ( TS_IEP_PRD_NSEC*200000000u/8000u )    /* 125us   (8Khz)   (CMP3)              */
/* Time Sync CMP Offsets */
#define TS_PRD_OFFSET1          ( -TS_IEP_PRD_NSEC*2000 )               /* 10 usec. pre-trigger */

/* Time Sync Compare Event Router input & output */
#define TS_CMPEVT_INTRTR_IN0    ( 35 )  /* ICSSG_0_IEP0_CMP_TIMER3_INT */
#define TS_CMPEVT_INTRTR_OUT0   ( 17 )  /* COMPEVT_RTR_COMP_17_EVT */

/* Simulated SYNC0 pulse, Time Sync CMP1 Period */
#define SIM_SYNC0_TS_PRD_COUNT0 ( TS_IEP_PRD_NSEC*200000000u/1000u )    /* 1ms     (1KHz)   (CMP1)    sim Sync0 */

/* Simulated SYNC0 pulse, Compare Event Router input & output */
#define SIM_SYNC0_CMPEVT_INTRTR_IN  \
    ( 33 )  /* ICSSG_0_IEP0_CMP_TIMER1_INT */
#define SIM_SYNC0_CMPEVT_INTRTR_OUT \
    ( 16 )  /* COMPEVT_RTR_COMP_16_EVT */
/* Simulated SYNC0 pulse, MCU interrupt number */
#define SIM_SYNC0_MAIN2MCU_RTR_PLS_MUX_INTR \
    ( 32 )  /* MAIN2MCU_RTR_PLS_MUX_INTR16 */

extern pinmuxPerCfg_t gFsiPinCfg;

#endif /* APP_PSL_CTRL_CFG_H_ */
