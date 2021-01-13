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

#ifndef APP_CFG_SOC_H_
#define APP_CFG_SOC_H_

#include <ti/board/board.h>
#include "ti/csl/soc.h"
#include "ti/csl/csl_timer.h"
#include <ti/csl/tistdtypes.h>
#include <ti/csl/arch/csl_arch.h>

/* benchmark timer */
#define NUM_MCU_CORE                 ( 4 )                                          /* number of R5F cores */
#define UINIT_TIMER_REGS_BASE_ADDR   ( 0x0 )                                        /* undefined timer base address register */
#define BENCHMARK_TIMER_FREQ_HZ      ( 25000000 )                                   /* Timer frequency, WKUP_HFOSC0_CLKOUT=25 MHz */
#define BENCHMARK_TIMER_TICK_PRD     ( 40 )                                         /* Timer tick period in ns. 1/BENCHMARK_TIMER_FREQ_HZ sec */
#define BENCHMARK_TIMER_COUNTER_VAL  ( 0xffff9e58 )                                 /* Timer counter value for 1000usec */
#define BENCHMARK_TIMER_PERIOD_USEC  ( 1000 )                                       /* Timer period (1000usec.) */
#define CPU_FREQUENCY                ( 800000000 )                                  /* CPU frequency */

#define BENCHMARK_TIMER_INT_TYPE               ( CSL_VIM_INTR_TYPE_LEVEL )          /* interrupt type: pulse,level */
#define BENCHMARK_TIMER_INT_MAP                ( CSL_VIM_INTR_MAP_IRQ )             /* interrupt route: 0-IRQ, 1-FIQ */
#define BENCHMARK_TIMER_INT_PRI                ( 0 )                                /* interrupt priority: 0-highest, 15-lowest */

#define BENCHMARK_TIMER_DM_TWPS_W_PEND_TCLR    (0x1u)
#define BENCHMARK_TIMER_DM_TWPS_W_PEND_TCRR    (0x2u)
#define BENCHMARK_TIMER_DM_TWPS_W_PEND_TLDR    (0x4u)

/* Unitialized values */
#define UINIT_VIM_REGS_BASE_ADDR               ( 0x0 ) /* VIM base address register */
#define NUM_MCU_INTR                           ( 5 )   /* number of MCU interrupts which can be configured */
#define MCU_INTR_IDX(x)                        ( x )   /* MCU interrupt index */

/* MCU interrupt registration parameters */
typedef struct McuIntrRegPrms_s {
    int32_t intrNum;                    /* VIM interrupt number */
    CSL_VimIntrType intrType;           /* VIM interrupt type {pulse, level} */
    CSL_VimIntrMap intrMap;             /* VIM interrupt map {IRQ, FIQ} */
    uint32_t intrPri;                   /* VIM interrupt priority (0(highest)..15(lowest)) */
    void (*isrRoutine)(void);           /* ISR routine */
} McuIntrRegPrms;

/* MCU interrupt configuration structure */
typedef struct McuIntrCfg_s {
    McuIntrRegPrms mcuIntrRegPrms;  /* interrupt configuration */
    bool intrCfgValid;              /* flag indicates whether interrupt configuration valid */
} McuIntrCfg;

typedef struct McuTimerCfg_s {
    uint32_t timerId;        /* timer ID */
    uint32_t timerBaseAddr;  /* timer base address */
    uint32_t intrNum;       /* interrupt number */
} McuTimerCfg;

#endif /* APP_CFG_SOC_H_ */
