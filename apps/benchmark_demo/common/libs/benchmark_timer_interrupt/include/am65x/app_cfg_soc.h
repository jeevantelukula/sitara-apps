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

/* benchmark timer */
#define BENCHMARK_TIMER_ID           ( 2 )           /* Timer ID */
#define BENCHMARK_TIMER_FREQ_HZ      ( 25000000 )    /* Timer frequency, WKUP_HFOSC0_CLKOUT=25 MHz */
#define BENCHMARK_TIMER_TICK_PRD     ( 40 )          /* Timer tick period in ns. 1/BENCHMARK_TIMER_FREQ_HZ sec */
#define BENCHMARK_TIMER_PERIOD_USEC  ( 1000 )        /* Timer period (1000usec.) */
#define BENCHMARK_TIMER_INTNUM       ( CSL_MCU0_INTR_TIMER2_INTR_PEND )          /* Timer interrupt, MCU_TIMER_2_INT */
#define CPU_FREQUENCY	              ( 800000000 )   /* CPU frequency */

#endif /* APP_CFG_SOC_H_ */
