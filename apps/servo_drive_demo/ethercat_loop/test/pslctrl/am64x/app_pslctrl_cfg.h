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

/* Timer parameters -- simulated SYNC pulse */
#define TIMER_ID                ( 0 )           /* Timer ID */
#define TIMER_FREQ_HZ           ( 25000000 )    /* Timer frequency, WKUP_HFOSC0_CLKOUT=25 MHz */
#define TIMER_PERIOD_USEC       ( 1000 )        /* Timer period (usec.) */
#define TIMER_INTNUM            ( 152 )         /* Timer interrupt, R5F1_0 DMTIMER0 INT */

extern pinmuxPerCfg_t gFsiPinCfg;

#endif /* APP_PSL_CTRL_CFG_H_ */
