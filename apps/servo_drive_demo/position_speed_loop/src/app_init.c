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

#include <stdio.h>
#include <string.h>
#include <ti/board/board.h>
#include <ti/board/src/am65xx_evm/am65xx_evm_pinmux.h>
#include <ti/csl/soc.h>
#include <misc/include/app_misc.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#include <misc/include/app_misc.h>
#include <sciclient/include/app_sciclient.h>
#include <logs/include/app_log.h>

#include "app_init.h"
#include "app_cfg.h"

static pinmuxPerCfg_t gFsiPinCfg[] =
{
    /* PRG2_PRU0_GPO12 -> N23 */
    {
        PIN_GPMC0_AD8, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* PRG2_PRU0_GPO13 -> N24 */
    {
        PIN_GPMC0_AD9, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* PRG2_PRU1_GPI12 -> N26 */
    {
        PIN_GPMC0_AD12, PIN_MODE(4) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* PRG2_PRU1_GPI13 -> N25 */
    {
        PIN_GPMC0_AD13, PIN_MODE(4) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* GPIO0_14 -> P24 */
    {   
        PIN_GPMC0_AD14, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* GPIO0_15 -> R27 */
    {
        PIN_GPMC0_AD15, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

int32_t appInit()
{
    int32_t status = 0;

    #ifdef ENABLE_BOARD
    {
	Board_initCfg boardCfg;

	/* Pad configurations */
	boardCfg = BOARD_INIT_UNLOCK_MMR | BOARD_INIT_UART_STDIO |
              BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG;
	Board_init(boardCfg);

	/* PINMUX configurations */
	appSetPinmux(gFsiPinCfg);
    }
    #endif

    appLogPrintf("APP: Init ... !!!\n");

    status = appSciclientInit();
    APP_ASSERT_SUCCESS(status);

    appLogPrintf("APP: Init ... Done !!!\n");

    return status;
}

void appDeInit()
{
    appLogPrintf("APP: Deinit ... !!!\n");

    appSciclientDeInit();

    appLogPrintf("APP: Deinit ... Done !!!\n");
}

