/*
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 *
 */

/**
 *
 *  \brief  Template application main file:
 *          The main code initializes the platform and then
 *          starts running the application tasks. The initialization
 *          includes board specific configuration and initialization
 *          of the peripherals used by the application. The application
 *          specific code is defined in a separate file: app.c.
 *
 */

#include <stdio.h>
#include <string.h>
#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/udma/udma.h>
#include <ti/drv/udma/examples/udma_apputils/udma_apputils.h>
#include "board_utils.h"

#define App_getGTCTimerTicks()  CSL_REG64_RD(CSL_GTC0_GTC_CFG1_BASE + 0x8U)
volatile uint64_t gTickDelay = 0;
volatile uint64_t gClockGTC = 0;

static void App_print(const char *str)
{
    UART_printf("%s", str);
    if(TRUE == Udma_appIsPrintSupported())
    {
        printf("%s", str);
    }

    return;
}

static void App_printNum(const char *str, uint32_t num)
{
    static char printBuf[200U];

    snprintf(printBuf, 200U, str, num);
    UART_printf("%s", printBuf);

    if(TRUE == Udma_appIsPrintSupported())
    {
        printf("%s", printBuf);
    }

    return;
}

int32_t App_getGTCClk(uint32_t moduleId,
                      uint32_t clkId)
{
    int32_t retVal;
    uint64_t clkVal;

    retVal = Sciclient_pmGetModuleClkFreq(moduleId,
                                          clkId,
                                          &clkVal,
                                          SCICLIENT_SERVICE_WAIT_FOREVER);

    gClockGTC = clkVal;
    App_printNum("GTC Clk running at %d Hz.\n", (uint32_t)gClockGTC);

    /* Enable GTC */
    CSL_REG64_WR(CSL_GTC0_GTC_CFG1_BASE + 0x0U, 0x1);

    /* Measure and store the time spent to do a getTime operation */
    gTickDelay = App_getGTCTimerTicks();
    gTickDelay = App_getGTCTimerTicks() - gTickDelay;
    App_printNum("Time taken to read GTC Timer ticks = %d ns ",
                 (uint32_t)((gTickDelay*1000000000U)/gClockGTC));
    App_printNum("(%d ticks).\n", (uint32_t)gTickDelay);

    return (retVal);
}

int main(void)
{
    Board_initCfg boardCfg;
    int status;

#if defined(am64x_evm)
    Board_initParams_t boardInitParams;
    Board_getInitParams(&boardInitParams);
    boardInitParams.uartInst = BOARD_UART3_INSTANCE;
    Board_setInitParams(&boardInitParams);
#endif

    boardCfg =
#ifndef SBL_BOOT
     /* Enabling Board Pinmux, clock when using without SBL boot
      * to act as stand alone application.
      */
     BOARD_INIT_MODULE_CLOCK |
     BOARD_INIT_PINMUX_CONFIG |
#endif
     /*  UART_STDIO initializes the default UART port on the board
      *  and supports stdio-like UART_printf which is used by appPrint
      */
    BOARD_INIT_UART_STDIO;

    /* Initialize Board */
    status = Board_init(boardCfg);
    if (status != BOARD_SOK) {
        App_printNum("Error: Board_init failed: error %d.\n", status);
    }
    App_print("Board Init complete.\n");

    App_getGTCClk(TISCI_DEV_GTC0, TISCI_DEV_GTC0_GTC_CLK);

    App_print("Template app ended.\n");
    return (0);
}
