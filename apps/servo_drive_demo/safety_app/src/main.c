/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <stdbool.h>

/* TI CSL Includes */
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
/* TI Driver Includes */
#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

/* Application Functions and Macros */
#include "app.h"
#include "esmcfg.h"
#include "ratcfg.h"

int main(void)
{
    int status;
    status = Board_init(BOARD_INIT_UART_STDIO);
    if(status != CSL_PASS)
    {
        goto application_exit;
    }

    UART_printf("\n\n");
    UART_printf("[Info] M4F Application Started!\n");

    /* This will set Control MMR bits for debug and reset isolation. */
    configure_isolation();

    /* Error Signaling Module aggregates device errors (Clock, ECC) allowing
        software or external hardware (via error pin) to make a response */
    status = configure_esm();
    if(status != CSL_PASS) {
        UART_printf("[Error] ESM config unsuccessful.\n");
        goto application_exit;
    }

    /* This enables a timer with ISR to flash an LED (still alive signal) */
    status = configure_ledPattern();
    if (status != CSL_PASS) {
        UART_printf("[Error] LED timer config unsuccessful.\n");
        goto application_exit;
    }

    configure_interrupts();
    application_loop();

application_exit:
    UART_printf("[Info] Exiting application.\n");
    Board_deinit(BOARD_DEINIT_UART_STDIO);
    return status;
}

