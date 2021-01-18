/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

/* standard C includes */
#include <stdio.h>
#include <string.h>
/* architecture support layer */
#include <ti/csl/arch/csl_arch.h>
/* chip support layer */
#include <ti/csl/soc.h>
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_intr_router.h>
/* board support layer */
#include <ti/board/board.h>
/* driver support layer */
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/sciclient/sciclient.h>

#define GPIO_MMR                CSL_GPIO1_BASE
#define GPIOMUX_INT             CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_8
#define GPIO_BUTTON0            43U
#define GPIO_BUTTON0_INTRTR_IN  CSLR_MAIN_GPIOMUX_INTROUTER0_IN_GPIO1_GPIO_43
#define GPIO_BUTTON0_INTRTR_OUT 8U

volatile uint32_t gInterruptCounter;

static void App_print(const char *str)
{
    UART_printf("%s", str);
    printf("%s", str);

    return;
}

static void App_printNum(const char *str, uint32_t num)
{
    static char printBuf[200U];

    snprintf(printBuf, 200U, str, num);
    UART_printf("%s", printBuf);
    printf("%s", printBuf);

    return;
}

void gpio_isr()
{
    GPIOIntrClear_v0(GPIO_MMR, GPIO_BUTTON0);
    while(GPIOIntrStatus_v0(GPIO_MMR, GPIO_BUTTON0));

    gInterruptCounter++;
    Intc_IntClrPend(GPIOMUX_INT);
}

void configure_intc()
{
    Intc_Init();
    Intc_IntClrPend(GPIOMUX_INT);
    Intc_IntRegister(GPIOMUX_INT, (IntrFuncPtr) &gpio_isr, NULL);
    Intc_IntEnable(GPIOMUX_INT);
    Intc_SystemEnable();
}

void configure_gpiomux(uint16_t src_index, uint16_t dst_index)
{
    int32_t                             status;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                              TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id         = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0;
    rmIrqReq.dst_id         = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0;
    rmIrqReq.src_index      = src_index;
    rmIrqReq.dst_host_irq   = dst_index;

    status = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);

    if(status != CSL_PASS)
    {
        App_printNum("[Error] configure_gpiomux() failed: Sciclient_rmIrqSet() returned %d.\n", status);
        App_printNum("rmIrqReq.src_id = %d, ", rmIrqReq.src_id);
        App_printNum("rmIrqReq.src_index = %d, ", rmIrqReq.src_index);
        App_printNum("dst_id = %d, ", rmIrqReq.dst_id);
        App_printNum("dst_host_irq = %d.\n", rmIrqReq.dst_host_irq);
    }
}

int main(void)
{
    int32_t status;
    Board_initCfg boardCfg =
#ifndef SBL_BOOT
     /* Enabling Board Pinmux and clock when using without SBL boot
      * to act as stand alone application.
      */
     BOARD_INIT_PINMUX_CONFIG |
     BOARD_INIT_MODULE_CLOCK |
#endif
     /*  UART_STDIO initializes the default UART port on the board
      *  and supports stdio-like UART_printf which is used by appPrint
      */
    BOARD_INIT_UART_STDIO;

    /* Initialize Board */
    status = Board_init(boardCfg);
    if (status != BOARD_SOK) {
        App_printNum("Error: Board_init() failed: error %d.\n", status);
    }
    App_print("Board_init() complete.\n");

    /* interrupt controller setup should be after Board_init() */
    configure_intc();

    GPIOSetDirMode_v0(GPIO_MMR, GPIO_BUTTON0, GPIO_DIRECTION_INPUT);
    GPIOIntrEnable_v0(GPIO_MMR, GPIO_BUTTON0, GPIO_INTR_MASK_RISE_EDGE);
    configure_gpiomux(GPIO_BUTTON0_INTRTR_IN, GPIO_BUTTON0_INTRTR_OUT);

    gInterruptCounter = 0;
    while(gInterruptCounter != 10);

    App_print("Template app ended.\n");

    Board_deinit(BOARD_DEINIT_UART_STDIO);
    return status;
}
