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
#include <ti/csl/soc/cslr_soc_ctrl_mmr.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/csl_mailbox.h>
#include <ti/csl/csl_timer.h>
#include <ti/csl/csl_gpio.h>
/* TI Driver Includes */
#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/osal/osal.h>

#include "app.h"
#include "esmcfg.h"
#include "ratcfg.h"
#include "appmbox.h"
#include "blackchannel.h"

/* ti/csl/soc/am64x/src/cslr_soc_baseaddress.h */
#define MCU_GPIO_MMR            (CSL_MCU_GPIO0_BASE + M4F_RAT_MODULES_OFFSET)
#define MCU_CTRL_MMR_BASE       (CSL_MCU_CTRL_MMR0_CFG0_BASE + M4F_RAT_MODULES_OFFSET)
#define MCU_TIMER2_MMR          (CSL_MCU_TIMER2_CFG_BASE + M4F_RAT_MODULES_OFFSET)
#define MCU_TIMER3_MMR          (CSL_MCU_TIMER3_CFG_BASE + M4F_RAT_MODULES_OFFSET)
#define MAIN_MBOX6_MMR          (CSL_MAILBOX0_REGS6_BASE + M4F_RAT_MAILBOX_OFFSET)

/* required for unlocking MCU CTRL MMR write protected registers */
#define UNLOCK0_VAL             (0x68EF3490U)
#define UNLOCK1_VAL             (0xD172BC5AU)

/* ti/csl/soc/am64x/src/cslr_mcu_ctrl_mmr.h */
#define MCU_RST_ISO_DONE_MASK   CSL_MCU_CTRL_MMR_CFG0_RST_CTRL_MCU_RESET_ISO_DONE_Z_MASK
#define MCU_DBG_ISO_EN_MASK     CSL_MCU_CTRL_MMR_CFG0_ISO_CTRL_MCU_DBG_ISO_EN_MASK
#define MCU_RST_ISO_EN_MASK     CSL_MCU_CTRL_MMR_CFG0_ISO_CTRL_MCU_RST_ISO_EN_MASK
#define MAIN_WARMRST_MASK       (0xFFFFFFF6U)

/* ti/csl/soc/am64x/src/cslr_intr_mcu_m4fss0_core0.h */
#define MCU_GPIOMUX_INT4        CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_4    // 4 through 7
#define MCU_TIMER2_INT          CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER2_INTR_PEND_0               // 0 through 3
#define MCU_TIMER3_INT          CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER3_INTR_PEND_0               // 0 through 3
#define MAIN_WARM_RSTz_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_RESETZ_SYNC_STRETCH_0
#define MAIN_COLD_RSTz_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_PORZ_SYNC_STRETCH_0
#define MCU_ESM_HI_PRI_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_HI_LVL_0
#define MCU_ESM_LO_PRI_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_LOW_LVL_0
#define PRU0_PROTOCOL_ACK       CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_ISO_RESET_PROTCOL_ACK_0
#define PRU1_PROTOCOL_ACK       CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_ISO_RESET_PROTCOL_ACK_0
#define MAILBOX0_IPC_INT        CSLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_6_MAILBOX_CLUSTER_PEND_3
#define MAILBOX1_IPC_INT        CSLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_7_MAILBOX_CLUSTER_PEND_3

/* MCU GPIO 5 (LED), MCU GPIO 6 (connected to SW5 on EVM), and
 * MCU GPIO 7 (safe torque off) are used in this application */
#define GPIO_LED0           5
#define GPIO_BUTTON0        6
#define GPIO_OUTPUT0        7
#define GPIOMUX_OUT4        4

/* global variables */
volatile uint32_t warm_rst_counter, esm_err_counter, button_counter, pru_ack_counter, mbox_msg_counter;
volatile uint32_t reset_requested, pru_ack_received, pru_ack_timeout, fsoe_data_available;
volatile uint32_t patternPhase, patternState;

void configure_interrupts()
{
    Intc_Init();

    Intc_IntClrPend(MCU_GPIOMUX_INT4);
    Intc_IntClrPend(MCU_TIMER3_INT);
    Intc_IntClrPend(MAIN_WARM_RSTz_INT);
    Intc_IntClrPend(MCU_ESM_HI_PRI_INT);
    Intc_IntClrPend(MCU_ESM_LO_PRI_INT);
    Intc_IntClrPend(PRU0_PROTOCOL_ACK);
    Intc_IntClrPend(PRU1_PROTOCOL_ACK);
    Intc_IntClrPend(MAILBOX0_IPC_INT);
    Intc_IntClrPend(MAILBOX1_IPC_INT);

    Intc_IntRegister(MCU_GPIOMUX_INT4, &gpio_button_isr, NULL);
    Intc_IntRegister(MCU_TIMER3_INT, &ledPattern_isr, NULL);
    Intc_IntRegister(MAIN_WARM_RSTz_INT, &main_warm_rst_req_isr, NULL);
    Intc_IntRegister(MCU_ESM_HI_PRI_INT, &mcu_esm_error_isr, NULL);
    Intc_IntRegister(MCU_ESM_LO_PRI_INT, &mcu_esm_error_isr, NULL);
    Intc_IntRegister(PRU0_PROTOCOL_ACK, &pru_protocol_ack_isr, NULL);
    Intc_IntRegister(PRU1_PROTOCOL_ACK, &pru_protocol_ack_isr, NULL);
    Intc_IntRegister(MAILBOX0_IPC_INT, &mailbox_isr, NULL);
    Intc_IntRegister(MAILBOX1_IPC_INT, &mailbox_isr, NULL);

    Intc_SystemEnable(MCU_GPIOMUX_INT4);
    Intc_SystemEnable(MCU_TIMER3_INT);
    Intc_SystemEnable(MAIN_WARM_RSTz_INT);
    Intc_SystemEnable(MCU_ESM_HI_PRI_INT);
    Intc_SystemEnable(MCU_ESM_LO_PRI_INT);
    Intc_SystemEnable(PRU0_PROTOCOL_ACK);
    Intc_SystemEnable(PRU1_PROTOCOL_ACK);
    Intc_SystemEnable(MAILBOX0_IPC_INT);
    Intc_SystemEnable(MAILBOX1_IPC_INT);

    warm_rst_counter = 0;
    esm_err_counter = 0;
    button_counter = 0;
    pru_ack_counter = 0;
    mbox_msg_counter = 0;

    Intc_IntEnable(0U);
}

void configure_gpiomux(uint16_t src_index, uint16_t dst_index)
{
    int32_t                             status;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                              TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id         = TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0;
    rmIrqReq.dst_id         = TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0;
    rmIrqReq.src_index      = src_index;
    rmIrqReq.dst_host_irq   = dst_index;

    status = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);

    if(status != CSL_PASS)
    {
        UART_printf("[Error] configure_gpiomux() failed.\n");
    }
}

void configure_isolation()
{
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;

    /* unlock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = UNLOCK0_VAL;
    MCU_CTRL_MMR->LOCK6_KICK1 = UNLOCK1_VAL;

    /* enable MCU domain debug isolation from JTAG - DISABLE WHILE DEBUGGING */
    //MCU_CTRL_MMR->ISO_CTRL |= MCU_DBG_ISO_EN_MASK;

    /* enable MCU domain reset isolation from MAIN domain */
    MCU_CTRL_MMR->ISO_CTRL |= MCU_RST_ISO_EN_MASK;

    /* then block MAIN domain warm resets */
    MCU_CTRL_MMR->RST_CTRL |= MCU_RST_ISO_DONE_MASK;

    /* above 2 steps only take effect after setting
        the magic mmr register -- any non-zero value will work */
    MCU_CTRL_MMR->RST_MAGIC_WORD = 0x1234ABCD;

    /* lock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = 0U;
    MCU_CTRL_MMR->LOCK6_KICK1 = 0U;
}

int32_t configure_timer(uint32_t timer_base, uint32_t compareVal)
{
    /* return false on success */
    int32_t retVal = CSL_PASS;

    /* clear config to default */
    retVal |= TIMERReset(timer_base);

    /* set initial compare value */
    retVal |= TIMERCompareSet(timer_base, compareVal);

    /* continuous mode (auto-reload) with compare */
    retVal |= TIMERModeConfigure(timer_base, TIMER_AUTORLD_CMP_ENABLE);

    /* enable timer compare mode interrupt */
    retVal |= TIMERIntEnable(timer_base, TIMER_INT_MAT_EN_FLAG);

    /* start timer */
    retVal |= TIMEREnable(timer_base);

    return retVal;
}

int32_t stop_timer(uint32_t timer_base, uint32_t timer_int)
{
    /* return false on success */
    int32_t retVal = CSL_PASS;

    /* this should clear everything */
    retVal |= TIMERIntStatusClear(timer_base, TIMER_INT_MAT_IT_FLAG);
    retVal |= TIMERReset(timer_base);

    Intc_SystemDisable(timer_int);
    Intc_IntClrPend(timer_int);

    return retVal;
}

int32_t configure_ledPattern()
{
    /* return false on success */
    int32_t retVal = false;
    patternState = GPIO_PIN_LOW;
    patternPhase = 0;

    GPIOSetDirMode_v0(MCU_GPIO_MMR, GPIO_LED0, GPIO_DIRECTION_OUTPUT);
    GPIOPinWrite_v0(MCU_GPIO_MMR, GPIO_LED0, patternState);

    /* configure timer & start with initial compare value */
    retVal |= configure_timer(MCU_TIMER3_MMR, 25000000);

    return retVal;
}

void set_EmergencyStop()
{
    /* gpio call will power OFF the C2000 motor drive boosterpack  */
    GPIOSetDirMode_v0(MCU_GPIO_MMR, GPIO_OUTPUT0, GPIO_DIRECTION_OUTPUT);
    GPIOPinWrite_v0(MCU_GPIO_MMR, GPIO_OUTPUT0, GPIO_PIN_HIGH);
}

void unset_EmergencyStop()
{
    /* gpio call will power ON the C2000 motor drive boosterpack  */
    GPIOSetDirMode_v0(MCU_GPIO_MMR, GPIO_OUTPUT0, GPIO_DIRECTION_OUTPUT);
    GPIOPinWrite_v0(MCU_GPIO_MMR, GPIO_OUTPUT0, GPIO_PIN_LOW);
}

void fsoe_stack_call()
{
    /* add fsoe stack call here to feed it latest data */
    black_channel_get_data();
}

void block_main_warmrst()
{
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;

    /* unlock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = UNLOCK0_VAL;
    MCU_CTRL_MMR->LOCK6_KICK1 = UNLOCK1_VAL;

    /* enable MAIN domain reset blocking */
    MCU_CTRL_MMR->RST_CTRL |= MCU_RST_ISO_DONE_MASK;

    /* lock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = 0U;
    MCU_CTRL_MMR->LOCK6_KICK1 = 0U;
}

void unblock_main_warmrst()
{
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;

    /* unlock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = UNLOCK0_VAL;
    MCU_CTRL_MMR->LOCK6_KICK1 = UNLOCK1_VAL;

    /* disable MAIN domain reset blocking */
    MCU_CTRL_MMR->RST_CTRL &= ~MCU_RST_ISO_DONE_MASK;

    /* lock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = 0U;
    MCU_CTRL_MMR->LOCK6_KICK1 = 0U;
}

void trigger_main_warmrst()
{
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;

    /* unlock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = UNLOCK0_VAL;
    MCU_CTRL_MMR->LOCK6_KICK1 = UNLOCK1_VAL;

    /* write to CTRL MMR to trigger a warm reset on MAIN domain */
    MCU_CTRL_MMR->RST_CTRL &= MAIN_WARMRST_MASK;

    /* lock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = 0U;
    MCU_CTRL_MMR->LOCK6_KICK1 = 0U;
}

void main_warm_rst_req_isr()
{
    Intc_IntDisable();
    Intc_SystemDisable(MAIN_WARM_RSTz_INT);

    /* stop motor immediately on reset signal */
    set_EmergencyStop();

    warm_rst_counter++;
    reset_requested = 1;

    /* start a timer while waiting for PRU acknowledgement
     * then reset after timer expires if PRU does not respond */
    configure_timer(MCU_TIMER2_MMR, 25000000);
    Intc_IntClrPend(MCU_TIMER2_INT);
    Intc_IntRegister(MCU_TIMER2_INT, &pru_timeout_isr, NULL);
    Intc_SystemEnable(MCU_TIMER2_INT);

    Intc_IntClrPend(MAIN_WARM_RSTz_INT);
    Intc_IntEnable(0U);
}

void mcu_esm_error_isr()
{
    Intc_IntDisable();

    /* stop motor immediately on error signal */
    set_EmergencyStop();
    trigger_main_warmrst();

    esm_err_counter++;
    main_esm_clear_error();

    Intc_IntClrPend(MCU_ESM_HI_PRI_INT);
    Intc_IntClrPend(MCU_ESM_LO_PRI_INT);
    Intc_IntEnable(0U);
}

void gpio_button_isr()
{
    Intc_IntDisable();

    /* stop motor immediately on reset signal */
    set_EmergencyStop();

    /* clear gpio source interrupt */
    GPIOIntrClear_v0(MCU_GPIO_MMR, GPIO_BUTTON0);
    while(GPIOIntrStatus_v0(MCU_GPIO_MMR, GPIO_BUTTON0));

    button_counter++;
    Intc_IntClrPend(MCU_GPIOMUX_INT4);
    Intc_IntEnable(0U);
}

void pru_protocol_ack_isr()
{
    pru_ack_counter++;
    pru_ack_received = 1;

    Intc_IntClrPend(PRU0_PROTOCOL_ACK);
    Intc_IntClrPend(PRU1_PROTOCOL_ACK);
}

void pru_timeout_isr()
{
    pru_ack_timeout = 1;

    /* this should clear everything */
    stop_timer(MCU_TIMER2_MMR, MCU_TIMER2_INT);
}

void mailbox_isr()
{
    Intc_IntDisable();
    uint32_t msg_data = 0;

    mbox_msg_counter++;
    MailboxGetMessage(MAIN_MBOX6_MMR, MAILBOX_QUEUE_0, &msg_data);
    MailboxClrNewMsgStatus(MAIN_MBOX6_MMR, MAILBOX_M4F_CPUID, MAILBOX_QUEUE_0);

    /* integrating Black Channel communication and Safe Torque Off demo */
    if (msg_data == CMD_MAILBOX_MSG_BOOT_COMPLETE) {
        block_main_warmrst();
        reset_requested = 0;
        pru_ack_timeout = 0;
        pru_ack_received = 0;

        /* Only enable Reset ISR when boot is complete */
        Intc_IntClrPend(MAIN_WARM_RSTz_INT);
        Intc_SystemEnable(MAIN_WARM_RSTz_INT);

        configure_esm();
    }
    else if (msg_data == CMD_MAILBOX_MSG_APPLY_STO)
        set_EmergencyStop();
    else if (msg_data == CMD_MAILBOX_MSG_LIFT_STO)
        unset_EmergencyStop();
    else if (msg_data == CMD_MAILBOX_MSG_BLACKCHANNEL_DATA)
        fsoe_stack_call();

    Intc_IntClrPend(MAILBOX0_IPC_INT);
    Intc_IntClrPend(MAILBOX1_IPC_INT);
    Intc_IntEnable(0U);
}

void ledPattern_isr()
{
    Intc_IntDisable();
    uint32_t compareVal = TIMERCounterGet(MCU_TIMER3_MMR);

    patternState ^= 1U;
    GPIOSetDirMode_v0(MCU_GPIO_MMR, GPIO_LED0, GPIO_DIRECTION_OUTPUT);
    GPIOPinWrite_v0(MCU_GPIO_MMR, GPIO_LED0, patternState);

    switch (patternPhase++) {
    case(0):
        compareVal += 2000000;
        break;
    case(1):
        compareVal += 4000000;
        break;
    case(2):
        compareVal += 4000000;
        break;
    case(3):
        compareVal += 15000000;
        patternPhase = 0;
        break;
    }

    TIMERCompareSet(MCU_TIMER3_MMR, compareVal);
    TIMERIntStatusClear(MCU_TIMER3_MMR, TIMER_INT_MAT_IT_FLAG);

    Intc_IntClrPend(MCU_TIMER3_INT);
    Intc_IntEnable(0U);
}

void application_loop()
{
    reset_requested = 0;
    pru_ack_timeout = 0;
    pru_ack_received = 0;

    GPIOSetDirMode_v0(MCU_GPIO_MMR, GPIO_OUTPUT0, GPIO_DIRECTION_OUTPUT);
    GPIOPinWrite_v0(MCU_GPIO_MMR, GPIO_OUTPUT0, GPIO_PIN_HIGH);

    GPIOSetDirMode_v0(MCU_GPIO_MMR, GPIO_BUTTON0, GPIO_DIRECTION_INPUT);
    GPIOIntrEnable_v0(MCU_GPIO_MMR, GPIO_BUTTON0, GPIO_INTR_MASK_FALL_EDGE);
    configure_gpiomux(GPIO_BUTTON0, GPIOMUX_OUT4);

    UART_printf("[Info] Application loop started.\n");
    while(1U) {
        if (reset_requested == 1 && (pru_ack_received == 1 || pru_ack_timeout == 1)) {
            if (pru_ack_received == 1) {
                /* stop the PRU time-out timer if it has not yet expired */
                stop_timer(MCU_TIMER2_MMR, MCU_TIMER2_INT);
            }

            /* all conditions to un-block reset are met */
            UART_printf("[Info] Main domain resetting now.\n");
            unblock_main_warmrst();

            /* clear flags */
            reset_requested = 0;
            pru_ack_timeout = 0;
            pru_ack_received = 0;
        }
    }
}

