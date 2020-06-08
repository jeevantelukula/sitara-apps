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
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_mailbox.h>

#include "app.h"
#include "esmcfg.h"
#include "ratcfg.h"
#include "appmbox.h"
#include "blackchannel.h"

/* ti/csl/soc/am64x/src/cslr_soc_baseaddress.h */
#define MAIN_MBOX6_MMR          (CSL_MAILBOX0_REGS6_BASE + MCU_RAT_OFFSET7)
/* ti/csl/soc/am64x/src/cslr_mcu_m4fss0_baseaddress.h */
#define MCU_CTRL_MMR_BASE       (CSL_MCU_CTRL_MMR0_CFG0_BASE + MCU_RAT_OFFSET0)

/* required for unlocking MCU CTRL MMR write protected registers */
#define UNLOCK0_VAL             (uint32_t) 0x68EF3490
#define UNLOCK1_VAL             (uint32_t) 0xD172BC5A

/* ti/csl/soc/am64x/src/cslr_mcu_ctrl_mmr.h */
#define MCU_RST_ISO_DONE_MASK   CSL_MCU_CTRL_MMR_CFG0_RST_CTRL_MCU_RESET_ISO_DONE_Z_MASK
#define MCU_DBG_ISO_EN_MASK     CSL_MCU_CTRL_MMR_CFG0_ISO_CTRL_MCU_DBG_ISO_EN_MASK
#define MCU_RST_ISO_EN_MASK     CSL_MCU_CTRL_MMR_CFG0_ISO_CTRL_MCU_RST_ISO_EN_MASK
#define MAIN_WARMRST_MASK       (uint32_t) 0xFFFFFFF6

/* ti/csl/soc/am64x/src/cslr_intr_mcu_m4fss0_core0.h */
#define MAIN_WARM_RSTz_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_RESETZ_SYNC_STRETCH_0
#define MAIN_COLD_RSTz_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_PORZ_SYNC_STRETCH_0
#define MCU_ESM_HI_PRI_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_HI_LVL_0
#define MCU_ESM_LO_PRI_INT      CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_LOW_LVL_0
#define PRU0_PROTOCOL_ACK       CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_ISO_RESET_PROTCOL_ACK_0
#define PRU1_PROTOCOL_ACK       CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_ISO_RESET_PROTCOL_ACK_0
#define MAILBOX0_IPC_INT        CSLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_6_MAILBOX_CLUSTER_PEND_3
#define MAILBOX1_IPC_INT        CSLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_6_MAILBOX_CLUSTER_PEND_3

/* global variables */
volatile uint32_t reset_received, pru_ack_received, fsoe_data_available;

void configure_nvic()
{
    Intc_Init();

    Intc_IntClrPend(MAIN_WARM_RSTz_INT);
    Intc_IntClrPend(MCU_ESM_HI_PRI_INT);
    Intc_IntClrPend(MCU_ESM_LO_PRI_INT);
    Intc_IntClrPend(PRU0_PROTOCOL_ACK);
    Intc_IntClrPend(PRU1_PROTOCOL_ACK);
    Intc_IntClrPend(MAILBOX0_IPC_INT);
    Intc_IntClrPend(MAILBOX1_IPC_INT);

    Intc_IntRegister(MAIN_WARM_RSTz_INT, &main_warm_rst_req_isr, NULL);
    Intc_IntRegister(MCU_ESM_HI_PRI_INT, &mcu_esm_error_isr, NULL);
    Intc_IntRegister(MCU_ESM_LO_PRI_INT, &mcu_esm_error_isr, NULL);
    Intc_IntRegister(PRU0_PROTOCOL_ACK, &pru_protocol_ack_isr, NULL);
    Intc_IntRegister(PRU1_PROTOCOL_ACK, &pru_protocol_ack_isr, NULL);
    Intc_IntRegister(MAILBOX0_IPC_INT, &mailbox_isr, NULL);
    Intc_IntRegister(MAILBOX1_IPC_INT, &mailbox_isr, NULL);

    Intc_SystemEnable(MAIN_WARM_RSTz_INT);
    Intc_SystemEnable(MCU_ESM_HI_PRI_INT);
    Intc_SystemEnable(MCU_ESM_LO_PRI_INT);
    Intc_SystemEnable(PRU0_PROTOCOL_ACK);
    Intc_SystemEnable(PRU1_PROTOCOL_ACK);
    Intc_SystemEnable(MAILBOX0_IPC_INT);
    Intc_SystemEnable(MAILBOX1_IPC_INT);

    Intc_IntEnable(0);
}

void configure_isolation()
{
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;

    /* unlock control mmr p6 */
    MCU_CTRL_MMR->LOCK6_KICK0 = UNLOCK0_VAL;
    MCU_CTRL_MMR->LOCK6_KICK1 = UNLOCK1_VAL;

    /* enable MCU domain debug isolation from JTAG - DISABLE WHILE DEBUGGING */
    MCU_CTRL_MMR->ISO_CTRL |= MCU_DBG_ISO_EN_MASK;

    /* enable MCU domain reset isolation from MAIN domain */
    MCU_CTRL_MMR->ISO_CTRL |= MCU_RST_ISO_EN_MASK;

    /* then block MAIN domain warm resets */
    MCU_CTRL_MMR->RST_CTRL |= MCU_RST_ISO_DONE_MASK;

    /* above 2 steps only take effect after setting
        the magic mmr register -- any non-zero value will work */
    MCU_CTRL_MMR->RST_MAGIC_WORD = 0x1234ABCD;
}

void set_EmergencyStop()
{
    /* SITSW-229: Add GPIO code to disable C2000 BoosterPack */
    /* gpio call will be filled in when EVM hardware details finalize */
    //GPIOPinWrite_v0(gpio_base_address, gpio_pin, GPIO_PIN_LOW);
}

void unset_EmergencyStop()
{
    /* SITSW-229: Add GPIO code to re-enable C2000 BoosterPack */
    /* gpio call will be filled in when EVM hardware details finalize */
    //GPIOPinWrite_v0(gpio_base_address, gpio_pin, GPIO_PIN_HIGH);
}

void fsoe_stack_call()
{
    /* customer adds fsoe stack call
        here to feed it latest data */

    /* SITSW-231: update usage example in blackchannel.c */
    black_channel_get_data();
}

void trigger_main_warmrst()
{
    /* write to CTRL MMR to trigger a warm reset on MAIN domain */
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;
    MCU_CTRL_MMR->RST_CTRL &= MAIN_WARMRST_MASK;
}

void block_main_warmrst()
{    
    /* enable MAIN domain reset blocking */
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;
    MCU_CTRL_MMR->RST_CTRL |= MCU_RST_ISO_DONE_MASK;
}

void unblock_main_warmrst()
{    
    /* enable MAIN domain reset blocking */
    CSL_mcu_ctrl_mmr_cfg0Regs * MCU_CTRL_MMR = (CSL_mcu_ctrl_mmr_cfg0Regs *) MCU_CTRL_MMR_BASE;
    MCU_CTRL_MMR->RST_CTRL &= ~MCU_RST_ISO_DONE_MASK;
}

void main_warm_rst_req_isr()
{
    reset_received = 1;

    /* stop motor immediately on reset signal */
    set_EmergencyStop();

    Intc_IntClrPend(MAIN_WARM_RSTz_INT);
}

void mcu_esm_error_isr()
{
    /* stop motor immediately on error signal */
    set_EmergencyStop();

    trigger_main_warmrst();

    main_esm_clear_error();

    Intc_IntClrPend(MCU_ESM_HI_PRI_INT);
    Intc_IntClrPend(MCU_ESM_LO_PRI_INT);
}

void pru_protocol_ack_isr()
{
    pru_ack_received = 1;

    Intc_IntClrPend(PRU0_PROTOCOL_ACK);
    Intc_IntClrPend(PRU1_PROTOCOL_ACK);
}

void mailbox_isr()
{
    uint32_t msg_data = 0;
    MailboxGetMessage(MAIN_MBOX6_MMR, MAILBOX_QUEUE_0, &msg_data);
    MailboxClrNewMsgStatus(MAIN_MBOX6_MMR, MAILBOX_M4F_CPUID, MAILBOX_QUEUE_0);

    /* integrating Black Channel communication and Safe Torque Off demo */
    if (msg_data == CMD_MAILBOX_MSG_BOOT_COMPLETE) {
        block_main_warmrst();
        configure_esm();
        unset_EmergencyStop();
    }
    else if (msg_data == CMD_MAILBOX_MSG_APPLY_STO)
        set_EmergencyStop();
    else if (msg_data == CMD_MAILBOX_MSG_LIFT_STO)
        unset_EmergencyStop();
    else if (msg_data == CMD_MAILBOX_MSG_BLACKCHANNEL_DATA)
        fsoe_stack_call();

    Intc_IntClrPend(MAILBOX0_IPC_INT);
    Intc_IntClrPend(MAILBOX1_IPC_INT);
}

/* HACK: M4F Interrupts Currently in Debug, Polling for Now */
bool MyIntCheckPending(uint16_t intNum)
{
    volatile uint32_t * irqPendSetAddr = (uint32_t *) 0xE000E280U;

    while (intNum > 31) {
        intNum -= 32;
        irqPendSetAddr++;
    }

    uint32_t reg_val = HW_RD_REG32(irqPendSetAddr);

    if (reg_val & (1U << intNum))
        return true;
    else
        return false;
}

void application_loop()
{
    bool looping = true;
    uint32_t loopCounter = 0;
    uint32_t heartbeatState = GPIO_PIN_LOW;
    reset_received = 0;
    pru_ack_received = 0;


    while(looping) {
        loopCounter = 10000;    /* adjust so heartbeat LED toggles 1Hz */

        while(loopCounter-- > 0) {
            /* HACK: will remove next four "if" statements when interrupts work */
            if (MyIntCheckPending(MAIN_WARM_RSTz_INT)) {
                main_warm_rst_req_isr();
            }

            if (MyIntCheckPending(MCU_ESM_HI_PRI_INT) || MyIntCheckPending(MCU_ESM_LO_PRI_INT)) {
                mcu_esm_error_isr();
            }

            if (MyIntCheckPending(PRU0_PROTOCOL_ACK) || MyIntCheckPending(PRU1_PROTOCOL_ACK)) {
                pru_protocol_ack_isr();
            }

            if (MyIntCheckPending(MAILBOX0_IPC_INT) || MyIntCheckPending(MAILBOX1_IPC_INT)) {
                mailbox_isr();
            }

            if (reset_received == 1 && pru_ack_received == 1) {
                reset_received = 0;
                pru_ack_received = 0;

                /* all conditions to un-block reset are met */
                unblock_main_warmrst();
            }
        }

        /* toggle LED via GPIO pin every time loopCounter hits 0 */
        heartbeatState = heartbeatState == 0 ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
        /* SITSW-229: will be filled in when EVM hardware details finalize */
        //GPIOPinWrite_v0(gpio_base_address, gpio_pin, heartbeatState);

    }
}

