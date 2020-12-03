/**
* tiesc_soc_am64x.c: Implements a mechanism to send message from R5F to M4F.
*/
/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <tiesc_soc.h>

#include <tiescbsp.h>
#include <tiesc_soc.h>

#ifndef TIESC_EMULATION_PLATFORM
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/spi/SPI.h>
#endif

#include <ti/drv/uart/UART_stdio.h>

#include <ti/board/board.h>

#include <ti/csl/soc.h>
#include <ti/csl/csl_spinlock.h>
#include <soc_icss_header.h>

#ifndef TIESC_EMULATION_PLATFORM
#include <ti/drv/gpio/GPIO.h>
#include <board_gpioLed.h>
#endif
#include <board_misc.h>

#ifndef TIESC_EMULATION_PLATFORM
#include <board_spi.h>
#include <board_phy.h>
#include <board_dpphy.h>
#include <board_rotary_switch.h>

#include <delay_us.h>
#include <board_i2cLed.h>
#endif
#include <version.h>

#ifndef TIESC_EMULATION_PLATFORM
#include <ti/board/src/am64x_evm/am64x_idk_pinmux.h>
#include <ti/board/src/am64x_evm/include/pinmux.h>
#endif

#if !defined (__aarch64__)
#ifndef TIESC_EMULATION_PLATFORM
#include <ti/drv/i2c/soc/I2C_soc.h>
#endif
#include <ti/csl/soc/am64x/src/cslr_intr_r5fss0_core0.h>
#include <ti/csl/soc/am64x/src/cslr_soc_baseaddress.h>
#include <ti/board/src/am64x_evm/include/board_cfg.h>
#endif

#include <ti/drv/sciclient/sciclient.h>

#if defined (__aarch64__)
/* A53 */
#define ARM_INTERRUPT_OFFSET_ICSS0 (286-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (294-20)
#else
/* R5F */
#define ARM_INTERRUPT_OFFSET_ICSS0 (120-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (248-20)
#endif

#define CSL_SEMAPHORE_REG_OFFSET(n)        (0x100 + ((n) * 0x04))

#ifndef TIESC_EMULATION_PLATFORM
SPI_Handle handle;                   /* SPI handle */
#endif

extern PRUICSS_Handle pruIcss1Handle;
extern PRUICSS_Config pruss_config[2 + 1];

/* Board specific definitions */
#define MCSPI_INSTANCE         (0U)

void tiesc_mii_pinmuxConfig (void);
void tiesc_icssg_route_interrupts (void);

/* FIXME: This is a workaround function. Need an API from PDK for the same. Tracked by PDK-7103*/
int32_t tiesc_setPLLClk(uint32_t modId, uint32_t clkId, uint64_t clkRate);

extern bool icssgResetIsolated;

/* "safedata" should be defined as a section in linker command file and should be placed in OCSRAM5.*/
#pragma DATA_SECTION(BlackChannel, ".safedata")
blackChannel_t BlackChannel;

void Configure_Rat(void)
{
    CSL_ratRegs * R5F_RAT_MMR = (CSL_ratRegs *) CSL_R5FSS0_RAT_CFG_BASE;
	
    CSL_ratDisableRegionTranslation(R5F_RAT_MMR, 0U);

    /* sizeInBytes, baseAddress, translatedAddress */
    /* 2MB, 32-bit local memory map, up to 64-bit SoC memory map */
    CSL_RatTranslationCfgInfo ratCfg0 = { OCSRAM_SIZE_INBYTES, OCSRAM_BASE_ADDRESS, OCSRAM_TRANSLATE_ADDRESS }; /* needed for OC-RAM */

    CSL_ratConfigRegionTranslation(R5F_RAT_MMR, 0U, &ratCfg0);

    CSL_ratEnableRegionTranslation(R5F_RAT_MMR, 1U);
}

void Configure_Mbox(void)
{
    //MailboxReset(MAILBOX_BASE_ADDRESS);
    MailboxDisableQueueNotFullInt(MAILBOX_BASE_ADDRESS, MAILBOX_R5F0_CPUID, MAILBOX_QUEUE_0);
    MailboxEnableNewMsgInt(MAILBOX_BASE_ADDRESS, MAILBOX_M4F_CPUID, MAILBOX_QUEUE_0);
}

void Send_BootComplete_Message_To_Partner(void)
{
	uint32_t msg_status;
	
    Configure_Rat();
    Configure_Mbox();

	/*Sending this data to test Black Channel data exchange only. This will not be used until FSoE is available on M4F. */
    BlackChannel.num_bytes = 3U;    /* data size */
    BlackChannel.data[0] = 0xA0U;   /* data[0] */
    BlackChannel.data[1] = 0x55U;   /* data[1] */
    BlackChannel.data[2] = 0x0AU;   /* data[size-1] */

    /* Send Message to M4F. */
    msg_status = MailboxSendMessage(MAILBOX_BASE_ADDRESS, MAILBOX_QUEUE_0, (uint32_t) CMD_MAILBOX_MSG_BOOT_COMPLETE);
	/*To avoid compiler warning.*/
	msg_status = msg_status;
}

uint8_t isEtherCATDevice(void)
{
    volatile uint32_t temp;
#if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
    temp = *((uint32_t *)(CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));
#elif (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
    temp = *((uint32_t *)(CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));
#endif
    temp = (temp && 0x01);
    if(0x00 == temp)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int16_t getARMInterruptOffset()
{
#if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
    return ARM_INTERRUPT_OFFSET_ICSS0;
#elif (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
    return ARM_INTERRUPT_OFFSET_ICSS1;
#else
    return ARM_INTERRUPT_OFFSET_ICSS2;
#endif
}

void initSpinlock()
{
    /* Setting up RAT config to map SOC Spinlock to C23 constant of PRUICSS */
    /* Mapping 0xC0000000 (C23 constant of PRUICSS) to 0x30E00800 (SPINLOCK) with size 2^11 = 0x800 */
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x24))) = (0xC0000000u); //rat0 base0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x28))) = (CSL_SPINLOCK0_BASE); //rat0 trans_low0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x2C))) = (0x00000000u); //rat0 trans_high0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x20))) = (1u << 31) | (15u); //rat0 ctrl0

    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x24))) = (0xC0000000u); //rat0 base0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x28))) = (CSL_SPINLOCK0_BASE); //rat0 trans_low0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x2C))) = (0x00000000u); //rat0 trans_high0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x20))) = (1u << 31) | (15u); //rat0 ctrl0

}

uint32_t getSpinlockLockReg0Offset()
{
    return (CSL_SPINLOCK0_BASE + CSL_SPINLOCK_LOCK_REG(0));
}

void bsp_soft_reset()
{
    //Device Reset not supported on AM654x
    return;
}


void bsp_soc_evm_init()
{

#ifndef TIESC_EMULATION_PLATFORM

#if !defined (__aarch64__)
    I2C_HwAttrs   i2c_cfg;
#endif
#endif

#ifndef TIESC_EMULATION_PLATFORM
    GPIO_init();
#endif

    if(icssgResetIsolated==FALSE)
    {
		/*This configuration is required only after poweron reset. Is not required after a warm reset.*/
		Board_init(BOARD_INIT_ICSS_ETH_PHY);
    }
    delay_us(100);

    /* FIXME: Remove this when it is being done in the board library*/
#ifdef TIESC_EMULATION_PLATFORM

    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0))) = 0x68EF3490;
    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1))) = 0xD172BC5A;

#endif

    //Making the clock for ICSSG core and IEP same
    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL))) |= 0x01;
    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL))) |= 0x01;

    pruIcss1Handle = PRUICSS_create(pruss_config, PRUICSS_INSTANCE);

    if(icssgResetIsolated==FALSE)
    {
		/* Selecting MII-RT mode in GPCFG mux */
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussCfgRegBase) + CSL_ICSSCFG_GPCFG0))) = 0x8000003;
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussCfgRegBase) + CSL_ICSSCFG_GPCFG1))) = 0x8000003;

#ifdef ECAT_RGMII
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiRtCfgRegBase) + 0x1000 + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG))) |= (0x28); // ICSS_G_CFG make it RGMII
#else
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiRtCfgRegBase) + 0x1000 + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG))) = (0x10001); // ICSS_G_CFG make it MII
#endif

#ifdef ECAT_RGMII
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiRtCfgRegBase) + 0x1000 + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG))) = 0x440000; //RGMII CFG make it 100Mbs
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0))) = 0x05F1001F; //Min frame size
		(*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1))) = 0x05F1001F; //min frame size

#ifndef TIESC_EMULATION_PLATFORM
		CSL_MDIO_phyRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 1), DPPHY_CFG1_REG, 0x100);
		CSL_MDIO_phyRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 2), DPPHY_CFG1_REG, 0x100);
#endif
#else

#ifndef TIESC_EMULATION_PLATFORM
		/* Setting up RX_ER/GPIO pin on the PHY as RX_ERR pin and COL/GPIO pin as LED_3 */
		MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 1), DPPHY_GPIO_MUX_CTRL2, 0x60);
		MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 2), DPPHY_GPIO_MUX_CTRL2, 0x60);

		/* Disable RGMII interface */
		MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 1), DPPHY_RGMIICTL, 0x50);
		MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 2), DPPHY_RGMIICTL, 0x50);
#endif

#endif


		PRUICSS_pinMuxConfig(pruIcss1Handle, 0x0);   // PRUSS pinmuxing
		//Disable PRUs - This is to ensure PRUs are not running when application is not initialized
		PRUICSS_pruDisable(pruIcss1Handle, 0);
		PRUICSS_pruDisable(pruIcss1Handle, 1);
    }

#if !defined (__aarch64__) && !defined(TIESC_EMULATION_PLATFORM)
    /* Route I2C0 interrupts to R5F */
    /* Main Domain I2C0 events are mapped to MAIN2MCU_INTRTR_LVL_IN_100 */
    /* Since 176 is used as I2C interrupt number, MAIN2MCU_INTRTR_LVL_IN_100 should be written to 0xA10044. */
    /* If 177 is used as I2C interrupt number, MAIN2MCU_INTRTR_LVL_IN_100 should be written to 0xA10048. */
    HW_WR_REG8_RAW(0xA10044, 100);

    if(icssgResetIsolated==FALSE)
    {
        /* Route ICSS Interrupts to R5F. */
        tiesc_icssg_route_interrupts();
    }
    /* LED's and Rotary Switch are connected to Main Domain I2C0. Reconfigure I2C to use I2C0 of main domain. */
    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(BOARD_I2C_IOEXP_INSTANCE, &i2c_cfg);

    i2c_cfg.baseAddr = CSL_I2C0_CFG_BASE;
    /*Use MCU_R5_CORE0_INT_IN_176, MAIN2MCU_LVL_INTRTR0 host interrupt 16.*/
    i2c_cfg.intNum = CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_16;

    I2C_socSetInitCfg(BOARD_I2C_IOEXP_INSTANCE, &i2c_cfg);
#endif

#ifndef TIESC_EMULATION_PLATFORM

    /* I2C Init */
    Board_i2cLedInit();

    /* Rotary Switch Init */
    Board_initRotarySwitch();

    QSPI_init();
#endif

#if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
    tiesc_setPLLClk(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK_PARENT_POSTDIV4_16FF_MAIN_0_HSDIVOUT9_CLK, 200000000);
#elif (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
    tiesc_setPLLClk(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK_PARENT_POSTDIV4_16FF_MAIN_0_HSDIVOUT9_CLK, 200000000);
#endif

}


void display_esc_version(uint16_t revision, uint16_t build)
{

    UART_printf("\n\rRevision/Type : x%04X", revision);
    UART_printf(" Build : x%04X", build);
    UART_printf("\n\rFirmware Version : %d.%d.%d\n\r", (revision >> 8),
                (build >> 8), (build & 0xFF));

}

void * tiesc_memcpy(uint8_t *dst, const uint8_t *src, uint32_t size_bytes)
{
    uint32_t i;
    volatile uint8_t * dst_ptr = dst;

    for (i = 0; i < size_bytes; i++)
    {
      *dst_ptr++ = *src++;
    }

    ASSERT_DMB();
    ASSERT_DSB();
    return dst;
}

void * tiesc_memset(uint8_t *dst, int8_t val, uint32_t size_bytes)
{
    uint32_t i;
    volatile uint8_t * dst_ptr = dst;

    for (i = 0; i < size_bytes; i++)
    {
      *dst_ptr++ = val;
    }

    ASSERT_DMB();
    ASSERT_DSB();
    return dst;
}

#ifndef TIESC_EMULATION_PLATFORM

static pinmuxPerCfg_t gPru_icssg0_mii_g_rt0PinCfg[] =
{
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii_mt0_clk -> AC24 */
    {
        PIN_PRG0_PRU1_GPO16, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_txen -> AE27 */
    {
        PIN_PRG0_PRU1_GPO15, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_txd3 -> AD24 */
    {
        PIN_PRG0_PRU1_GPO14, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_txd2 -> AD25 */
    {
        PIN_PRG0_PRU1_GPO13, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_txd1 -> AC25 */
    {
        PIN_PRG0_PRU1_GPO12, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_txd0 -> AB24 */
    {
        PIN_PRG0_PRU1_GPO11, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxdv -> Y24 */
    {
        PIN_PRG0_PRU0_GPO4, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii_mr0_clk -> Y25 */
    {
        PIN_PRG0_PRU0_GPO6, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxd3 -> AA27 */
    {
        PIN_PRG0_PRU0_GPO3, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxd2 -> W24 */
    {
        PIN_PRG0_PRU0_GPO2, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxer -> V28 */
    {
        PIN_PRG0_PRU0_GPO5, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxd1 -> W25 */
    {
        PIN_PRG0_PRU0_GPO1, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxd0 -> V24 */
    {
        PIN_PRG0_PRU0_GPO0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii0_rxlink -> V27 */
    {
        PIN_PRG0_PRU0_GPO8, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii_mt1_clk -> AD28 */
    {
        PIN_PRG0_PRU0_GPO16, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_txen -> AA24 */
    {
        PIN_PRG0_PRU0_GPO15, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_txd3 -> AD26 */
    {
        PIN_PRG0_PRU0_GPO14, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_txd2 -> AC26 */
    {
        PIN_PRG0_PRU0_GPO13, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_txd1 -> AD27 */
    {
        PIN_PRG0_PRU0_GPO12, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_txd0 -> AB25 */
    {
        PIN_PRG0_PRU0_GPO11, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxdv -> AA25 */
    {
        PIN_PRG0_PRU1_GPO4, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii_mr1_clk -> AB27 */
    {
        PIN_PRG0_PRU1_GPO6, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxd3 -> AB26 */
    {
        PIN_PRG0_PRU1_GPO3, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxd2 -> AC27 */
    {
        PIN_PRG0_PRU1_GPO2, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxer -> U23 */
    {
        PIN_PRG0_PRU1_GPO5, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxd1 -> AC28 */
    {
        PIN_PRG0_PRU1_GPO1, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxd0 -> AB28 */
    {
        PIN_PRG0_PRU1_GPO0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG0_MII_G_RT1 -> pr0_mii1_rxlink -> W27 */
    {
        PIN_PRG0_PRU1_GPO8, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gPru_icssg0_mii_g_rtPinCfg[] =
{
    {0, TRUE, gPru_icssg0_mii_g_rt0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gPru_icssg1_mii_g_rt0PinCfg[] =
{
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii_mt0_clk -> AE19 */
    {
        PIN_PRG1_PRU1_GPO16, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_txen -> AG19 */
    {
        PIN_PRG1_PRU1_GPO15, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_txd3 -> AH19 */
    {
        PIN_PRG1_PRU1_GPO14, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_txd2 -> AF19 */
    {
        PIN_PRG1_PRU1_GPO13, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_txd1 -> AE20 */
    {
        PIN_PRG1_PRU1_GPO12, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_txd0 -> AC20 */
    {
        PIN_PRG1_PRU1_GPO11, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxdv -> AG23 */
    {
        PIN_PRG1_PRU0_GPO4, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii_mr0_clk -> AF22 */
    {
        PIN_PRG1_PRU0_GPO6, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxd3 -> AD21 */
    {
        PIN_PRG1_PRU0_GPO3, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxd2 -> AF23 */
    {
        PIN_PRG1_PRU0_GPO2, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxer -> AF27 */
    {
        PIN_PRG1_PRU0_GPO5, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxd1 -> AG24 */
    {
        PIN_PRG1_PRU0_GPO1, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxd0 -> AE22 */
    {
        PIN_PRG1_PRU0_GPO0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii0_rxlink -> AF28 */
    {
        PIN_PRG1_PRU0_GPO8, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii_mt1_clk -> AD20 */
    {
        PIN_PRG1_PRU0_GPO16, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_txen -> AD19 */
    {
        PIN_PRG1_PRU0_GPO15, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_txd3 -> AG20 */
    {
        PIN_PRG1_PRU0_GPO14, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_txd2 -> AH21 */
    {
        PIN_PRG1_PRU0_GPO13, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_txd1 -> AH20 */
    {
        PIN_PRG1_PRU0_GPO12, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_txd0 -> AF21 */
    {
        PIN_PRG1_PRU0_GPO11, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxdv -> AE21 */
    {
        PIN_PRG1_PRU1_GPO4, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii_mr1_clk -> AG22 */
    {
        PIN_PRG1_PRU1_GPO6, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxd3 -> AH22 */
    {
        PIN_PRG1_PRU1_GPO3, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxd2 -> AG21 */
    {
        PIN_PRG1_PRU1_GPO2, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxer -> AC22 */
    {
        PIN_PRG1_PRU1_GPO5, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxd1 -> AH23 */
    {
        PIN_PRG1_PRU1_GPO1, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxd0 -> AH24 */
    {
        PIN_PRG1_PRU1_GPO0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyPRU_ICSSG1_MII_G_RT1 -> pr1_mii1_rxlink -> AE24 */
    {
        PIN_PRG1_PRU1_GPO8, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gPru_icssg1_mii_g_rtPinCfg[] =
{
    {0, TRUE, gPru_icssg1_mii_g_rt0PinCfg},
    {PINMUX_END}
};

pinmuxBoardCfg_t gAM65xxMIIPinmuxData[] =
{
    {0, gPru_icssg0_mii_g_rtPinCfg},
    {1, gPru_icssg1_mii_g_rtPinCfg},
    {PINMUX_END}
};


#endif


void tiesc_icssg_route_interrupts (void)
{
    /* Route ICSS Interrupts to R5F. */
    /* ICSSG0 events are mapped to MAIN2MCU_INTRTR_LVL_IN[32:39]*/
    /* ICSSG1 events are mapped to MAIN2MCU_INTRTR_LVL_IN[40:47]*/    
    /* MAIN2MCU_RTR_LVL_MX_INTR[63:0] is mapped to MCU_R5_CORE0_INT_IN[223:160] */
    HW_WR_REG8_RAW(0xA10004,
                  32);  //MAIN2MCU_RTR_LVL_MX_INTR0 mapped to MAIN2MCU_RTR_LVL_MX_INTR32
    HW_WR_REG8_RAW(0xA10008,
                  33);  //MAIN2MCU_RTR_LVL_MX_INTR1 mapped to MAIN2MCU_RTR_LVL_MX_INTR33
    HW_WR_REG8_RAW(0xA1000C,
                  34);  //MAIN2MCU_RTR_LVL_MX_INTR2 mapped to MAIN2MCU_RTR_LVL_MX_INTR34
    HW_WR_REG8_RAW(0xA10010,
                  35);  //MAIN2MCU_RTR_LVL_MX_INTR3 mapped to MAIN2MCU_RTR_LVL_MX_INTR35
    HW_WR_REG8_RAW(0xA10014,
                  36);  //MAIN2MCU_RTR_LVL_MX_INTR4 mapped to MAIN2MCU_RTR_LVL_MX_INTR36
    HW_WR_REG8_RAW(0xA10018,
                  37);  //MAIN2MCU_RTR_LVL_MX_INTR5 mapped to MAIN2MCU_RTR_LVL_MX_INTR37
    HW_WR_REG8_RAW(0xA1001C,
                  38);  //MAIN2MCU_RTR_LVL_MX_INTR6 mapped to MAIN2MCU_RTR_LVL_MX_INTR38
    HW_WR_REG8_RAW(0xA10020,
                  39);  //MAIN2MCU_RTR_LVL_MX_INTR7 mapped to MAIN2MCU_RTR_LVL_MX_INTR39

    HW_WR_REG8_RAW(0xA10024,
                  40);  //MAIN2MCU_RTR_LVL_MX_INTR8 mapped to MAIN2MCU_RTR_LVL_MX_INTR40
    HW_WR_REG8_RAW(0xA10028,
                  41);  //MAIN2MCU_RTR_LVL_MX_INTR9 mapped to MAIN2MCU_RTR_LVL_MX_INTR41
    HW_WR_REG8_RAW(0xA1002C,
                  42);  //MAIN2MCU_RTR_LVL_MX_INTR10 mapped to MAIN2MCU_RTR_LVL_MX_INTR42
    HW_WR_REG8_RAW(0xA10030,
                  43);  //MAIN2MCU_RTR_LVL_MX_INTR11 mapped to MAIN2MCU_RTR_LVL_MX_INTR43
    HW_WR_REG8_RAW(0xA10034,
                  44);  //MAIN2MCU_RTR_LVL_MX_INTR12 mapped to MAIN2MCU_RTR_LVL_MX_INTR44
    HW_WR_REG8_RAW(0xA10038,
                  45);  //MAIN2MCU_RTR_LVL_MX_INTR13 mapped to MAIN2MCU_RTR_LVL_MX_INTR45
    HW_WR_REG8_RAW(0xA1003C,
                  46);  //MAIN2MCU_RTR_LVL_MX_INTR14 mapped to MAIN2MCU_RTR_LVL_MX_INTR46
    HW_WR_REG8_RAW(0xA10040,
                  47);  //MAIN2MCU_RTR_LVL_MX_INTR15 mapped to MAIN2MCU_RTR_LVL_MX_INTR47

    return;
}

#include <stdint.h>

//Region of pinmux padconfigs
//Can't find the CSL defines for the padconfig0 offset, need to replace
//The offsets were found from the AM64X MAIN and MCU padcfg_ctrl pdf documents
#define TEMP_REPLACE_WITH_CSL_MAIN_PADCONFIG0_OFFSET    (0x4000)
#define PINMUX_MAIN_REG_BASE                            (CSL_PADCFG_CTRL0_CFG0_BASE + TEMP_REPLACE_WITH_CSL_MAIN_PADCONFIG0_OFFSET)

//All defaults except:
//Receiver Disabled
//High-Z
#define PINMUX_DEFAULT_REG_VALUE        (0x08010000U)
#define PINMUX_DEFAULT                  (0xffU)

//Pullup/down/none
#define PINMUX_PD                       (0x0U)
#define PINMUX_PU                       (0x2U)
#define PINMUX_NP                       (0x1U)

#define PINMUX_DRIVESTRENGTH_0          (0x0U)
#define PINMUX_DRIVESTRENGTH_1          (0x1U)
#define PINMUX_DRIVESTRENGTH_2          (0x2U)
#define PINMUX_DRIVESTRENGTH_3          (0x3U)

//The pinmux mmrs are stupid about this:
#define PINMUX_RX_ACTIVE                (0x1U)
#define PINMUX_RX_DISABLE               (0x0U)
#define PINMUX_TX_ACTIVE                (0x0U)
#define PINMUX_TX_DISABLE               (0x1U)

typedef enum {
    PINMUX_MUX_MODE_0 = 0,
    PINMUX_MUX_MODE_1,
    PINMUX_MUX_MODE_2,
    PINMUX_MUX_MODE_3,
    PINMUX_MUX_MODE_4,
    PINMUX_MUX_MODE_5,
    PINMUX_MUX_MODE_6,
    PINMUX_MUX_MODE_7,
    PINMUX_MUX_MODE_8,
    PINMUX_MUX_MODE_9,
    PINMUX_MUX_MODE_10,
    PINMUX_MUX_MODE_11,
    PINMUX_MUX_MODE_12,
    PINMUX_MUX_MODE_13,
    PINMUX_MUX_MODE_14,
    PINMUX_MUX_MODE_15,
}muxmode_t;

typedef struct{
    uint32_t    mmrBaseAddress;
    uint32_t    pinmuxRegOffset;
    muxmode_t   muxmode; //0 through 15
    uint8_t     pu_pd;
    uint8_t     driveStrength;
    uint8_t     rx;
    uint8_t     tx;
} pinmux_t;

//Loops through an array of pinmuxs, setting the MMRs for each one.
//arraysize is the size of the whole array of structs in bytes.
//Leaves registers unchanged for DEFAULT configurations; so make sure that is the start state.
//Does not check if MMRs are unlocked before writing.
void set_pinmux(pinmux_t *Array, uint8_t arraysize);

#if !defined(ECAT_RGMII)

pinmux_t PINMUX_MAIN_ICSSG0_MII0_APP2_array [] = {
//pinmux reg base,  Reg offset, Muxmode,  PUPD,    DRIVE_STRENGTH,    RX,  TX

{PINMUX_MAIN_REG_BASE, 0x0160, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO0 ; PIN ->PRG0_MII0_RD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG88
{PINMUX_MAIN_REG_BASE, 0x0164, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO1 ; PIN ->PRG0_MII0_RD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG89
{PINMUX_MAIN_REG_BASE, 0x0168, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO2 ; PIN ->PRG0_MII0_RD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG90
{PINMUX_MAIN_REG_BASE, 0x016C, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO3 ; PIN ->PRG0_MII0_RD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG91
{PINMUX_MAIN_REG_BASE, 0x0170, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO4 ; PIN ->PRG0_MII0_RX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG92
{PINMUX_MAIN_REG_BASE, 0x0174, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO5 ; PIN ->PRG0_MII0_RXER ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG93
{PINMUX_MAIN_REG_BASE, 0x0178, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO6 ; PIN ->PRG0_MII0_RXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG94
{PINMUX_MAIN_REG_BASE, 0x0180, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO8 ; PIN ->PRG0_MII0_RXLINK ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG96
{PINMUX_MAIN_REG_BASE, 0x018C, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO11 ; PIN ->PRG0_MII0_TD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG99
{PINMUX_MAIN_REG_BASE, 0x0190, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO12 ; PIN ->PRG0_MII0_TD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG100
{PINMUX_MAIN_REG_BASE, 0x0194, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO13 ; PIN ->PRG0_MII0_TD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG101
{PINMUX_MAIN_REG_BASE, 0x0198, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO14 ; PIN ->PRG0_MII0_TD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG102
{PINMUX_MAIN_REG_BASE, 0x019C, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO15 ; PIN ->PRG0_MII0_TX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG103
{PINMUX_MAIN_REG_BASE, 0x01A0, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}  //PAD ->PRG0_PRU0GPO16 ; PIN ->PRG0_MII0_TXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG104
};

pinmux_t PINMUX_MAIN_ICSSG0_MII1_APP2_array [] = {
//pinmux reg base,  Reg offset, Muxmode,  PUPD,    DRIVE_STRENGTH,    RX,  TX

{PINMUX_MAIN_REG_BASE, 0x01B0, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE},  //PAD ->PRG0_PRU1GPO0 ; PIN ->PRG0_MII1_RD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG108
{PINMUX_MAIN_REG_BASE, 0x01B4, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO1 ; PIN ->PRG0_MII1_RD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG109
{PINMUX_MAIN_REG_BASE, 0x01B8, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO2 ; PIN ->PRG0_MII1_RD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG110
{PINMUX_MAIN_REG_BASE, 0x01BC, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO3 ; PIN ->PRG0_MII1_RD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG111
{PINMUX_MAIN_REG_BASE, 0x01C0, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO4 ; PIN ->PRG0_MII1_RX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MOdE_1, PADCONFIG112
{PINMUX_MAIN_REG_BASE, 0x01C4, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO5 ; PIN ->PRG0_MII1_RXER ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG113
{PINMUX_MAIN_REG_BASE, 0x01C8, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO6 ; PIN ->PRG0_MII1_RXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG114
{PINMUX_MAIN_REG_BASE, 0x01d0, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO8 ; PIN ->PRG0_MII1_RXLINK ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG116
{PINMUX_MAIN_REG_BASE, 0x01DC, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO11 ; PIN ->PRG0_MII1_TD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG119
{PINMUX_MAIN_REG_BASE, 0x01E0, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO12 ; PIN ->PRG0_MII1_TD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG120
{PINMUX_MAIN_REG_BASE, 0x01E4, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO13 ; PIN ->PRG0_MII1_TD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG121
{PINMUX_MAIN_REG_BASE, 0x01E8, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO14 ; PIN ->PRG0_MII1_TD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG122
{PINMUX_MAIN_REG_BASE, 0x01EC, PINMUX_MUX_MODE_0, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO15 ; PIN ->PRG0_MII1_TX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_0, PADCONFIG123
{PINMUX_MAIN_REG_BASE, 0x01F0, PINMUX_MUX_MODE_1, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}  //PAD ->PRG0_PRU1GPO16 ; PIN ->PRG0_MII1_TXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_1, PADCONFIG124
};

#else

pinmux_t PINMUX_MAIN_ICSSG0_RGMII1_APP2_array [] = {
//pinmux reg base,  Reg offset, Muxmode,  PUPD,    DRIVE_STRENGTH,    RX,  TX

{PINMUX_MAIN_REG_BASE, 0x0160, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO0 ; PIN ->PRG0_RGMII1_RD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0164, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO1 ; PIN ->PRG0_RGMII1_RD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0168, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO2 ; PIN ->PRG0_RGMII1_RD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x016C, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO3 ; PIN ->PRG0_RGMII1_RD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0170, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO4 ; PIN ->PRG0_RGMII1_RX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0178, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU0GPO6 ; PIN ->PRG0_RGMII1_RXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x018C, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO11 ; PIN ->PRG0_RGMII1_TD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0190, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO12 ; PIN ->PRG0_RGMII1_TD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0194, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO13 ; PIN ->PRG0_RGMII1_TD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x0198, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO14 ; PIN ->PRG0_RGMII1_TD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x019C, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU0GPO15 ; PIN ->PRG0_RGMII1_TX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01A0, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE } //PAD ->PRG0_PRU0GPO16 ; PIN ->PRG0_RGMII1_TXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
};

pinmux_t PINMUX_MAIN_ICSSG0_RGMII2_APP2_array [] = {
//pinmux reg base,  Reg offset, Muxmode,  PUPD,    DRIVE_STRENGTH,    RX,  TX

{PINMUX_MAIN_REG_BASE, 0x01B0, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE},  //PAD ->PRG0_PRU1GPO0 ; PIN ->PRG0_RGMII2_RD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01B4, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO1 ; PIN ->PRG0_RGMII2_RD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01B8, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO2 ; PIN ->PRG0_RGMII2_RD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01BC, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO3 ; PIN ->PRG0_RGMII2_RD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01C0, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO4 ; PIN ->PRG0_RGMII2_RX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01C8, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_ACTIVE , PINMUX_TX_DISABLE}, //PAD ->PRG0_PRU1GPO6 ; PIN ->PRG0_RGMII2_RXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01DC, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO11 ; PIN ->PRG0_RGMII2_TD0 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01E0, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO12 ; PIN ->PRG0_RGMII2_TD1 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01E4, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO13 ; PIN ->PRG0_RGMII2_TD2 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01E8, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO14 ; PIN ->PRG0_RGMII2_TD3 ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01EC, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE }, //PAD ->PRG0_PRU1GPO15 ; PIN ->PRG0_RGMII2_TX_CTL ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
{PINMUX_MAIN_REG_BASE, 0x01F0, PINMUX_MUX_MODE_2, PINMUX_DEFAULT, PINMUX_DEFAULT, PINMUX_RX_DISABLE, PINMUX_TX_ACTIVE } //PAD ->PRG0_PRU1GPO16 ; PIN ->PRG0_RGMII2_TXC ; PINMUX_MUX_MODE_ ->PINMUX_MUX_MODE_2
};

#endif

void set_pinmux(pinmux_t *Array, uint8_t arraysize){
    uint32_t c = 0;
    uint32_t i = 0;
    uint32_t mmr_reg_value = 0;
    uint32_t errors = 0;

    //Pointer for loop
    pinmux_t * arrayPtr = NULL;
    arrayPtr = Array;

    //Loop on each pinmux_t struct
    for (i = 0; i < arraysize; i++){
        //Set PADCONFIG register to defined PINMUX_DEFAULTs
        *(volatile uint32_t *)(uintptr_t)(arrayPtr->mmrBaseAddress + arrayPtr->pinmuxRegOffset) = PINMUX_DEFAULT_REG_VALUE;

        //Read the PADCONFIG register
        mmr_reg_value = *(volatile uint32_t *)(uintptr_t)(arrayPtr->mmrBaseAddress + arrayPtr->pinmuxRegOffset);

        //Mux mode slection
        if(arrayPtr->muxmode != PINMUX_DEFAULT){
            mmr_reg_value  &= (0xFFFFFFF0);     // Mux mode
            mmr_reg_value  |= (arrayPtr->muxmode & 0x0000000F);
        }

        //PU-PD Selection
        if(arrayPtr->pu_pd != PINMUX_DEFAULT){
            mmr_reg_value &= (0xFFFCFFFF);       // PU-PD
            mmr_reg_value |= ((arrayPtr->pu_pd) << 16);
        }

        //Drive strength selection
        if(arrayPtr->driveStrength != PINMUX_DEFAULT){
            mmr_reg_value &= (0xFFE7FFFF);       // Drive strength
            mmr_reg_value |= ((arrayPtr->driveStrength) << 19);
        }

        //Receive enable/disable
        if(arrayPtr->rx != PINMUX_DEFAULT){
            mmr_reg_value &= (0xFFFBFFFF);       // Drive strength
            mmr_reg_value |= ((arrayPtr->rx) << 18);
        }

        //Transmit enable/disable
        if(arrayPtr->tx != PINMUX_DEFAULT){
            mmr_reg_value &= (0xFFDFFFFF);       // Drive strength
            mmr_reg_value |= ((arrayPtr->tx) << 21);
        }

        //Writing to Padconfig register
        *(volatile uint32_t *)(uintptr_t)(arrayPtr->mmrBaseAddress + arrayPtr->pinmuxRegOffset) = mmr_reg_value;

        for(c = 0; c < 20; c++);

        //Verify write
        if((*(volatile uint32_t *)(uintptr_t)(arrayPtr->mmrBaseAddress + arrayPtr->pinmuxRegOffset)) != mmr_reg_value) errors++;

        arrayPtr++;
    }

    if (errors>0) { printf("set_pinmux(): Error");}
    else {          printf("set_pinmux(): Done");}

}

void tiesc_mii_pinmuxConfig (void)
{
#if !defined(ECAT_RGMII)
            set_pinmux(PINMUX_MAIN_ICSSG0_MII0_APP2_array, (sizeof(PINMUX_MAIN_ICSSG0_MII0_APP2_array)/sizeof(pinmux_t)));
            set_pinmux(PINMUX_MAIN_ICSSG0_MII1_APP2_array, (sizeof(PINMUX_MAIN_ICSSG0_MII1_APP2_array)/sizeof(pinmux_t)));
#else
            set_pinmux(PINMUX_MAIN_ICSSG0_RGMII1_APP2_array, (sizeof(PINMUX_MAIN_ICSSG0_RGMII1_APP2_array)/sizeof(pinmux_t)));
            set_pinmux(PINMUX_MAIN_ICSSG0_RGMII2_APP2_array, (sizeof(PINMUX_MAIN_ICSSG0_RGMII2_APP2_array)/sizeof(pinmux_t)));
#endif
}

int32_t tiesc_setPLLClk(uint32_t modId, uint32_t clkId, uint64_t clkRate)
{
    uint8_t parentId = 0;
    uint32_t i = 0U;
    int32_t status   = 0;
    uint64_t respClkRate = 0;
    uint32_t clockBaseId = 0U;
    uint32_t numParents = 0U;
    uint32_t clockStatus;

    status = Sciclient_pmQueryModuleClkFreq(modId,
                                        clkId,
                                        clkRate,
                                        &respClkRate,
                                        SCICLIENT_SERVICE_WAIT_FOREVER);
    if(status == CSL_PASS)
    {
        /* Check if the clock is enabled or not */
        status = Sciclient_pmModuleGetClkStatus(modId,
                                                clkId,
                                                &clockStatus,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
    }

    if(status == CSL_PASS)
    {
        /* Check if the clock is enabled or not */
        status = Sciclient_pmGetModuleClkNumParent(modId,
                                                clkId,
                                                &numParents,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
    }

    if ((status == CSL_PASS) && (respClkRate == clkRate))
    {
        status = Sciclient_pmSetModuleClkFreq(
                                modId,
                                clkId,
                                clkRate,
                                TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE,
                                SCICLIENT_SERVICE_WAIT_FOREVER);
        if (status == CSL_PASS)
        {
            if (clockStatus == TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ)
            {
                /* Enable the clock */
                status = Sciclient_pmModuleClkRequest(
                                                    modId,
                                                    clkId,
                                                    TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                                                    0U,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
            }
        }
    }
    else if (status == CSL_PASS)
    {
        /* Try to loop and change parents of the clock */
        for(i = 0U; i < numParents; i++)
        {
            /* Disabling the clock */
            status = Sciclient_pmModuleClkRequest(modId,
                                                  clkId,
                                                  TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ,
                                                  0U,
                                                  SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status == CSL_PASS)
            {
                clockStatus = TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ;
                /* Setting the new parent */
                status = Sciclient_pmSetModuleClkParent(
                                        modId,
                                        clkId,
                                        clkId+i+1,
                                        SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            /* Check if the clock can be set to desirable freq. */
            if (status == CSL_PASS)
            {
                status = Sciclient_pmQueryModuleClkFreq(modId,
                                                        clkId,
                                                        clkRate,
                                                        &respClkRate,
                                                        SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            if (status == CSL_PASS)
            {
                if(respClkRate == clkRate)
                {
                    break;
                }
                else
                {
                    status = CSL_EFAIL;
                }
            }
            parentId++;
            clockBaseId++;
        }

        if (status == CSL_PASS)
        {
            /* Set the clock at the desirable frequency*/
            status = Sciclient_pmSetModuleClkFreq(
                                    modId,
                                    clkId,
                                    clkRate,
                                    TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE,
                                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }
        if((status == CSL_PASS) && (clockStatus == TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ))
        {
            /*Enable the clock again */
            status = Sciclient_pmModuleClkRequest(
                                                modId,
                                                clkId,
                                                TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                                                0U,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
        }
    }

    return status;
}
