/**
* tiesc_soc_am65x.c: Implements EVM/IDK initialization and Pinmux.
*                    Does EtherCAT Device select and interrupt allocation.
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

#include <tiescbsp.h>
#include <tiesc_soc.h>

#include <ti/drv/i2c/I2C.h>
#include <ti/drv/spi/SPI.h>

#include <ti/drv/uart/UART_stdio.h>

#include <ti/board/board.h>

#include <ti/csl/soc.h>
#include <ti/csl/csl_spinlock.h>
#include <soc_icss_header.h>

#include <ti/drv/gpio/GPIO.h>
#include <board_gpioLed.h>
#include <board_misc.h>

#include <board_spi.h>
#include <board_phy.h>
#include <board_dpphy.h>
#include <board_rotary_switch.h>

#include <delay_us.h>
#include <board_i2cLed.h>
#include <version.h>

#include <ti/board/src/am65xx_idk/am65xx_idk_pinmux.h>
#include <ti/board/src/am65xx_idk/include/pinmux.h>

#if !defined (__aarch64__)
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <ti/csl/soc/am65xx/src/cslr_intr_mcu0.h>
#include <ti/csl/soc/am65xx/src/cslr_soc_baseaddress.h>
#include <ti/board/src/am65xx_idk/include/board_cfg.h>
#include <interruptroute.h>
#endif

#if defined (__aarch64__)
/* A53 */
#define ARM_INTERRUPT_OFFSET_ICSS0 (286-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (294-20)
#define ARM_INTERRUPT_OFFSET_ICSS2 (302-20)
#endif

#define CSL_SEMAPHORE_REG_OFFSET(n)        (0x100 + ((n) * 0x04))

SPI_Handle handle;                   /* SPI handle */

extern PRUICSS_Handle pruIcss1Handle;
extern PRUICSS_Config pruss_config[2 + 1];

/* Board specific definitions */
#define MCSPI_INSTANCE         (0U)

void tiesc_mii_pinmuxConfig (void);


extern bool icssgResetIsolated;


int16_t icssInterruptOffset = 0;
uint32_t i2cInterruptOffset = 0;

uint8_t isEtherCATDevice(void)
{
    volatile uint32_t temp;
#if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
    temp = *((uint32_t *)(CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));
#elif (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
    temp = *((uint32_t *)(CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));
#else
    temp = *((uint32_t *)(CSL_PRU_ICSSG2_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));
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
#if defined (__aarch64__)
#if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
    return ARM_INTERRUPT_OFFSET_ICSS0;
#elif (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
    return ARM_INTERRUPT_OFFSET_ICSS1;
#else
    return ARM_INTERRUPT_OFFSET_ICSS2;
#endif
#else
    return (icssInterruptOffset - 20);
#endif

}

void initSpinlock()
{
    /* Setting up RAT config to map SOC Spinlock to C23 constant of PRUICSS */
    /* Mapping 0xC0000000 (C23 constant of PRUICSS) to 0x30E00800 (SPINLOCK) with size 2^11 = 0x800 */
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x24))) = (0xC0000000u); //rat0 base0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x28))) = (CSL_NAVSS0_SPINLOCK_BASE); //rat0 trans_low0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x2C))) = (0x00000000u); //rat0 trans_high0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_0_BASE + 0x20))) = (1u << 31) | (15u); //rat0 ctrl0

    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x24))) = (0xC0000000u); //rat0 base0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x28))) = (CSL_NAVSS0_SPINLOCK_BASE); //rat0 trans_low0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x2C))) = (0x00000000u); //rat0 trans_high0
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr) + CSL_ICSS_G_RAT_REGS_1_BASE + 0x20))) = (1u << 31) | (15u); //rat0 ctrl0

}

uint32_t getSpinlockLockReg0Offset()
{
    return (CSL_NAVSS0_SPINLOCK_BASE + CSL_SPINLOCK_LOCK_REG(0));
}

void bsp_soft_reset()
{   
	Sciclient_pmDeviceReset(0xFFFFFFFFU);
    return;
}


void bsp_soc_evm_init()
{

#if !defined (__aarch64__)
    I2C_HwAttrs   i2c_cfg;
#endif

    if(icssgResetIsolated==FALSE)
    {
		tiesc_mii_pinmuxConfig();

		Board_init(BOARD_INIT_ICSS_ETH_PHY);
    }

    GPIO_init();
    delay_us(100);

    //Making the clock for ICSSG core and IEP same
    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL))) |= 0x01;
    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL))) |= 0x01;
    (*((volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_ICSSG2_CLKSEL))) |= 0x01;

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

    CSL_MDIO_phyRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 1), DPPHY_CFG1_REG, 0x100);
    CSL_MDIO_phyRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 2), DPPHY_CFG1_REG, 0x100);
#else

    /* Setting up RX_ER/GPIO pin on the PHY as RX_ERR pin and COL/GPIO pin as LED_3 */
    MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 1), DPPHY_GPIO_MUX_CTRL2, 0x60);
    MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 2), DPPHY_GPIO_MUX_CTRL2, 0x60);

    /* Disable RGMII interface */
    MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 1), DPPHY_RGMIICTL, 0x50);
    MDIO_phyExtRegWrite((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), Board_getPhyAddress(PRUICSS_INSTANCE, 2), DPPHY_RGMIICTL, 0x50);

#endif


    PRUICSS_pinMuxConfig(pruIcss1Handle, 0x0);   // PRUSS pinmuxing
    //Disable PRUs - This is to ensure PRUs are not running when application is not initialized
    PRUICSS_pruDisable(pruIcss1Handle, 0);
    PRUICSS_pruDisable(pruIcss1Handle, 1);
    }

#if !defined (__aarch64__)

    /* Route Interrupts to R5F. */
	icssInterruptOffset = route_icss_interrupts_to_r5f(PRUICSS_INSTANCE);
	if(INTERRUPT_ROUTE_ERROR != icssInterruptOffset)
	{
	    icssInterruptOffset = (int16_t)icssInterruptOffset;
	}
	else
	{
		/*Wait here in an endless loop as there is an error. */
		while(1);
	}

	i2cInterruptOffset = route_i2c_interrupts_to_r5f();
	if(INTERRUPT_ROUTE_ERROR == i2cInterruptOffset)
	{
		/*Wait here in an endless loop as there is an error. */
		while(1);
	}

    /* LED's and Rotary Switch are connected to Main Domain I2C0. Reconfigure I2C to use I2C0 of main domain. */
    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(BOARD_I2C_IOEXP_INSTANCE, &i2c_cfg);

    i2c_cfg.baseAddr = CSL_I2C0_CFG_BASE;
    i2c_cfg.intNum = i2cInterruptOffset;

    I2C_socSetInitCfg(BOARD_I2C_IOEXP_INSTANCE, &i2c_cfg);
#endif

    /* I2C Init */
    Board_i2cLedInit();

    /* Rotary Switch Init */
    Board_initRotarySwitch();

    QSPI_init();

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

static pinmuxPerCfg_t gPru_icssg2_mii_g_fsi0PinCfg[] =
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

static pinmuxModuleCfg_t gPru_icssg2_mii_g_fsiPinCfg[] =
{
    {0, TRUE, gPru_icssg2_mii_g_fsi0PinCfg},
    {PINMUX_END}
};

pinmuxBoardCfg_t gAM65xxMIIPinmuxData[] =
{
    {0, gPru_icssg0_mii_g_rtPinCfg},
    {1, gPru_icssg1_mii_g_rtPinCfg},
    {2, gPru_icssg2_mii_g_fsiPinCfg},
    {PINMUX_END}
};

void tiesc_mii_pinmuxConfig (void)
{
    pinmuxModuleCfg_t* pModuleData = NULL;
    pinmuxPerCfg_t* pInstanceData = NULL;
    int32_t i, j, k;
    uint32_t rdRegVal;

    for(i = 0; PINMUX_END != gAM65xxMIIPinmuxData[i].moduleId; i++)
    {
        pModuleData = gAM65xxMIIPinmuxData[i].modulePinCfg;
        for(j = 0; (PINMUX_END != pModuleData[j].modInstNum); j++)
        {
            if(pModuleData[j].doPinConfig == TRUE)
            {
                pInstanceData = pModuleData[j].instPins;
                for(k = 0; (PINMUX_END != pInstanceData[k].pinOffset); k++)
                {
                    rdRegVal = HW_RD_REG32((MAIN_PMUX_CTRL + pInstanceData[k].pinOffset));
                    rdRegVal = (rdRegVal & PINMUX_BIT_MASK);
                    HW_WR_REG32((MAIN_PMUX_CTRL + pInstanceData[k].pinOffset),
                                (pInstanceData[k].pinSettings));
                }
            }
        }
    }

    return;
}

/* This is a dummy function in context of AM65x as there is no partner. */
void Send_BootComplete_Message_To_Partner()
{
	/* In AM65x, there is no partner. */
	return;
}
