/**
* tiesc_soc.c: Implements a mechanism to send message from R5F to M4F.
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

#include <ti/drv/uart/UART_stdio.h>

#include <ti/board/board.h>

#include <ti/csl/soc.h>
#include <ti/csl/csl_spinlock.h>
#include <soc_icss_header.h>

#include <ti/drv/gpio/GPIO.h>
#include <board_gpioLed.h>

#include <board_misc.h>
#include <board_eeprom.h>
#include <board_phy.h>
#include <board_dpphy.h>
#include <delay_us.h>
#include <board_spi.h>

#include <board_rotary_switch.h>
#include <board_i2cLed.h>

#include <version.h>

#include <ti/board/src/am64x_evm/include/board_pinmux.h>
#include <ti/board/src/am64x_evm/include/pinmux.h>
#include <ti/board/src/am64x_evm/AM64x_pinmux.h>

#include <ti/drv/i2c/soc/I2C_soc.h>
#include <ti/csl/soc/am64x/src/cslr_intr_r5fss0_core0.h>
#include <ti/csl/soc/am64x/src/cslr_soc_baseaddress.h>

#include <ti/board/src/am64x_evm/include/board_cfg.h>

#include <ti/drv/sciclient/sciclient.h>

#include <ti/csl/src/ip/rat/V0/csl_rat.h>

#define ARM_INTERRUPT_OFFSET_ICSS0 (120-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (248-20)

#define CSL_SEMAPHORE_REG_OFFSET(n)        (0x100 + ((n) * 0x04))

extern I2C_Handle i2c0Handle;
I2C_Handle eepromFlashHandle = NULL;

extern PRUICSS_Handle pruIcss1Handle;
extern PRUICSS_Config pruss_config[2 + 1];

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
    if(PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
        temp = *((uint32_t *)(CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));
    else if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
        temp = *((uint32_t *)(CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE + CSL_ICSSCFG_HWDIS));

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
    if(PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
        return ARM_INTERRUPT_OFFSET_ICSS0;
    else if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
        return ARM_INTERRUPT_OFFSET_ICSS1;
}

void initSpinlock()
{
    uintptr_t icssgBaseAddr = (((PRUICSS_HwAttrs *)((pruIcss1Handle)->hwAttrs))->baseAddr);
    CSL_ratRegs *pRatRegs;
    CSL_RatTranslationCfgInfo   TranslationCfg;

    /* Setting up RAT config to map SOC Spinlock to C23 constant of PRUICSS */
    /* Mapping 0xC0000000 (C23 constant of PRUICSS) to (SPINLOCK0) */

    TranslationCfg.baseAddress  = (0xC0000000u);
    TranslationCfg.sizeInBytes  = CSL_SPINLOCK0_SIZE;
    TranslationCfg.translatedAddress    = CSL_SPINLOCK0_BASE;

    pRatRegs = (CSL_ratRegs *)(icssgBaseAddr + CSL_ICSS_G_RAT_REGS_0_BASE);
    CSL_ratConfigRegionTranslation(pRatRegs, 0, &TranslationCfg);

    pRatRegs = (CSL_ratRegs *)(icssgBaseAddr + CSL_ICSS_G_RAT_REGS_1_BASE);
    CSL_ratConfigRegionTranslation(pRatRegs, 0, &TranslationCfg);

}

uint32_t getSpinlockLockReg0Offset()
{
    return (CSL_SPINLOCK0_BASE + CSL_SPINLOCK_LOCK_REG(0));
}

void bsp_soft_reset()
{
    //Device Reset not supported on AM64x
    return;
}

void tiesc_eeprom_init()
{
    int32_t ret;
    ret = Board_i2cEepromInit();
    if (BOARD_SOK != ret)
    {
        UART_printf("Board_i2cEepromInit returned error = %d\n", ret);
    }
    eepromFlashHandle = i2c0Handle;
}

int32_t tiesc_eeprom_read(uint32_t  offset,
                      uint8_t  *buf,
                      uint32_t  len)
{
    int32_t ret;

    ret = Board_i2cEepromRead(eepromFlashHandle, offset, buf, len, BOARD_I2C_EEPROM_ADDR);

    if (BOARD_SOK != ret)
    {
        UART_printf("Board_i2cEepromRead returned error = %d\n", ret);
    }

    return ret;
}

int32_t tiesc_eeprom_write(uint32_t  offset,
                       uint8_t  *buf,
                       uint32_t  len)
{
    int32_t ret;

    ret = Board_i2cEepromWrite(eepromFlashHandle, offset, buf, len, BOARD_I2C_EEPROM_ADDR);

    if (BOARD_SOK != ret)
    {
        UART_printf("Board_i2cEepromWrite returned error = %d\n", ret);
    }

    return ret;
}

void bsp_soc_evm_init()
{
    int8_t phy_addr1, phy_addr2;

    GPIO_init();

    if(icssgResetIsolated==FALSE)
    {
	/*This configuration is required only after poweron reset. Is not required after a warm reset.*/
        Board_init(BOARD_INIT_ICSS_ETH_PHY | BOARD_INIT_UNLOCK_MMR);
    }

#ifndef ECAT_RGMII
    Board_PinmuxConfig_t icssPinmux;

    Board_pinmuxGetCfg(&icssPinmux);
    icssPinmux.muxCfg = BOARD_PINMUX_CUSTOM;
    icssPinmux.icssMux = BOARD_PINMUX_ICSS_MII;
    Board_pinmuxSetCfg(&icssPinmux);
#endif
    Board_init(BOARD_INIT_PINMUX_CONFIG);

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
#else
		phy_addr1 = Board_getPhyAddress(PRUICSS_INSTANCE, 1);
		phy_addr2 = Board_getPhyAddress(PRUICSS_INSTANCE, 2);

		/* Select MII mode */
		Board_enablePhyMII((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), phy_addr1);
		Board_enablePhyMII((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), phy_addr2);

		/* Disable 1G advertisement */
		Board_phyAutoNeg1000AdvDisable((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), phy_addr1);
		Board_phyAutoNeg1000AdvDisable((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), phy_addr2);

		/* Soft-reset PHY */
		Board_phySoftRestart((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), phy_addr1);
		Board_phySoftRestart((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussMiiMdioRegBase), phy_addr2);

#endif

		PRUICSS_pinMuxConfig(pruIcss1Handle, 0x0);   // PRUSS pinmuxing
		//Disable PRUs - This is to ensure PRUs are not running when application is not initialized
		PRUICSS_pruDisable(pruIcss1Handle, 0);
		PRUICSS_pruDisable(pruIcss1Handle, 1);
    }

#ifndef TIESC_EMULATION_PLATFORM
    /* Rotary Switch Init */
    Board_initRotarySwitch();
#endif

    /* I2C LED Init */
    Board_i2cLedInit();

    /* Initialize flash handle for i2c eeprom */
    tiesc_eeprom_init();

    /* Initialize flash handle for OSPI memory*/
    OSPI_init();

    if(PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE)
        tiesc_setPLLClk(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, 200000000);
    else if (PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO)
        tiesc_setPLLClk(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK, 200000000);

    //Making the clock for ICSSG core and IEP same
    (*((volatile uint32_t *)((((PRUICSS_HwAttrs *)(pruIcss1Handle->hwAttrs))->prussCfgRegBase) + CSL_ICSSCFG_IEPCLK))) |= CSL_ICSSCFG_IEPCLK_OCP_EN_MASK;

}


void display_esc_version(uint16_t revision, uint16_t build)
{
#ifndef DISABLE_UART_PRINT
    UART_printf("\n\rRevision/Type : x%04X", revision);
    UART_printf(" Build : x%04X", build);
    UART_printf("\n\rFirmware Version : %d.%d.%d\n\r", (revision >> 8),
                (build >> 8), (build & 0xFF));
#endif
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

void tiesc_boardConfig(void)
{
    Board_IDInfo_v2 boardInfo;
    Board_STATUS status;

#ifndef DISABLE_UART_PRINT
    memset(&boardInfo, 0, sizeof(Board_IDInfo_v2));
    status = Board_getIDInfo_v2(&boardInfo, BOARD_I2C_EEPROM_ADDR);
    if(status != BOARD_SOK)
    {
        UART_printf("Board_getIDInfo_v2 returned error = %d",status);
    }

    UART_printf("\nBoard name \t: ");
    UART_printf(boardInfo.boardInfo.boardName);

    UART_printf("\nBoard Revision \t: ");
    UART_dataWrite((char *)&boardInfo.boardInfo.designRev, BOARD_DESIGN_REV_LEN);
#endif
}
