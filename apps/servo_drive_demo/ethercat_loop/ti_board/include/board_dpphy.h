/**
 * \file board_dpphy.h
 * \brief
 *
 *
*/

/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
#ifndef _BOARD_DPPHY_H_
#define _BOARD_DPPHY_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include  <ti/csl/src/ip/mdio/V5/csl_mdio.h>

#define DPPHY_BMCR_REG          0x00
#define DPPHY_BMSR_REG          0x01
#define DPPHY_ID1_REG           0x02
#define DPPHY_ANAR_REG          0x04
#define DPPHY_CFG1_REG          0x09
#define DPPHY_REGCR_REG         0x0D
#define DPPHY_ADDR_REG          0x0E
#define DPPHY_PHYCR_REG         0x10
#define DPPHY_PHYSTS_REG        0x11
#define DPPHY_LEDCR1_REG        0x18
#define DPPHY_LEDCR2_REG        0x19
#define DPPHY_LEDCR3_REG        0x1A
#define DPPHY_CFG3_REG          0x1E
#define DPPHY_GEN_CTRL          0x1F
#define DPPHY_FLD_CFG_REG       0x2D
#define DPPHY_RGMIICTL          0x32
#define DPPHY_100CR_REG         0x43
#define DPPHY_GPIO_MUX_CTRL2    0x172

/**Auto neg Enable bit in PHY_BMCR_REG*/
#define DPPHY_AUTO_NEGOTIATE_EN   (1<<12)

#define DPPHY_ANAR_100FD      (1u<<8)
#define DPPHY_ANAR_100HD      (1u<<7)
#define DPPHY_ANAR_10FD       (1u<<6)
#define DPPHY_ANAR_10HD       (1u<<5)

#define DPPHY_1000FD_ADV      (1u<<9)
#define DPPHY_1000HD_ADV      (1u<<8)

#define DPPHY_EXT_FD_ENABLE         (1u<<11)

#define DPPHY_ENHANCED_IPG_DET_ENABLE       (1u<<4)
#define DPPHY_ODDNIBBLE_DET_ENABLE           (1u<<1)
#define DPPHY_FASTRXDV_DET_ENABLE           (1u<<0)

#define DPPHY_SPEED_STATUS                     (3<<14)
#define DPPHY_DUPLEX_STATUS                    (1<<13)

#define DPPHY_SPEED_1000M   (0x2u<<14)
#define DPPHY_SPEED_100M    (0x1u<<14)
#define DPPHY_SPEED_10M     (0x0u<<14)


#define DPPHY_AUTOMDIX_ENABLE       (1u<<6)
#define DPPHY_FORCEMDIX_ENABLE      (1u<<5)

#define DPPHY_PSMODE_ENABLE   (1u<<14)

#define DPPHY_PSMODE_BIT1   (1u<<8)
#define DPPHY_PSMODE_BIT2   (1u<<9)

#define DPPHY_LEDCR_LED0      0x00
#define DPPHY_LEDCR_LED1      0x01
#define DPPHY_LEDCR_LED2      0x02
#define DPPHY_LEDCR_LED3      0x03

#define DPPHY_LEDCR_MODE0     0x00 // Link
#define DPPHY_LEDCR_MODE1     0x01 // Rx/Tx activity
#define DPPHY_LEDCR_MODE2     0x02 // Tx Activity
#define DPPHY_LEDCR_MODE3     0x03 // Rx Activity
#define DPPHY_LEDCR_MODE4     0x04 // Collision
#define DPPHY_LEDCR_MODE5     0x05 // 1000BT Link
#define DPPHY_LEDCR_MODE6     0x06 // 100BTX link
#define DPPHY_LEDCR_MODE7     0x07 // 10BT link
#define DPPHY_LEDCR_MODE8     0x08 // 10/100BT link
#define DPPHY_LEDCR_MODE9     0x09 // 100/1000BT link
#define DPPHY_LEDCR_MODE10    0x0A // Full duplex
#define DPPHY_LEDCR_MODE11    0x0B // Link and blink for tx/rx activity
#define DPPHY_LEDCR_MODE13    0x0D // Rx/Tx error
#define DPPHY_LEDCR_MODE14    0x0E // Rx Error

#define EXT_REG_ADDRESS_ACCESS      0x001F
#define EXT_REG_DATA_NORMAL_ACCESS  0x401F

#define FAST_LINKDOWN_SIGENERGY        1u
#define FAST_LINKDOWN_LOWSNR           (1u<<1)
#define FAST_LINKDOWN_MLT3ERR          (1u<<2)
#define FAST_LINKDOWN_RXERR            (1u<<3)

#define DPPHY_SPEED_MASK             (0x2040U)
#define DPPHY_AUTONEG_MASK           (0x1000U)
#define DPPHY_SPEED_100MPBS          (0x2000U)
#define DPPHY_SPEED_1000MPBS         (0x0040U)


/*PHY LED BLINK Rates*/
#define LED_BLINK_500     500
#define LED_BLINK_200     200
#define LED_BLINK_100     100
#define LED_BLINK_50      50

#define  PHY_CONFIG_AUTONEG         0u
#define  PHY_CONFIG_100FD           1u
#define  PHY_CONFIG_10FD            2u
#define  PHY_CONFIG_100HD           3u
#define  PHY_CONFIG_10HD            4u
#define  PHY_CONFIG_1000FD          5u
#define  PHY_CONFIG_1000HD          6u
#define  PHY_CONFIG_INVALID         7u

#define PHY_ENABLE_AUTO_MDIX  0u
#define PHY_ENABLE_FORCE_MDI  1u
#define PHY_ENABLE_FORCE_MDIX  2u
#define PHY_POWERMODE_NORMAL 0u
#define PHY_POWERMODE_DOWN 1u
#define PHY_POWERMODE_ACTIVE_SLEEP 2u
#define PHY_POWERMODE_PASSIVE_SLEEP 3u

#define DPPHY_SOFT_RESET    (1u<<14)

#define PHY_AUTO_NEGOTIATE_EN   (1<<12)
#define PHY_SPEEDSEL_100        (1<<13)
#define PHY_SPEEDSEL_100_MSB    (1<<6)
#define PHY_DUPLEXMODE_FULL     (1<<8)

#ifdef INCLUDE_DPPHY_WORKAROUND

#define MDIX_FIX_TASKSLEEP_TICK (2500)
#define MDIO_MAX_PHYNUM         (32)

/**Structure to pass to the MDIX workaround task*/
typedef struct dpphyMDIXTaskParam
{
    uint32_t mdioBaseAddress;
    uint8_t numPorts;
    uint8_t phyAddress[MDIO_MAX_PHYNUM];
} dpphyMDIXTaskParam_t;

/**
* @brief Initializes the workarounds for DPPHY issues. This API should be called to make the DP PHY
*           work in Forced Mode.
*
*       This API implements the DPPHY specific workarounds.
*       1) AutoMDIX workaround
*        A task is created which will constantly check whether the PHY is in forced mode, In this case
*        the AutoMDIX is disabled and Software MDI/X is done.
*
*        MDIO init shall be done before using this function
*
* @param mdioBaseAddress    [IN] MDIO Base Address
*
*  @retval none
*/
void Board_phyMDIXFixInit(dpphyMDIXTaskParam_t *params);
/**
* @brief Shutdown the DPPHY issue workaround
*
*       Deletes the DPPHY issue workaround task
*
*
*  @retval none
*/
void Board_phyMDIXFixDeInit();

#endif

/** @addtogroup PHY_VENDOR_SPECIFIC_FUNCTIONS
 @{ */

/**
* @brief Function to disable AutoMDIX
*
*       MDIO init shall be done before using this function.The Sem handle need to be passed if theThread safe MDIO read is used,
*       Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
*  @retval none
*/
extern void Board_disablePhyAutoMDIX(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Enable AutoMDIX for the particular Port
*
*       MDIO init shall be done before using this function.The Sem handle need to be passed if theThread safe MDIO read is used,
*       Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
*  @retval none
*/

extern void Board_enablePhyAutoMDIX(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Function to get the MDIX status of a particular PHY
*
*       MDIO init shall be done before using this function.The Sem handle need to be passed if theThread safe MDIO read is used,
*       Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
*  @retval PHY_ENABLE_AUTO_MDIX if PHY is in Auto MDIX
*          PHY_ENABLE_FORCE_MDIX if PHY is in Forced MDIX
*          PHY_ENABLE_FORCE_MDI if PHY is in Forced MDI
*/
extern uint8_t Board_getPhyMDIXStat(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Function to Configure MDI/X Mode of PHY
*
*       API to enable MDI/MDIX or AutoMDIX mode
*       MDIO init shall be done before using this function.The Sem handle need to be passed if theThread safe MDIO read is used,
*       Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
* @param mdiState           [IN] MDI/MDIX mode to be set
*
* @retval none
*/
extern void Board_setPhyMDIX(uint32_t mdioBaseAddress, uint32_t phyNum,
                      uint8_t mdiState);
/**
* @brief Function to Enable Power Saving modes of the PHY
*
*       This function should be called in advance to use the Power save mode of the PHY.
*       MDIO init shall be done before using this function.The Sem handle need to be passed if theThread safe MDIO read is used,
*       Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
* @retval none
*/
extern void Board_phyEnablePowerSaveMode(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Function to force Power Saving mode in PHY
*
*       Following modes are supported
*       PHY_POWERMODE_NORMAL        Normal opearion with PHY fully functional
*       PHY_POWERMODE_DOWN          Shuts down all internal circuitry except SMI functionality
*       PHY_POWERMODE_ACTIVE_SLEEP  Shuts down all internal circuitry except SMI and energy detect functionalities.
*                                   In this mode the PHY sends NLP every 1.4 Sec to wake up link-partner.
*                                   Automatic power-up is done when link partner is detected
*       PHY_POWERMODE_PASSIVE_SLEEP Shuts down all internal circuitry except SMI and energy detect functionalities.
*                                   Automatic power-up is done when link partner is detected
*
*       Power Save mode should be enabled to use the Power Saving feaures. MDIO init shall be done before using this function
*       The Sem handle need to be passed if theThread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
* @param phyPowerMode       [IN] Mode to be set
*
* @retval none
*/
extern void Board_setPhyPowerSaveMode(uint32_t mdioBaseAddress, uint32_t phyNum,
                               uint8_t phyPowerMode);



/**
* @brief Function to enable Fast RXDV Detection
*
*       MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
* @retval none
*/
extern void Board_phyFastRXDVDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Function to enable Fast Link Down Detection
*
*       MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
* @param val                [IN] Phy address of the port
*
* @retval none
*/
extern void Board_phyFastLinkDownDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum,
                                    uint8_t val);
/**
* @brief Function to enable Extended Full Duplex ability
*
*       MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
* @retval none
*/
extern void Board_phyExtFDEnable(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Function to enable Enhanced LED Link Functionality
*
*       MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
* @retval none
*/
extern void Board_phyEnhLEDLinkEnable(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @brief Function to enable ODD Nibble detection
*
*       MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
* @retval none
*/
extern void Board_phyODDNibbleDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum);

/**
* @brief Function does the  LED Configuration of PHY
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port
* @param ledNum led number to be configured
* @param mode Led Config mode
*
* @retval none
*/
extern void Board_phyLedConfig(uint32_t mdioBaseAddress, uint32_t phyNum,
                        uint8_t ledNum, uint8_t mode);

/**
* @brief Function to Configure EthernetLED Blink rate
*
*       Function can be used to configure the PHY LED Blink rate.Valid values incase of TLK PHYs are
*       500ms,200ms,100ms and 50 ms. MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
* @param val                [IN] Supported rate mode
*
* @retval none
*/
extern void Board_phyLedBlinkConfig(uint32_t mdioBaseAddress, uint32_t phyNum,
                             uint16_t val);

/**
* @brief Function to Configure Ethernet LED polarities
*
*       Function can be used to configure polarity of SPEED(Bit2), LINK (Bit1) and ACT (Bit0) LEDs
*       See tlk datasheet for more details MDIO init shall be done before using this function,
*       the Sem handle need to be passed if the Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
* @param mask               [IN] Speed | Link | Activity. Set mask bit for Active HIGH
*
* @retval none
*/

extern void Board_phyLedSpeedLinkActPolarity(uint32_t mdioBaseAddress, uint32_t phyNum,
                                      uint16_t mask);
/**
* @brief Function to get the PHY Speed and duplexity
*
*       This API returns the Speed and Duplexity Configuration of the PHY
*       MDIO init shall be done before using this function, The Sem handle need to be passed if the
*       Thread safe MDIO read is used, Pass NULL otherwise
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyNum             [IN] Phy address of the port
*
*  @retval PHY_CONFIG_10FD 10 Mbps and Full duplex
*          PHY_CONFIG_10HD 10 Mbps and Half duplex
*          PHY_CONFIG_100FD 100 Mbps and Full duplex
*          PHY_CONFIG_100HD 100 Mbps and Half duplex
*/

extern uint8_t Board_getPhyConfig(uint32_t mdioBaseAddress, uint32_t phyNum);

/**
@}
*/

/**
* @brief Function to get the PHY ready status
*
**      Function returns whether the PHY is identified and ready
*
*       MDIO init shall be done before using this function
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyAddr            [IN] Phy address of the port
*
*  @retval TRUE if Phy is ready
*          FALSE if Phy not ready
*/
extern uint8_t Board_getPhyIdentifyStat(uint32_t mdioBaseAddress, uint32_t phyAddr);
/**
* @brief Function returns AutoNeg status of the PHY
*
**      Function returns whether the PHY is identified and ready
*
*       MDIO init shall be done before using this function,
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyAddr            [IN] Phy address of the port
*
*  @retval TRUE if Phy is ready
*          FALSE if Phy not ready
*/
extern uint8_t Board_getAutoNegStat(uint32_t mdioBaseAddress, uint32_t phyAddr);

/**
* @brief Function to set ANAR register of the PHY to support 100Base-TX Full-Duplex
*
*       MDIO init shall be done before using this function,
*
* @param mdioBaseAddress    [IN] MDIO Base Address
* @param phyAddr            [IN] Phy address of the port
*
*  @retval none
*/
extern void Board_phyAutoNegAdvConfigFD(uint32_t mdioBaseAddress, uint32_t phyAddr);

extern void Board_phyAutoNeg1000AdvDisable(uint32_t mdioBaseAddress, uint32_t phyAddr);

extern void Board_setEthPhySpeed(uint32_t mdioBaseAddress, uint32_t phyNum,
                          uint16_t speed);

extern void Board_setAdv100M_FD(uint32_t mdioBaseAddress, uint32_t phyNum);

/**
* @brief Function to configure PHY in MII mode
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port

* @retval none
*/
extern void Board_enablePhyMII(uint32_t mdioBaseAddress, uint32_t phyNum);

extern void MDIO_phyExtRegRead(uint32_t mdioBaseAddress, uint32_t phyNum,
                        uint32_t regNum, uint16_t *phyregval);

extern void MDIO_phyExtRegWrite(uint32_t mdioBaseAddress, uint32_t phyNum,
                         uint32_t regNum, uint16_t phyregval);

/**
* @brief Function to perform a soft restart of PHY
*
*        Restarts the PHY without affecting registers
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port

* @retval none
*/
extern void Board_phySoftRestart(uint32_t mdioBaseAddress, uint32_t phyNum);

/**
* @brief API to Change Phy Configuration
*
* @param mdioBaseAddress MDIO Base Address
* @param phyMode    PHY_CONFIG_AUTONEG
*                   PHY_CONFIG_100FD
*                   PHY_CONFIG_10FD
*                   PHY_CONFIG_100HD
* @param phyNum Phy address of the port
* @param mdioSemhandle Semaphore handle if thread safe MDIO access is used
*
* @retval none
*/
void MDIO_setPhyConfig(uint32_t mdioBaseAddress, uint32_t phyNum,
                       uint8_t phyMode);

/**
* @brief Function to get the PHY Speed and duplexity
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port
*
*  @retval PHY_CONFIG_10FD 10 Mbps and Full duplex
*          PHY_CONFIG_10HD 10 Mbps and Half duplex
*          PHY_CONFIG_100FD 100 Mbps and Full duplex
*          PHY_CONFIG_100HD 100 Mbps and Half duplex
*/
uint8_t MDIO_getPhyConfig(uint32_t mdioBaseAddress, uint32_t phyNum);

/**
* @brief Function to disable Fast Link Drop Detection
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port

* @retval none
*/
extern void Board_phyFastLinkDownDetDisable(uint32_t mdioBaseAddress, uint32_t phyNum);

/**
* @brief Function to enable Enhanced Interpacket Gap Detection
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port

* @retval none
*/
extern void Board_phyEnhancedIPGDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum);
#endif
