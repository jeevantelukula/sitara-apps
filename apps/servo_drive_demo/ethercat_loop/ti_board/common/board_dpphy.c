/**
 * \file board_dpphy.c
 * \brief Contains ICSS PHY (DP) Specific APIs
 *
*/
/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdlib.h>
#include <string.h>

#include <board_dpphy.h>
#include <soc_icss_header.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#ifdef INCLUDE_DPPHY_WORKAROUND
#include <TaskP.h>

/**Task Handle for the DPPHY thread which swaps MDI/MDI-X bits for link*/
void *dpphyTask;

/**
* @internal
* @brief API to Swap the MDIO/MDIX setting if Link is down and Phy is in forced mode
*
*       This API is used by DPPHY Task to do Software MDI/X. This Function checks for Link status.Incase down,it checks for
*       Autonegotiation status. If the PHY is in forced mode, MDI/X configuration of the PHY is swapped until
*       link is detected
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum PHY Address of the Port
*
* @retval none
*/
void Board_dpphyMDIXSwap(uint32_t mdioBaseAddress, uint32_t phyNum);
/**
* @internal
* @brief Main Task which implements MDI/MDI-X for the PHY in Software
*
*       This Task Does the software MDI/X for the DP PHY. Once created the task  will constantly
*       Check for Link UP. Incase Link is down, the taks will check whether the PHY is in forced mode.
*       If in Forced mode, the AutoMDIX is disabled and  Software MDI/X is done. It will continuosuly swap
*       MDI/MDIX bit until the Link is established.
*
* @param a0     [IN] Argument passed (MDIO Base Address)
* @param a1     [IN] Argument passed (None)
*
*  @retval none
*/
void Board_dpphyMDIXTask(UArg a0, UArg a1);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
* @internal
* @brief API to Swap the MDIO/MDIX setting if Link is down and Phy is in forced mode
*
*       This API is used by DP Task to do Software MDI/X. This Function checks for Link status.Incase down,it checks for
*       Autonegotiation status. If the PHY is in forced mode, MDI/X configuration of the PHY is swapped until
*       link is detected
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum PHY Address of the Port
*
* @retval none
*/
void Board_dpphyMDIXSwap(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    if(!CSL_MDIO_phyLinkStatus(mdioBaseAddress, phyNum))
    {
        if(!Board_getAutoNegStat(mdioBaseAddress, phyNum))
        {
            if(PHY_ENABLE_FORCE_MDIX == Board_getPhyMDIXStat(mdioBaseAddress, phyNum))
            {
                Board_setPhyMDIX(mdioBaseAddress, phyNum, PHY_ENABLE_FORCE_MDI);
            }

            else if(PHY_ENABLE_FORCE_MDI == Board_getPhyMDIXStat(mdioBaseAddress, phyNum))
            {
                Board_setPhyMDIX(mdioBaseAddress, phyNum, PHY_ENABLE_FORCE_MDIX);
            }

            else
            {
                Board_setPhyMDIX(mdioBaseAddress, phyNum, PHY_ENABLE_FORCE_MDI);
            }
        }
    }
}

/**
* @internal
* @brief Main Task which implements MDI/MDI-X for the DP PHY in Software
*
*       This Task Does the software MDI/X for the DP PHY. Once created the task  will constantly
*       Check for LInk UP. In case link is down, the task will check whether the PHY is in forced mode.
*       In this case the AutoMDIX is disabled and  Software MDI/X is done. It will continuously swap
*       MDI/MDIX bit until the Link is established.
*
*       The NLP Fix is also enabled in the task
*
* @param a0     [IN] Argument passed (MDIO Base Address)
* @param a1     [IN] Argument passed (None)
*
*  @retval none
*/
void Board_dpphyMDIXTask(UArg a0, UArg a1)
{
    uint32_t i = 0;
    dpphyMDIXTaskParam_t params;
    memcpy((void *)&params, (void *)a0, sizeof(dpphyMDIXTaskParam_t));

    TaskP_sleep(MDIX_FIX_TASKSLEEP_TICK);

    while(1)
    {
        for(i = 0; i < (params.numPorts); i++)
        {
            Board_dpphyMDIXSwap(params.mdioBaseAddress, (uint32_t)(params.phyAddress[i]));
            TaskP_sleep(MDIX_FIX_TASKSLEEP_TICK);
        }
    }
}

/**
* @brief Initializes the workarounds for DP PHY issues. This API should be called to make the DP PHY
*           work in Forced Mode.
*
*       This API implements the DP PHY specific workarounds.
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
void Board_phyMDIXFixInit(dpphyMDIXTaskParam_t *params)
{
    TaskP_Params taskParams;

    uint8_t taskname[] = "dpphyMDIXTask";
    TaskP_Params_init(&taskParams);
    taskParams.priority = 3;
    taskParams.stacksize = 0x1000;
    taskParams.name = (uint8_t *)taskname;
    taskParams.arg0 = (void *)params;

    dpphyTask = TaskP_create(Board_dpphyMDIXTask, &taskParams);
}
/**
* @brief Shut down the DP PHY issue workaround
*
*       Deletes the DP PHY issue workaround task
*
*
*  @retval none
*/
void Board_phyMDIXFixDeInit()
{
    TaskP_delete((TaskP_Handle *)&dpphyTask);
}

#endif
void MDIO_phyExtRegRead(uint32_t mdioBaseAddress, uint32_t phyNum,
                        uint32_t regNum, uint16_t *phyregval)
{
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_REGCR_REG,
                         EXT_REG_ADDRESS_ACCESS);
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_ADDR_REG, regNum);
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_REGCR_REG,
                         EXT_REG_DATA_NORMAL_ACCESS);
    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_ADDR_REG, phyregval);
    return;
}

void MDIO_phyExtRegWrite(uint32_t mdioBaseAddress, uint32_t phyNum,
                         uint32_t regNum, uint16_t phyregval)
{
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_REGCR_REG,
                         EXT_REG_ADDRESS_ACCESS);
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_ADDR_REG, regNum);
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_REGCR_REG,
                         EXT_REG_DATA_NORMAL_ACCESS);
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_ADDR_REG, phyregval);
    return;
}

void Board_disablePhyAutoMDIX(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t phyregVal = 0;
    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, &phyregVal);
    phyregVal &= ~(DPPHY_AUTOMDIX_ENABLE);
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, phyregVal);
}

void Board_enablePhyAutoMDIX(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t phyregval = 0;
    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, &phyregval);
    phyregval |= DPPHY_AUTOMDIX_ENABLE;
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, phyregval);
}

uint8_t Board_getPhyMDIXStat(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t phyregval = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, &phyregval);

    if(phyregval & DPPHY_AUTOMDIX_ENABLE)
    {
        return PHY_ENABLE_AUTO_MDIX;
    }

    else if(phyregval & DPPHY_FORCEMDIX_ENABLE)
    {
        return PHY_ENABLE_FORCE_MDIX;
    }

    else
    {
        return PHY_ENABLE_FORCE_MDI;
    }
}

void Board_setPhyMDIX(uint32_t mdioBaseAddress, uint32_t phyNum,
                      uint8_t mdiState)
{
    uint16_t phyregval = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, &phyregval);

    switch(mdiState)
    {
        case PHY_ENABLE_FORCE_MDI:
            phyregval &= ~(DPPHY_AUTOMDIX_ENABLE);
            phyregval &= ~(DPPHY_FORCEMDIX_ENABLE);
            break;

        case PHY_ENABLE_FORCE_MDIX:
            phyregval &= ~(DPPHY_AUTOMDIX_ENABLE);
            phyregval |= DPPHY_FORCEMDIX_ENABLE;
            break;

        case PHY_ENABLE_AUTO_MDIX:
        default:
            phyregval |= DPPHY_AUTOMDIX_ENABLE;
            break;
    }

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, phyregval);
}

void Board_phyEnablePowerSaveMode(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    //only setting the Power Save mode is required.
    //Enable not required.
}

void Board_setPhyPowerSaveMode(uint32_t mdioBaseAddress, uint32_t phyNum,
                               uint8_t phyPowerMode)
{
    uint16_t phyregval = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, &phyregval);

    switch(phyPowerMode)
    {
        case PHY_POWERMODE_NORMAL:
            phyregval &= (~(DPPHY_PSMODE_BIT1) & ~(DPPHY_PSMODE_BIT2));
            break;

        case PHY_POWERMODE_DOWN:
            phyregval |= DPPHY_PSMODE_BIT1;
            phyregval &= ~(DPPHY_PSMODE_BIT2);
            break;

        case PHY_POWERMODE_ACTIVE_SLEEP:
            phyregval |= DPPHY_PSMODE_BIT2;
            phyregval &= ~(DPPHY_PSMODE_BIT1);
            break;

        case PHY_POWERMODE_PASSIVE_SLEEP:
            phyregval |= (DPPHY_PSMODE_BIT1 | DPPHY_PSMODE_BIT2);

        default:
            phyregval &= (~(DPPHY_PSMODE_BIT1) & ~(DPPHY_PSMODE_BIT2));
            break;
    }

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_PHYCR_REG, phyregval);
}

void Board_phyLedBlinkConfig(uint32_t mdioBaseAddress, uint32_t phyNum,
                             uint16_t val)
{
    uint16_t phyregval = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_LEDCR3_REG, &phyregval);

    switch(val)
    {
        case LED_BLINK_500:
            phyregval &= 0xFFFC;
            phyregval |= 0x0003;
            break;

        case LED_BLINK_200:
            phyregval &= 0xFFFC;
            phyregval |= 0x0002;
            break;

        case LED_BLINK_100:
            phyregval &= 0xFFFC;
            phyregval |= 0x0001;
            break;

        case LED_BLINK_50:
            phyregval &= 0xFFFC;
            break;
    }

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_LEDCR3_REG, phyregval);
}

void Board_phyLedSpeedLinkActPolarity(uint32_t mdioBaseAddress, uint32_t phyNum,
                                      uint16_t mask)
{
    //TODO : implement this

    //    uint16_t phyregval = 0;
    //    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, TLKPHY_LEDCR_REG, &phyregval);
    //    phyregval |= (mask << 6);
    //    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, TLKPHY_LEDCR_REG, phyregval);
}

void Board_phyFastRXDVDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t phyregval = 0;
    MDIO_phyExtRegRead(mdioBaseAddress, phyNum, DPPHY_100CR_REG, &phyregval);
    phyregval |= (DPPHY_FASTRXDV_DET_ENABLE);
    MDIO_phyExtRegWrite(mdioBaseAddress, phyNum, DPPHY_100CR_REG, phyregval);
}

void Board_phyFastLinkDownDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum,
                                    uint8_t val)
{
    uint16_t phyregval = 0;
    MDIO_phyExtRegRead(mdioBaseAddress, phyNum, DPPHY_FLD_CFG_REG, &phyregval);
    phyregval &= ~(0x1F);
    phyregval |= (val & 0x1F);
    MDIO_phyExtRegWrite(mdioBaseAddress, phyNum, DPPHY_FLD_CFG_REG, phyregval);
}

void Board_phyExtFDEnable(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t phyregval = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_CFG3_REG, &phyregval);
    phyregval |= DPPHY_EXT_FD_ENABLE;
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_CFG3_REG, phyregval);
}


void Board_phyEnhLEDLinkEnable(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    // Seperate Link pin available in the PHY
    // This setting doesnt exist and not required

    //    uint16_t phyregval = 0;
    //    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, TLKPHY_CR2_REG, &phyregval);
    //    phyregval |= ENH_LEDLINK_ENABLE;
    //    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, TLKPHY_CR2_REG, phyregval);
}

void Board_phyODDNibbleDetEnable(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t phyregval = 0;
    MDIO_phyExtRegRead(mdioBaseAddress, phyNum, DPPHY_100CR_REG, &phyregval);
    phyregval |= (DPPHY_ODDNIBBLE_DET_ENABLE);
    MDIO_phyExtRegWrite(mdioBaseAddress, phyNum, DPPHY_100CR_REG, phyregval);
}

void Board_phyRxErrIdleEnable(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    //TODO: Implement this function

    //    uint16_t phyregval = 0;
    //    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, TLKPHY_CR2_REG, &phyregval);
    //    phyregval |= RXERROR_IDLE_ENABLE;
    //    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, TLKPHY_CR2_REG, phyregval);
}

uint8_t Board_getPhyConfig(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t regStatus = 0;
    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_PHYSTS_REG, &regStatus);

    if((regStatus & DPPHY_SPEED_STATUS) == DPPHY_SPEED_10M)    /*Speed is 10*/
    {
        if(regStatus & DPPHY_DUPLEX_STATUS)
        {
            return PHY_CONFIG_10FD;
        }

        else
        {
            return PHY_CONFIG_10HD;
        }
    }

    else if((regStatus & DPPHY_SPEED_STATUS) ==
            DPPHY_SPEED_100M)    /*Speed is 100*/
    {
        if(regStatus & DPPHY_DUPLEX_STATUS)
        {
            return PHY_CONFIG_100FD;
        }

        else
        {
            return PHY_CONFIG_100HD;
        }
    }

    else    /*Speed is 1000*/
    {
        if(regStatus & DPPHY_DUPLEX_STATUS)
        {
            return PHY_CONFIG_100FD;
        }

        else
        {
            return PHY_CONFIG_100HD;
        }
    }
}

uint8_t Board_getAutoNegStat(uint32_t mdioBaseAddress, uint32_t phyAddr)
{
    uint16_t regStatus = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyAddr, DPPHY_BMCR_REG, &regStatus);

    if(regStatus & DPPHY_AUTO_NEGOTIATE_EN)
    {
        return TRUE;
    }

    else
    {
        return FALSE;
    }
}

uint8_t Board_getPhyIdentifyStat(uint32_t mdioBaseAddress, uint32_t phyAddr)
{
    uint16_t regStatus = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyAddr, DPPHY_ID1_REG, &regStatus);

    if(regStatus == 0x2000)
    {
        return TRUE;
    }

    else
    {
        return FALSE;
    }
}

void Board_phyAutoNegAdvConfigFD(uint32_t mdioBaseAddress, uint32_t phyAddr)
{
    uint16_t regStatus = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyAddr, DPPHY_ANAR_REG, &regStatus);

    regStatus |= DPPHY_ANAR_100FD;
    regStatus |= DPPHY_ANAR_100HD;
    regStatus |= DPPHY_ANAR_10FD;
    regStatus |= DPPHY_ANAR_10HD;

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyAddr, DPPHY_ANAR_REG, regStatus);
}

void Board_phyAutoNeg1000AdvDisable(uint32_t mdioBaseAddress, uint32_t phyAddr)
{
    uint16_t regStatus = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyAddr, DPPHY_CFG1_REG, &regStatus);

    regStatus &= ~DPPHY_1000FD_ADV;
    regStatus &= ~DPPHY_1000HD_ADV;

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyAddr, DPPHY_CFG1_REG, regStatus);
}

void Board_setEthPhySpeed(uint32_t mdioBaseAddress, uint32_t phyNum,
                          uint16_t speed)
{
    uint16_t regData = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_BMCR_REG, &regData);

    /* Reset the speed bits and disable the auto negotiation */
    regData &= ~(DPPHY_SPEED_MASK | DPPHY_AUTONEG_MASK);

    regData |= speed;

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_BMCR_REG, regData);
    return;
}

void Board_setAdv100M_FD(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t regData = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_ANAR_REG, &regData);

    /* Only enable 100M FD advertisement */
    regData &= ~(0xE0);
    regData |= 0x100;

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_ANAR_REG, regData);


    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_BMCR_REG, &regData);

    /* Enable and restart Auto negotiation */
    regData |= 0x1200;

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_BMCR_REG, regData);

    return;
}

void Board_phySoftRestart(uint32_t mdioBaseAddress, uint32_t phyNum)
{
    uint16_t regData = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_GEN_CTRL, &regData);
    regData |= DPPHY_SOFT_RESET;
    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_GEN_CTRL, regData);
}
