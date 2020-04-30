/**
 * \file board_dp83867.c
 * \brief Contains DP83867 PHY Specific Apis
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
#include <board_dpphy.h>
#include <soc_icss_header.h>

/**
* @brief Function does the  LED Configuration of DP83867 PHY
*
* @param mdioBaseAddress MDIO Base Address
* @param phyNum Phy address of the port
* @param led_num Led number
* @param mode Led Config mode

* @retval none
*/
void Board_phyLedConfig(uint32_t mdioBaseAddress, uint32_t phyNum,
                        uint8_t ledNum, uint8_t mode)
{
    uint16_t phyregval = 0;

    CSL_MDIO_phyRegRead(mdioBaseAddress, phyNum, DPPHY_LEDCR1_REG, &phyregval);

    switch(ledNum)
    {
        case DPPHY_LEDCR_LED0:
            phyregval &= ~(0xF << 0);
            phyregval |= (mode << 0);
            break;

        case DPPHY_LEDCR_LED1:
            phyregval &= ~(0xF << 4);
            phyregval |= (mode << 4);
            break;

        case DPPHY_LEDCR_LED2:
            phyregval &= ~(0xF << 8);
            phyregval |= (mode << 8);
            break;

        case DPPHY_LEDCR_LED3:
            phyregval &= ~(0xF << 12);
            phyregval |= (mode << 12);
            break;
    }

    CSL_MDIO_phyRegWrite(mdioBaseAddress, phyNum, DPPHY_LEDCR1_REG, phyregval);
}
