/**
 * tlk105_tiescbsp.c
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


#include <tiescbsp.h>
#include <board_dpphy.h>
#include <board_spi.h>

#include <soc_icss_header.h>

void bsp_ethphy_init(PRUICSS_Handle pruIcssHandle, uint8_t phy0addr,
                     uint8_t phy1addr, uint8_t enhancedlink_enable)
{
    if(TIESC_MDIO_RX_LINK_ENABLE == enhancedlink_enable)
    {
        Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr, DPPHY_LEDCR_LED0, DPPHY_LEDCR_MODE0);
        Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr, DPPHY_LEDCR_LED0, DPPHY_LEDCR_MODE0);
    }

    while(!Board_getPhyIdentifyStat((((PRUICSS_HwAttrs *)(
                                          pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr))
    {
    }

    while(!Board_getPhyIdentifyStat((((PRUICSS_HwAttrs *)(
                                          pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr))
    {
    }

    //SWSCR2
    //Bit2: Enable RXERR during IDLE detection
    //Bit1: Disable detection of transmit error in odd-nibble boundary for odd nibble insertion)
    //Bit5: Force Full-Duplex while working with link partner in forced 100B-TX. When the
    //PHY is set to Auto-Negotiation or Force 100B-TX and the link partner is operated
    //in Force 100B-TX, the link is always Full Duplex
    //For EtherCAT : Disable enhanced LED link function

    Board_phyExtFDEnable((((PRUICSS_HwAttrs *)(
                               pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr);
    Board_phyExtFDEnable((((PRUICSS_HwAttrs *)(
                               pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr);
    Board_phyODDNibbleDetEnable((((PRUICSS_HwAttrs *)(
                                      pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr);
    Board_phyODDNibbleDetEnable((((PRUICSS_HwAttrs *)(
                                      pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr);
    Board_phyRxErrIdleEnable((((PRUICSS_HwAttrs *)(
                                   pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr);
    Board_phyRxErrIdleEnable((((PRUICSS_HwAttrs *)(
                                   pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr);

    //Enable enhanced link detection in TLK110 for EtherCAT
    //Bit3: Drop the link based on RX Error count of the MII interface, when a predefined number
    // of 32 RX Error occurrences in a 10us interval is reached, the link will be dropped
    // Bit0: Drop the link based on Signal/Energy loss indication, when the Energy detector
    //indicates Energy Loss, the link will be dropped. Typical reaction time is 10us.

    /* PHY pin LED_0 as link for fast link detection */
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr, DPPHY_LEDCR_LED0, DPPHY_LEDCR_MODE0);
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr, DPPHY_LEDCR_LED0, DPPHY_LEDCR_MODE0);

    /* PHY pin LED_1 as 1G link established */
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr, DPPHY_LEDCR_LED1, DPPHY_LEDCR_MODE5);
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr, DPPHY_LEDCR_LED1, DPPHY_LEDCR_MODE5);

    /* PHY pin LED_2 as Rx/Tx Activity */
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr, DPPHY_LEDCR_LED2, DPPHY_LEDCR_MODE11);
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr, DPPHY_LEDCR_LED2, DPPHY_LEDCR_MODE11);

    /* PHY pin LED_3 as 100M link established */
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr, DPPHY_LEDCR_LED3, DPPHY_LEDCR_MODE8);
    Board_phyLedConfig((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr, DPPHY_LEDCR_LED3, DPPHY_LEDCR_MODE8);


    Board_phyLedBlinkConfig((((PRUICSS_HwAttrs *)(
                                  pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr, LED_BLINK_200);
    Board_phyLedBlinkConfig((((PRUICSS_HwAttrs *)(
                                  pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr, LED_BLINK_200);


    Board_phyFastLinkDownDetEnable((((PRUICSS_HwAttrs *)(
                                         pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy0addr,
                                   FAST_LINKDOWN_SIGENERGY | FAST_LINKDOWN_RXERR);
    Board_phyFastLinkDownDetEnable((((PRUICSS_HwAttrs *)(
                                         pruIcssHandle->hwAttrs))->prussMiiMdioRegBase), phy1addr,
                                   FAST_LINKDOWN_SIGENERGY | FAST_LINKDOWN_RXERR);
}
