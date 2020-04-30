/**
 *  \file   board_phy.c
 *
 *  \brief  AM65xx IDK Board specific phy parameters.
 *
 *   This file contains the phy hardware parameters specific to board.
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
#include "stdio.h"
#include "string.h"
#include <stdlib.h>

#include <ti/csl/soc.h>
#include <ti/csl/src/ip/gpio/V0/gpio.h>
#include <soc_icss_header.h>

#include <ti/board/board.h>
#include <board_phy.h>
#include <board_misc.h>
#include <board_gpioLed.h>

#define AM65xx_CPSW_PORT1_PHY_ADDR 0
#define AM65xx_CPSW_PORT2_PHY_ADDR 3

#ifdef AM65XX_ALPHA_BOARD
#define AM65xx_ICSS1_PORT1_PHY_ADDR 3
#define AM65xx_ICSS1_PORT2_PHY_ADDR 0
#else
#define AM65xx_ICSS1_PORT1_PHY_ADDR 0
#define AM65xx_ICSS1_PORT2_PHY_ADDR 3
#endif

#define AM65xx_ICSS2_PORT1_PHY_ADDR 0
#define AM65xx_ICSS2_PORT2_PHY_ADDR 3

#define AM65xx_ICSS3_PORT1_PHY_ADDR 0
#define AM65xx_ICSS3_PORT2_PHY_ADDR 3

#define MAX_ICSS_EMAC_PORTS     6


int8_t Board_getPhyAddress(uint8_t instance, uint8_t portNumber)
{
    if(CPSW_PHY_ADDRESS == instance)
    {
        if(1u == portNumber)
        {
            return AM65xx_CPSW_PORT1_PHY_ADDR;
        }

        else if(2u == portNumber)
        {
            return AM65xx_CPSW_PORT2_PHY_ADDR;
        }

        else
        {
            return -1;
        }
    }

    else if(PRUICSS1_PHY_ADDRESS == instance)
    {
        if(1u == portNumber)
        {
            return AM65xx_ICSS1_PORT1_PHY_ADDR;
        }

        else if(2u == portNumber)
        {
            return AM65xx_ICSS1_PORT2_PHY_ADDR;
        }

        else
        {
            return -1;
        }
    }

    else if(PRUICSS2_PHY_ADDRESS == instance)
    {
        if(1u == portNumber)
        {
            return AM65xx_ICSS2_PORT1_PHY_ADDR;
        }

        else if(2u == portNumber)
        {
            return AM65xx_ICSS2_PORT2_PHY_ADDR;
        }

        else
        {
            return -1;
        }
    }

    else if(PRUICSS3_PHY_ADDRESS == instance)
    {
        if(1u == portNumber)
        {
            return AM65xx_ICSS3_PORT1_PHY_ADDR;
        }

        else if(2u == portNumber)
        {
            return AM65xx_ICSS3_PORT2_PHY_ADDR;
        }

        else
        {
            return -1;
        }
    }

    else
    {
        return -1;
    }
}


int32_t Board_phyReset(uint8_t numPorts)
{
    //TODO: Not working!

    //resetting PHYs of ICSS0 and ICSS1

    GPIOSetDirMode_v0(CSL_GPIO1_BASE, 58, 0);
    GPIOSetDirMode_v0(CSL_GPIO1_BASE, 38, 0);

    GPIOPinWrite_v0(CSL_GPIO1_BASE, 58, 1);
    GPIOPinWrite_v0(CSL_GPIO1_BASE, 38, 1);
    GPIOPinWrite_v0(CSL_GPIO1_BASE, 58, 0);
    GPIOPinWrite_v0(CSL_GPIO1_BASE, 38, 0);
    delay_us(10);
    GPIOPinWrite_v0(CSL_GPIO1_BASE, 58, 1);
    GPIOPinWrite_v0(CSL_GPIO1_BASE, 38, 1);
    delay_us(3000);

    return 0;
}

