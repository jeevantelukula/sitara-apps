/**
 *  \file   board_gpioLed.c
 *
 *  \brief:
 *  AM64x EVM GPIO LED configurations
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

#ifndef TIESC_EMULATION_PLATFORM

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <ti/csl/soc.h>
#include <ti/csl/csl_types.h>
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include <board_gpioLed.h>

/***********************************************************************/
/* Macros                                        */
/***********************************************************************/

/* Used for McSPI HVS */
/* Port and pin number mask for GPIO Load pin.
   Bits 7-0: Pin number  and Bits 15-8: Port number */
#define AM57X_IDK_GPIO_LD_PIN    (0x0313)

/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] =
{
    /* Output pin : ETH_LED1 */
    GPIO_DEVICE_CONFIG(1, 46) |
    GPIO_CFG_OUTPUT ,

    /* Output pin : ETH_LED2 */
    GPIO_DEVICE_CONFIG(1, 66) |
    GPIO_CFG_OUTPUT,

    /* Output pin : ETH_LED3 */
    GPIO_DEVICE_CONFIG(1, 48) |
    GPIO_CFG_OUTPUT,

    /* Output pin : ETH_LED4 */
    GPIO_DEVICE_CONFIG(1, 68) |
    GPIO_CFG_OUTPUT,

    /* Output pin : ETH_LED5 */
    GPIO_DEVICE_CONFIG(0, 73) |
    GPIO_CFG_OUTPUT,

    /* Output pin : ETH_LED6 */
    GPIO_DEVICE_CONFIG(0, 93) |
    GPIO_CFG_OUTPUT,

    /* Output pin : ETH_LED7 */
    GPIO_DEVICE_CONFIG(0, 75) |
    GPIO_CFG_OUTPUT,

    /* Output pin : ETH_LED8 */
    GPIO_DEVICE_CONFIG(0, 95) |
    GPIO_CFG_OUTPUT,

    /* Output pin : IDK_IOEXP_LDn_1V8 */
    /* Used for McSPI HVS LDn */
    GPIO_DEVICE_CONFIG(0, 65) |
    GPIO_CFG_OUTPUT,

    //    /* Output pin : GPIO_ETH0/1_RESETn */
    //    GPIO_DEVICE_CONFIG(1, 58) |
    //    GPIO_CFG_OUTPUT,
    //
    //    /* Output pin : GPIO_ETH2/3_RESETn */
    //    GPIO_DEVICE_CONFIG(1, 38) |
    //    GPIO_CFG_OUTPUT,

};

/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] =
{
    NULL
};

/* GPIO Driver configuration structure */
GPIO_v0_Config GPIO_v0_config =
{
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    0x2U,
};

#ifdef AM65XX_ALPHA_BOARD
#define AM65XX_IDK_RED0_GPIO        0
#define AM65XX_IDK_GREEN0_GPIO      1
#define AM65XX_IDK_YELLOW0_GPIO     2
#define AM65XX_IDK_RED1_GPIO        4
#define AM65XX_IDK_GREEN1_GPIO      5
#define AM65XX_IDK_YELLOW1_GPIO     6
#else
#define AM65XX_IDK_RED0_GPIO        1
#define AM65XX_IDK_GREEN0_GPIO      0
#define AM65XX_IDK_YELLOW0_GPIO     2
#define AM65XX_IDK_RED1_GPIO        5
#define AM65XX_IDK_GREEN1_GPIO      4
#define AM65XX_IDK_YELLOW1_GPIO     6
#endif

void  Board_setTriColorLED(uint32_t gpioLeds, uint8_t value)
{
    if(gpioLeds & BOARD_TRICOLOR0_RED)
    {
        GPIO_write(AM65XX_IDK_RED0_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR0_GREEN)
    {
        GPIO_write(AM65XX_IDK_GREEN0_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR0_YELLOW)
    {
        GPIO_write(AM65XX_IDK_YELLOW0_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_RED)

    {
        GPIO_write(AM65XX_IDK_RED1_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_GREEN)
    {
        GPIO_write(AM65XX_IDK_GREEN1_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_YELLOW)
    {
        GPIO_write(AM65XX_IDK_YELLOW1_GPIO, value);
    }
}

#endif /* TIESC_EMULATION_PLATFORM */
