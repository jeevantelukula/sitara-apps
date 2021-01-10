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

#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>
#include <ti/board/src/am64x_evm/include/board_cfg.h>
#include <board_gpioLed.h>

/*NOTE: Tri-color LEDs are not present on AM64x EVM. Toggling
 *      AM64X_EVM_RED0_GPIO, AM64X_EVM_GREEN0_GPIO and AM64X_EVM_YELLOW0_GPIO
 *      just toggles the TEST_LED2. Similarly AM64X_EVM_RED1_GPIO,
 *      AM64X_EVM_GREEN1_GPIO and AM64X_EVM_YELLOW1_GPIO toggles TEST_LED1*/

/***********************************************************************/
/* Macros                                        */
/***********************************************************************/

/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] =
{
    /* Output pin : TEST_LED1 */
    GPIO_DEVICE_CONFIG(BOARD_GPIO_TEST_LED1_PORT_NUM, BOARD_GPIO_TEST_LED1_PIN_NUM) |
    GPIO_CFG_OUTPUT ,

    /* Output pin : TEST_LED2 */
    GPIO_DEVICE_CONFIG(BOARD_MCU_GPIO_TEST_LED2_PORT_NUM, BOARD_MCU_GPIO_TEST_LED2_PIN_NUM) |
    GPIO_CFG_OUTPUT,
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

#define AM64X_EVM_RED0_GPIO        1
#define AM64X_EVM_GREEN0_GPIO      1
#define AM64X_EVM_YELLOW0_GPIO     1
#define AM64X_EVM_RED1_GPIO        0
#define AM64X_EVM_GREEN1_GPIO      0
#define AM64X_EVM_YELLOW1_GPIO     0

void  Board_setTriColorLED(uint32_t gpioLeds, uint8_t value)
{
    if(gpioLeds & BOARD_TRICOLOR0_RED)
    {
        GPIO_write(AM64X_EVM_RED0_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR0_GREEN)
    {
        GPIO_write(AM64X_EVM_GREEN0_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR0_YELLOW)
    {
        GPIO_write(AM64X_EVM_YELLOW0_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_RED)

    {
        GPIO_write(AM64X_EVM_RED1_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_GREEN)
    {
        GPIO_write(AM64X_EVM_GREEN1_GPIO, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_YELLOW)
    {
        GPIO_write(AM64X_EVM_YELLOW1_GPIO, value);
    }
}
