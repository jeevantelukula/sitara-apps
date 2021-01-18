/**
 *  \file   board_i2cLed.c
 *
 *  \brief  AM64x EVM i2cLed APIs
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

#include <board_i2cLed.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/board/src/am64x_evm/include/board_cfg.h>


I2C_Handle i2cLedhandle = NULL;
static I2C_Transaction i2cLedTransaction;
static I2C_Transaction i2cTestLedTransaction;
static char i2cLedtxBuf[2] = {0x44 , 0x00};
static char i2cTestLedtxBuf[2];

void Board_i2cLedInit()
{
    /*I2C Init */
    I2C_Params i2cParams;

    I2C_init();
    I2C_Params_init(&i2cParams);
    //    i2cParams.bitRate = I2C_400kHz;
    i2cLedhandle = I2C_open(BOARD_I2C_LED_INSTANCE, &i2cParams);

    i2cLedTransaction.slaveAddress = BOARD_I2C_LED_ADDR;
    i2cLedTransaction.writeBuf = (uint8_t *)&i2cLedtxBuf[0];
    i2cLedTransaction.writeCount = 2;

    i2cTestLedTransaction.slaveAddress = BOARD_I2C_IOEXP_DEVICE1_ADDR;
    i2cTestLedTransaction.writeBuf = (uint8_t *)&i2cTestLedtxBuf[0];
    i2cTestLedTransaction.writeCount = 2;

    /* Configure P20 (corresponding to TEST_LED1) as output pin */
    /* Command Byte for Configuration Port 2 register */
    i2cTestLedtxBuf[0] = 0x0E;
    /* Set last bit (corresponding to P20) to 0 */  
    i2cTestLedtxBuf[1] = 0xFE;
    I2C_transfer(i2cLedhandle, &i2cTestLedTransaction);

    /* Command byte for Output Port 2 register. Needed for subsequent calls to 
       control LED */
    i2cTestLedtxBuf[0] = 0x06;
    /* Default value of output bit is 1. Set it to 0*/
    Board_setTestLED1(0);
}

void Board_setDigOutput(uint8_t ledData)
{
    i2cLedtxBuf[1] = ledData;
    I2C_transfer(i2cLedhandle, &i2cLedTransaction);
}

void Board_setTestLED1(uint8_t value)
{
    /*NOTE: For controlling LED, we need to control the LSB in Output Port 2 
            register. The operation needed is read-modify-write, but that is
            expensive as it needs two I2C transactions. In EVM design, P21 to 
            P27 (corresponding to other bits in Output Port 2 register) are 
            connected to TP, so sending just one transaction(for write) works.*/

    i2cTestLedtxBuf[1] = value;
    I2C_transfer(i2cLedhandle, &i2cTestLedTransaction);
}
