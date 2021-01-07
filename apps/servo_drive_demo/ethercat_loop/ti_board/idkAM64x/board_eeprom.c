/**
 *  \file   board_eeprom.c
 *
 *  \brief  AM64x EVM Board EEPROM specific functions
 *
 *   This file contains the GPIO hardware parameters specific to board.
 */

/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <ti/board/src/am64x_evm/include/board_cfg.h>
#include <board_eeprom.h>
#include <delay_us.h>

#define I2C_EEPROM_PAGE_SIZE        (256)    /** Page size for I2C EEPROM */
/* FIXME: Write cycle time is 5 ms, but delay of 5ms does not look enough
 *        10 ms is working okay. Check if delay_us() is working fine*/
#define I2C_EEPROM_WRITE_CYCLE_TIME (10000)  /** Write cycle time = 5ms = 5000 us*/

#define MAX_NUM_MACADDRESS 6

extern const I2C_Config I2C_config[];
I2C_Handle i2c0Handle;

/* TODO: This function is not verified */
#if 0
Board_STATUS Board_getMACAddress(uint8_t inst, uint8_t *info)
{
    Board_STATUS ret = BOARD_SOK;
    I2C_Transaction i2cTransaction;
    I2C_Handle i2cHandle = NULL;
    char txBuf[2] = {0x00, 0x00};
    bool status;
    I2C_Params i2cParams;
    uint32_t i = 0;
    uint8_t test[6] = {0};
    uint8_t flag = 0;

    if(MAX_NUM_MACADDRESS < inst)
    {
        ret = BOARD_INVALID_PARAM;
        return ret;
    }

    /* Board i2c usage does not depend on interrupts */
    for(i = 0; I2C_config[i].fxnTablePtr != NULL; i++)
    {
        /* Check if interrupt set to true. Change it to false and raise a flag
            Note that number of i2c instances will not exceed 8 */
        if(((I2C_HwAttrs *)I2C_config[i].hwAttrs)->enableIntr == true)
        {
            flag |= (1 << i);
            ((I2C_HwAttrs *)I2C_config[i].hwAttrs)->enableIntr = false;
        }
    }

    I2C_init();

    I2C_Params_init(&i2cParams);

    i2cHandle = I2C_open(BOARD_I2C_EEPROM_INSTANCE, &i2cParams);


    /* Restore the original I2C_config table */
    for(i = 0; I2C_config[i].fxnTablePtr != NULL; i++)
    {
        if(flag & (1 << i))
            ((I2C_HwAttrs *)I2C_config[i
                                      ].hwAttrs)->enableIntr = true;
    }

    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = BOARD_I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = 2;

    /* Get header info */
    txBuf[0] = (char)(((uint32_t) 0xFF00 & BOARD_EEPROM_MACID0_ADDR) >> 8);
    txBuf[1] = (char)((uint32_t) 0xFF & BOARD_EEPROM_MACID0_ADDR);
    i2cTransaction.readBuf = (uint8_t *)test;
    i2cTransaction.readCount = BOARD_EEPROM_MACID_LENGTH;
    status = I2C_transfer(i2cHandle, &i2cTransaction);

    if(status == false)
    {
        ret = BOARD_I2C_TRANSFER_FAIL;
        I2C_close(i2cHandle);
        i2cHandle = NULL;
        return ret;
    }

    test[5] += (inst - 1);

    for(i = 0; i < 6; i++)
    {
        info[i] = test[i];
    }

    I2C_close(i2cHandle);
    i2cHandle = NULL;

    return ret;
}
#endif

Board_STATUS Board_i2cEepromInit(void)
{
    I2C_Params i2cParams;
//    int i;
    Board_STATUS ret = BOARD_SOK;
//    I2C_HwAttrs   i2c_cfg;
//    uint8_t flag = 0;

    if(i2c0Handle == NULL)
    {
        /*FIXME: Remove this if not needed*/
        /* Board i2c usage does not depend on interrupts */
//        for (i=0; I2C_config[i].fxnTablePtr != NULL; i++)
//        {
//            /* Check if interrupt set to true. Change it to false and raise a flag
//                Note that number of i2c instances will not exceed 8 */
//            if (((I2C_HwAttrs *)I2C_config[i].hwAttrs)->enableIntr == true)
//            {
//                I2C_socGetInitCfg(i, &i2c_cfg);
//                flag |= (1<<i);
//                i2c_cfg.enableIntr = false;
//                I2C_socSetInitCfg(i, &i2c_cfg);
//            }
//        }

        I2C_init();

        I2C_Params_init(&i2cParams);

        /* Note:- the Board ID EEPROM and the I2C IO EXPANDER are connected to
           same i2c instance. */
        i2c0Handle = I2C_open(BOARD_I2C_EEPROM_INSTANCE, &i2cParams);
        if (i2c0Handle == NULL)
            ret = BOARD_I2C_OPEN_FAIL;
        else
            ret = BOARD_SOK;

        /*FIXME: Remove this if not needed*/

//        /* Restore the original I2C_config table */
//        for (i=0; I2C_config[i].fxnTablePtr != NULL; i++)
//        {
//            if (flag & (1<<i))
//            {
//                I2C_socGetInitCfg(i, &i2c_cfg);
//                i2c_cfg.enableIntr = true;
//                I2C_socSetInitCfg(i, &i2c_cfg);
//            }
//        }
    }
    /* Enable Wake-up I2C port for EEPROM access */
//    enableWKUPI2C();

    return ret;
}


Board_STATUS Board_i2cEepromRead(I2C_Handle    handle,
                                 uint32_t      offsetAddress,
                                 uint8_t       *buf,
                                 uint32_t      len,
                                 uint8_t       slaveAddress)
{
    Board_STATUS ret = BOARD_SOK;
    I2C_Transaction i2cTransaction;
    char txBuf[2] = {0x00, 0x00};
    bool status;

    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = slaveAddress;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = 2;

    txBuf[0] = (char)(((uint32_t) 0xFF00 & offsetAddress)>>8);
    txBuf[1] = (char)((uint32_t) 0xFF & offsetAddress);
    i2cTransaction.readBuf = buf;
    i2cTransaction.readCount = len;

    status = I2C_transfer(handle, &i2cTransaction);
    if (status == false)
    {
        ret = BOARD_I2C_TRANSFER_FAIL;
    }

    return ret;
}

Board_STATUS Board_i2cEepromWrite(I2C_Handle    handle,
                                  uint32_t      offsetAddress,
                                  uint8_t       *buf,
                                  uint32_t      len,
                                  uint8_t       slaveAddress)
{
    Board_STATUS ret = BOARD_SOK;
    I2C_Transaction i2cTransaction;
    uint16_t offsetSize = 2;
    uint32_t bytesToWrite = len;
    bool status;
    char txBuf[I2C_EEPROM_PAGE_SIZE + 2 + 1];


    while(bytesToWrite >= I2C_EEPROM_PAGE_SIZE)
    {
        I2C_transactionInit(&i2cTransaction);
        i2cTransaction.slaveAddress = slaveAddress;
        i2cTransaction.writeBuf = &txBuf[0];
        i2cTransaction.writeCount = I2C_EEPROM_PAGE_SIZE + offsetSize;
        txBuf[0] = (char)(((uint32_t) 0xFF00 & offsetAddress) >> 8);
        txBuf[1] = (char)((uint32_t) 0xFF & offsetAddress);
        memcpy(&txBuf[2], buf, I2C_EEPROM_PAGE_SIZE);

        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0;

        status = I2C_transfer(handle, &i2cTransaction);
        if (status == false)
        {
            ret = BOARD_I2C_TRANSFER_FAIL;
            return ret;
        }

        buf = buf + I2C_EEPROM_PAGE_SIZE;
        offsetAddress = offsetAddress + I2C_EEPROM_PAGE_SIZE;
        bytesToWrite = bytesToWrite - I2C_EEPROM_PAGE_SIZE;


        delay_us(I2C_EEPROM_WRITE_CYCLE_TIME);
    }

    if (0 != bytesToWrite)
    {
        I2C_transactionInit(&i2cTransaction);
        i2cTransaction.slaveAddress = slaveAddress;
        i2cTransaction.writeBuf = &txBuf[0];
        i2cTransaction.writeCount = bytesToWrite + offsetSize;
        txBuf[0] = (char)(((uint32_t) 0xFF00 & offsetAddress) >> 8);
        txBuf[1] = (char)((uint32_t) 0xFF & offsetAddress);
        memcpy(&txBuf[2], buf, bytesToWrite);

        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0;

        status = I2C_transfer(handle, &i2cTransaction);
        if (status == false)
        {
            ret = BOARD_I2C_TRANSFER_FAIL;
        }
        delay_us(I2C_EEPROM_WRITE_CYCLE_TIME);
    }
    return ret;
}
