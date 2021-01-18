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

/**
 *  \file   current_monitor_test.c
 *
 *  \brief  Current Monitor demo
 *
 *  Operation: Configure I2C bus for Sitara EVMs with INA devices connected,
 *  verification of INA device by measuring the shunt voltage, bus voltage,
 *  power, and current then printing it on to the serial console.
 *
 *  Supported SoCs: AM65x and AM64x
 *
 *  Supported Platforms: am65xx_evm, am65xx_idk, and am64x_evm.
 *
 */

#include "current_monitor_test.h"

extern I2C_config_list I2C_config;

#if defined(am65xx_evm) || defined(am65xx_idk)
inaCfgObj_t inaDevice[NUM_OF_INA_DEVICES] = {
    {"VDD_CORE",      {  2, 150}, 0x40},
    {"VDD_MCU",       { 10,  25}, 0x41},
    {"VDD_MPU",       {  2, 100}, 0x42},
    {"SoC_DVDD3V3",   {  2, 100}, 0x43},
    {"SoC_DVDD1V8",   { 10,  50}, 0x44},
    {"SoC_AVDD1V8",   { 10,  50}, 0x45},
    {"SoC_VDDS_DDR",  { 10,  25}, 0x46},
    {"VDD_DDR",       { 10,  25}, 0x47}
};
#elif defined(am64x_evm)
inaCfgObj_t inaDevice[NUM_OF_INA_DEVICES] = {
    {"VDD_CORE",      {  2, 100}, 0x40},
    {"VDDAR_CORE",    { 10,  20}, 0x41},
    {"VDDS_DDR",      { 10,  20}, 0x46},
    {"SoC_DVDD1V8",   { 10,  20}, 0x4B},
    {"SoC_DVDD3V3",   { 10,  20}, 0x4C},
    {"SoC_AVDD1V8",   { 10,  20}, 0x4E}
};
#endif

static int8_t INA_read_register(I2C_Handle handle, uint8_t slaveAddress, uint8_t regAddr, uint16_t *regData)
{
    int8_t  ret = 0;
    uint8_t  rx[2];
    I2C_Transaction transaction;

    /* Initializes the I2C transaction structure with default values */
    I2C_transactionInit(&transaction);

    transaction.slaveAddress = slaveAddress;
    transaction.writeBuf     = &regAddr;
    transaction.writeCount   = 1;
    transaction.readBuf      = NULL;
    transaction.readCount    = 0;

    ret = I2C_transfer(handle, &transaction);
    if(ret != I2C_STS_SUCCESS)
    {
        ret = -1;
        return ret;
    }

    BOARD_delay(100);

    transaction.writeBuf     = NULL;
    transaction.writeCount   = 0;
    transaction.readBuf      = &rx[0];
    transaction.readCount    = 2;

    ret = I2C_transfer(handle, &transaction);
    if(ret != I2C_STS_SUCCESS)
    {
        ret = -1;
        return ret;
    }
    else
    {
        ret = 0;
    }

    /* Note:- Slave device responds with MSB first for the read sequence sent */
    *regData = ((((uint16_t)rx[0]) << 8) | ((uint16_t)rx[1]));

    return ret;
}

static int8_t INA_write_register(I2C_Handle handle, uint8_t slaveAddress, uint8_t regAddr, uint16_t regData)
{
    int8_t ret = 0;
    uint8_t tx[3];
    I2C_Transaction transaction;

    I2C_transactionInit(&transaction);

    transaction.slaveAddress = slaveAddress;
    transaction.writeBuf     = &tx[0];
    transaction.writeCount   = 3;
    transaction.readBuf      = NULL;
    transaction.readCount    = 0;

    tx[0] = regAddr;

    /* MSB of 16-bit data should be sent first followed by the LSB */
    tx[1] = (uint8_t)((regData & 0xFF00) >> 8);
    tx[2] = (uint8_t)(regData & 0x00FF);

    BOARD_delay(100);

    ret = I2C_transfer(handle, &transaction);
    if(ret != I2C_STS_SUCCESS)
    {
        ret = -1;
    }
    else
    {
        ret = 0;
    }

    return ret;
}

static int8_t INA_set_default_config(I2C_Handle handle, inaCfgObj_t *inaDevice)
{
    int8_t ret = 0;

    ret = INA_write_register(handle, inaDevice->slaveAddr, CONFIGURATION_REG_ADDR_OFFSET, INA_DEFAULT_CONFIG_VAL);

    return ret;
}

static int8_t INA_set_calibration(I2C_Handle handle, inaCfgObj_t *inaDevice)
{
    int8_t ret = 0;
    calParams_t *calParams = &inaDevice->inaCalParams;
    uint32_t calibration = INA_CALIBRATION_CONSTANT / ((uint32_t)(calParams->shuntMiliOhms * calParams->lsbMicroAmps));

    if (calibration > INT16_MAX) {
        ret = -1;
        UART_printf("[Error] (INA_CALIBRATION_CONSTANT / (shuntMiliOhms * lsbMicroAmps)) must be less than 32768.\n");
    }
    else
    {
        ret = INA_write_register(handle, inaDevice->slaveAddr, CALIBRATION_REG_ADDR_OFFSET, (uint16_t)calibration);
    }

    return ret;
}

static int8_t INA_read_shunt_microvolts(I2C_Handle handle, inaCfgObj_t *inaDevice, float *shuntVoltage)
{
    int8_t ret = 0;
    float sign = 1;
    uint16_t readRegData = 0;

    ret = INA_read_register(handle, inaDevice->slaveAddr, SHUNT_VOLTAGE_REG_ADDR_OFFSET, &readRegData);
    if(ret == 0)
    {
        if (readRegData & SIGNED_REG_MASK)
        {
            readRegData = (~readRegData)+1;
            sign = -1;
        }
        else
        {
            *shuntVoltage = (readRegData & SHUNT_VOLTAGE_REG_MASK) * SHUNT_MICROVOLT_CONSTANT * sign;
        }
    }

    return ret;
}

static int8_t INA_read_bus_milivolts(I2C_Handle handle, inaCfgObj_t *inaDevice, float *busVoltage)
{
    int8_t ret = 0;
    uint16_t readRegData = 0;

    ret = INA_read_register(handle, inaDevice->slaveAddr, BUS_VOLTAGE_REG_ADDR_OFFSET, &readRegData);
    if(ret == 0)
    {
        *busVoltage = (readRegData & BUS_VOLTAGE_REG_MASK) * BUS_MILIVOLT_CONSTANT;
    }

    return ret;
}

static int8_t INA_read_bus_microwatts(I2C_Handle handle, inaCfgObj_t *inaDevice, float *powerMeasured)
{
    int8_t ret = 0;
    uint16_t readRegData = 0;
    calParams_t *calParams = &inaDevice->inaCalParams;

    ret = INA_read_register(handle, inaDevice->slaveAddr, POWER_REG_ADDR_OFFSET, &readRegData);
    if(ret != 0)
    {
        return ret;
    }
    else
    {
        *powerMeasured = (readRegData & POWER_REG_MASK) * calParams->lsbMicroAmps * POWER_MICROWATT_CONSTANT;
    }

    return ret;
}

static int8_t INA_read_bus_microamps(I2C_Handle handle, inaCfgObj_t *inaDevice, float *currentMeasured)
{
    int8_t ret = 0;
    float sign = 1;
    uint16_t readRegData = 0;
    calParams_t *calParams = &inaDevice->inaCalParams;

    ret = INA_read_register(handle, inaDevice->slaveAddr, CURRENT_REG_ADDR_OFFSET, &readRegData);
    if(ret == 0)
    {
        if (readRegData & SIGNED_REG_MASK)
        {
            readRegData = (~readRegData)+1;
            sign = -1;
        }
        else
        {
            *currentMeasured = (readRegData & CURRENT_REG_MASK) * calParams->lsbMicroAmps * sign;
        }
    }

    return ret;
}

static int8_t currentMonitorDemo(void)
{
    int8_t ret = 0;
    uint8_t index;
    float retData = 0;

    I2C_Params i2cParams;
    I2C_HwAttrs i2cConfig;
    I2C_Handle handle = NULL;

    for(index = 0; index < I2C_HWIP_MAX_CNT; index++)
    {
        I2C_socGetInitCfg(index, &i2cConfig);
        i2cConfig.enableIntr = false;
        I2C_socSetInitCfg(index, &i2cConfig);
    }

    /* Initializes the I2C */
    I2C_init();

    /* Initializes the I2C Parameters */
    I2C_Params_init(&i2cParams);
#if defined(SOC_AM65XX)
    i2cParams.bitRate = I2C_400kHz;
#endif
    /* Configures the I2C instance with the passed parameters*/
    handle = I2C_open(BOARD_I2C_CURRENT_MONITOR_INSTANCE, &i2cParams);
    if(handle == NULL)
    {
        UART_printf("\nI2C Open failed!\n");
        ret = -1;
        return ret;
    }

    UART_printf("Setting the configuration & calibration registers...\n");
    for(index = 0; index < NUM_OF_INA_DEVICES; index++)
    {
        ret = INA_set_default_config(handle, &inaDevice[index]);
        if(ret != 0)
        {
            UART_printf(inaDevice[index].deviceID);
            UART_printf(": Unable to set the configuration register\n\r");
            break;
        }

        ret = INA_set_calibration(handle, &inaDevice[index]);
        if(ret != 0)
        {
            UART_printf(inaDevice[index].deviceID);
            UART_printf(": Unable to set the calibration register\n\r");
            break;
        }
    }

    BOARD_delay(1000 * 1000);   /* microseconds (1000 x 1000 = 1 second) */

    for(index = 0; index < NUM_OF_INA_DEVICES; index++)
    {
        UART_printf("\n");
        UART_printf(inaDevice[index].deviceID);
        UART_printf("\n");
        UART_printf("Reading the Bus Voltage register...\n\r");
        ret = INA_read_bus_milivolts(handle, &inaDevice[index], &retData);
        if(ret != 0)
        {
            UART_printf("Failed to read the Bus Voltage register\n\r");
            break;
        }
        else
        {
            UART_printf("Bus Voltage = %d mV\n", (uint32_t) retData);
        }
        UART_printf("Reading the Shunt Voltage register...\n\r");
        ret = INA_read_shunt_microvolts(handle, &inaDevice[index], &retData);
        if(ret != 0)
        {
            UART_printf("Failed to read the Shunt Voltage register\n\r");
            break;
        }
        else
        {
            UART_printf("Shunt Voltage = %d uV\n", (uint32_t) retData);
        }

        UART_printf("Reading the Current register...\n\r");
        ret = INA_read_bus_microamps(handle, &inaDevice[index], &retData);
        if(ret != 0)
        {
            UART_printf("Failed to read the Current register\n\r");
            break;
        }
        else
        {
            UART_printf("Current = %d mA\n", ((uint32_t) retData) / 1000);
        }

        UART_printf("Reading the Power register...\n\r");
        ret = INA_read_bus_microwatts(handle, &inaDevice[index], &retData);
        if(ret != 0)
        {
            UART_printf("Failed to read the Power register\n\r");
            break;
        }
        else
        {
            UART_printf("Power = %d mW\n", ((uint32_t) retData) / 1000);
        }
    }

    /* Closing the current monitor i2c instance */
    I2C_close(handle);

    return ret;
}

#if defined(SOC_AM65XX) || defined(SOC_AM64X)
int main(void)
{
    Board_STATUS status;
    int8_t ret = 0;

#if defined(am64x_evm)
    Board_initParams_t boardInitParams;
    Board_getInitParams(&boardInitParams);
    boardInitParams.uartInst = BOARD_UART3_INSTANCE;
    Board_setInitParams(&boardInitParams);
#endif

    status = Board_init(BOARD_INIT_UART_STDIO | BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_MODULE_CLOCK);
    if(status != BOARD_SOK)
    {
        return -1;
    }

    UART_printf("\n**********************************************\n");
    UART_printf  ("*            Current Monitor Demo            *\n");
    UART_printf  ("**********************************************\n");

    ret = currentMonitorDemo();

    if(ret == 0)
    {
        UART_printf("\nCurrent Monitor Demo Passed\n");
    }
    else
    {
        UART_printf("\nCurrent Monitor Demo Failed\n");
    }

    Board_deinit(BOARD_DEINIT_UART_STDIO);

    return ret;
}
#else
#error
#endif
