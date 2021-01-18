/**
 *  \file   board_eeprom.h
 *
 *  \brief:
 *
 */

/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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
#ifndef _BOARD_EEPROM_H_
#define _BOARD_EEPROM_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ti/board/board.h>
#include <ti/drv/i2c/I2C.h>

#if defined(SOC_AM65XX)
/*NOTE: For R5F on AM65xx, I2C driver allows access by default only to
 *      MCU_I2C0 instance. MCU_I2C0 is accessible using i2cInitCfg[0].
 *      Board ID EEPROM is connected to WKUP_I2C0 instance. In order to
 *      access WKUP_I2C0, we need to add an entry in i2cInitCfg[] array
 *      which is done here. We add the required details in i2cInitCfg[1].
 *      Similarly i2cInitCfg[0] is used for LEDs and Rotary Switch access*/
#define I2C_BOARD_ID_EEPROM_INSTANCE    (1)
#endif
/**
* @brief Function to get MAC address
*
* @param inst    [IN] Instance
* @param info    [OUT] MAC address of the input instance
*
*  @retval status
*/
Board_STATUS Board_getMACAddress(uint8_t inst, uint8_t *info);

/**
* @brief Function to initialize I2C EEPROM on IDK
*
*  @retval status
*/
Board_STATUS Board_i2cEepromInit(void);

/**
* @brief Function to get read data from I2C EEPROM
*
* @param handle         [IN] Initialized I2C_Handle
* @param offsetAddress  [IN] Offset to start the read from
* @param buf            [IN] Pointer to a buffer to read the data into
* @param len            [IN] Amount of data to read
* @param slaveAddress   [IN] I2C address for EEPROM

*  @retval status
*/
Board_STATUS Board_i2cEepromRead(I2C_Handle    handle,
                                 uint32_t      offsetAddress,
                                 uint8_t       *buf,
                                 uint32_t      len,
                                 uint8_t       slaveAddress);

/**
* @brief Function to get write data to I2C EEPROM
*
* @param handle         [IN] Initialized I2C_Handle
* @param offsetAddress  [IN] Offset to start the write from
* @param buf            [IN] Pointer to  data to write
* @param len            [IN] Length of the data pointed to by buf
* @param slaveAddress   [IN] I2C address for EEPROM

*  @retval status
*/
Board_STATUS Board_i2cEepromWrite(I2C_Handle    handle,
                                  uint32_t      offsetAddress,
                                  uint8_t       *buf,
                                  uint32_t      len,
                                  uint8_t       slaveAddress);
#endif /* _BOARD_EEPROM_H_*/
