/******************************************************************************
 * Copyright (c) 2018-2020 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/**
 *
 * \file   current_monitor_test.h
 *
 * \brief  This is the header file for current monitor diagnostic test.
 *
 */

#ifndef _CURRENT_MONITOR_TEST_H_
#define _CURRENT_MONITOR_TEST_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#include <ti/csl/soc.h>

#include "board.h"
#include "board_cfg.h"

#include "diag_common_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Number of INA devices installed */
#if defined(am65xx_evm) || defined(am65xx_idk)
#define NUM_OF_INA_DEVICES                (0x08U)
#elif defined(am64x_evm)
#define NUM_OF_INA_DEVICES                (0x06U)
#endif

/* INA Device Register Address Offsets */
#define CONFIGURATION_REG_ADDR_OFFSET     (0x00U)
#define SHUNT_VOLTAGE_REG_ADDR_OFFSET     (0x01U)
#define BUS_VOLTAGE_REG_ADDR_OFFSET       (0x02U)
#define POWER_REG_ADDR_OFFSET             (0x03U)
#define CURRENT_REG_ADDR_OFFSET           (0x04U)
#define CALIBRATION_REG_ADDR_OFFSET       (0x05U)

/* INA Device Register masks */
#define SHUNT_VOLTAGE_REG_MASK            (0x7fffU)
#define BUS_VOLTAGE_REG_MASK              (0x7fffU)
#define POWER_REG_MASK                    (0xffffU)
#define CURRENT_REG_MASK                  (0x7fffU)
#define SIGNED_REG_MASK                   (0x8000U)

/* INA Device Constant values */
#define SHUNT_MICROVOLT_CONSTANT          (float)(2.50) // constant 2.5  uV/bit
#define BUS_MILIVOLT_CONSTANT             (float)(1.25) // constant 1.25 mV/bit
#define POWER_MICROWATT_CONSTANT          (float)(25.0) // constant 25 * current_LSB
#define INA_DEFAULT_CONFIG_VAL            (uint16_t)(0x4737U)
#define INA_CALIBRATION_CONSTANT          (uint32_t)(5120000U)

/**
 *  \brief Structure defining INA Device calibration.
 */
typedef struct CalibrationParams {
    uint16_t shuntMiliOhms;
    uint16_t lsbMicroAmps;
} calParams_t;

/**
 *  \brief Structure defining INA Device configuration.
 *
 */
typedef struct inaCfgObj {
    char deviceID[20];
    calParams_t inaCalParams;
    uint8_t slaveAddr;
} inaCfgObj_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _CURRENT_MONITOR_TEST_H_ */
