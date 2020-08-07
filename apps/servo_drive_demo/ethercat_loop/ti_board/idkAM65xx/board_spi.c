/**
 *  \file   board_spi.c
 *
 *  \brief  AM65xx IDK Board SPI specific APIS
 *
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
#include <stdbool.h>
#include <ti/csl/soc.h>
#include <ti/drv/gpio/GPIO.h>
#include <board_spi.h>
/* Flash header file */
#include <ti/drv/spi/SPI.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/board/board.h>
#include <ti/board/src/am65xx_idk/include/board_cfg.h>
#include <ti/drv/uart/UART_stdio.h>


#define QSPI_PER_CNT            (1U)
#define QSPI_INSTANCE           (1U)
#define QSPI_OFFSET             (4U)

/** \brief QSPI Flash handle. Can be extern from app*/
Board_flashHandle boardFlashHandle;

extern SPI_Handle handle;                   /* SPI handle */

#define USE_INDUS_INPUTS 0

void LoadData(void)
{
#if USE_INDUS_INPUTS
    GPIO_write(8, 0);
    delay_us(1);
    GPIO_write(8, 1);
    delay_us(1);
    return;
#endif
}

void Board_readHVS(uint8_t *switches)
{
#if USE_INDUS_INPUTS
    /* MCSPI params required */
    /* Buffer containing the known data that needs to be written to flash */
    uint8_t txBuf[1U];
    /* Buffer containing the received data */
    uint8_t rxBuf[1U] = {0xFFU};
    /* Transfer length */
    uint32_t transferLength;
    SPI_Transaction transaction;         /* SPI transaction */

    /* Load data */
    LoadData();

    /* Initiate transfer */
    txBuf[0] = 0xAAU;
    transferLength = 1U;

    transaction.count = transferLength;
    transaction.txBuf = &txBuf[0];
    transaction.rxBuf = &rxBuf[0];
    SPI_transfer(handle, &transaction);

    *switches = *rxBuf & 0xFF;
#endif
}

void QSPI_init()
{
    OSPI_v0_HwAttrs ospi_cfg;

    /* Get the default OSPI init configurations */
    OSPI_socGetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    /* Indirect access controller mode always uses polling, interrupt is not supported */
    ospi_cfg.intrEnable = false;
    ospi_cfg.phyEnable = false;


    /* Set the default OSPI init configurations */
    OSPI_socSetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);


    /* Open the Board QSPI NOR device with QSPI port 0
       and use default QSPI configurations */
    boardFlashHandle = Board_flashOpen(BOARD_FLASH_ID_MT35XU512ABA1G12,
                                       BOARD_OSPI_NOR_INSTANCE, NULL);

    if(!boardFlashHandle)
    {
        UART_printf("\n Board_flashOpen failed. \n");
        return;
    }
}
