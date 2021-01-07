/**
 *  \file   board_spi.c
 *
 *  \brief  AM64x EVM Board SPI specific APIS
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

#include <board_spi.h>
#include <ti/board/src/am64x_evm/include/board_cfg.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/spi/soc/SPI_soc.h>

/** \brief OSPI Flash handle. Can be extern from app*/
Board_flashHandle boardFlashHandle;

void OSPI_init()
{
    OSPI_v0_HwAttrs ospi_cfg;
    uint32_t          tuneEnable;

    /* NOTE: DAC writes are not supported on Cypress xSPI Flash - Switch to INDAC mode for write as WA to PDK-7115 */
    /* Get the default OSPI init configurations */
    OSPI_socGetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);
    ospi_cfg.dacEnable = false;
    ospi_cfg.phyEnable = false;
    ospi_cfg.dmaEnable = false;
    ospi_cfg.intrEnable = true;
    /* Set the default OSPI init configurations */
    OSPI_socSetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    tuneEnable = FALSE;

    /* Open the Board OSPI NOR device
       and use default OSPI configurations */
    boardFlashHandle = Board_flashOpen(BOARD_FLASH_ID_S28HS512T, BOARD_OSPI_NOR_INSTANCE, (void *)(&tuneEnable));

    if(!boardFlashHandle)
    {
        UART_printf("\n Board_flashOpen failed. \n");
        return;
    }
}

