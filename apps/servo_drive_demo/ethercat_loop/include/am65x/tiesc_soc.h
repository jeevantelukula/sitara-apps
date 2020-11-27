/**
 *  \file   tiesc_soc.h
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

#ifndef _TIESC_SOC_H_
#define _TIESC_SOC_H_

#include <ti/drv/pruss/soc/pruicss_soc.h>
#include <ti/drv/pruss/pruicss_ver.h>

/* The PRUSS drivers before version 1.0.0.16 have a typo in macro names*/
#if (PRUICSS_DRV_VERSION_ID < 0x01000010)
#define PRUICSS_INSTANCE_ONE   (PRUICCSS_INSTANCE_ONE)
#define PRUICSS_INSTANCE_TWO   (PRUICCSS_INSTANCE_TWO)
#define PRUICSS_INSTANCE_THREE (PRUICCSS_INSTANCE_THREE)
#endif

/* EEPROM data offset in SPI/QSPI Flash */
#define SPI_EEPROM_DATA_OFFSET      0x100000
#define DEFAULT_PRUICSS_INSTANCE    PRUICSS_INSTANCE_ONE
#define TIESC_TASK_STACK_SIZE_MUL   2

/* Change this define to switch between PRUICSS for AM571x */
#define PRUICSS_INSTANCE        DEFAULT_PRUICSS_INSTANCE

#define TIESC_LINK0_POL   TIESC_LINK_POL_ACTIVE_HIGH
#define TIESC_LINK1_POL   TIESC_LINK_POL_ACTIVE_HIGH

#define SPINLOCK_GRANTED       0
#define SPINLOCK_UNLOCK        0


uint8_t isEtherCATDevice(void);

void tiesc_mii_pinmuxConfig (void);

int16_t getARMInterruptOffset();

uint32_t getSpinlockClkCtrlOffset();

uint32_t getSpinlockLockReg0Offset();

void bsp_soft_reset();

void bsp_soc_evm_init();

void display_esc_version(uint16_t revision, uint16_t build);

void initSpinlock();

void Send_BootComplete_Message_To_Partner(void);

void * tiesc_memcpy(uint8_t *dst, const uint8_t *src, uint32_t size_bytes);
void * tiesc_memset(uint8_t *dst, int8_t val, uint32_t size_bytes);

#endif /* _TIESC_SOC_H_*/
