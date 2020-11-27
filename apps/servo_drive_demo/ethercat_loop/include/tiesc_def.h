/**
 *  \file   tiesc_def.h
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
#ifndef _TIESC_DEF_H_
#define _TIESC_DEF_H_

#if (defined(ECAT_LIMITED_DEMO) || defined(TIESC_SPI_SLAVE_MODE))

/**
ECAT_TIMER_INT: If this switch is set, then the watchdog time for the EtherCAT watchdog will be checked in a timer interrupt routine. */
#ifndef ECAT_TIMER_INT
#define ECAT_TIMER_INT                          (0)
#endif

/**
ESC_EEPROM_EMULATION: If this switch is set EEPROM emulation is supported. Not all ESC types support EEPROM emulation. See ESC datasheet for more information. */
#ifndef ESC_EEPROM_EMULATION
#define ESC_EEPROM_EMULATION                    (1)
#endif

#ifndef ESC_SYSTEMTIME_OFFSET
#define ESC_SYSTEMTIME_OFFSET                   (0x0910)
#endif

#ifndef ESC_DC_SYNC_ACTIVATION_OFFSET
#define ESC_DC_SYNC_ACTIVATION_OFFSET           (0x0981)
#endif

#ifndef ESC_DC_SYNC0_CYCLETIME_OFFSET
#define ESC_DC_SYNC0_CYCLETIME_OFFSET           (0x09A0)
#endif

#ifndef ESC_DC_SYNC1_CYCLETIME_OFFSET
#define ESC_DC_SYNC1_CYCLETIME_OFFSET           (0x09A4)
#endif

#ifndef TIESC_APPLICATION
/*TIESC_APPLICATION: Should be set if the Slave Sample Code runs on an ICE/IDK EVM. */
#define TIESC_APPLICATION                         1
#endif

#ifndef AL_EVENT_ENABLED
#define AL_EVENT_ENABLED                        (1)
#endif

#ifndef ESC_EEPROM_SIZE
#define ESC_EEPROM_SIZE                         (0x800)
#endif

#ifndef MEM_ADDR
#define MEM_ADDR                                uint8_t
#endif

#ifndef EEPROM_READ_SIZE
#define EEPROM_READ_SIZE                        (0x8)
#endif

#ifndef EEPROM_WRITE_SIZE
#define EEPROM_WRITE_SIZE                       (0x2)
#endif

#ifndef ESC_EEPROM_CONTROL_OFFSET
#define ESC_EEPROM_CONTROL_OFFSET               (0x0502)
#endif

#ifndef ESC_EEPROM_BUSY_MASK
#define ESC_EEPROM_BUSY_MASK                    (0x8000)
#endif

#ifndef ESC_EEPROM_CMD_MASK
#define ESC_EEPROM_CMD_MASK                     (0x0700)
#endif

#ifndef ESC_EEPROM_ADDRESS_OFFSET
#define ESC_EEPROM_ADDRESS_OFFSET               (0x0504)
#endif

#ifndef ESC_EEPROM_DATA_OFFSET
#define ESC_EEPROM_DATA_OFFSET                  (0x0508)
#endif

#ifndef STATE_INIT
#define STATE_INIT                              (0x01)
#endif

#ifndef STATE_PREOP
#define STATE_PREOP                             (0x02)
#endif

#ifndef STATE_BOOT
#define STATE_BOOT                              (0x03)
#endif

#ifndef STATE_SAFEOP
#define STATE_SAFEOP                            (0x04)
#endif

#ifndef STATE_OP
#define STATE_OP                                (0x08)
#endif

#endif

#endif /* _TIESC_DEF_H_*/
