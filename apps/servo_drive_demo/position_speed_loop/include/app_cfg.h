/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

#ifndef APP_CFG_H_
#define APP_CFG_H_

#define ENABLE_BOARD

#define MAX_NUM_AXES            ( 3 )           /* MAX number of independent axis supported */

/* IPC CPU ID should match with EtherCAT CPU configuration */
#define IPC_ETHERCAT_CPU_ID     ( MAILBOX_IPC_CPUID_MCU1_0 )
#define IPC_PSL_MC_CPU_ID       ( MAILBOX_IPC_CPUID_MCU1_1 )

/* Simulated ECAT timer */
#define SIM_ECAT_TIMER_ID           ( 2 )           /* Timer ID */
#define SIM_ECAT_TIMER_FREQ_HZ      ( 25000000 )    /* Timer frequency, WKUP_HFOSC0_CLKOUT=25 MHz */
#define SIM_ECAT_TIMER_PERIOD_USEC  ( 125 )         /* Timer period (usec.) */
#define SIM_ECAT_TIMER_INTNUM       ( 40 )          /* Timer interrupt, R5F0 MCU_TIMER_0_INT */

#endif /* APP_CFG_H_ */
