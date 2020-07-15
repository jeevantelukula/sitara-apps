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

/* This is intended for Time Sync verification in the case that the EthCAT 
   slave stack (ESC) isn't present in the system. This code should be 
   excluded from the system when ESC is present. */

#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include "app_pslctrl_esc_sim.h"

/* ESC register offsets (bytes) */
#define ESC_AL_STATUS_OFFSET            ( 0x0130 )  /* Register Description: Actual State of the Device State Machine */
#define ESC_AL_STATUS_CUR_STATE_MASK    ( 0x000F )  /* ALStatus, current state mask */

#define ESC_DC_SYNC0_CYCLETIME_OFFSET   ( 0x09A0 )  /* Register Description: 32Bit Time between two consecutive SYNC0 pulses in ns */

/* Pointer to Shared DMEM containing ESC Registers */
static uint8_t * pEscRegs;

/* Initialize ESC firmware regs */
int32_t escFwRegsInit(
    PRUICSS_MaxInstances icssInstId,    /* ICSSG hardware instance ID */
    uint16_t alStatus,                  /* Value to write to "AL Status" ESC FW reg */
    uint32_t escSync0CycleTime_nsec     /* Value to write to "SYNC0 Cycle Time" ESC FW reg */
)
{
    /* Translate ICSSG hardware module ID to PWM API */
    if (icssInstId == PRUICCSS_INSTANCE_ONE) {
        pEscRegs = (uint8_t *)CSL_PRU_ICSSG0_RAM_SLV_RAM_BASE;
    }
    else if (icssInstId == PRUICCSS_INSTANCE_TWO) {
        pEscRegs = (uint8_t *)CSL_PRU_ICSSG1_RAM_SLV_RAM_BASE;
    }
    else {
        return APP_PSLCTRL_ESC_INV_PRMS;
    }

    /* Write AL Status:currentState to STATE_SAFEOP */
    alStatus = *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET];
    alStatus &= ~ESC_AL_STATUS_CUR_STATE_MASK;
    alStatus |= STATE_SAFEOP;
    *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] = alStatus;
    
    /* Write SYNC0 Cycle Time */
    *(volatile uint32_t *)&pEscRegs[ESC_DC_SYNC0_CYCLETIME_OFFSET] = escSync0CycleTime_nsec;
    
    return APP_PSLCTRL_ESC_SIM_SOK;
}
