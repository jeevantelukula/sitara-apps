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

#ifndef APP_PSL_CTRL_ESC_SIM_H_
#define APP_PSL_CTRL_ESC_SIM_H_

#include <ti/csl/tistdtypes.h>

/* Status codes */
#define APP_PSLCTRL_ESC_SIM_SOK         (  0 )  /* no error */
#define APP_PSLCTRL_ESC_INV_PRMS        ( -1 )  /* error, invalid parameters */

/* ESC AL Status, current state */
#define STATE_PREOP     ((uint8_t) 0x02)    /* State PreOP */
#define STATE_SAFEOP    ((uint8_t) 0x04)    /* State SafeOP */

/* Initialize ESC firmware regs */
int32_t escFwRegsInit(
    PRUICSS_MaxInstances icssInstId,    /* ICSSG hardware instance ID */
    uint16_t alStatus,                  /* Value to write to "AL Status" ESC FW reg */
    uint32_t escSync0CycleTime_nsec     /* Value to write to "SYNC0 Cycle Time" ESC FW reg */
);

#endif /* APP_PSL_CTRL_ESC_SIM_H_ */
