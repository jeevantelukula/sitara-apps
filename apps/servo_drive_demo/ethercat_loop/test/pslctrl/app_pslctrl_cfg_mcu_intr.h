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

#ifndef _APP_PSLCTRL_CFG_MCU_INTR_H_
#define _APP_PSLCTRL_CFG_MCU_INTR_H_

#include <stdint.h>

/* Status codes */
#define APP_CSLCTRL_CFG_MCU_INTR_SOK                (  0 )  /* no error */
#define APP_CSLCTRL_CFG_MCU_INTR_SERR_CFG_INTR_RTR  ( -1 )  /* interrupt router configuration error */
#define APP_CSLCTRL_CFG_MCU_INTR_SERR_REG_INTR      ( -2 )  /* interrupt registration error */

/* Configures Compare Event router */
int32_t configureCmpEventInterruptRouter(
    int32_t intrRtrInIntNum, 
    int32_t intrRtrOutIntNum
);

/* Registers an interrupt for event from compare event router */
int32_t registerIntrOnCmpEvent(
    int32_t intrNum,
    void (*isrRoutine)(uintptr_t) /* The ISR routine to hook the corepacEventNum to */
);

/* Disables interrupt for event from PRU */
void disableIntrOnPruEvent(
    int32_t intrNum
);

/* Enables Host interrupt for event from PRU */
void enableIntrOnPruEvent(
    int32_t intrNum
);

#endif /* _APP_PSLCTRL_CFG_MCU_INTR_H_ */
