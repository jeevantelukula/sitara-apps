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

#ifndef _CFG_MCU_INTR_H_
#define _CFG_MCU_INTR_H_

#include <ti/csl/tistdtypes.h>
#include <ti/csl/arch/csl_arch.h>

/* Status codes */
#define CFG_MCU_INTR_SOK                      (  0 )  /* no error */
#define CFG_MCU_INTR_SERR_INV_GROUP_ID        ( -1 )  /* invalid Group ID for processors groups within system */
#define CFG_MCU_INTR_SERR_INV_ID              ( -2 )  /* invalid MCU interrupt ID */
#define CFG_MCU_INTR_SERR_CFG_INTR_RTR        ( -3 )  /* interrupt router configuration error */
#define CFG_MCU_INTR_SERR_REG_INTR            ( -4 )  /* interrupt registration error */

#define NUM_MCU_INTR                          ( 5 )   /* number of MUC interrupts which can be configured */
#define MCU_INTR_IDX(x)                       ( x )   /* MCU interrupt index */

/* MCU interrupt registration parameters */
typedef struct McuIntrRegPrms_s {
    int32_t intrNum;                    /* VIM interrupt number */
    CSL_VimIntrType intrType;           /* VIM interrupt type {pulse, level} */
    CSL_VimIntrMap intrMap;             /* VIM interrupt map {IRQ, FIQ} */
    uint32_t intrPri;                   /* VIM interrupt priority (0(highest)..15(lowest)) */
    void (*isrRoutine)(void);           /* ISR routine */
} McuIntrRegPrms;

/* MCU MAIN2MCU interrupt routing parameters */
typedef struct McuIntrRtrPrms_s {
    uint16_t    tisciSrcId;             /* TISCI device source ID */
    uint16_t    tisciSrcIndex;          /* TISCI device source IRQ index */
} McuIntrRtrPrms;

/* Initialize MCU INTC */
int32_t McuIntc_Init(void);

/* Register MCU interrupt */
int32_t McuIntc_registerIntr(
    McuIntrRegPrms *pMcuIntrRegPrms,    /* MCU interrupt registration parameters */
    uint8_t mcuIntrIdx                  /* MCU interrupt index */
);

/* Configure MCU interrupt */
int32_t McuIntc_cfgIntr(
    McuIntrRegPrms *pMcuIntrRegPrms,    /* MCU interrupt registration parameters */
    McuIntrRtrPrms *pMcuIntrRtrPrms,    /* MCU interrupt MAIN2MCU interrupt router parameters */
    uint8_t mcuIntrIdx                  /* MCU interrupt index */
);

/* Enable MCU interrupt */
int32_t McuIntc_enableIntr(
    uint8_t mcuIntrIdx                  /* MCU interrupt index (0..NUM_MCU_INTR) */
);

/* VIM registers base address */
extern uint32_t gVimRegsBaseAddr;

#endif /* _CFG_MCU_INTR_H_ */
