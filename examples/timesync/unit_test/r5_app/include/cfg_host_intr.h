/*
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _CFG_HOST_INTR_H_
#define _CFG_HOST_INTR_H_

#include <stdint.h>

/* Status codes */
#define CFG_HOST_INTR_ERR_NERR                  (  0 )  /* no error */
#define CFG_HOST_INTR_ERR_CFG_INTR_RTR          ( -1 )  /* interrupt router configuration error */
#define CFG_HOST_INTR_ERR_REG_INTR              ( -2 )  /* interrupt registration error */

/* MAIN2MCU_LVL_INTRTR0 input interrupt */
/* 32: ICSSG_0_HOST_INT0, PRU_ICSSG0 host interrupt 2, see TRM 11.3.3.1 CMPEVT_INTRTR0 Event Map */
#define CMPEVENT_INTRTR0_IN_ICSSG_0_IEP0_CMP_TIMER0      ( 32 )  /* ICSSG_0_HOST_INT0, PRU_ICSSG0 host interrupt 2 */

#define DEF_CMPEVENT_INTRTR0_OUT                ( 16 )  /* COMPEVT_RTR_COMP_16_EVT MAIN2MCU_INTRTR_PLS_IN_BIT0_32 MAIN2MCU_PLS_INTRTR0 */

/* Configures MAIN2MCU_LVL_INTRTR0 interrupt router */
int32_t configureCmpEventInterruptRouter(
    uint16_t intrRtrSrcDevId,
    uint16_t intrRtrSrcIdx,
    uint16_t intrRtrDstDevId,
    uint16_t intrRtrDstIrq,
    uint8_t intrRtrSecHost
);

/* MAIN2MCU_LVL_INTRTR0 input interrupt */
/* 32: ICSSG_0_HOST_INT0, PRU_ICSSG0 host interrupt 2, see TRM 9.4.10 MAIN2MCU_LVL_INTRTR0 Interrupt Map */
#define MAIN2MCU_LVL_INTR0_IN_ICSSG0_HINT2      ( 32 )  /* ICSSG_0_HOST_INT0, PRU_ICSSG0 host interrupt 2 */
#define MAIN2MCU_LVL_INTR0_IN_ICSSG1_HINT2      ( 40 )  /* ICSSG_1_HOST_INT0, PRU_ICSSG1 host interrupt 2 */
#define MAIN2MCU_LVL_INTR0_IN_ICSSG1_HINT3      ( 41 )  /* ICSSG_1_HOST_INT1, PRU_ICSSG1 host interrupt 3 */

/* Default MAIN2MCU_LVL_INTRTR0 input interrupt */
#define DEF_MAIN2MCU_LVL_INTR0_IN               ( MAIN2MCU_LVL_INTR0_IN_ICSSG1_HINT2 )  
#define DEF_MAIN2MCU_LVL_INTR0_OUT              ( 3 ) 

/* R5F input interrupt */
/* See TRM 9.3.3.2 MAIN2MCU_LVL_INTRTR0 Integration:
 * MAIN2MCU_RTR_LVL_MUX_INTR[63:0] is connected to MCU_R5_CORE0_INT_IN[223:160]
 * Therefore we have a offset of 160. [0] <-> [160]
 */
#define DEF_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUT    ( CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_0 + DEF_MAIN2MCU_LVL_INTR0_OUT )

/* Configures MAIN2MCU_LVL_INTRTR0 interrupt router */
int32_t configureMain2MCUInterruptRouter(
    int32_t intrRtrInIntNum, 
    int32_t intrRtrOutIntNum
);

/*
 * Registers an interrupt for event from PRU.
 * MAIN2MCU_LVL_INTRTR0 interrupt router must be configured before this function is called.
 */
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

#endif /* _CFG_PIN_MUX_H_ */
