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

#ifndef _CFG_SOC_H_
#define _CFG_SOC_H_

/* Test ICSSG instance ID */
#define TEST_ICSSG_INST_ID              ( PRUICCSS_INSTANCE_ONE )

/* Test PRU instance ID */
#define TEST_PRU_INST_ID                ( PRUICCSS_RTU0 )

/* Compare event router i*/
#define CMPEVT7_INTRTR_IN    ( 39 )  /* ICSSG_0_IEP0_CMP_TIMER7_INT */

/* Compare event router output */
#define CMPEVT7_INTRTR_OUT   ( 16 )  /* COMPEVT_RTR_COMP_16_EVT */

/* Compare event router ouput R5 interrupt */
#define TS_CMPEVT_INTRTR_R5 ( 224+32 )

/* Compare event router CFG base */
#define CMPEVENT_INTRTR0_INTR_ROUTER_CFG_BASE ( CSL_CMPEVENT_INTRTR0_INTR_ROUTER_CFG_BASE )

/* Configure interrupts */
int32_t configureInterrupts();

#endif /* _CFG_SOC_ */
