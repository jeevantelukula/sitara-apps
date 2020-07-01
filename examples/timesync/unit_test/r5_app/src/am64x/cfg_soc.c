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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/csl/csl_intr_router.h>
#include <ti/csl/csl_intaggr.h>
#include <ti/osal/osal.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include "cfg_host_intr.h"
#include "cfg_soc.h"
#include "test_utils.h"

/* Configure interrupt aggregator */
int32_t configureInterruptAggregator(
    int32_t iaLevi,
    int32_t iaSevi,
    int32_t iaVintr,
    int32_t vintrStatusBit
)
{
    int32_t retVal = 0;
    uint32_t vintrBitNum;

    /* Initialize Interrupt Aggregator config structure */
    CSL_IntaggrCfg iaRegs =
    {
        .pCfgRegs       = (CSL_intaggr_cfgRegs *) CSL_DMASS0_INTAGGR_CFG_BASE,
        .pImapRegs      = (CSL_intaggr_imapRegs *) CSL_DMASS0_INTAGGR_IMAP_BASE,
        .pIntrRegs      = (CSL_intaggr_intrRegs *) CSL_DMASS0_INTAGGR_INTR_BASE,
        .pL2gRegs       = (CSL_intaggr_l2gRegs *) CSL_DMASS0_INTAGGR_L2G_BASE,
        .pMcastRegs     = (CSL_intaggr_mcastRegs *) CSL_DMASS0_INTAGGR_MCAST_BASE,
        .pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) CSL_DMASS0_INTAGGR_GCNTCFG_BASE,
        .pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) CSL_DMASS0_INTAGGR_GCNTRTI_BASE,
        .srcEventCnt    = 1536U,
        .virtIntrCnt    = 184U,
        .localEventCnt  = 32U,
        .globalEventCnt = 256U,
        .mcastEventCnt  = 128U
    };

    vintrBitNum = ((iaVintr << 6) & 0x3FC0) | (vintrStatusBit & 0x3F);
    retVal += CSL_intaggrMapEventToLocalEvent(&iaRegs,
                                             iaSevi,
                                             iaLevi,
                                             CSL_INTAGGR_EVT_DETECT_MODE_ACTIVE_HIGH_PULSE);
    retVal += CSL_intaggrMapEventIntr(&iaRegs,
                                      iaSevi,
                                      vintrBitNum);
    retVal += CSL_intaggrClrIntr(&iaRegs, vintrBitNum);
    retVal += CSL_intaggrSetIntrEnable(&iaRegs, vintrBitNum, TRUE);
    if (retVal < 0) {
        return -1;
    }

    return 0;
}

/* Configure interrupts: the R5F core is used to configure the CMP EVT router to direct interrupts
   from ICSSG to all other cores (R5, M4, A53). The R5 also does the configuration of the interrupt
   aggregator needed to route interrupts from CPT EVT router to M4. Each core is then responsible for
   configuring its own interrupt controller (GIC, VIM, NVIC) to accept that interrupt. A disabled
   interrupt at core level has no impact on that cores operation.*/
int32_t configureInterrupts()
{
    int32_t status = 0;

    /* Configure CompareEvent Interrupt Router */
    status = configureCmpEventInterruptRouter(CMPEVT3_INTRTR_IN, CMPEVT3_INTRTR_OUT);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* CompareEvent Interrupt Router CMP5 -> M4 */
    status = configureCmpEventInterruptRouter(CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_5,
                                              32);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Configure Interrupt Aggregator (for M4 interrupt)*/
    status = configureInterruptAggregator(0, 1500, 168, 0);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    return 0;
}
