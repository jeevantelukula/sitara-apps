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
    int32_t retVal = CSL_PASS;
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
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

    /* Configure the local to global (L2G) mapping of local event to global event */
    vintrBitNum = ((iaVintr << 6) & 0x3FC0) | (vintrStatusBit & 0x3F);
    /* NOTICE: the following line is subject to change as the sciclient API to configure L2G registers becomes available */
    retVal += CSL_intaggrMapEventToLocalEvent(&iaRegs,
                                             iaSevi,
                                             iaLevi,
                                             CSL_INTAGGR_EVT_DETECT_MODE_ACTIVE_HIGH_PULSE);

    /* Configure the global event to interrupt aggregator output mapping through sciclient call */
    rmIrqReq.valid_params           =  TISCI_MSG_VALUE_RM_IA_ID_VALID
                                      | TISCI_MSG_VALUE_RM_VINT_VALID
                                      | TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID
                                      | TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID
                                      | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;

    rmIrqReq.src_id                 = TISCI_DEV_DMASS0_INTAGGR_0;
    rmIrqReq.ia_id                  = TISCI_DEV_DMASS0_INTAGGR_0;
    rmIrqReq.vint                   = iaVintr;
    rmIrqReq.global_event           = iaSevi;
    rmIrqReq.vint_status_bit_index  = vintrStatusBit;
    rmIrqReq.secondary_host         = (uint8_t)TISCI_HOST_ID_M4_0;
    retVal += Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);

    /* Clear the interrupt aggregator interrupt and enable it */
    retVal += CSL_intaggrClrIntr(&iaRegs, vintrBitNum);
    retVal += CSL_intaggrSetIntrEnable(&iaRegs, vintrBitNum, TRUE);

    return retVal;
}

/* Configure interrupts: the R5F core is used to configure the CMP EVT router to direct interrupts
   from ICSSG to all other cores (R5, M4, A53). The R5 also does the configuration of the interrupt
   aggregator needed to route interrupts from CPT EVT router to M4. Each core is then responsible for
   configuring its own interrupt controller (GIC, VIM, NVIC) to accept that interrupt. A disabled
   interrupt at core level has no impact on that cores operation.*/
int32_t configureInterrupts()
{
    int32_t status = 0;

    /* Configure Compare Event Router ICSSG1_IEP0_CMP7 -> R5F1_0 VIM INT 48 */
    status += configureCmpEventInterruptRouter(TISCI_DEV_PRU_ICSSG1, CMPEVT7_SRC_IDX, TISCI_DEV_R5FSS0_CORE0, CMPEVT7_R5F_VIM_IN, TISCI_HOST_ID_MAIN_0_R5_1);
    /* Configure Compare Event Router ICSSG1_IEP0_CMP8 -> A53 GIC INT 48 */
    status += configureCmpEventInterruptRouter(TISCI_DEV_PRU_ICSSG1, CMPEVT8_SRC_IDX, TISCI_DEV_GICSS0, CMPEVT8_A53_GIC_IN, TISCI_HOST_ID_A53_2);
    /* Configure Compare Event Router ICSSG1_IEP0_CMP9 -> INTAGGR (L2G) input 0 */
    status += configureCmpEventInterruptRouter(TISCI_DEV_PRU_ICSSG1, CMPEVT9_SRC_IDX, TISCI_DEV_DMASS0_INTAGGR_0, CMPEVT9_INTAGGR_IN, TISCI_HOST_ID_M4_0);

    if (status != CFG_HOST_INTR_ERR_NERR) {
        UART_printf("\n\rError=%d", status);
        System_printf("configureInterrupts CmpEventInterruptRouter: Error=%d: ", status);
        System_exit(-1);
    }

    /* Configure Interrupt Aggregator (for M4 interrupt)*/
    status = configureInterruptAggregator(INTAGGR_LOCAL_EVT_NUM, INTAGGR_GLOBAL_EVT_NUM, INTAGGR_VINT_NUM, INTAGGR_VINT_STATUS_BIT);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("configureInterrupts InterruptAggregator: Error=%d: ", status);
        System_exit(-1);
    }

    return 0;
}
