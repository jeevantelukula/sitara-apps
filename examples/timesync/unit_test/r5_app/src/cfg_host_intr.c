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
#include <ti/csl/csl_intr_router.h>
#include <ti/osal/osal.h>
#include "cfg_host_intr.h"
#include "cfg_soc.h"

/* CMPEVENT_INTRTR0 number of input interrupts */
#define NUM_CMPEVENT_INTRTR0_IN          ( 128 )
/* CMPEVENT_INTRTR0 number of output interrupts */
#define NUM_CMPEVENT_INTRTR0_OUT         ( 40 )

/* MAIN2MCU_LVL_INTRTR0 number of input interrupts */
#define NUM_MAIN2MCU_LVL_INTR0_IN        ( 192 )
/* MAIN2MCU_LVL_INTRTR0 number of output interrupts */
#define NUM_MAIN2MCU_LVL_INTR0_OUT       ( 64 )
/* Interrupt priority of timesync App */
#define INTERRUPT_PRIORITY               ( 10 )

HwiP_Handle hwiHandle = NULL;

/* Configures Compare Event router */
int32_t configureCmpEventInterruptRouter(
    uint16_t intrRtrSrcDevId,
    uint16_t intrRtrSrcIdx,
    uint16_t intrRtrDstDevId,
    uint16_t intrRtrDstIrq,
    uint8_t intrRtrSecHost
)
{
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    int32_t retVal = 0;

    rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID
                                  | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID
                                  | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    rmIrqReq.src_id         = intrRtrSrcDevId;
    rmIrqReq.src_index      = intrRtrSrcIdx;
    rmIrqReq.dst_id         = intrRtrDstDevId;
    rmIrqReq.dst_host_irq   = intrRtrDstIrq;
    rmIrqReq.secondary_host = intrRtrSecHost;

    /* Config event */
    retVal = Sciclient_rmIrqSet(
                 &rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);

    if(retVal != CSL_PASS)
        return CFG_HOST_INTR_ERR_CFG_INTR_RTR;
    else
        return CFG_HOST_INTR_ERR_NERR;
}

/*
 * Registers an interrupt for event from compare event router
 * CMPEVENT_INTRTR0 interrupt router must be configured before this function is called.
 */
int32_t registerIntrOnCmpEvent(
    int32_t intrNum,
    void (*isrRoutine)(uintptr_t) /* The ISR routine to hook the corepacEventNum to */
)
{
    /* Register interrupt from PRU */
    OsalInterruptRetCode_e retCode;

    OsalRegisterIntrParams_t interruptRegParams;
    /* Initialize interrupt with defaults */
    Osal_RegisterInterrupt_initParams(&interruptRegParams);
    interruptRegParams.corepacConfig.triggerSensitivity = OSAL_ARM_GIC_TRIG_TYPE_EDGE;
    interruptRegParams.corepacConfig.isrRoutine = isrRoutine;
    /* R5F VIM and TIRTOS support only 16 priority levels (0-15) */
    interruptRegParams.corepacConfig.priority = INTERRUPT_PRIORITY;
    interruptRegParams.corepacConfig.name = NULL;
    interruptRegParams.corepacConfig.corepacEventNum = intrNum;
    interruptRegParams.corepacConfig.intVecNum = intrNum; /* Host Interrupt vector */
    /* Register interrupt */
    retCode = Osal_RegisterInterrupt(&interruptRegParams, &(hwiHandle));
    if (retCode != osal_OK)
    {
        return CFG_HOST_INTR_ERR_REG_INTR;
    }

    /* Disable and clear interrupts */
    Osal_DisableInterrupt(intrNum, intrNum);
    Osal_ClearInterrupt(intrNum, intrNum);

    return CFG_HOST_INTR_ERR_NERR;
}

/* Disables interrupt for event from PRU */
void disableIntrOnPruEvent(
    int32_t intrNum
)
{
    /* Enable the interrupt */
    Osal_DisableInterrupt(intrNum, intrNum);
}

/* Enables interrupt for event from PRU */
void enableIntrOnPruEvent(
    int32_t intrNum
)
{
    /* Enable the interrupt */
    Osal_EnableInterrupt(intrNum, intrNum);
}
