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

#include <xdc/std.h>
#include <ti/csl/csl_intr_router.h>
#include <ti/osal/osal.h>
#include "app_ts_cfg_mcu_intr.h"

/* CMPEVENT_INTRTR0 number of input interrupts */
#define NUM_CMPEVENT_INTRTR0_IN          ( 128 )
/* CMPEVENT_INTRTR0 number of output interrupts */
#define NUM_CMPEVENT_INTRTR0_OUT         ( 32 )

/* MAIN2MCU_LVL_INTRTR0 number of input interrupts */
#define NUM_MAIN2MCU_LVL_INTR0_IN        ( 192 )
/* MAIN2MCU_LVL_INTRTR0 number of output interrupts */
#define NUM_MAIN2MCU_LVL_INTR0_OUT       ( 64 )

HwiP_Handle hwiHandle = NULL;

/* Configures Compare Event router */
int32_t appTs_configureCmpEventInterruptRouter(
    int32_t intrRtrInIntNum,
    int32_t intrRtrOutIntNum
)
{
    CSL_IntrRouterCfg intrRouterCmpEventCfg;
    int32_t retVal;

    /* Initialize Main to MCU Interrupt Router config structure */
    intrRouterCmpEventCfg.pIntrRouterRegs = (CSL_intr_router_cfgRegs *)(uintptr_t)(CSL_CMPEVENT_INTRTR0_INTR_ROUTER_CFG_BASE);
    intrRouterCmpEventCfg.pIntdRegs       = (CSL_intr_router_intd_cfgRegs *)NULL;
    intrRouterCmpEventCfg.numInputIntrs   = NUM_CMPEVENT_INTRTR0_IN;
    intrRouterCmpEventCfg.numOutputIntrs  = NUM_CMPEVENT_INTRTR0_OUT;

    /* Route PRU INTC output interrupt to MAIN2MCU LEVEL interrupt router output */
    retVal = CSL_intrRouterCfgMux(&intrRouterCmpEventCfg, intrRtrInIntNum, intrRtrOutIntNum);
    if (retVal < 0) {
        return APP_TS_CFG_MCU_INTR_SERR_CFG_INTR_RTR;
    }

    return APP_TS_CFG_MCU_INTR_SOK;
}

/* Registers an interrupt for event from compare event router */
int32_t appTs_registerIntrOnCmpEvent(
    int32_t intrNum,
    uint32_t priority,     
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
    interruptRegParams.corepacConfig.priority = priority;
    interruptRegParams.corepacConfig.name = NULL;
    interruptRegParams.corepacConfig.corepacEventNum = intrNum;
    interruptRegParams.corepacConfig.intVecNum = intrNum; /* Host Interrupt vector */
    /* Register interrupt */
    retCode = Osal_RegisterInterrupt(&interruptRegParams, &(hwiHandle));
    if (retCode != osal_OK)
    {
        return APP_TS_CFG_MCU_INTR_SERR_REG_INTR;
    }

    /* Disable and clear interrupts */
    Osal_DisableInterrupt(intrNum, intrNum);
    Osal_ClearInterrupt(intrNum, intrNum);

    return APP_TS_CFG_MCU_INTR_SOK;
}

/* Disables interrupt for event from PRU */
void appTs_disableIntrOnPruEvent(
    int32_t intrNum
)
{
    /* Enable the interrupt */
    Osal_DisableInterrupt(intrNum, intrNum);
}

/* Enables interrupt for event from PRU */
void appTs_enableIntrOnPruEvent(
    int32_t intrNum
)
{
    /* Enable the interrupt */
    Osal_EnableInterrupt(intrNum, intrNum);
}
