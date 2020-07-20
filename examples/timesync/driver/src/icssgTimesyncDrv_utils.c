/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the
 *        distribution.
 *
 *      * Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
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

#include <ti/csl/soc.h>
#include <ti/csl/cslr_icss.h>
#include "icssgTimesyncDrv.h"
#include "timesyncDrv_api.h"

/* Start IEP0 counter */
void icssgTsDrv_startIepCount(
    IcssgTsDrv_Handle handle
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    CSL_icss_g_pr1_iep1_slvRegs *pIepHwRegs;

    /* Get pointer to IEP0 CMP hardware registers */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pIepHwRegs = pTsDrv->tsCmpCtrl.pIepHwRegs;

    /* Enable IEP Counter -- this shouldn't be needed when EtherCAT is running! (TBD/FIXME) */
    pIepHwRegs->GLOBAL_CFG_REG |= 0x1;
}

/* Read IEP and comparator */
void icssgTsDrv_readIepCmp(
    IcssgTsDrv_Handle handle,
    uint32_t   *curIep,
    uint32_t   *curCmp7,
    uint32_t   *curCmp8,
    uint32_t   *curCmp9,
    uint32_t   *curCmp10
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    CSL_icss_g_pr1_iep1_slvRegs *pIepHwRegs;
        
    /* Get pointer to IEP0 CMP hardware registers */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pIepHwRegs = pTsDrv->tsCmpCtrl.pIepHwRegs;

    /* Do IEP first (as soon as possible) for benchmarking */
    if (curIep) {
        *curIep = pIepHwRegs->COUNT_REG0;
    }
    if (curCmp7) {
        *curCmp7 = pIepHwRegs->CMP7_REG0;
    }
    if (curCmp8) {
        *curCmp8 = pIepHwRegs->CMP8_REG0;
    }
    if (curCmp9) {
        *curCmp9 = pIepHwRegs->CMP9_REG0;
    }
    if (curCmp10) {
        *curCmp10 = pIepHwRegs->CMP10_REG0;
    }
}

