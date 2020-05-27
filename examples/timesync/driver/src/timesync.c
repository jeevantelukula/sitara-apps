/*
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <string.h>
#include <ti/csl/soc.h>
#include "timesyncFwDefs.h"
#include "timesyncHwRegs.h"
#include "timesync.h"

IcssgTsCtrlObj gIcssgTsCtrlObj; /* IEP TS control object */
IcssgTsObj gIcssgIep0TsObj;     /* IEP 0 TS object */
IcssgTsObj gIcssgIep1TsObj;     /* IEP 1 TS object */
/* ------------------------------------------------------------------------- *
 * External Functions                                                        *
 * ------------------------------------------------------------------------- */

/* Reset TS FW control object */
int32_t resetTsCtrlObj(
    IcssgTsCtrlObj *pIcssgTsCtrlObj
)
{
    uint8_t i;

    for (i = 0; i < ICSSG_NUM_IEP; i++)
    {
        pIcssgTsCtrlObj->iepTsGblEn[i] = FALSE;
    }

    pIcssgTsCtrlObj->pIcssgCfgHwRegs = (CSL_IcssCfgRegs *)ICSS_CFG_BASE;
    return IEP_STS_NERR;
}

/* Reset IEP TS object */
int32_t resetIepTsObj(
    IcssgTsObj *pIcssgTsObj,
    IepId iepId
)
{
    memset(pIcssgTsObj, 0, sizeof(IcssgTsObj));
    pIcssgTsObj->iepId = iepId;
    if (iepId == IEP_ID_0) {
        pIcssgTsObj->pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)ICSS_IEP0_CFG_BASE;
    }
    else if (iepId == IEP_ID_1) {
        pIcssgTsObj->pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)ICSS_IEP1_CFG_BASE;
    }
    else {
        return IEP_STS_ERR_INV_IEP_ID;
    }

    return IEP_STS_NERR;
}

/* Initialize IEP TS */
void initIepTs(
    IcssgTsObj *pIcssgTsObj
)
{
    /* Enable IEP Counter -- this shouldn't be needed when EtherCAT is running! (TBD/FIXME) */
    pIcssgTsObj->pIepHwRegs->GLOBAL_CFG_REG |= 0x1;
}

/* Read IEP and comparator */
void readIepCmp(
    IcssgTsObj *pIcssgTsObj,
    uint32_t   *curIep,
    uint32_t   *curCmp3,
    uint32_t   *curCmp4,
    uint32_t   *curCmp5,
    uint32_t   *curCmp6
)
{
    /* Do IEP first (as soon as possible) for benchmarking */
    if (curIep) {
        *curIep = pIcssgTsObj->pIepHwRegs->COUNT_REG0;
    }
    if (curCmp3) {
        *curCmp3 = pIcssgTsObj->pIepHwRegs->CMP3_REG0;
    }
    if (curCmp4) {
        *curCmp4 = pIcssgTsObj->pIepHwRegs->CMP4_REG0;
    }
    if (curCmp5) {
        *curCmp5 = pIcssgTsObj->pIepHwRegs->CMP5_REG0;
    }
    if (curCmp6) {
        *curCmp6 = pIcssgTsObj->pIepHwRegs->CMP6_REG0;
    }
}
