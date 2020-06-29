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

#ifndef _ICSSG_TIMESYNC_DRV_H_
#define _ICSSG_TIMESYNC_DRV_H_

#include <ti/csl/tistdtypes.h>
#include "tsFwRegs.h"

/* TS info object */
typedef struct IcssgTsDrv_TsInfoObj_s
{
    /* TS info firmware registers */
    TsInfoFwRegs *pTsInfoFwRegs;
} IcssgTsDrv_TsInfoObj;

/* TS control object */
typedef struct IcssgTsDrv_TsCtrlObj_s
{
    /* TS control firmware registers */
    TsCtrlFwRegs *pTsCtrlFwRegs;
} IcssgTsDrv_TsCtrlObj;

/* TS IEP0 CMP control object */
typedef struct IcssgTsDrv_TsCmpCtrlObj_s
{
    /* TS CMP firmware registers */
    TsCmpFwRegs *pTsCmpFwRegs;
    /* IEP0 CMP hardware registers */
    CSL_icss_g_pr1_iep1_slvRegs *pIepHwRegs;
} IcssgTsDrv_TsCmpCtrlObj;

/* TS DRV object */
typedef struct IcssgTsDrv_TsDrvObj_s
{
    /* ICSSG hardware module ID */
    uint8_t                 icssgId;
    /* PRU hardware module ID */
    uint8_t                 pruId;
    /* TS info */
    IcssgTsDrv_TsInfoObj    tsInfo;
    /* TS control */
    IcssgTsDrv_TsCtrlObj    tsCtrl;
    /* TS IEP0/CMP control */
    IcssgTsDrv_TsCmpCtrlObj tsCmpCtrl;
} IcssgTsDrv_TsDrvObj;

#endif /* _ICSSG_TS_DRV_H_ */
