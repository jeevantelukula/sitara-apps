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

#ifndef _TIMESYNC_FW_REGS_H_
#define _TIMESYNC_FW_REGS_H_

#include <ti/csl/cslr_icss.h>

/* TS Firmware information registers */
typedef struct TsInfoFwRegs_s
{
    volatile uint32_t  FwMagicNumber;         /* Firmware Magic Number Fw Reg */
    volatile uint32_t  FwType;                /* Firmware Type Information Fw Reg */
    volatile uint32_t  FwVersion;             /* Firmware Version Information Fw Reg */
    volatile uint32_t  FwFeature;             /* Firmware Feature Information Fw Reg */
    volatile uint32_t  FwExtendedFeature;     /* Firmware Extended Feature Information Fw Reg */
} TsInfoFwRegs;

/* TS Firmware Control/Status registers */
typedef struct TsCtrlFwRegs_s
{
    volatile uint32_t  TS_CTRL;              /* TS Control Fw Reg */
    volatile uint32_t  TS_STAT;              /* TS Status Fw Reg */
} TsCtrlFwRegs;

/* TS Firmware registers per IEP */
typedef struct IepTsFwRegs_s
{
    volatile uint32_t  TS_RECFG;         /* TS Reconfiguration Fw Reg */
    volatile uint32_t  TS_MODE;          /* TS Mode Fw Reg */
    volatile uint32_t  TS_EN;            /* TS Enable Fw Reg */
    volatile uint32_t  TS_PRD_COUNT[5];  /* TS Period Count Fw Reg */
    volatile int32_t   TS_PRD_OFFSET[4]; /* Signed offset from TS_PRD_COUNT to start firing */
} IepTsFwRegs;

/* FW registers */
extern const TsInfoFwRegs gTsInfoFwRegs;
extern const TsCtrlFwRegs gTsCtrlFwRegs;
extern const IepTsFwRegs gIep0TsFwRegs;
extern const IepTsFwRegs gIep1TsFwRegs;

#endif /* _TIMESYNC_FW_REGS_H_ */
