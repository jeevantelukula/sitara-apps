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
 
#ifndef _TS_FW_REGS_H_
#define _TS_FW_REGS_H_

#include <ti/csl/tistdtypes.h>

/* TS Firmware information registers */
typedef struct TsInfoFwRegs_s
{
    volatile uint32_t  FwMagicNumber;       /* Firmware Magic Number Fw Reg */
    volatile uint32_t  FwType;              /* Firmware Type Information Fw Reg */
    volatile uint32_t  FwVersion;           /* Firmware Version Information Fw Reg */
    volatile uint32_t  FwFeature;           /* Firmware Feature Information Fw Reg */
    volatile uint32_t  FwExtendedFeature;   /* Firmware Extended Feature Information Fw Reg */
} TsInfoFwRegs;

/* TS Firmware Control/Status registers */
typedef struct TsCtrlFwRegs_s
{
    volatile uint32_t  TS_CTRL;             /* TS Control Fw Reg */
    volatile uint32_t  TS_STAT;             /* TS Status Fw Reg */
    volatile uint32_t  TS_IEP_PRD_NSEC;     /* TS IEP period in nsec */
} TsCtrlFwRegs;

/* TS Firmware Compare registers */
typedef struct TsCmpFwRegs_s
{
    volatile uint32_t  TS_CMP1_COUNT;       /* TS CMP1 Fw Reg */
    volatile uint32_t  TS_CMP3_COUNT;       /* TS CMP3 Fw Reg Count Fw 1 */
    volatile uint32_t  TS_CMP4_COUNT;       /* TS CMP4 Fw Reg Count Fw Reg */
    volatile uint32_t  TS_CMP5_COUNT;       /* TS CMP5 Fw Reg Count Fw Reg */
    volatile uint32_t  TS_CMP6_COUNT;       /* TS CMP6 Fw Reg Count Fw Reg */
    volatile int32_t   TS_CMP3_OFFSET;      /* TS CMP3 Fw Reg Offset Fw Reg */
    volatile int32_t   TS_CMP4_OFFSET;      /* TS CMP4 Fw Reg Offset Fw Reg */
    volatile int32_t   TS_CMP5_OFFSET;      /* TS CMP5 Fw Reg Offset Fw Reg */
    volatile int32_t   TS_CMP6_OFFSET;      /* TS CMP6 Fw Reg Offset Fw Reg */
} TsCmpFwRegs;

/* TS Firmware registers */
typedef struct TsFwRegs_s
{
    TsInfoFwRegs    tsInfoFwRegs;
    TsCtrlFwRegs    tsCtrlFwRegs;   
    TsCmpFwRegs     tsCmpFwRegs;
} TsFwRegs;

extern TsFwRegs gTsFwRegs;

#endif /* _TS_FW_REGS_H_ */
