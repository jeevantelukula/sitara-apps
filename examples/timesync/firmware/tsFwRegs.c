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

#include <ti/csl/tistdtypes.h>
#include "firmware_version.h"
#include "tsFwRegs.h"

/* TS Info defaults */
#define DEF_FW_MAGIC_NUMBER     ( 0x5354434D )                          /* Firmware Magic Number - 'MCTS' */
#define DEF_FW_TYPE             ( TS_FIRMWARE_TYPE )                    /* Firmware Type */
#define DEF_FW_VERSION          ( TS_FIRMWARE_VERSION )                 /* Firmware Version */
#define DEF_FW_FEATURE          ( TS_FIRMWARE_FEATURE )                 /* Firmware Feature */
#define DEF_FW_EXTENDED_FEATURE ( TS_FIRMWARE_EXTENDED_FEATURE_PTR )    /* Firmware Extended Feature */

/* TS control defaults */
#define DEF_TS_CTRL             ( 0x00000000 )  /* TS disabled */
#define DEF_TS_STAT             ( 0x00000000 )  /* FW uninitialized */
#define DEF_TS_IEP_PRD_NSEC     ( 0x00000005 )  /* TS IEP period in nanoseconds */

/* IEP clock @ 200 MHz */
#define DEF_TS_CMP_COUNT        ( 0 )   /* disabled */

/* IEP clock @ 200 MHz */
#define DEF_TS_CMP3_OFFSET      ( 0 )   /* 0 usec before Sync0 */
#define DEF_TS_CMP4_OFFSET      ( 0 )       /* 0 usec before Sync0 */
#define DEF_TS_CMP5_OFFSET      ( 0 )       /* 0 usec before Sync0 */
#define DEF_TS_CMP6_OFFSET      ( 0 )       /* 0 usec before Sync0 */

/* TS FW register defaults */
#pragma RETAIN(gTsFwRegs)
#pragma DATA_SECTION(gTsFwRegs, ".fwRegs")
TsFwRegs gTsFwRegs = 
{
    {
        DEF_FW_MAGIC_NUMBER,    /* FwMagicNumber */
        DEF_FW_TYPE,            /* FwType */
        DEF_FW_VERSION,         /* FwVersion */
        DEF_FW_FEATURE,         /* FwFeature */
        DEF_FW_EXTENDED_FEATURE /* FwExtendedFeature - Reserved for future use */        
    }, 
    {
        DEF_TS_CTRL,            /* TS_CTRL */
        DEF_TS_STAT,            /* TS_STAT */
        DEF_TS_IEP_PRD_NSEC     /* TS_IEP_NSEC */
    },
    {0, 0, 0, 0, 0, 0, 0, 0},
    {
        DEF_TS_CMP_COUNT,       /* TS_CMP1_COUNT */
        DEF_TS_CMP_COUNT,       /* TS_CMP3_COUNT */
        DEF_TS_CMP_COUNT,       /* TS_CMP4_COUNT */
        DEF_TS_CMP_COUNT,       /* TS_CMP5_COUNT */
        DEF_TS_CMP_COUNT,       /* TS_CMP6_COUNT */
        DEF_TS_CMP3_OFFSET,     /* TS_CMP3_OFFSET */
        DEF_TS_CMP4_OFFSET,     /* TS_CMP4_OFFSET */
        DEF_TS_CMP5_OFFSET,     /* TS_CMP5_OFFSET */
        DEF_TS_CMP6_OFFSET      /* TS_CMP6_OFFSET */
    }
};
