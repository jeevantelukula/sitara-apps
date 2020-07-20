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

#ifndef _APP_TS_H_
#define _APP_TS_H_

#include <ti/csl/tistdtypes.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include "timesyncDrv_api.h"

/* Status codes */
#define APP_TS_SOK                      (  0 )  /* no error */
#define APP_TS_SERR_INV_PRMS            ( -1 )  /* invalid parameters */
#define APP_TS_SERR_CFG_HOST_INTR       ( -2 )  /* interrupt configuration error */
#define APP_TS_SERR_INIT_PRU            ( -3 )  /* initialize PRU error */
#define APP_TS_SERR_INIT_TS_DRV         ( -4 )  /* initialize TS DRV error */
#define APP_TS_SERR_START_TS            ( -5 )  /* start TS error */
#define APP_TS_SERR_INIT                ( -6 )  /* initialize TS error */ 

/* Bits for TS CMP configuration mask */
#define TS_CFG_CMP7                     ( 1<<0 )
#define TS_CFG_CMP8                     ( 1<<1 )
#define TS_CFG_CMP9                     ( 1<<2 )
#define TS_CFG_CMP10                    ( 1<<3 )
#define TS_CFG_CMP_ALL  \
    ( TS_CFG_CMP7 | TS_CFG_CMP8 | TS_CFG_CMP9 | TS_CFG_CMP10 )

/* Time Sync configuration parameters */
typedef struct TsPrmsObj_s {
    PRUICSS_MaxInstances icssInstId;    /* ICSSG hardware instance ID */
    PRUSS_PruCores pruInstId;           /* PRU hardware instance ID */

    /* Period Count */
    uint32_t prdCount[ICSSG_TS_DRV__NUM_IEP_CMP];
    /* Positive/Negative Offset */
    int32_t prdOffset[ICSSG_TS_DRV__NUM_IEP_CMP];
    /* Compare Event Router input & output IDs */
    int32_t cmpEvtRtrInIntNum[ICSSG_TS_DRV__NUM_IEP_CMP];
    int32_t cmpEvtRtrOutIntNum[ICSSG_TS_DRV__NUM_IEP_CMP];
    /* Period/Offset configuration mask */
    uint8_t prdCfgMask;
} TsPrmsObj;

/* Time Sync control object */
typedef struct TsCtrlObj_s {
    PRUICSS_Handle pruIcssHandle;           /* PRUSS DRV handle */
    TsPrmsObj tsPrms;                       /* TS configuration parameters */
    uint8_t icssgId;                        /* TS DRV ICSSG hardware module ID */
    uint8_t pruId;                          /* TS DRV PRU hardware module ID */
    IcssgTsDrv_Handle hTsDrv;               /* TS DRV handle */
} TsObj;

/* Get ICSSG ID for PWM DRV */
int32_t appTs_getIcssgId(
    PRUICSS_MaxInstances icssInstId,
    uint8_t *pIcssgId
);

/* Get PRU ID for PWM DRV */
int32_t appTs_getPruId(
    PRUSS_PruCores pruInstId,
    uint8_t *pPruId
);

/* Initialize PRU for TS */
int32_t appTs_initPruTs(
    PRUICSS_Handle pruIcssHandle,
    PRUSS_PruCores pruInstId
);

/* Initialize ICSSG TS DRV */
int32_t appTs_initIcssgTsDrv(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTs
);

/* Start ICSSG TS */
int32_t appTs_startTs(
    TsObj *pTs
);

/* Initialize Time Sync */
int32_t appTs_initTs(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTs
);

#endif /* _APP_TS_H_ */
