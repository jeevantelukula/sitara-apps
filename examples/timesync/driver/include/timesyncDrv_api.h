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

#ifndef _TIMESYNC_DRV_API_H_
#define _TIMESYNC_DRV_API_H_

#include <stdint.h>

#include "icssg_timesync.h"

/* Return status codes */
#define ICSSG_TS_DRV__STS_NERR          ( 0 )   /* no error */
#define ICSSG_TS_DRV__STS_ERR_INV_PRM   ( 1 )   /* error, invalid parameters */

#define ICSSG_TS_DRV__ICSSG_ID_0        ( 0 )   /* ICSSG0 hardware module ID */
#define ICSSG_TS_DRV__ICSSG_ID_1        ( 1 )   /* ICSSG1 hardware module ID */
#define ICSSG_TS_DRV__ICSSG_ID_2        ( 2 )   /* ICSSG2 hardware module ID */
#define ICSSG_TS_DRV__NUM_ICSSG         ( 3 )   /* AM65xx number of ICSSG */

#define ICSSG_TS_DRV__PRU_ID_0          ( 0 )   /* PRU0 hardware module ID */
#define ICSSG_TS_DRV__PRU_ID_1          ( 1 )   /* PRU1 hardware module ID */
#define ICSSG_TS_DRV__RTU_ID_0          ( 2 )   /* RTU0 hardware module ID */
#define ICSSG_TS_DRV__RTU_ID_1          ( 3 )   /* RTU1 hardware module ID */
#define ICSSG_TS_DRV__TPRU_ID_0         ( 4 )   /* TPRU0 hardware module ID */
#define ICSSG_TS_DRV__TPRU_ID_1         ( 5 )   /* TPRU1 hardware module ID */
#define ICSSG_TS_DRV__NUM_PRU           ( 6 )   /* ICSSG number of PRUs */

#define ICSSG_TS_DRV__NUM_ICSSG_SLICE   ( 2 )   /* ICSSG number of ICSSG Slices */

#define ICSSG_TS_DRV__NUM_IEP_CMP       ( 4 )   /* Number of TS IEP CMP */

/* Settings for icssgTsDrv_setTsGblEn() */
#define ICSSG_TS_DRV__TS_GBL_EN_DISABLE             ( 0 )   /* Global Enable, disable setting */
#define ICSSG_TS_DRV__TS_GBL_EN_ENABLE              ( 1 )   /* Global Enable, enable setting */

/* Settings for icssgTsDrv_waitTsGblEnAck() */
#define ICSSG_TS_DRV__TS_GBL_EN_ACK_DISABLE         ( 0 )   /* Global Enable ACK, disable setting */
#define ICSSG_TS_DRV__TS_GBL_EN_ACK_ENABLE          ( 1 )   /* Global Enable ACK, enable setting */

/* Settings for icssgTsDrv_waitFwInit() */
#define ICSSG_TS_DRV__TS_FW_INIT_UNINIT             ( 0 )   /* FW init, uninitialized setting */
#define ICSSG_TS_DRV__TS_FW_INIT_INIT               ( 1 )   /* FW init, initialized setting */

/* FW register base address */
#define ICSSG_TS_BASE_ADDR  ( ICSSG_TS_FW_REGS_BASE )

/* ICSSG TS DRV handle */
typedef struct IcssgTsDrv_TsDrvObj *IcssgTsDrv_Handle;

/**
 *  @name   icssgTsDrv_initDrv
 *  @brief  Initialize TS DRV instance
 *
 *  @param[in]  icssgId     ICSSG hardware module ID, 0...ICSSG_TS_DRV__NUM_ICSSG-1
 *  @param[in]  pruId       ICSSG hardware module ID, 0...ICSSG_TS_DRV__NUM_PRU-1
 *
 *  @retval TS DRV instance handle
 *
 */
IcssgTsDrv_Handle icssgTsDrv_initDrv(
    uint8_t icssgId,
    uint8_t pruId
);

/**
 *  @name   icssgTsDrv_setTsGblEn
 *  @brief  Set TS Global Enable
 *
 *  @param[in]  handle              TS DRV instance handle
 *  @param[in]  tsGblEnFlag         TS global enable flag: 0/1 - disable/enable
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_setTsGblEn(
    IcssgTsDrv_Handle handle,
    uint8_t tsGblEnFlag
);

/**
 *  @name   icssgTsDrv_waitTsGblEnAck
 *  @brief  Wait for TS Global Enable ACK
 *
 *  @param[in]  handle              TS DRV instance handle
 *  @param[in]  tsGblEnFlag         TS global enable ACK flag: 0/1 - disable/enable
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_waitTsGblEnAck(
    IcssgTsDrv_Handle handle,
    uint8_t tsGblEnAckFlag
);

/**
 *  @name   icssgTsDrv_waitFwInit
 *  @brief  Wait for FW initialization flag
 *
 *  @param[in]  handle              TS DRV instance handle
 *  @param[in]  tsGblEnFlag         TS FW init flag: 0/1 - uninit/init
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_waitFwInit(
    IcssgTsDrv_Handle handle, 
    uint8_t fwInitFlag
);

/**
 *  @name   icssgTsDrv_cfgTsPrdCount
 *  @brief  Configure TS IEP0 Period Counts & Offsets
 *
 *  @param[in]  handle          TS DRV instance handle
 *  @param[in]  tsPrdCount      Compare periods/reloads (ICSSG_TS_DRV__NUM_IEP_CMP)
 *  @param[in]  tsPrdOffset     Compare offsets         (ICSSG_TS_DRV__NUM_IEP_CMP)
 *  @param[in]  cfgBf           Mask for TS Period Count & Offset configuration request: Bit X: CMP(3+X), 0<=X<=3
 *  @param[in]  testTsPrdCount  Test compare period, CMP1 period if not zero
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_cfgTsPrdCount(
    IcssgTsDrv_Handle handle,
    uint32_t tsPrdCount[],
    int32_t tsPrdOffset[],
    uint32_t cfgBf,
    uint32_t testTsPrdCount
);

#endif /* _TIMESYNC_DRV_API_H_ */
