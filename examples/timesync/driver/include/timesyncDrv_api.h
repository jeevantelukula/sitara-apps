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
#define ICSSG_TS_DRV__STS_NERR         ( 0 )   /* no error */
#define ICSSG_TS_DRV__STS_ERR_INV_PRM  ( 1 )   /* error, invalid parameters */

#define ICSSG_TS_DRV__ICSSG_ID_0       ( 0 )   /* ICSSG0 hardware module ID */
#define ICSSG_TS_DRV__ICSSG_ID_1       ( 1 )   /* ICSSG1 hardware module ID */
#define ICSSG_TS_DRV__ICSSG_ID_2       ( 2 )   /* ICSSG2 hardware module ID */
#define ICSSG_TS_DRV__NUM_ICSSG        ( 3 )   /* AM65xx number of ICSSG */

#define ICSSG_TS_DRV__PRU_ID_0         ( 0 )   /* PRU0 hardware module ID */
#define ICSSG_TS_DRV__PRU_ID_1         ( 1 )   /* PRU1 hardware module ID */
#define ICSSG_TS_DRV__RTU_ID_0         ( 2 )   /* RTU0 hardware module ID */
#define ICSSG_TS_DRV__RTU_ID_1         ( 3 )   /* RTU1 hardware module ID */
#define ICSSG_TS_DRV__TPRU_ID_0        ( 4 )   /* TPRU0 hardware module ID */
#define ICSSG_TS_DRV__TPRU_ID_1        ( 5 )   /* TPRU1 hardware module ID */
#define ICSSG_TS_DRV__NUM_PRU          ( 6 )   /* ICSSG number of ICSSG PRUs */

#define ICSSG_TS_DRV__SLICE_PRU_ID_0   ( 0 )   /* Slice PRU0 ID */
#define ICSSG_TS_DRV__SLICE_PRU_ID_1   ( 1 )   /* Slice PRU1 ID */
#define ICSSG_NUM_SLICE                ( 2 )   /* ICSSG number of Slices */

#define ICSSG_TS_DRV__IEP_ID_0         ( 0 )   /* IEP0 hardware module ID */
#define ICSSG_TS_DRV__IEP_ID_1         ( 1 )   /* IEP1 hardware module ID */
#define ICSSG_TS_DRV__ICSSG_NUM_IEP    ( 2 )   /* ICSSG number of IEPs */

/* Shifts & Masks for icssgTsDrv_setIepTsGblEn(), iepTsGblEnMask */
#define ICSSG_TS_DRV__IEP_TS_GBL_EN_DISABLE               ( 0 )   /* Global Enable, disable setting */
#define ICSSG_TS_DRV__IEP_TS_GBL_EN_ENABLE                ( 1 )   /* Global Enable, enable setting */
#define ICSSG_TS_DRV__BF_IEP_TS_GBL_EN_MASK               ( 0x1 )
#define ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_SHIFT             ( 0 )
#define ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_MASK              ( ICSSG_TS_DRV__BF_IEP_TS_GBL_EN_MASK << ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_SHIFT )
#define ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_SHIFT             ( 1 )
#define ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_MASK              ( ICSSG_TS_DRV__BF_IEP_TS_GBL_EN_MASK << ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_SHIFT )

/* Shifts & Masks for icssgTsDrv_waitIepTsGblEnAck(), iepTsGblEnAckMask */
#define ICSSG_TS_DRV__IEP_TS_GBL_EN_ACK_DISABLE           ( 0 )   /* Global Enable ACK, disable setting */
#define ICSSG_TS_DRV__IEP_TS_GBL_EN_ACK_ENABLE            ( 1 )   /* Global Enable ACK, enable setting */
#define ICSSG_TS_DRV__BF_IEP_TS_GBL_EN_ACK_MASK           ( 0x1 )
#define ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_ACK_SHIFT         ( 0 )
#define ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_ACK_MASK          ( ICSSG_TS_DRV__BF_IEP_TS_GBL_EN_ACK_MASK << ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_ACK_SHIFT )
#define ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_ACK_SHIFT         ( 1 )
#define ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_ACK_MASK          ( ICSSG_TS_DRV__BF_IEP_TS_GBL_EN_ACK_MASK << ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_ACK_SHIFT )

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
 *  @name   icssgTsDrv_setIepTsGblEn
 *  @brief  Set IEP PWM Global Enable flags
 *
 *  @param[in]  handle              PWM DRV instance handle
 *  @param[in]  iepTsGblEnMask     IEP PWM global enable mask: BitX, X=0...1: global disable/enable flag for IEP<X> PWMs
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_setIepTsGblEn(
    IcssgTsDrv_Handle handle,
    uint8_t iepTsGblEnMask
);

/**
 *  @name   icssgTsDrv_waitFwInit
 *  @brief  Wait for FW initialization completion
 *
 *  @param[in]  handle      TS DRV instance handle
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_waitFwInit(
    IcssgTsDrv_Handle handle
);

/**
 *  @name   icssgTsDrv_prepRecfgTsPrdCount
 *  @brief  Prepare IEP Period Count reconfiguration
 *
 *  @param[in]  handle          TS DRV instance handle
 *  @param[in]  iepId           IEP hardware module ID, 0...ICSSG_TS_DRV__ICSSG_NUM_IEP-1
 *  @param[in]  tsPrdCount      Compare periods/reloads (nPrdCount)
 *  @param[in]  tsPrdOffset     Compare offsets         (nPrdCount-1)
 *  @param[in]  nPrdCount       TS Period Count
 *  @param[out] pRecfgBf        Pointer to mask for TS Period Count reconfiguration request.
 *
 *  @retval Status code
 *
 */
int32_t icssgTsDrv_prepRecfgTsPrdCount(
    IcssgTsDrv_Handle handle,
    uint8_t iepId,
    uint32_t tsPrdCount[],
    int32_t tsPrdOffset[],
    uint8_t  nPrdCount,
    uint32_t *pRecfgBf
);

#endif /* _TIMESYNC_DRV_API_H_ */
