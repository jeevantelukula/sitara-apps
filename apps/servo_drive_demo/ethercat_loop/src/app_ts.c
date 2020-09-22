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

#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include <app_log.h>
#include "timesync_array.h"     /* TS PRU FW image data */
#include "timesyncDrv_utils.h"  /* TS driver utilities */
#include "timesyncDrv_api.h"    /* TS driver utilities */
#include "app_ts_cfg_mcu_intr.h"
#include "app_ts.h"

/*
 *  ======== appTs_getIcssgId ========
 */
/* Get ICSSG ID for TS DRV */
int32_t appTs_getIcssgId(
    PRUICSS_MaxInstances icssInstId,
    uint8_t *pIcssId
)
{
    /* Translate ICSSG hardware module ID to TS API */
    if (icssInstId == PRUICSS_INSTANCE_ONE) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_0;
        return APP_TS_SOK;
    }
    else if (icssInstId == PRUICSS_INSTANCE_TWO) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_1;
        return APP_TS_SOK;
    }
    else if (icssInstId == PRUICSS_INSTANCE_MAX) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_2;
        return APP_TS_SOK;
    }
    else {
        *pIcssId = 0;
        return APP_TS_SERR_INV_PRMS;
    }
}

/*
 *  ======== appTs_getPruId ========
 */
/* Get ICSSG ID for TS DRV */
int32_t appTs_getPruId(
    PRUSS_PruCores pruInstId,
    uint8_t *pPruId
)
{
    int32_t retVal = 0;
    
    /* Translate PRU hardware module ID to TS API */
    switch (pruInstId)
    {
        /* 
           TS firmware co-exists w/ EthCAT firmware.
           EthCAT firmware uses PRU0/1, 
           so TS firmware can only execute on RTU0/1 & TX_PRU0/1 
        */
        case PRUICSS_RTU0:
            *pPruId = ICSSG_TS_DRV__RTU_ID_0;
            retVal = APP_TS_SOK;
            break;
        case PRUICSS_RTU1:
            *pPruId = ICSSG_TS_DRV__RTU_ID_1;
            retVal = APP_TS_SOK;
            break;
        case PRUICSS_TPRU0:
            *pPruId = ICSSG_TS_DRV__TPRU_ID_0;
            retVal = APP_TS_SOK;
            break;
        case PRUICSS_TPRU1:
            *pPruId = ICSSG_TS_DRV__TPRU_ID_1;
            retVal = APP_TS_SOK;
            break;
        default:
            *pPruId = 0;
            retVal = APP_TS_SERR_INV_PRMS;
            break;
    }
    
    return retVal;
}

/*
 *  ======== appTs_initPruTs ========
 */
/* Initialize PRU for timesync */
int32_t appTs_initPruTs(
    PRUICSS_Handle pruIcssHandle,
    PRUSS_PruCores pruInstId
)
{
    uint8_t slicePruInstId;
    uint32_t pruDMem, pruIMem;
    int32_t size;
    const uint32_t *sourceMem;    /* Source memory[ Array of uint32_t ] */
    uint32_t offset;              /* Offset at which write will happen */
    uint32_t byteLen;             /* Total number of bytes to be written */
    int32_t status;

    /* Disable PRU */
    status = PRUICSS_pruDisable(pruIcssHandle, pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return APP_TS_SERR_INIT_PRU;
    }

    /* Reset PRU */
    status = PRUICSS_pruReset(pruIcssHandle, pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return APP_TS_SERR_INIT_PRU;
    }

    /* Determine PRU ID in slice */
    slicePruInstId = pruInstId - (uint8_t)pruInstId/ICSSG_TS_DRV__NUM_ICSSG_SLICE * ICSSG_TS_DRV__NUM_ICSSG_SLICE;
    /* Determine PRU DMEM address */
    pruDMem = PRU_ICSS_DATARAM(slicePruInstId);
    /* Determine PRU IMEM address */
    switch (pruInstId)
    {
        case PRUICSS_PRU0:
        case PRUICSS_PRU1:
            pruIMem = PRU_ICSS_IRAM_PRU(slicePruInstId);
            break;
        case PRUICSS_RTU0:
        case PRUICSS_RTU1:
            pruIMem = PRU_ICSS_IRAM_RTU(slicePruInstId);
            break;
        case PRUICSS_TPRU0:
        case PRUICSS_TPRU1:
            pruIMem = PRU_ICSS_IRAM_TXPRU(slicePruInstId);
            break;
        default:
            break;
    }

    /* Initialize DMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, pruDMem);
    if (size == 0) {
        return APP_TS_SERR_INIT_PRU;
    }

    /* Initialize IMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, pruIMem);
    if (size == 0)
    {
        return APP_TS_SERR_INIT_PRU;
    }

    /* Write DMEM */
    offset = ICSSG_TS_BASE_ADDR;
    sourceMem = (uint32_t *)pru_timesync_image_1;
    byteLen = sizeof(pru_timesync_image_1);
    size = PRUICSS_pruWriteMemory(pruIcssHandle, pruDMem, offset, sourceMem, byteLen);
    if (size == 0)
    {
        return APP_TS_SERR_INIT_PRU;
    }

    /* Write IMEM */
    offset = 0;
    sourceMem = pru_timesync_image_0;
    byteLen = sizeof(pru_timesync_image_0);
    size = PRUICSS_pruWriteMemory(pruIcssHandle, pruIMem, offset, sourceMem, byteLen);
    if (size == 0)
    {
        return APP_TS_SERR_INIT_PRU;
    }

    return APP_TS_SOK;
}

/*
 *  ======== appTs_initIcssgTsDrv ========
 */
/* Initialize ICSSG timesync DRV */
int32_t appTs_initIcssgTsDrv(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTsCtrl
)
{
    IcssgTsDrv_Handle hTsDrv;
    int32_t status;

    /* Copy TS parameters to TS object */
    pTsCtrl->tsPrms = *pTsPrms;
    /* Copy ICSS handle to TS object */
    pTsCtrl->pruIcssHandle = pruIcssHandle;

    /* Translate ICSSG & PRU hardware module IDs to TS API */
    status = appTs_getIcssgId(pTsPrms->icssInstId, &pTsCtrl->icssgId);
    if (status != APP_TS_SOK) {
        return APP_TS_SERR_INIT_TS_DRV;
    }
    appTs_getPruId(pTsPrms->pruInstId, &pTsCtrl->pruId);
    if (status != APP_TS_SOK) {
        return APP_TS_SERR_INIT_TS_DRV;
    }

    /* Initialize TS DRV instance */
    hTsDrv = icssgTsDrv_initDrv(pTsCtrl->icssgId, pTsCtrl->pruId);
    if (hTsDrv == NULL) {
        return APP_TS_SERR_INIT_TS_DRV;
    }

    /* Set non-default configuration */

    /* Configure IEP0 Period Counts & Offsets */
    status = icssgTsDrv_cfgTsPrdCount(hTsDrv, pTsPrms->prdCount, 
        pTsPrms->prdOffset, pTsPrms->prdCfgMask, 0);
    if (status != ICSSG_TS_DRV__STS_NERR) {
        return APP_TS_SERR_INIT_TS_DRV;
    }

    /* Store TS DRV handle to TS object */
    pTsCtrl->hTsDrv = hTsDrv;    

    return APP_TS_SOK;
}

/*
 *  ======== appTs_startTs ========
 */
/* Start ICSSG Timesync */
int32_t appTs_startTs(
    TsObj *pTsCtrl
)
{
    int32_t status;

    /* Enable PRU */
    status = PRUICSS_pruEnable(pTsCtrl->pruIcssHandle, pTsCtrl->tsPrms.pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return APP_TS_SERR_START_TS;
    }

    /* Set TS Global Enable */
    status = icssgTsDrv_setTsGblEn(pTsCtrl->hTsDrv, ICSSG_TS_DRV__TS_GBL_EN_ENABLE);
    if (status != ICSSG_TS_DRV__STS_NERR) {
        return APP_TS_SERR_START_TS;
    }

    /* Wait TS Global Enable ACK */
    status = icssgTsDrv_waitTsGblEnAck(pTsCtrl->hTsDrv, ICSSG_TS_DRV__TS_GBL_EN_ACK_ENABLE);
    if (status != ICSSG_TS_DRV__STS_NERR) {
        return APP_TS_SERR_START_TS;
    }
    
    return APP_TS_SOK;
}

/*
 *  ======== appTs_initTs ========
 */
/* Initialize Time Sync */
int32_t appTs_initTs(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTs
)
{
    uint32_t i;
    int32_t status;

    /* Configure IEP CMP Period & Offset */
    for (i = 0; i < ICSSG_TS_DRV__NUM_IEP_CMP; i++) 
    {
        if ((pTsPrms->prdCfgMask >> i) & 0x1) {
            /* Configure Compare Event Interrupt Router */
            status = appTs_configureCmpEventInterruptRouter(pTsPrms->cmpEvtRtrInIntNum[i], 
                pTsPrms->cmpEvtRtrOutIntNum[i], pTsPrms->cmpEvtRtrHostId[i]);
            if (status != APP_TS_SOK) {
                appLogPrintf("appPslCtrlTsInit: configureCmpEventInterruptRouter() failed.\n");
                return APP_TS_SERR_INIT;
            }
        }
    }
    
    /* Initialize PRU for Timesync */
    status = appTs_initPruTs(pruIcssHandle, pTsPrms->pruInstId);
    if (status != APP_TS_SOK) {
        appLogPrintf("appPslCtrlTsInit: initPruTimesync() failed.\n");
        return APP_TS_SERR_INIT;
    }
    
    /* Initialize ICSSG TS DRV */
    /* Initialize PRU for TS */
    status = appTs_initIcssgTsDrv(pruIcssHandle, pTsPrms, pTs);
    if (status != APP_TS_SOK) {
        appLogPrintf("appPslCtrlTsInit: initIcssgTsDrv() failed.\n");
        return APP_TS_SERR_INIT;
    }

    return APP_TS_SOK;
}
