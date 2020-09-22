/*
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include "timesync_array.h"         /* TS PRU FW image data */
#include "timesyncDrv_utils.h"      /* TS driver utilities */
#include "timesyncDrv_api.h"        /* TS driver API */
#include "cfg_host_intr.h"
#include "app_timesync.h"

/* Time Sync ICSSG default pin mux setting */
#define PRUICSS_PINMUX_DEF           ( 0x0 )

/* Get ICSSG ID for DRV */
int32_t getIcssgId(
    PRUICSS_MaxInstances icssInstId,
    uint8_t *pIcssId
)
{
    /* Translate ICSSG hardware module ID to API */
    if (icssInstId == PRUICSS_INSTANCE_ONE) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_0;
        return APP_TS_ERR_NERR;
    }
    else if (icssInstId == PRUICSS_INSTANCE_TWO) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_1;
        return APP_TS_ERR_NERR;
    }
    else {
        *pIcssId = 0;
        return APP_TS_ERR_INV_PRMS;
    }
}

/* Get ICSSG ID for DRV */
int32_t getPruId(
    PRUSS_PruCores pruInstId,
    uint8_t *pPruId
)
{
    int32_t retVal = 0;
    
    /* Translate PRU hardware module ID to API */
    switch (pruInstId)
    {
        case PRUICSS_PRU0:
            *pPruId = ICSSG_TS_DRV__PRU_ID_0;
            retVal = APP_TS_ERR_NERR;
            break;
        case PRUICSS_PRU1:
            *pPruId = ICSSG_TS_DRV__PRU_ID_1;
            retVal = APP_TS_ERR_NERR;
            break;
        case PRUICSS_RTU0:
            *pPruId = ICSSG_TS_DRV__RTU_ID_0;
            retVal = APP_TS_ERR_NERR;
            break;
        case PRUICSS_RTU1:
            *pPruId = ICSSG_TS_DRV__RTU_ID_1;
            retVal = APP_TS_ERR_NERR;
            break;
        case PRUICSS_TPRU0:
            *pPruId = ICSSG_TS_DRV__TPRU_ID_0;
            retVal = APP_TS_ERR_NERR;
            break;
        case PRUICSS_TPRU1:
            *pPruId = ICSSG_TS_DRV__TPRU_ID_1;
            retVal = APP_TS_ERR_NERR;
            break;
        default:
            *pPruId = 0;
            retVal = APP_TS_ERR_INV_PRMS;
            break;
    }
    
    return retVal;
}

/*
 *  ======== cfgIcssgClkSel ========
 */
/* Configure ICSSG clock selection */
int32_t cfgIcssgClkSel(
    PRUICSS_MaxInstances icssInstId,
    uint8_t source
)
{
    CSL_main_ctrl_mmr_cfg0Regs *pCtrlMmrCfg0Regs = (CSL_main_ctrl_mmr_cfg0Regs *)CSL_CTRL_MMR0_CFG0_BASE;
    uint32_t regVal;

    if (icssInstId == PRUICSS_INSTANCE_ONE) {
        regVal = HW_RD_REG32(&pCtrlMmrCfg0Regs->ICSSG0_CLKSEL);
        regVal &= ~CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_CORE_CLKSEL_MASK;
        regVal |= source & 0x1;
        HW_WR_REG32(&pCtrlMmrCfg0Regs->ICSSG0_CLKSEL, regVal);
    }
    else if (icssInstId == PRUICSS_INSTANCE_TWO) {
        regVal = HW_RD_REG32(&pCtrlMmrCfg0Regs->ICSSG1_CLKSEL);
        regVal &= ~CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_CORE_CLKSEL_MASK;
        regVal |= source & 0x1;
        HW_WR_REG32(&pCtrlMmrCfg0Regs->ICSSG1_CLKSEL, regVal);
    }
    else {
        return APP_TS_ERR_CFG_ICSSG_CLKSEL;
    }

    return APP_TS_ERR_NERR;
}

/*
 *  ======== initIcss ========
 */
/* Initialize ICSSG */
int32_t initIcss(
    PRUICSS_MaxInstances icssInstId,
    PRUICSS_Handle *pPruIcssHandle
)
{
    PRUICSS_Config *pruIcssCfg; /* ICSS configuration */
    PRUICSS_Handle pruIcssHandle;
    uint8_t i;
    int32_t status;

    /* Get SoC level PRUICSS initial configuration */
    status = PRUICSS_socGetInitCfg(&pruIcssCfg);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return APP_TS_ERR_INIT_ICSSG;
    }

    /* Create ICSS PRU instance */
    pruIcssHandle = PRUICSS_create(pruIcssCfg, icssInstId);
    if (pruIcssHandle == NULL) {
        return APP_TS_ERR_INIT_ICSSG;
    }

    /* Disable PRUs & RTUs */
    for (i = 0; i < PRUICSS_MAX_PRU; i++)
    {
        status = PRUICSS_pruDisable(pruIcssHandle, i);
        if (status != PRUICSS_RETURN_SUCCESS) {
            return APP_TS_ERR_INIT_ICSSG;
        }
    }

    /* Set ICSS pin mux to default */
    PRUICSS_pinMuxConfig(pruIcssHandle, PRUICSS_PINMUX_DEF);

    *pPruIcssHandle = pruIcssHandle;

    return APP_TS_ERR_NERR;
}

/* Initialize PRU for timesync */
int32_t initPruTimesync(
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

    /* Reset PRU */
    status = PRUICSS_pruReset(pruIcssHandle, pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return APP_TS_ERR_INIT_PRU;
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
        return APP_TS_ERR_INIT_PRU;
    }

    /* Initialize IMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, pruIMem);
    if (size == 0)
    {
        return APP_TS_ERR_INIT_PRU;
    }

    /* Write DMEM */
    offset = ICSSG_TS_BASE_ADDR;
    sourceMem = (uint32_t *)pru_timesync_image_1;
    byteLen = sizeof(pru_timesync_image_1);
    size = PRUICSS_pruWriteMemory(pruIcssHandle, pruDMem, offset, sourceMem, byteLen);
    if (size == 0)
    {
        return APP_TS_ERR_INIT_PRU;
    }

    /* Write IMEM */
    offset = 0;
    sourceMem = pru_timesync_image_0;
    byteLen = sizeof(pru_timesync_image_0);
    size = PRUICSS_pruWriteMemory(pruIcssHandle, pruIMem, offset, sourceMem, byteLen);
    if (size == 0)
    {
        return APP_TS_ERR_INIT_PRU;
    }

    return APP_TS_ERR_NERR;
}

/* Initialize ICSSG timesync DRV */
int32_t initIcssgTsDrv(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTs
)
{
    IcssgTsDrv_Handle hTsDrv;
    int32_t status;

    /* Copy TS parameters to TS object */
    pTs->tsPrms = *pTsPrms;
    /* Copy ICSS handle to TS object */
    pTs->pruIcssHandle = pruIcssHandle;

    /* Translate ICSSG & PRU hardware module IDs to TS API */
    status = getIcssgId(pTsPrms->icssInstId, &pTs->icssgId);
    if (status != APP_TS_ERR_NERR) {
        return APP_TS_ERR_INIT_TS_DRV;
    }
    getPruId(pTsPrms->pruInstId, &pTs->pruId);
    if (status != APP_TS_ERR_NERR) {
        return APP_TS_ERR_INIT_TS_DRV;
    }

    /* Initialize TS DRV instance */
    hTsDrv = icssgTsDrv_initDrv(pTs->icssgId, pTs->pruId);
    if (hTsDrv == NULL) {
        return APP_TS_ERR_INIT_TS_DRV;
    }

    /* Set TS Global Enable */
    status = icssgTsDrv_setTsGblEn(hTsDrv, ICSSG_TS_DRV__TS_GBL_EN_ENABLE);
    if (status != ICSSG_TS_DRV__STS_NERR) {
        return APP_TS_ERR_INIT_TS_DRV;
    }

    /* Set non-default configuration */

    /* Configure IEP0 Period Count */
    status = icssgTsDrv_cfgTsPrdCount(hTsDrv, pTsPrms->prdCount, 
        pTsPrms->prdOffset, pTsPrms->cfgMask, pTsPrms->testTsPrdCount);
    if (status != ICSSG_TS_DRV__STS_NERR) {
        return APP_TS_ERR_INIT_TS_DRV;
    }

    /* Store TS DRV handle to TS object */
    pTs->hTsDrv = hTsDrv;    

    return APP_TS_ERR_NERR;
}

/* Start ICSSG Timesync */
int32_t startTs(
    TsObj *pTs
)
{
    int32_t status;

    /* Start IEP0 counter */
    icssgTsDrv_startIepCount(pTs->hTsDrv);

    /* Enable PRU */
    status = PRUICSS_pruEnable(pTs->pruIcssHandle, pTs->tsPrms.pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return APP_TS_ERR_START_TS;
    }

    /* Wait for PRU FW initialization complete */
    status = icssgTsDrv_waitFwInit(pTs->hTsDrv, ICSSG_TS_DRV__TS_FW_INIT_INIT);
    if (status != ICSSG_TS_DRV__STS_NERR)
    {
        return APP_TS_ERR_START_TS;
    }

    return APP_TS_ERR_NERR;
}
