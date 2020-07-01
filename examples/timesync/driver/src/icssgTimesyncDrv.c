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

#include <stdint.h>
#include <string.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr_icss.h>
#include "tsFwRegs.h"
#include "icssg_timesync.h"
#include "icssgTimesyncDrv.h"
#include "timesyncDrv_api.h"

#define ICSSG_SLICE_PRU_ID_0    ( 0 )   /* Slice PRU0 ID */
#define ICSSG_SLICE_PRU_ID_1    ( 1 )   /* Slice PRU1 ID */

/* Internal structure for managing TSs for each ICSSG */
/* static */ IcssgTsDrv_TsDrvObj gTsDrvObj;


/* ------------------------------------------------------------------------- *
 * External Functions                                                        *
 * ------------------------------------------------------------------------- */

/* Initialize DRV */
IcssgTsDrv_Handle icssgTsDrv_initDrv(
    uint8_t icssgId,
    uint8_t pruId
)
{
    IcssgTsDrv_TsDrvObj *pTsDrvObj;
    uint8_t slicePruId;
    uint32_t baseAddr;

    if ((icssgId < ICSSG_TS_DRV__NUM_ICSSG) &&
        (pruId < ICSSG_TS_DRV__NUM_PRU))
    {
        pTsDrvObj = &gTsDrvObj;
        memset(pTsDrvObj, 0, sizeof(pTsDrvObj));
        
        /* Store ICSSG & PRU IDs */
        pTsDrvObj->icssgId = icssgId;
        pTsDrvObj->pruId = pruId;
        
        /* Determine PRU ID in slice */
        slicePruId = pruId - (uint8_t)pruId/ICSSG_TS_DRV__NUM_ICSSG_SLICE * ICSSG_TS_DRV__NUM_ICSSG_SLICE;
        
        /* Determine DMEM base address & 
           IEP CMP hardware register address */
        if (icssgId == ICSSG_TS_DRV__ICSSG_ID_0)
        {
            /* Assign ICSSG0 addresses */
            if (slicePruId == ICSSG_SLICE_PRU_ID_0) {
                baseAddr = CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE;
            }
            else if (slicePruId == ICSSG_SLICE_PRU_ID_1) {
                baseAddr = CSL_PRU_ICSSG0_DRAM1_SLV_RAM_BASE;
            }
            else {
                return NULL;
            }
            pTsDrvObj->tsCmpCtrl.pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)CSL_PRU_ICSSG0_IEP0_BASE;
        }
        else if (icssgId == ICSSG_TS_DRV__ICSSG_ID_1)
        {
            /* Assign ICSSG1 addresses */
            if (slicePruId == ICSSG_SLICE_PRU_ID_0) {
                baseAddr = CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE;
            }
            else if (slicePruId == ICSSG_SLICE_PRU_ID_1) {
                baseAddr = CSL_PRU_ICSSG1_DRAM1_SLV_RAM_BASE;
            }
            else {
                return NULL;
            }
            pTsDrvObj->tsCmpCtrl.pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)CSL_PRU_ICSSG1_IEP0_BASE;
        }
        else 
        {
            return NULL;
        }
        
        /* Initialize TS driver firmware register pointers */
        pTsDrvObj->tsInfo.pTsInfoFwRegs = (TsInfoFwRegs *)(baseAddr + FW_REG_MAGIC_NUMBER);
        pTsDrvObj->tsCtrl.pTsCtrlFwRegs = (TsCtrlFwRegs *)(baseAddr + FW_REG_TS_CTRL);
        pTsDrvObj->tsCmpCtrl.pTsCmpFwRegs = (TsCmpFwRegs *)(baseAddr + FW_REG_TS_CMP1_COUNT);        
    }
    else {
        pTsDrvObj = NULL;
    }

    return (IcssgTsDrv_Handle)pTsDrvObj;
}

/* Set TS Global Enable */
int32_t icssgTsDrv_setTsGblEn(
    IcssgTsDrv_Handle handle,
    uint8_t tsGblEnFlag
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    TsCtrlFwRegs *tsCtrlFwRegs;
    uint32_t tsCtrlFwReg;

    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    tsCtrlFwRegs = pTsCtrl->pTsCtrlFwRegs;

    /* Read TS Global Control FW register */
    tsCtrlFwReg = tsCtrlFwRegs->TS_CTRL;

    /* Set TS global enable in FW register */
    tsCtrlFwReg &= ~TS_CTRL_IEP0_TS_GBL_EN_MASK;
    if (tsGblEnFlag == ICSSG_TS_DRV__TS_GBL_EN_DISABLE) {
        tsCtrlFwReg |= BF_TS_GBL_EN_DISABLE << TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    }
    else if (tsGblEnFlag == ICSSG_TS_DRV__TS_GBL_EN_ENABLE) {
        tsCtrlFwReg |= BF_TS_GBL_EN_ENABLE << TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    }
    else
    {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Write TS Global Control FW register */
    tsCtrlFwRegs->TS_CTRL = tsCtrlFwReg;

    return ICSSG_TS_DRV__STS_NERR;
}

/* Wait for TS Global Enable ACK */
int32_t icssgTsDrv_waitTsGblEnAck(
    IcssgTsDrv_Handle handle,
    uint8_t tsGblEnAckFlag
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    TsCtrlFwRegs *pTsCtrlFwRegs;
    uint32_t tsStatFwReg;
    uint8_t tsGblEnAckFwReg;
    
    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    pTsCtrlFwRegs = pTsCtrl->pTsCtrlFwRegs;

    /* Wait for TS Global Enable ACK */
    do {
        /* Read TS Status FW register */
        tsStatFwReg = pTsCtrlFwRegs->TS_STAT;
        /* Extract TS Global Enable ACK flag */
        tsGblEnAckFwReg = (tsStatFwReg & TS_STAT_IEP0_TS_GBL_EN_ACK_MASK) >> TS_STAT_IEP0_TS_GBL_EN_ACK_SHIFT;
    } while (tsGblEnAckFwReg == tsGblEnAckFlag);
    
    return ICSSG_TS_DRV__STS_NERR;    
}

/* Wait for FW initialization flag */
int32_t icssgTsDrv_waitFwInit(
    IcssgTsDrv_Handle handle, 
    uint8_t fwInitFlag
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    TsCtrlFwRegs *pTsCtrlFwRegs;
    uint32_t tsStatFwReg;
    uint8_t fwInitFwReg;

    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    pTsCtrlFwRegs = pTsCtrl->pTsCtrlFwRegs;

    /* Wait for FW init */
    do {
        /* Read TS Status FW register */
        tsStatFwReg = pTsCtrlFwRegs->TS_STAT;
        /* Extract FW init flag */
        fwInitFwReg = (tsStatFwReg & TS_STAT_FW_INIT_MASK) >> TS_STAT_FW_INIT_SHIFT;
    } while (fwInitFwReg == fwInitFlag);

    return ICSSG_TS_DRV__STS_NERR;
}

/* Configure TS IEP0 Period (nsec) */
int32_t icssgTsDrv_cfgTsIepPrdNsec(
    IcssgTsDrv_Handle handle,
    uint32_t tsIepPrdNsec
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    TsCtrlFwRegs *pTsCtrlFwRegs;

    /* Get pointer to IEP TS control registers */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    pTsCtrlFwRegs = pTsCtrl->pTsCtrlFwRegs;

    /* Write TS IEP Perdiod FW register */
    pTsCtrlFwRegs->TS_IEP_PRD_NSEC = tsIepPrdNsec;

    return ICSSG_TS_DRV__STS_NERR;
}

/* Configure TS IEP0 Period Counts & Offsets */
int32_t icssgTsDrv_cfgTsPrdCount(
    IcssgTsDrv_Handle handle,
    uint32_t tsPrdCount[],
    int32_t tsPrdOffset[],
    uint32_t cfgBf,
    uint32_t testTsPrdCount
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    TsCmpFwRegs *pTsCmpFwRegs;
    volatile uint32_t *pTsCmpCountFwReg;
    volatile int32_t *pTsCmpOffsetFwReg;
    int32_t i;

    /* Get pointer to IEP TS control registers */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCmpFwRegs = pTsDrv->tsCmpCtrl.pTsCmpFwRegs;
    pTsCmpCountFwReg = &pTsCmpFwRegs->TS_CMP3_COUNT;
    pTsCmpOffsetFwReg = &pTsCmpFwRegs->TS_CMP3_OFFSET;

    /* Configure IEP CMP Period & Offset */
    for (i = 0; i < ICSSG_TS_DRV__NUM_IEP_CMP; i++)
    {
        if ((cfgBf >> i) & 0x1)
        {
            /* Write Period Count firmware register */
            *pTsCmpCountFwReg++ = tsPrdCount[i];   
            /* Write Period Offset firmware register */
            *pTsCmpOffsetFwReg++ = tsPrdOffset[i];
        }        
    }

    /* Configure test IEP CMP */
    pTsCmpCountFwReg = &pTsCmpFwRegs->TS_CMP1_COUNT;
    *pTsCmpCountFwReg = 0;
    if (testTsPrdCount != 0)
    {
        *pTsCmpCountFwReg = testTsPrdCount;
    }

    return ICSSG_TS_DRV__STS_NERR;
}

