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
#include <ti/csl/soc.h>
#include "tsFwRegs.h"
#include "icssg_timesync.h"
#include "timesyncDrv_api.h"

/**    @brief    reconfigure TS Period Count */
#define RECFG_TS_PRD_COUNT_MASK    ( 1<<1 )

/* Default (reset) IEP TS Global Enable Mask */
#define DEF_TS_GBL_EN_MASK     ( 0 )

typedef TsInfoFwRegs IcssgTsDrv_TsInfoRegs;
typedef TsCtrlFwRegs IcssgTsDrv_TsCtrlRegs;
typedef TsCmpFwRegs IcssgTsDrv_TsCmpRegs;

/* TS info object */
typedef struct IcssgTsDrv_TsInfoObj_s
{
    /* TS info registers */
    IcssgTsDrv_TsInfoRegs *pTsInfoRegs;
} IcssgTsDrv_TsInfoObj;

/* TS control object */
typedef struct IcssgTsDrv_TsCtrlObj_s
{
    /* TS control registers */
    IcssgTsDrv_TsCtrlRegs *pTsCtrlRegs;
} IcssgTsDrv_TsCtrlObj;

/* TS IEP CMP control object */
typedef struct IcssgTsDrv_TsCmpObj_s
{
    /* TS CMP registers */
    IcssgTsDrv_TsCmpRegs *pTsCmpRegs;
} IcssgTsDrv_TsCmpObj;

/* TS DRV object */
typedef struct IcssgTsDrv_TsDrvObj_s
{
    /* ICSSG hardware module ID */
    uint8_t                icssgId;
    /* PRU hardware module ID */
    uint8_t                pruId;
    /* TS info */
    IcssgTsDrv_TsInfoObj   tsInfo;
    /* TS control */
    IcssgTsDrv_TsCtrlObj   tsCtrl;
    /* IEPx TS/CMP control */
    IcssgTsDrv_TsCmpObj    tsCmp[ICSSG_TS_DRV__ICSSG_NUM_IEP];
} IcssgTsDrv_TsDrvObj;

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
        
        /* Store ICSSG & PRU IDs */
        pTsDrvObj->icssgId = icssgId;
        pTsDrvObj->pruId = pruId;
        
        /* Determine PRU ID in slice */
        slicePruId = pruId - (uint8_t)pruId/ICSSG_NUM_SLICE * ICSSG_NUM_SLICE;
        
        /* Determine DMEM base address */
        if (icssgId == ICSSG_TS_DRV__ICSSG_ID_0)
        {
            baseAddr = (slicePruId == ICSSG_TS_DRV__SLICE_PRU_ID_0) ? CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE : 
                CSL_PRU_ICSSG0_DRAM1_SLV_RAM_BASE;
        }
        else if (icssgId == ICSSG_TS_DRV__ICSSG_ID_1)
        {
            baseAddr = (slicePruId == ICSSG_TS_DRV__SLICE_PRU_ID_0) ? CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE : 
                CSL_PRU_ICSSG1_DRAM1_SLV_RAM_BASE;
        }
        else if (icssgId == ICSSG_TS_DRV__ICSSG_ID_2)
        {
            baseAddr = (slicePruId == ICSSG_TS_DRV__SLICE_PRU_ID_0) ? CSL_PRU_ICSSG2_DRAM0_SLV_RAM_BASE : 
                CSL_PRU_ICSSG2_DRAM1_SLV_RAM_BASE;
        }
        
        /* Initialize TS driver object pointers */
        pTsDrvObj->tsInfo.pTsInfoRegs = (IcssgTsDrv_TsInfoRegs *)(baseAddr + FW_REG_MAGIC_NUMBER);
        pTsDrvObj->tsCtrl.pTsCtrlRegs = (IcssgTsDrv_TsCtrlRegs *)(baseAddr + FW_REG_TS_CTRL);
        pTsDrvObj->tsCmp[ICSSG_TS_DRV__IEP_ID_0].pTsCmpRegs = (IcssgTsDrv_TsCmpRegs *)(baseAddr + FW_REG_TS_CMP1_COUNT);
        pTsDrvObj->tsCmp[ICSSG_TS_DRV__IEP_ID_1].pTsCmpRegs = NULL;
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
    IcssgTsDrv_TsCtrlRegs *tsCtrlRegs;
    uint32_t tsCtrl;

    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    tsCtrlRegs = pTsCtrl->pTsCtrlRegs;

    /* Read TS Global Control FW register */
    tsCtrl = tsCtrlRegs->TS_CTRL;

    /* Set TS global enable in FW register */
    tsCtrl &= ~TS_CTRL_IEP0_TS_GBL_EN_MASK;
    if (tsGblEnFlag == ICSSG_TS_DRV__IEP_TS_GBL_EN_DISABLE) {
        tsCtrl |= BF_TS_GBL_EN_DISABLE << TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    }
    else if (tsGblEnFlag == ICSSG_TS_DRV__IEP_TS_GBL_EN_ENABLE) {
        tsCtrl |= BF_TS_GBL_EN_ENABLE << TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    }
    else
    {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Write IEP TS Global Control Mask FW register */
    tsCtrlRegs->TS_CTRL = tsCtrl;

    return ICSSG_TS_DRV__STS_NERR;
}

/* Wait for TS Global Enable ACK complete */
int32_t icssgTsDrv_waitTsGblEnAck(
    IcssgTsDrv_Handle handle
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    IcssgTsDrv_TsCtrlRegs *pTsCtrlRegs;
    uint32_t tsStat;
    uint8_t tsGblEnAckFlag;
    
    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    pTsCtrlRegs = pTsCtrl->pTsCtrlRegs;

    /* Wait for TS Global Enable ACK */
    do {
        /* Read TS Status FW register */
        tsStat = pTsCtrlRegs->TS_STAT;
        /* Extract TS Global Enable ACK flag */
        tsGblEnAckFlag = (tsStat & TS_STAT_IEP0_TS_GBL_EN_ACK_MASK) >> TS_STAT_IEP0_TS_GBL_EN_ACK_SHIFT;
    } while (tsGblEnAckFlag == BF_TS_GBL_EN_ACK_DISABLE);
    
    return ICSSG_TS_DRV__STS_NERR;    
}

/* Wait for FW initialization complete */
int32_t icssgTsDrv_waitFwInit(
    IcssgTsDrv_Handle handle
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    IcssgTsDrv_TsCtrlRegs *pTsCtrlRegs;
    uint32_t tsStat;
    uint8_t fwInitFlag;

    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    pTsCtrlRegs = pTsCtrl->pTsCtrlRegs;

    /* Wait for FW init */
    do {
        /* Read TS Status FW register */
        tsStat = pTsCtrlRegs->TS_STAT;
        /* Extract FW init flag */
        fwInitFlag = (tsStat & TS_STAT_FW_INIT_MASK) >> TS_STAT_FW_INIT_SHIFT;
    } while (fwInitFlag == BF_TS_FW_INIT_UNINIT);

    return ICSSG_TS_DRV__STS_NERR;
}

/* Prepare IEP Period Count reconfiguration */
int32_t icssgTsDrv_prepRecfgTsPrdCount(
    IcssgTsDrv_Handle handle,
    uint8_t iepId,
    uint32_t tsPrdCount[],
    int32_t tsPrdOffset[],
    uint8_t  nPrdCount,
    uint32_t *pRecfgBf
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCmpRegs *pTsCmpRegs;
    volatile uint32_t *pTsCmpCount;
    volatile int32_t *pTsCmpOffset;
    int32_t i;

    /* Check IEP ID */
    if (iepId >= ICSSG_TS_DRV__ICSSG_NUM_IEP) {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Get pointer to IEP TS control registers */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCmpRegs = pTsDrv->tsCmp[iepId].pTsCmpRegs;
    pTsCmpCount = &pTsCmpRegs->TS_CMP1_COUNT;
    pTsCmpOffset = &pTsCmpRegs->TS_CMP3_OFFSET;

    for (i = 0; i < nPrdCount; i++)
    {
        /* Write Period Count register */
        *pTsCmpCount++ = tsPrdCount[i];   
        
        /* Write Period Offset register */
        if (i > 0)
        {
            *pTsCmpOffset++ = tsPrdOffset[i-1];
        }
    }

    /* Set flag indicating reconfiguration request parameter */
    *pRecfgBf = RECFG_TS_PRD_COUNT_MASK;

    return ICSSG_TS_DRV__STS_NERR;
}
