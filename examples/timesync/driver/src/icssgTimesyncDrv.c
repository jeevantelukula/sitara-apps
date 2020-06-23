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
#include "timesyncFwRegs.h"
#include "timesyncFwDefs.h"
#include "timesyncDrv_api.h"

/**    @brief    reconfigure TS enable */
#define RECFG_TS_EN_MASK           ( 1<<0 )
/**    @brief    reconfigure TS Period Count */
#define RECFG_TS_PRD_COUNT_MASK    ( 1<<1 )
/**    @brief    reconfigure TS Duty Cycle Count */
#define RECFG_TS_DC_COUNT_MASK     ( 1<<2 )
/**    @brief    reconfigure TS Deadband Count */
#define RECFG_TS_DB_COUNT_MASK     ( 1<<3 )

/* Default (reset) IEP TS Global Enable Mask */
#define DEF_TS_GBL_EN_MASK     ( 0 )
/* Default (reset) IEPx TS Enable Mask */
#define DEF_IEPx_TS_EN_MASK        ( 0 )
/* Default (reset) IEPx TS Mode Mask */
#define DEF_IEPx_TS_MODE_MASK      ( 0 )

typedef TsInfoFwRegs IcssgTsDrv_TsInfoRegs;
typedef TsCtrlFwRegs IcssgTsDrv_TsCtrlRegs;
typedef IepTsFwRegs IcssgTsDrv_IepTsRegs;

/* TS info object */
typedef struct IcssgTsDrv_TsInfoObj_s
{
    /* TS info registers */
    IcssgTsDrv_TsInfoRegs *pTsInfoRegs;
} IcssgTsDrv_TsInfoObj;

/* TS control object */
typedef struct IcssgTsDrv_TsCtrlObj_s
{
    /* TS global enable mask */
    uint8_t tsGblEnMask;
    /* TS control registers */
    IcssgTsDrv_TsCtrlRegs *pTsCtrlRegs;
} IcssgTsDrv_TsCtrlObj;

/* IEP CMP/TS control object */
typedef struct IcssgTsDrv_IepTsCtrlObj_s
{
    /* IEPx TS enable mask */
    Uint16 tsEnMask;

    /* IEPx TS registers */
    IcssgTsDrv_IepTsRegs *pIepTsRegs;
} IcssgTsDrv_IepTsCtrlObj;

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
    IcssgTsDrv_IepTsCtrlObj iepTsCtrl[ICSSG_TS_DRV__ICSSG_NUM_IEP];
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
        pTsDrvObj->tsInfo.pTsInfoRegs = (IcssgTsDrv_TsInfoRegs *)(baseAddr + ICSSG_TS_FW_MAGIC_NUMBER_ADDR);
        pTsDrvObj->tsCtrl.pTsCtrlRegs = (IcssgTsDrv_TsCtrlRegs *)(baseAddr + ICSSG_TS_TS_CTRL_ADDR);
        pTsDrvObj->iepTsCtrl[ICSSG_TS_DRV__IEP_ID_0].pIepTsRegs = (IcssgTsDrv_IepTsRegs *)(baseAddr + ICSSG_TS_IEP0_TS_BASE_ADDR);
        pTsDrvObj->iepTsCtrl[ICSSG_TS_DRV__IEP_ID_1].pIepTsRegs = (IcssgTsDrv_IepTsRegs *)(baseAddr + ICSSG_TS_IEP1_TS_BASE_ADDR);

        /* Reset IEP TS global enable mask */
        pTsDrvObj->tsCtrl.tsGblEnMask = DEF_TS_GBL_EN_MASK;

        /* Reset IEPx TS enable mask */
        pTsDrvObj->iepTsCtrl[ICSSG_TS_DRV__IEP_ID_0].tsEnMask = DEF_IEPx_TS_EN_MASK;
        pTsDrvObj->iepTsCtrl[ICSSG_TS_DRV__IEP_ID_1].tsEnMask = DEF_IEPx_TS_EN_MASK;
    }
    else {
        pTsDrvObj = NULL;
    }

    return (IcssgTsDrv_Handle)pTsDrvObj;
}

/* Set IEP TS Global Enable flags */
int32_t icssgTsDrv_setIepTsGblEn(
    IcssgTsDrv_Handle handle,
    uint8_t tsGblEnMask
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_TsCtrlObj *pTsCtrl;
    IcssgTsDrv_TsCtrlRegs *tsCtrlRegs;
    uint32_t tsCtrl;
    uint8_t iepGblEnPrm;

    /* Get pointer to TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pTsCtrl = &pTsDrv->tsCtrl;
    tsCtrlRegs = pTsCtrl->pTsCtrlRegs;

    /* Stash IEP global control mask parameter */
    pTsCtrl->tsGblEnMask = tsGblEnMask;

    /* Read IEP TS Global Control FW register */
    tsCtrl = tsCtrlRegs->TS_CTRL;

    /* Extract IEP0 TS global enable parameter */
    iepGblEnPrm = (tsGblEnMask & ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_MASK) >> ICSSG_TS_DRV__BF_IEP0_TS_GBL_EN_SHIFT;
    /* Set IEP TS global enable in FW register */
    tsCtrl &= ~TS_CTRL_IEP0_TS_GBL_EN_MASK;
    if (iepGblEnPrm == ICSSG_TS_DRV__IEP_TS_GBL_EN_DISABLE) {
        tsCtrl |= BF_TS_GBL_EN_DISABLE << TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    }
    else if (iepGblEnPrm == ICSSG_TS_DRV__IEP_TS_GBL_EN_ENABLE) {
        tsCtrl |= BF_TS_GBL_EN_ENABLE << TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    }
    else
    {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Extract IEP1 TS global enable parameter */
    iepGblEnPrm = (tsGblEnMask & ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_MASK) >> ICSSG_TS_DRV__BF_IEP1_TS_GBL_EN_SHIFT;
    /* Set IEP TS global enable in FW register */
    tsCtrl &= ~TS_CTRL_IEP1_TS_GBL_EN_MASK;
    if (iepGblEnPrm == ICSSG_TS_DRV__IEP_TS_GBL_EN_DISABLE) {
        tsCtrl |= BF_TS_GBL_EN_DISABLE << TS_CTRL_IEP1_TS_GBL_EN_SHIFT;
    }
    else if (iepGblEnPrm == ICSSG_TS_DRV__IEP_TS_GBL_EN_ENABLE) {
        tsCtrl |= BF_TS_GBL_EN_ENABLE << TS_CTRL_IEP1_TS_GBL_EN_SHIFT;
    }
    else
    {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Write IEP TS Global Control Mask FW register */
    tsCtrlRegs->TS_CTRL = tsCtrl;

    return ICSSG_TS_DRV__STS_NERR;
}

/* Wait FW initialization complete */
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

    /* Wait for IEP TS global enable ACK */
    do {
        /* Read IEP TS Global Status FW register */
        tsStat = pTsCtrlRegs->TS_STAT;
        /* Extract IEP TS global enable ACK */
        fwInitFlag = (tsStat & TS_STAT_FW_INIT_MASK) >> TS_STAT_FW_INIT_SHIFT;
    } while (fwInitFlag == BF_TS_FW_INIT_UNINIT);

    return ICSSG_TS_DRV__STS_NERR;
}

/* Prepare IEP TS enable reconfiguration */
int32_t icssgTsDrv_prepRecfgTsEn(
    IcssgTsDrv_Handle handle,
    uint8_t iepId,
    Uint16 tsEnMask,
    uint32_t *pRecfgBf
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_IepTsCtrlObj *pIepTsCtrl;
    uint32_t iepTsEn;
    uint8_t tsEnPrm;
    uint8_t i;

    /* Check IEP ID */
    if (iepId >= ICSSG_TS_DRV__ICSSG_NUM_IEP) {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Get pointer to IEP TS info & control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pIepTsCtrl = &pTsDrv->iepTsCtrl[iepId];

    /* Write IEPx TS Enable FW register */
    pIepTsCtrl->tsEnMask = tsEnMask;

    /* Read IEPx TS Enable FW register */
    iepTsEn = pIepTsCtrl->pIepTsRegs->TS_EN;

    for (i = 0; i < 4; i++)
    {
        /* Extract TS enable parameter */
        tsEnPrm = (tsEnMask >> i) & ICSSG_TS_DRV__BF_TS_EN_MASK;
        /* Set TS enable in FW register */
        iepTsEn &= ~(TS_EN_MASK << i);
        if (tsEnPrm == ICSSG_TS_DRV__IEP_TS_EN_DISABLE) {
            iepTsEn |= BF_TS_EN_DISABLE << i;
        }
        else if (tsEnPrm == ICSSG_TS_DRV__IEP_TS_EN_ENABLE) {
            iepTsEn |= BF_TS_EN_ENABLE << i;
        }
        else {
            return ICSSG_TS_DRV__STS_ERR_INV_PRM;
        }
    }

    /* Write IEPx TS Enable FW register */
    pIepTsCtrl->pIepTsRegs->TS_EN = iepTsEn;

    /* Set flag indicating reconfiguration request parameter */
    *pRecfgBf = RECFG_TS_EN_MASK;

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
    IcssgTsDrv_IepTsRegs *pIepTsRegs;
    uint32_t prdCount;
    int32_t i;

    /* Check IEP ID */
    if (iepId >= ICSSG_TS_DRV__ICSSG_NUM_IEP) {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Get pointer to IEP TS control registers */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pIepTsRegs = pTsDrv->iepTsCtrl[iepId].pIepTsRegs;

    for (i = 0; i < nPrdCount; i++)
    {
        /* Read IEP TS Period Count register */
        prdCount = pIepTsRegs->TS_PRD_COUNT[i];
        /* Update IEP TS Period Count w/ Period Count parameter */
        prdCount &= ~IEP_TS_PRD_COUNT_MASK;
        prdCount |= (tsPrdCount[i] & PRD_COUNT_MASK) << IEP_TS_PRD_COUNT_SHIFT;
        /* Write IEP TS Period Count register */
        pIepTsRegs->TS_PRD_COUNT[i] = prdCount;
        /* Write offset */
        if (i > 0)
        {
            pIepTsRegs->TS_PRD_OFFSET[i-1] = tsPrdOffset[i-1];
        }
    }

    /* Set flag indicating reconfiguration request parameter */
    *pRecfgBf = RECFG_TS_PRD_COUNT_MASK;

    return ICSSG_TS_DRV__STS_NERR;
}

/* Execute prepared reconfigurations */
int32_t icssgTsDrv_commitRecfg(
    IcssgTsDrv_Handle handle,
    uint8_t iepId,
    uint32_t recfgBf
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_IepTsCtrlObj *pIepTsCtrl;
    IcssgTsDrv_IepTsRegs *pIepTsRegs;
    uint32_t tsRecfg;

    /* Check IEP ID */
    if (iepId >= ICSSG_TS_DRV__ICSSG_NUM_IEP) {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Get IEP TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pIepTsCtrl = &pTsDrv->iepTsCtrl[iepId];
    pIepTsRegs = pIepTsCtrl->pIepTsRegs;

    /* Update reconfiguration register */
    tsRecfg = pIepTsRegs->TS_RECFG;
    tsRecfg &= ~IEP_TS_RECFG_MASK;
    tsRecfg |= recfgBf;
    pIepTsRegs->TS_RECFG = tsRecfg;

    return ICSSG_TS_DRV__STS_NERR;
}

/* Wait for reconfiguration completion */
int32_t icssgTsDrv_waitRecfg(
    IcssgTsDrv_Handle handle,
    uint8_t iepId
)
{
    IcssgTsDrv_TsDrvObj *pTsDrv;
    IcssgTsDrv_IepTsCtrlObj *pIepTsCtrl;
    IcssgTsDrv_IepTsRegs *pIepTsRegs;
    uint32_t tsRecfg;

    /* Check IEP ID */
    if (iepId >= ICSSG_TS_DRV__ICSSG_NUM_IEP) {
        return ICSSG_TS_DRV__STS_ERR_INV_PRM;
    }

    /* Get IEP TS control */
    pTsDrv = (IcssgTsDrv_TsDrvObj *)handle;
    pIepTsCtrl = &pTsDrv->iepTsCtrl[iepId];
    pIepTsRegs = pIepTsCtrl->pIepTsRegs;

    /* Wait for FW to clear reconfiguration register */
    do {
        tsRecfg = pIepTsRegs->TS_RECFG;
        tsRecfg &= IEP_TS_RECFG_MASK;
    } while (tsRecfg != 0);

    return ICSSG_TS_DRV__STS_NERR;
}
