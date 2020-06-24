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

#include <ti/csl/cslr_icss.h>
#include <ti/csl/soc.h>
#include "tsFwRegs.h"
#include "icssg_timesync.h"

/* Pointer to FW registers */
TsFwRegs * const pTsFwRegs = &gTsFwRegs;
/* Pointer to IEP0 registers */
CSL_icss_g_pr1_iep1_slvRegs * const pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)CSL_ICSS_IEP0_CFG_BASE;

void main(void)
{
    uint32_t tsCtrl, tsStat;
    uint8_t iepTsGblEn;
    uint8_t mask = 0;
    uint32_t temp;
    uint64_t lastCmp1;
    uint64_t lastCmp3, lastCmp4, lastCmp5, lastCmp6;
    TsCtrlFwRegs *pTsCtrlFwRegs;
    TsCmpFwRegs *pTsCmpFwRegs;

    pTsCtrlFwRegs = &pTsFwRegs->tsCtrlFwRegs;   /* get pointer to firmware control registers */
    pTsCmpFwRegs = &pTsFwRegs->tsCmpFwRegs;     /* get pointer to firmware CMP registers */

    /* Wait for enable */
    do {
        /* Read IEP TS Global Status FW register */
        tsCtrl = pTsCtrlFwRegs->TS_CTRL;
        /* Extract IEP TS global enable */
        iepTsGblEn = (tsCtrl & TS_CTRL_IEP0_TS_GBL_EN_MASK) >> TS_CTRL_IEP0_TS_GBL_EN_SHIFT;
    } while (iepTsGblEn == BF_TS_GBL_EN_DISABLE);
    
    /* Set enable ACK */
    tsStat = pTsCtrlFwRegs->TS_STAT;
    tsStat &= ~TS_STAT_IEP0_TS_GBL_EN_ACK_MASK;
    tsStat |= BF_TS_GBL_EN_ACK_ENABLE << TS_STAT_IEP0_TS_GBL_EN_ACK_SHIFT;
    pTsCtrlFwRegs->TS_STAT = tsStat;
    
    /* Set up CMP1 for testing purposes */
    if (pTsCmpFwRegs->TS_CMP1_COUNT != 0) 
    {
        mask |= (1<<1);
        lastCmp1 = *(volatile uint64_t *)&pIepHwRegs->COUNT_REG0;
        lastCmp1 += pTsCmpFwRegs->TS_CMP1_COUNT;
        *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0 = lastCmp1;
    }
    
    /* Initialize CMP3 */
    if (pTsCmpFwRegs->TS_CMP3_COUNT != 0)
    {
        mask |= (1<<3);
        lastCmp3 = lastCmp1;
        lastCmp3 += pTsCmpFwRegs->TS_CMP3_COUNT;
        lastCmp3 += pTsCmpFwRegs->TS_CMP3_OFFSET;
        *(volatile uint64_t *)&pIepHwRegs->CMP3_REG0 = lastCmp3;
    }
    
    /* Initialize CMP4 */
    if (pTsCmpFwRegs->TS_CMP4_COUNT != 0)
    {
        mask |= (1<<4);
        lastCmp4 = lastCmp1;
        lastCmp4 += pTsCmpFwRegs->TS_CMP4_COUNT;
        lastCmp4 += pTsCmpFwRegs->TS_CMP4_OFFSET;
        *(volatile uint64_t *)&pIepHwRegs->CMP4_REG0 = lastCmp4;
    }

    /* Initialize CMP5 */
    if (pTsCmpFwRegs->TS_CMP5_COUNT != 0)
    {
        mask |= (1<<5);
        lastCmp5 = lastCmp1;
        lastCmp5 += pTsCmpFwRegs->TS_CMP5_COUNT;
        lastCmp5 += pTsCmpFwRegs->TS_CMP5_OFFSET;
        *(volatile uint64_t *)&pIepHwRegs->CMP5_REG0 = lastCmp5;
    }

    /* Initialize CMP6 */
    if (pTsCmpFwRegs->TS_CMP6_COUNT != 0)
    {
        mask |= (1<<6);
        lastCmp6 = lastCmp1;
        lastCmp6 += pTsCmpFwRegs->TS_CMP6_COUNT;
        lastCmp6 += pTsCmpFwRegs->TS_CMP6_OFFSET;
        *(volatile uint64_t *)&pIepHwRegs->CMP6_REG0 = lastCmp6;
    }
    
    /* Clear pending events on CMP1, 3,4,5,6 */
    temp = pIepHwRegs->CMP_STATUS_REG;
    temp &= ~0xFF;
    temp |= mask;
    pIepHwRegs->CMP_STATUS_REG = temp;
    
    /* Enable events on CMP1, 3,4,5,6 */
    temp = pIepHwRegs->CMP_CFG_REG;
    temp &= ~(0xFF << 1);
    temp |= mask << 1;
    pIepHwRegs->CMP_CFG_REG = temp;

    /* Set firmware init flag */
    tsStat = pTsCtrlFwRegs->TS_STAT;
    tsStat &= ~TS_STAT_FW_INIT_MASK;
    tsStat |= BF_TS_FW_INIT_INIT << TS_STAT_FW_INIT_SHIFT;
    pTsCtrlFwRegs->TS_STAT = tsStat;
    
    while (1)
    {
        /* Read CMP status */
        temp = pIepHwRegs->CMP_STATUS_REG;
        temp &= 0x7A;   /* only keep CMP1,3,4,5,6 */
        
        /* Reload CMP1 */
        if (temp & (1<<1))
        {
            lastCmp1 += pTsCmpFwRegs->TS_CMP1_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0 = lastCmp1;
        }
        
        /* Reload CMP3 */
        if (temp & (1<<3))
        {
            lastCmp3 += pTsCmpFwRegs->TS_CMP3_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP3_REG0 = lastCmp3;
        }
        
        /* Reload CMP4 */
        if (temp & (1<<4))
        {
            lastCmp4 += pTsCmpFwRegs->TS_CMP4_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP4_REG0 = lastCmp4;
        }
        
        /* Reload CMP5 */
        if (temp & (1<<5))
        {
            lastCmp5 += pTsCmpFwRegs->TS_CMP5_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP5_REG0 = lastCmp5;
        }
        
        /* Reload CMP6 */
        if (temp & (1<<6))
        {
            lastCmp6 += pTsCmpFwRegs->TS_CMP6_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP6_REG0 = lastCmp6;
        }
        
        /* Clear status */
        pIepHwRegs->CMP_STATUS_REG = temp;
    }
}
