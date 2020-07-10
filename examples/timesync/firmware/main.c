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

/* ESC register offsets (bytes) */
#define ESC_AL_STATUS_OFFSET            ( 0x0130 )  /* Register Description: Actual State of the Device State Machine */
#define ESC_AL_STATUS_CUR_STATE_MASK    ( 0x000F )  /* ALStatus, current state mask */

#define ESC_DC_SYNC0_CYCLETIME_OFFSET   ( 0x09A0 )  /* Register Description: 32Bit Time between two consecutive SYNC0 pulses in ns */

/* ESC AL Status, current state */
#define STATE_PREOP     ((uint8_t) 0x02)    /* State PreOP */
#define STATE_SAFEOP    ((uint8_t) 0x04)    /* State SafeOP */
#define STATE_OP        ((uint8_t) 0x08)    /* State OP */

/* Determines wether ESC SYNC0 Cycle Time register used for calculating next CMP1 location during initialization:
    not defined:    ESC SYNC0 CT not:   base CMP1 time for first CMPx (3,4,5,6) is CMP1 read from HW register
    defined:        ESC SYNC0 CT used:  base CMP1 time for first CMPx (3,4,5,6) is CMP1 read from HW register + calculated CT */
/* #define USE_SYNC0_CT */

/* Pointer to Shared DMEM containing ESC Registers */
uint8_t * const pEscRegs = (uint8_t *)CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE;

/* Pointer to FW registers */
TsFwRegs * const pTsFwRegs = &gTsFwRegs;
/* Pointer to IEP0 registers */
CSL_icss_g_pr1_iep1_slvRegs * const pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)CSL_ICSS_IEP0_CFG_BASE;

void main(void)
{
    TsCtrlFwRegs *pTsCtrlFwRegs;
    TsCmpFwRegs *pTsCmpFwRegs;
    uint32_t tsCtrl, tsStat;
    uint8_t iepTsGblEn;
    uint8_t iepDefInc;
    uint8_t mask = 0;
    uint64_t countReg, nextCmp1;
    uint64_t nextCmp3, nextCmp4, nextCmp5, nextCmp6;
    uint16_t escCurState;
    uint32_t escSync0CycleTime_nsec;
    uint32_t temp;
#ifdef USE_SYNC0_CT
    uint32_t cmp1Count;
#endif

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

    while (1)
    {
        if (pTsCmpFwRegs->TS_CMP1_COUNT == 0) 
        {
            /* Test mode disabled */

            /* Wait for ESC current state to transition to SAFEOP */
            do {
                escCurState = *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] & 
                    ESC_AL_STATUS_CUR_STATE_MASK;
            } while (escCurState != STATE_SAFEOP);

#ifdef USE_SYNC0_CT
            /* Read ESC SYNC0 Cycle Time register */
            escSync0CycleTime_nsec = *(volatile uint32_t *)&pEscRegs[ESC_DC_SYNC0_CYCLETIME_OFFSET];
            
            /* Read IEP default count */
            iepDefInc = (pIepHwRegs->GLOBAL_CFG_REG & CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_MASK) 
                >> CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT;
            
            /* Compute CMP1 count corresponding to SYNC0 period */
            cmp1Count = (escSync0CycleTime_nsec / pTsCtrlFwRegs->TS_IEP_PRD_NSEC) * iepDefInc;            
#endif
            
            /* Wait for IEP enable */
            do {
                temp = *(volatile uint32_t *)&pIepHwRegs->GLOBAL_CFG_REG & 0x1;
            } while (temp == 0);
            
            /* Compute count for next CMP1 event */
            countReg = *(volatile uint64_t *)&pIepHwRegs->COUNT_REG0;            
            do {
                nextCmp1 = *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0;
            } while (nextCmp1 < countReg);
            
#ifdef USE_SYNC0_CT
            nextCmp1 += cmp1Count;            
#endif
        }
        else
        {
            /* Test mode enabled */
            
            /* Set up CMP1 */
            mask |= (1<<1);
            nextCmp1 = *(volatile uint64_t *)&pIepHwRegs->COUNT_REG0;
            nextCmp1 += pTsCmpFwRegs->TS_CMP1_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0 = nextCmp1;
        }
        
        /* Initialize CMP3 */
        if (pTsCmpFwRegs->TS_CMP3_COUNT != 0)
        {
            mask |= (1<<3);
            nextCmp3 = nextCmp1;
            nextCmp3 += pTsCmpFwRegs->TS_CMP3_COUNT;
            nextCmp3 += pTsCmpFwRegs->TS_CMP3_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP3_REG0 = nextCmp3;
        }
        
        /* Initialize CMP4 */
        if (pTsCmpFwRegs->TS_CMP4_COUNT != 0)
        {
            mask |= (1<<4);
            nextCmp4 = nextCmp1;
            nextCmp4 += pTsCmpFwRegs->TS_CMP4_COUNT;
            nextCmp4 += pTsCmpFwRegs->TS_CMP4_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP4_REG0 = nextCmp4;
        }

        /* Initialize CMP5 */
        if (pTsCmpFwRegs->TS_CMP5_COUNT != 0)
        {
            mask |= (1<<5);
            nextCmp5 = nextCmp1;
            nextCmp5 += pTsCmpFwRegs->TS_CMP5_COUNT;
            nextCmp5 += pTsCmpFwRegs->TS_CMP5_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP5_REG0 = nextCmp5;
        }

        /* Initialize CMP6 */
        if (pTsCmpFwRegs->TS_CMP6_COUNT != 0)
        {
            mask |= (1<<6);
            nextCmp6 = nextCmp1;
            nextCmp6 += pTsCmpFwRegs->TS_CMP6_COUNT;
            nextCmp6 += pTsCmpFwRegs->TS_CMP6_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP6_REG0 = nextCmp6;
        }
        
        /* Clear pending events on CMP1, 3,4,5,6 */
        temp = pIepHwRegs->CMP_STATUS_REG;
        temp &= ~0xFFFF;
        temp |= mask;
        pIepHwRegs->CMP_STATUS_REG = temp;
        
        /* Enable events on CMP1, 3,4,5,6 */
        temp = pIepHwRegs->CMP_CFG_REG;
        temp |= mask << 1;
        pIepHwRegs->CMP_CFG_REG = temp;
        
        /* Set firmware init flag */
        tsStat = pTsCtrlFwRegs->TS_STAT;
        tsStat &= ~TS_STAT_FW_INIT_MASK;
        tsStat |= BF_TS_FW_INIT_INIT << TS_STAT_FW_INIT_SHIFT;
        pTsCtrlFwRegs->TS_STAT = tsStat;

        /* Read ESC current state */
        escCurState = (pTsCmpFwRegs->TS_CMP1_COUNT == 0) ? 
            *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] & ESC_AL_STATUS_CUR_STATE_MASK : STATE_SAFEOP;
        while ((escCurState == STATE_SAFEOP) || (escCurState == STATE_OP))
        {
            /* 
               ESC firmware clears IEP_CMP_CFG_REG CMP3 enable bit.
               Temporary workaround for this issue it to continusly re-enable IEP_CMP_CFG_REG CMP3.
               Note a similar workaround would need to be added for other enabled CMPx (4,5,6).
            */
            temp = pIepHwRegs->CMP_CFG_REG;
            temp |= 1<<4;
            pIepHwRegs->CMP_CFG_REG = temp;
            
            /* Read CMP status */
            temp = pIepHwRegs->CMP_STATUS_REG & mask;

            /* Clear status */
            pIepHwRegs->CMP_STATUS_REG = temp;           
            
            if (pTsCmpFwRegs->TS_CMP1_COUNT != 0)
            {
                /* Test mode enabled */               
                /* Reload CMP1 */
                if (temp & (1<<1))
                {
                    nextCmp1 += pTsCmpFwRegs->TS_CMP1_COUNT;
                    *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0 = nextCmp1;
                }
            }
            
            /* Reload CMP3 */
            if (temp & (1<<3))
            {
                nextCmp3 += pTsCmpFwRegs->TS_CMP3_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP3_REG0 = nextCmp3;
            }
            
            /* Reload CMP4 */
            if (temp & (1<<4))
            {
                nextCmp4 += pTsCmpFwRegs->TS_CMP4_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP4_REG0 = nextCmp4;
            }
            
            /* Reload CMP5 */
            if (temp & (1<<5))
            {
                nextCmp5 += pTsCmpFwRegs->TS_CMP5_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP5_REG0 = nextCmp5;
            }
            
            /* Reload CMP6 */
            if (temp & (1<<6))
            {
                nextCmp6 += pTsCmpFwRegs->TS_CMP6_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP6_REG0 = nextCmp6;
            }
            
            /* Read ESC current state */
            escCurState = (pTsCmpFwRegs->TS_CMP1_COUNT == 0) ? 
                *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] & ESC_AL_STATUS_CUR_STATE_MASK : STATE_SAFEOP;
        }
       
        /* Disable events on CMP1, 3,4,5,6 */
        temp = pIepHwRegs->CMP_CFG_REG;
        temp &= ~mask << 1;
        pIepHwRegs->CMP_CFG_REG = temp;
        
        /* Clear firmware init flag */
        tsStat = pTsCtrlFwRegs->TS_STAT;
        tsStat &= ~TS_STAT_FW_INIT_MASK;
        tsStat |= BF_TS_FW_INIT_UNINIT << TS_STAT_FW_INIT_SHIFT;
        pTsCtrlFwRegs->TS_STAT = tsStat;        
    }
}
