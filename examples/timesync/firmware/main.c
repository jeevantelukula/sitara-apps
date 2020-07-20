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

/* Pointer to Shared DMEM containing ESC Registers */
uint8_t * const pEscRegs = (uint8_t *)CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE;

/* Pointer to FW registers */
TsFwRegs * const pTsFwRegs = &gTsFwRegs;
/* Pointer to IEP0 HW registers */
CSL_icss_g_pr1_iep1_slvRegs * const pIepHwRegs = (CSL_icss_g_pr1_iep1_slvRegs *)CSL_ICSS_IEP0_CFG_BASE;

void main(void)
{
    TsCtrlFwRegs *pTsCtrlFwRegs;        /* pointer to TS control FW registers */
    TsCmpFwRegs *pTsCmpFwRegs;          /* pointer to TS CMPx count & offset FW registers */
    uint32_t tsCtrl, tsStat;            /* TS control and status FW control registers */
    uint8_t iepTsGblEn;                 /* TS global enable */
    uint16_t mask = 0;                  /* TS CMPx enable mask, bitX=1/0 => CMPx enabled/disabled */
    uint64_t countReg, nextCmp1;        /* IEP counter reg, next CMP1 count */
    uint64_t nextCmp7, nextCmp8, nextCmp9, nextCmp10; /* next CMP7,8,9,10 count */
    uint16_t escCurState;               /* ESC PRU firmware current state */
    uint32_t escSync0CycleTime_nsec;    /* ESC PRU firmware SYNC0 Cycle Time (nsec.) */
    uint32_t temp;

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
            /* Test mode disabled: SYNC0 cycle time (IEP CMP1 timing) determined by 
               EtherCAT Slave Controller (ESC) PRU firmware */
            
            /* Enter TS init state:
                Time Sync (TS) firmware is sychronized to ESC PRU firmware state & 
                TwinCAT SYNC0 Cycle Time. TS firmware state machine transitions depend 
                on ESC state transitions.
            */

            /* 
               Wait for ESC current state to transition to SAFEOP.
               ESC SYNC0 Cycle Time (nsec.) is not valid before this ESC state.
            */
            do {
                escCurState = *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] & 
                    ESC_AL_STATUS_CUR_STATE_MASK;
            } while (escCurState != STATE_SAFEOP);

            /* Read ESC SYNC0 Cycle Time register */
            escSync0CycleTime_nsec = *(volatile uint32_t *)&pEscRegs[ESC_DC_SYNC0_CYCLETIME_OFFSET];
                       
            /* Wait for IEP enable */
            do {
                temp = *(volatile uint32_t *)&pIepHwRegs->GLOBAL_CFG_REG & 0x1;
            } while (temp == 0);
            
            /* Determine CMP1 count for next SYNC0 period (n) boundary:
                - Sample current IEP counter register,
                - Wait for CMP1 to be set greater than counter by ESC firmware. */
            countReg = *(volatile uint64_t *)&pIepHwRegs->COUNT_REG0;            
            do {
                nextCmp1 = *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0;
            } while (nextCmp1 < countReg);
            
            /* Compute CMP1 count for SYNC0 period (n+1) boundary using 
               ESC SYNC0 Cycle Time firmware register.
               - Use ESC SYNC0 Cycle Time in compution.
               - ESC sets IEP clock to 200 MHz & IEP_GLOBAL_CFG_REG:DEFAULT_INC HW regiseter to 5 (1/200 MHz).
                 These values are assumed (ignored) in the calculation, i.e.:
                    count = esc_ct_nsec / iep_prd_nsec * iep_def_inc
                          = esc_ct_nsec / 5 * 5 
                          = esc_ct_nsec
               - Use computed CMP1 for initializing TS CMPx (7,8,9,10), CMPx will then
                 track with SYNC0 period. 
            */
            nextCmp1 += escSync0CycleTime_nsec;            
        }
        else
        {
            /* Test mode enabled: SYNC0 cycle time (IEP CMP1 timing) determined by 
               by TS firmware.*/
            
            /* Set up CMP1, start of SYNC0 period is based on current IEP count
               and is arbitrary */
            mask |= (1<<1);
            nextCmp1 = *(volatile uint64_t *)&pIepHwRegs->COUNT_REG0;
            nextCmp1 += pTsCmpFwRegs->TS_CMP1_COUNT;
            *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0 = nextCmp1;
        }
        
        /* Initialize CMPx (7,8,9,10)
            - use CMP1 for SYNC0 (n+1) period boundary as base time
            - (CMPx period==0) indicates CMPx is diabled
            - use CMPx period and offset (pre- and post-trigger) for initialization */
            
        /* Initialize CMP7 */
        if (pTsCmpFwRegs->TS_CMP7_COUNT != 0)
        {
            mask |= (1<<7);
            nextCmp7 = nextCmp1;
            nextCmp7 += pTsCmpFwRegs->TS_CMP7_COUNT;
            nextCmp7 += pTsCmpFwRegs->TS_CMP7_OFFSET; // *** negative offset shouldnt be greater than period
            *(volatile uint64_t *)&pIepHwRegs->CMP7_REG0 = nextCmp7;
        }
        
        /* Initialize CMP8 */
        if (pTsCmpFwRegs->TS_CMP8_COUNT != 0)
        {
            mask |= (1<<8);
            nextCmp8 = nextCmp1;
            nextCmp8 += pTsCmpFwRegs->TS_CMP8_COUNT;
            nextCmp8 += pTsCmpFwRegs->TS_CMP8_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP8_REG0 = nextCmp8;
        }

        /* Initialize CMP9 */
        if (pTsCmpFwRegs->TS_CMP9_COUNT != 0)
        {
            mask |= (1<<9);
            nextCmp9 = nextCmp1;
            nextCmp9 += pTsCmpFwRegs->TS_CMP9_COUNT;
            nextCmp9 += pTsCmpFwRegs->TS_CMP9_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP9_REG0 = nextCmp9;
        }

        /* Initialize CMP10 */
        if (pTsCmpFwRegs->TS_CMP10_COUNT != 0)
        {
            mask |= (1<<10);
            nextCmp10 = nextCmp1;
            nextCmp10 += pTsCmpFwRegs->TS_CMP10_COUNT;
            nextCmp10 += pTsCmpFwRegs->TS_CMP10_OFFSET;
            *(volatile uint64_t *)&pIepHwRegs->CMP10_REG0 = nextCmp10;
        }
        
        /* Clear pending events on enabled CMPx 7,8,9,10 & CMP1 when in test mode */
        temp = pIepHwRegs->CMP_STATUS_REG;
        temp &= ~0xFFFF;
        temp |= mask;
        pIepHwRegs->CMP_STATUS_REG = temp;
        
        /* Enable events on enabled CMPx 7,8,9,10 & CMP1 when in test mode */
        temp = pIepHwRegs->CMP_CFG_REG;
        temp |= mask << 1;
        pIepHwRegs->CMP_CFG_REG = temp;
        
        /* Set firmware init flag -- indicate FW initialized to host CPU */
        tsStat = pTsCtrlFwRegs->TS_STAT;
        tsStat &= ~TS_STAT_FW_INIT_MASK;
        tsStat |= BF_TS_FW_INIT_INIT << TS_STAT_FW_INIT_SHIFT;
        pTsCtrlFwRegs->TS_STAT = tsStat;

        /* Enter TS active state:
            TS remains active as long as ESC firmware remains in SAFEOP or OP states.
            Otherwise TS exits active mode and re-enters init state to sync with ESC firmare.
        */
        
        /* Read ESC current state */
        escCurState = (pTsCmpFwRegs->TS_CMP1_COUNT == 0) ? 
            *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] & ESC_AL_STATUS_CUR_STATE_MASK : STATE_SAFEOP;
        while ((escCurState == STATE_SAFEOP) || (escCurState == STATE_OP))
        {
            /* Read CMP status */
            temp = pIepHwRegs->CMP_STATUS_REG & mask;
            /* Clear status */
            pIepHwRegs->CMP_STATUS_REG = temp;           
            
            if (pTsCmpFwRegs->TS_CMP1_COUNT != 0)
            {
                /* Test mode enabled */               
                
                /* Compute CMP1 for next period & reload CMP1 */
                if (temp & (1<<1))
                {
                    nextCmp1 += pTsCmpFwRegs->TS_CMP1_COUNT;
                    *(volatile uint64_t *)&pIepHwRegs->CMP1_REG0 = nextCmp1;
                }
            }
            
            /* Compute CMP7 for next period & reload CMP7 */
            if (temp & (1<<7))
            {
                nextCmp7 += pTsCmpFwRegs->TS_CMP7_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP7_REG0 = nextCmp7;
            }
            
            /* Compute CMP8 for next period & reload CMP8 */
            if (temp & (1<<8))
            {
                nextCmp8 += pTsCmpFwRegs->TS_CMP8_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP8_REG0 = nextCmp8;
            }
            
            /* Compute CMP9 for next period & reload CMP9 */
            if (temp & (1<<9))
            {
                nextCmp9 += pTsCmpFwRegs->TS_CMP9_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP9_REG0 = nextCmp9;
            }
            
            /* Compute CMP10 for next period & reload CMP10 */
            if (temp & (1<<10))
            {
                nextCmp10 += pTsCmpFwRegs->TS_CMP10_COUNT;
                *(volatile uint64_t *)&pIepHwRegs->CMP10_REG0 = nextCmp10;
            }
            
            /* Read ESC current state */
            escCurState = (pTsCmpFwRegs->TS_CMP1_COUNT == 0) ? 
                *(volatile uint16_t *)&pEscRegs[ESC_AL_STATUS_OFFSET] & ESC_AL_STATUS_CUR_STATE_MASK : STATE_SAFEOP;
        }
       
        /* Disable events on enabled CMPx 7,8,9,10 & CMP1 when in test mode */
        temp = pIepHwRegs->CMP_CFG_REG;
        temp &= ~mask << 1;
        pIepHwRegs->CMP_CFG_REG = temp;
        
        /* Set firmware init flag -- indicate FW not initialized to host CPU */
        tsStat = pTsCtrlFwRegs->TS_STAT;
        tsStat &= ~TS_STAT_FW_INIT_MASK;
        tsStat |= BF_TS_FW_INIT_UNINIT << TS_STAT_FW_INIT_SHIFT;
        pTsCtrlFwRegs->TS_STAT = tsStat;        
    }
}
