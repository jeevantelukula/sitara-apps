/**
 *  \file   profile.c
 *
 *  \brief  This file contains the profile specific functions and
 *          macros.
 *
 */

/*
 * Copyright (C) 2014 - 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 *
 */

#include <stdint.h>
#include "ti/csl/arch/r5/csl_arm_r5_pmu.h"

#define PMU_CNTR_NUM_BRANCH             (2u)
#define PMU_CNTR_NUM_ICACHE_MISS        (1u)
#define PMU_CNTR_NUM_DCACHE_MISS        (0u)

uint64_t gOverheadTime;
uint64_t gStartTime, gEndTime, gTotalTime;

void init_profiling(void)
{
    uint32_t val;

    CSL_armR5PmuCfg(0, 0, 1);
    /* Clear the overflow */
    val = CSL_armR5PmuReadCntrOverflowStatus();
    val &= 0x80000007;
    CSL_armR5PmuClearCntrOverflowStatus(val);
    CSL_armR5PmuCfgCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM, CSL_ARM_R5_PMU_EVENT_TYPE_CYCLE_CNT);
    /* I-Cache */
    CSL_armR5PmuCfgCntr(PMU_CNTR_NUM_ICACHE_MISS, CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS);
    /* D-Cache */
    CSL_armR5PmuCfgCntr(PMU_CNTR_NUM_DCACHE_MISS, CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_MISS);
    /* Branch */
    CSL_armR5PmuCfgCntr(PMU_CNTR_NUM_BRANCH, CSL_ARM_R5_PMU_EVENT_TYPE_B_IMMEDIATE);
    CSL_armR5PmuEnableAllCntrs(0);
    CSL_armR5PmuResetCycleCnt();      /* Set PMCR C-bit */
    CSL_armR5PmuResetCntrs();

    CSL_armR5PmuEnableCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM, 1);
    CSL_armR5PmuEnableCntr(PMU_CNTR_NUM_BRANCH, 1);
    CSL_armR5PmuEnableCntr(PMU_CNTR_NUM_ICACHE_MISS, 1);
    CSL_armR5PmuEnableCntr(PMU_CNTR_NUM_DCACHE_MISS, 1);
    CSL_armR5PmuEnableAllCntrs(1);
    CSL_armR5PmuEnableCntrOverflowIntr(PMU_CNTR_NUM_BRANCH, 1U);
    CSL_armR5PmuEnableCntrOverflowIntr(PMU_CNTR_NUM_BRANCH, 0U);


}

void resetPmuEventCounters(void)
{
    CSL_armR5PmuResetCntrs();
}

uint32_t readPmuInstCacheMiss(void)
{
    return (CSL_armR5PmuReadCntr(PMU_CNTR_NUM_ICACHE_MISS));
}

uint32_t readPmuDataCacheMiss(void)
{
    return (CSL_armR5PmuReadCntr(PMU_CNTR_NUM_DCACHE_MISS));
}

uint32_t readPmuBranch(void)
{
    return (CSL_armR5PmuReadCntr(PMU_CNTR_NUM_BRANCH));
}

uint32_t readPmu(void)
{
    return (CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM));
}
