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

#include <stdint.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>

#include "test_utils.h"
#include "timesyncDrv_api.h"

#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>

uint64_t gOverheadTime;
uint64_t gStartTime, gEndTime, gTotalTime;

void init_profiling(void)
{
    CSL_armR5PmuEnableAllCntrs(1);  /* Set/clear PMCR E-bit */
    CSL_armR5PmuResetCntrs();       /* Set PMCR P-bit */
    CSL_armR5PmuResetCycleCnt();    /* Set PMCR C-bit */
    CSL_armR5PmuEnableCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM, 1);    /* Set PMCNTENSET for event */
    CSL_armR5PmuClearCntrOverflowStatus(0x80000007);
}

uint32_t readPmu(void)
{
    return (CSL_armR5PmuReadCntr(0x1F));
}

/* Get ICSSG ID for PWM DRV */
int32_t getIcssgId(
    PRUICSS_MaxInstances icssInstId,
    uint8_t *pIcssId
)
{
    /* Translate ICSSG hardware module ID to PWM API */
    if (icssInstId == PRUICCSS_INSTANCE_ONE) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_0;
        return TEST_UTILS_ERR_NERR;
    }
    else if (icssInstId == PRUICCSS_INSTANCE_TWO) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_1;
        return TEST_UTILS_ERR_NERR;
    }
    else if (icssInstId == PRUICCSS_INSTANCE_MAX) {
        *pIcssId = ICSSG_TS_DRV__ICSSG_ID_2;
        return TEST_UTILS_ERR_NERR;
    }
    else {
        *pIcssId = 0;
        return TEST_UTILS_ERR_INV_PRMS;
    }
}

/* Get ICSSG ID for PWM DRV */
int32_t getPruId(
    PRUSS_PruCores pruInstId,
    uint8_t *pPruId
)
{
    int32_t retVal = 0;
    
    /* Translate PRU hardware module ID to PWM API */
    switch (pruInstId)
    {
        case PRUICCSS_PRU0:
            *pPruId = ICSSG_TS_DRV__PRU_ID_0;
            retVal = TEST_UTILS_ERR_NERR;
            break;
        case PRUICCSS_PRU1:
            *pPruId = ICSSG_TS_DRV__PRU_ID_1;
            retVal = TEST_UTILS_ERR_NERR;
            break;
        case PRUICCSS_RTU0:
            *pPruId = ICSSG_TS_DRV__RTU_ID_0;
            retVal = TEST_UTILS_ERR_NERR;
            break;
        case PRUICCSS_RTU1:
            *pPruId = ICSSG_TS_DRV__RTU_ID_1;
            retVal = TEST_UTILS_ERR_NERR;
            break;
        case PRUICCSS_TPRU0:
            *pPruId = ICSSG_TS_DRV__TPRU_ID_0;
            retVal = TEST_UTILS_ERR_NERR;
            break;
        case PRUICCSS_TPRU1:
            *pPruId = ICSSG_TS_DRV__TPRU_ID_1;
            retVal = TEST_UTILS_ERR_NERR;
            break;
        default:
            *pPruId = 0;
            retVal = TEST_UTILS_ERR_INV_PRMS;
            break;
    }
    
    return retVal;
}
