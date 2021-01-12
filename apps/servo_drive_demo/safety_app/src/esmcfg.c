/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <stdbool.h>

/* TI CSL Includes */
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/csl_esm.h>

#include "esmcfg.h"
#include "ratcfg.h"

/* ti/csl/soc/am64x/src/cslr_soc_baseaddress.h */
#define MAIN_ESM_MMR        (CSL_ESM0_CFG_BASE + M4F_RAT_MODULES_OFFSET)
#define MCU_ESM_MMR         (CSL_MCU_ESM0_CFG_BASE + M4F_RAT_MODULES_OFFSET)

int32_t configure_esm()
{
    /* return false on success */
    int32_t retVal = CSL_PASS;

    /* ti/csl/soc/am64x/src/cslr_intr_esm0.h */
    uint32_t error_list[] = {0,1,2,3,6,8,13,14,18,19,20,21,22,23,29,44,62,63,64,65,66,67,69,71,76,77,80,81,82,83,84,85,90,103,104,128,129,130,131,132,133,134,135};

    uint32_t i = sizeof(error_list) / sizeof(uint32_t);

    /* reset Error Signaling Module before configuration */
    retVal |= ESMReset(MAIN_ESM_MMR);

    /* make sure we're not in force error mode */
    retVal |= ESMSetMode(MAIN_ESM_MMR, ESM_OPERATION_MODE_NORMAL);

    /* Number of clock cycles to show the error signal on the error pin */
    retVal |= ESMSetErrPinLowTimePreload(MAIN_ESM_MMR, 65535);

    /* Enable ESM Error Signals from error_list[] */
    /* ti/csl/soc/am64x/src/cslr_intr_esm0.h */
    while(i-- != 0) {
        retVal |= ESMEnableIntr(MAIN_ESM_MMR, error_list[i]);
    }

    /* Enable for all ESM Error Signals */
    retVal |= ESMEnableGlobalIntr(MAIN_ESM_MMR);

    /**********************************************************************/

    /* reset Error Signaling Module before configuration */
    retVal |= ESMReset(MCU_ESM_MMR);

    /* make sure we're not in force error mode */
    retVal |= ESMSetMode(MCU_ESM_MMR, ESM_OPERATION_MODE_NORMAL);

    /* Number of clock cycles to show the error signal on the error pin */
    retVal |= ESMSetErrPinLowTimePreload(MCU_ESM_MMR, 65535);

    /* Enable ESM Error Signals */
    /* ti/csl/soc/am64x/src/cslr_intr_esm0.h */
    retVal |= ESMEnableIntr(MCU_ESM_MMR, 1);
    retVal |= ESMEnableIntr(MCU_ESM_MMR, 2);

    /* Enable for all ESM Error Signals */
    retVal |= ESMEnableGlobalIntr(MCU_ESM_MMR);

    return retVal;
}

int32_t main_esm_clear_error()
{
    /* return false on success */
    int32_t retVal = CSL_PASS;

    uint32_t activeError, intrStatus;
    esmGroupIntrStatus_t groupStatus; 

    /* retreive pending error number, clear it, then validate it is clear */
    ESMGetGroupIntrStatus(MAIN_ESM_MMR, ESM_INTR_PRIORITY_LEVEL_LOW, &groupStatus);
    activeError = groupStatus.highestPendLvlIntNum;

    retVal |= ESMClearIntrStatus(MAIN_ESM_MMR, activeError);

    do {
        retVal |= ESMGetIntrStatus(MAIN_ESM_MMR, activeError, &intrStatus);
    } while (intrStatus != 0);

    /* clear MCU ESM only after clearing MAIN ESM above
        otherwise the MCU ESM may not re-trigger */
    retVal |= ESMClearIntrStatus(MCU_ESM_MMR, 1);
    retVal |= ESMClearIntrStatus(MCU_ESM_MMR, 2);

    return retVal;
}

