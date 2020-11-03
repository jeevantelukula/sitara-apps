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
#include <ti/csl/csl_rat.h>

#include "ratcfg.h"

/* ti/csl/soc/am64x/src/cslr_mcu_m4fss0_baseaddress.h */
#define MCU_RAT_MMR_BASE    0x44200000UL

bool configure_rat()
{
    /* return false on success */
    bool retVal = false;
    CSL_ratRegs * MCU_RAT_MMR = (CSL_ratRegs *) MCU_RAT_MMR_BASE;

    /* ti/csl/arch/m4/src/startup.c */
    /* CSL already uses regions 0 - 7 by default */
    retVal |= !CSL_ratDisableRegionTranslation(MCU_RAT_MMR, 6);    // re-defining for MAIN ESM

    /* sizeInBytes, baseAddress, translatedAddress */
    /* { byte size, 32-bit local M4F memory map, 64-bit SoC memory map */
    CSL_RatTranslationCfgInfo ratCfg6 = { 0x00001000, 
        CSL_ESM0_CFG_BASE + MCU_RAT_OFFSET6, CSL_ESM0_CFG_BASE };  // 4kB for ESM

    retVal |= !CSL_ratConfigRegionTranslation(MCU_RAT_MMR, 6, &ratCfg6);

    return retVal;
}

