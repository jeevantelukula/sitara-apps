/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <ti/csl/tistdtypes.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include "ti_fsi_pruss_intc_mapping.h"  /* INTC configuration */
#include "cfg_icss.h"

static const PRUICSS_IntcInitData gPruicssIntcInitdata = PRUICSS_INTC_INITDATA;

/*
 *  ======== initIcss ========
 */
/* Initialize ICSSG */
int32_t initIcss(
    PRUICSS_MaxInstances icssInstId, 
    PRUICSS_Handle *pPruIcssHandle    
)
{
    PRUICSS_Config *pruIcssCfg; /* ICSS configuration */
    PRUICSS_Handle pruIcssHandle;
    uint8_t i;
    int32_t status;
    
    /* Get SoC level PRUICSS initial configuration */
    status = PRUICSS_socGetInitCfg(&pruIcssCfg);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return CFG_ICSS_SERR_INIT_ICSS;
    }
    
    /* Create ICSS PRU instance */
    pruIcssHandle = PRUICSS_create(pruIcssCfg, icssInstId);
    if (pruIcssHandle == NULL) {
        return CFG_ICSS_SERR_INIT_ICSS;
    }
    
    /* Disable PRUs & RTUs */
    for (i = 0; i < PRUICSS_MAX_PRU; i++)
    {
        status = PRUICSS_pruDisable(pruIcssHandle, i);
        if (status != PRUICSS_RETURN_SUCCESS) {
            return CFG_ICSS_SERR_INIT_ICSS;
        }
    }
    
    /* Set ICSS pin mux to default */
    PRUICSS_pinMuxConfig(pruIcssHandle, PRUICSS_PINMUX_DEF);
    
    /* Initialize ICSS INTC */
    status = PRUICSS_pruIntcInit(pruIcssHandle, &gPruicssIntcInitdata);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return CFG_ICSS_SERR_INIT_ICSS;        
    }
    
    *pPruIcssHandle = pruIcssHandle;
    
    return CFG_ICSS_SOK;
}

/*
 *  ======== initPruFsi ========
 */
/* Initialize PRU for FSI Transmit */
int32_t initPruFsi(
    PRUICSS_Handle pruIcssHandle,
    PRUSS_PruCores pruInstId,
    const uint32_t *sourceMemData,
    uint32_t dataSize,
    const uint32_t *sourceMemInstr,
    uint32_t instrSize
)
{
    int32_t size;
    uint32_t offset;            /* Offset at which write will happen */
    int32_t status;
    
    /* Reset PRU */
    status = PRUICSS_pruReset(pruIcssHandle, pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return CFG_ICSS_SERR_INIT_PRU;
    }
    
    /* Initialize DMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, PRU_ICSS_DATARAM(pruInstId));
    if (size == 0) {
        return CFG_ICSS_SERR_INIT_PRU;
    }
    
    /* Initialize IMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, PRU_ICSS_IRAM(pruInstId));
    if (size == 0)
    {
        return CFG_ICSS_SERR_INIT_PRU;
    }

    /* Write DMEM */
    offset = 0;
    size = PRUICSS_pruWriteMemory(pruIcssHandle, PRU_ICSS_DATARAM(pruInstId), offset, sourceMemData, dataSize);
    if (size == 0)
    {
        return CFG_ICSS_SERR_INIT_PRU;
    }
    
    /* Write IMEM */
    offset = 0;
    size = PRUICSS_pruWriteMemory(pruIcssHandle, PRU_ICSS_IRAM(pruInstId), offset, sourceMemInstr, instrSize);
    if (size == 0)
    {
        return CFG_ICSS_SERR_INIT_PRU;
    }    
    
    /* Enable PRU */
    status = PRUICSS_pruEnable(pruIcssHandle, pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return CFG_ICSS_SERR_INIT_PRU;
    }
    
    return CFG_ICSS_SOK;
}
