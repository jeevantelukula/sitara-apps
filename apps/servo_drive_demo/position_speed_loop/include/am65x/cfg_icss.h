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

#ifndef _CFG_ICSS_H_
#define _CFG_ICSS_H_

#include <ti/csl/tistdtypes.h>

#define CFG_ICSS_SOK            (  0 )  /* no error */      
#define CFG_ICSS_SERR_INIT_ICSS ( -1 )  /* initialize ICSS error */
#define CFG_ICSS_SERR_INIT_PRU  ( -2 )  /* initialize PRU error */
#define CFG_ICSS_SERR_INIT_FSI  ( -3 )  /* initialize FSI error */

/* Default ICSS pin mux setting */
#define PRUICSS_PINMUX_DEF      ( 0x0 )

/* Initialize ICSSG */
int32_t initIcss(
    PRUICSS_MaxInstances icssInstId, 
    PRUICSS_Handle *pPruIcssHandle    
);

/* Initialize PRU for FSI */
int32_t initPruFsi(
    PRUICSS_Handle pruIcssHandle,
    PRUSS_PruCores pruInstId,
    const uint32_t *sourceMemData,
    uint32_t dataSize,
    const uint32_t *sourceMemInstr,
    uint32_t instrSize
);

#endif /* _CFG_ICSS_H_ */
