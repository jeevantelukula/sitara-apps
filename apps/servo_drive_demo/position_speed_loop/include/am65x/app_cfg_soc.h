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

#ifndef APP_CFG_SOC_H_
#define APP_CFG_SOC_H_

#include <ti/board/board.h>
#include <ti/board/src/am65xx_evm/am65xx_evm_pinmux.h>
#include "cfg_mcu_intr_soc.h"

//#define ENABLE_BOARD

extern pinmuxPerCfg_t gFsiPinCfg[];

/* TS MCU interrupt number */
#define TS_MAIN2MCU_RTR_PLS_MUX_INTR0 \
    ( 33 )  /* MAIN2MCU_RTR_PLS_MUX_INTR17 */

/* Time Sync Int */
#define TS_INT_NUM                  ( CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_0 + TS_MAIN2MCU_RTR_PLS_MUX_INTR0 )
#define TS_INT_TYPE                 ( CSL_VIM_INTR_TYPE_PULSE )
#define TS_INT_MAP                  ( CSL_VIM_INTR_MAP_IRQ )
#define TS_INT_PRI                  ( 1 ) /* 0(highest)..15(lowest) */

#endif /* APP_CFG_SOC_H_ */
