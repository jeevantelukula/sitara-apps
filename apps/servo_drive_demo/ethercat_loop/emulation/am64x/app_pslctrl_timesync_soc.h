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

#ifndef _APP_PSL_CTRL_TIMESYNC_SOC_H_
#define _APP_PSL_CTRL_TIMESYNC_SOC_H_

/* ICSSG functional clock source selection options */
/* CTRLMMR_ICSSGn_CLKSEL:CORE_CLKSEL */
#define APP_PSLCTRL_TS_CORE_CLKSEL_MAIN_PLL2_HSDIV0_CLKOUT \
    ( 0 )   /* ICSSG functional clock source MAIN_PLL2_HSDIV0_CLKOUT */
#define APP_PSLCTRL_TS_CORE_CLKSEL_MAIN_PLL0_HSDIV9_CLKOUT \
    ( 1 )   /* ICSSG functional clock source MAIN_PLL0_HSDIV9_CLKOUT */

/* CTRLMMR_ICSSGn_CLKSEL:IEP_CLKSEL */
#define APP_PSLCTRL_TS_IEP_CLKSEL_MAIN_PLL2_HSDIV5_CLKOUT   ( 0 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_MAIN_PLL0_HSDIV6_CLKOUT   ( 1 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_CPSW2G_CPTS_RFT_CLK       ( 2 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_CPTS_RFT_CLK              ( 3 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_MCU_EXT_REFCLK0           ( 4 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_EXT_REFCLK1               ( 5 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_SERDES0_IP1_LN0_TXMCLK    ( 6 )
#define APP_PSLCTRL_TS_IEP_CLKSEL_MAIN_SYSCLK0              ( 7 )

/* ICSSGn_CORE_CLK selection */
#define APP_PSLCTRL_TS_CORE_CLKSEL  ( APP_PSLCTRL_TS_CORE_CLKSEL_MAIN_PLL2_HSDIV0_CLKOUT )
/* ICSSGn_IEP_CLK selection */
#define APP_PSLCTRL_TS_IEP_CLKSEL   ( APP_PSLCTRL_TS_IEP_CLKSEL_MAIN_PLL0_HSDIV6_CLKOUT )

#endif /* _APP_PSL_CTRL_TIMESYNC_SOC_H_ */
