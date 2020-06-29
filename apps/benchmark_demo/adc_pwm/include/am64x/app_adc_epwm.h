/**
 *  \file   app_adc_epwm.h
 *
 *  \brief  This file contains the prototypes for the ADC/PWM benchmark functions.
 *
 */

/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _APP_ADC_EPWM_H_
#define _APP_ADC_EPWM_H_

#include <stdint.h>

#include <stdio.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/board/board.h>
#include <ti/osal/osal.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/csl_adc.h>
#include <ti/csl/csl_epwm.h>
#include <ti/csl/csl_intr_router.h>

#ifndef IO_CONSOLE
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#endif

#include "benchmark_log.h"
#include "profile.h"
#include "benchmark_stat.h"
#include "benchmark_timer_interrupt.h"

#include "arm_math.h"
#include "math_helper.h"

/* ----------------------------------------------------------------------
** Structures and Enums
** ------------------------------------------------------------------- */
/** \brief Structure holding the EPWM configuration parameters. */
typedef struct CSL_AppEpwmCfg
{
    CSL_EpwmTimebaseCfg_t       tbCfg;
    /**< Timebase Sub-module configuration data structure. */
    CSL_EpwmCounterCmpCfg_t     ccCfg;
    /**< Counter comparator values . */
    CSL_EpwmAqActionCfg_t       aqCfg;
    /**< Action Qualifier Sub-module configuration data structure. */
    CSL_EpwmDeadbandCfg_t       dbCfg;
    /**< Dead band Sub-module configuration data structure. */
    CSL_EpwmChopperCfg_t        chpCfg;
    /**< Chopper sub-module configuration data structure. */
    CSL_EpwmTripzoneCfg_t       tzCfg;
    /**< Trip-zone sub-module configuration data structure. */
    CSL_EpwmEtCfg_t             etCfg;
    /**< Event Trigger sub-module configuration data structure. */
    CSL_EpwmHighResolutionCfg_t hrCfg;
    /**< High Resolution sub-module configuration data structure. */
} CSL_AppEpwmCfg_t;

/**< \brief Structure holding the EPWM object data structure. */
typedef struct CSL_AppEpwmObj
{
    uint32_t                pwmCh;
    /**< EPWM channel [A or B]. */
    uint32_t                instAddr;
    /**< EPWM instance address. */
    uint32_t                funcClk;
    /**< Functional clock(in Hz) input to the PWMSS. */
    uint32_t                enableDeadband;
    /**< Enable dead band sub-module processing. */
    uint32_t                enableChopper;
    /**< Enable chopper sub-module processing. */
    uint32_t                enableTripZone;
    /**< Enable Trip zone processing. */
    uint32_t                enableEventTrigger;
    /**< Enable Event trigger. */
    uint32_t                enableHighResolution;
    /**< Enable High resolution pwm feature. */
    CSL_AppEpwmCfg_t     pwmCfg;
    /**< EPWM configuration data structure. */
} CSL_AppEpwmObj_t;

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */
typedef struct
{
    int64_t adcIsrCnt;               /* ADC interrupt counter */
    int64_t adcIsrCntPrev;           /* previous ADC interrupt counter */
    int64_t pwmIsrCnt;               /* PWM interrupt counter */
    int64_t pwmIsrCntPrev;           /* previous PWM interrupt counter */
    int32_t pwmIsrErrCnt;            /* PWM interrupt error count */
} adcpwm_int_stat;

extern adcpwm_int_stat gAdcPwmIntStat;

/* Function prototypes */
/* ADC related functions */
static void appADCWait(void);
static void appADCStop(void);
static void appADCConfigureInterrupt(void);
static void appADCModuleEnable(void);
static void appADCModuleDisable(void);

int32_t appADCPWMBench(uint32_t *adcInData, int32_t *adcInDataSize);
void appADCPWMBenchInit(int32_t freq);
void appADCPWMBenchDeInit(void);

/* PWM related functions */
static int32_t appEpwmBoardInit(void);
static void appEpwmCfgEPwmPads(void);
static void appEpwmTbClockEnable(uint32_t pwmId);
static void appEpwmIntrISR(void *handle);
static void appEpwmPwmCfg(CSL_AppEpwmObj_t *pObj);
static int32_t appEpwmGetPwmFuncClock(CSL_AppEpwmObj_t *pObj);
static void appEpwmTimebaseModuleCfg(uint32_t baseAddr,
                                     uint32_t pwmFuncClk,
                                     CSL_EpwmTimebaseCfg_t *pTbCfg);
static void appEpwmCounterComparatorCfg(uint32_t baseAddr,
                                        CSL_EpwmCounterCmpCfg_t *pCcCfg);

int32_t appEpwmInit(void);
int32_t appEpwmSetOutFreq(int32_t freq);
int32_t appEpwmSetDutyCycle(int32_t dc);

#endif /* _APP_ADC_EPWM_H_ */
