/**
 *  \file   app_adc_epwm.c
 *
 *  \brief  This file contains functions and macros for ADC/PWM benchmark.
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

/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */

#include "app_adc_epwm.h"

/* -------------------------------------------------------------------
 * Declare Macros
 * ------------------------------------------------------------------- */
/* ADC related Macros */
#define APP_ADC_MODULE          (CSL_ADC0_BASE)
#define APP_ADC_RANGE_MAX       (4096U)

#define APP_ADC_DIV             (1U)
/* Reference voltage for ADC - should be given in mV*/
#define APP_ADC_REF_VOLTAGE     (1800U)

/** \brief Pad configuration settings. */
#define PADCFG_GPI_SELECT_MASK          (0xFFFFFFF0)    /* no write to PULLUDEN */
#define PADCFG_GPI_INPUT_ENABLE         (0x01240000)    /* PULLUDEN=1 => disabled */
#define PADCFG_GPO_SELECT_MASK          (0xFFDAFFF0)    /* PULLUDEN=0 => enabled */
#define PADCFG_GPO_OUTPUT_ENABLE        (0x00000000)    /* no write to PULLUDEN */
#define CTRLMMR_PADCONFIG33_MMODE       ( 5 )           /* EHRPWM0_A pad mux mode setting */

/** \brief ePWM instance IDs */
#define APP_EPWM0_INST_ID   ( 0 )
#define APP_EPWM1_INST_ID   ( 1 )
#define APP_EPWM2_INST_ID   ( 2 )
#define APP_EPWM3_INST_ID   ( 3 )
#define APP_EPWM4_INST_ID   ( 4 )
#define APP_EPWM5_INST_ID   ( 5 )

/** \brief Kick0 protection register unlock value. */
#define KICK0_UNLOCK_VAL    ( 0x68EF3490 )
/** \brief Kick1 protection register unlock value. */
#define KICK1_UNLOCK_VAL    ( 0xD172BC5A )

/** \brief Enable clocks to epwm. */
#define APP_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_TB_CLKEN_EN   ( 0x1U )

/* Configure EPWM pads */
static void appEpwmCfgEPwmPads(void);
/* Enable ePWM clock */
static void appEpwmTbClockEnable(uint32_t pwmId);

/* Status codes */
#define APP_ERR_SOK                 (  0 )  /* no error */
#define APP_ERR_PAD_CFG_PM_ERR      ( -1 )  /* pad config, PM error */
#define APP_ERR_GET_FUNC_CLK        ( -2 )  /* get functional clock error */
#define APP_ERR_BOARD_INIT          ( -3 )  /* board init error */

/*
 * Configurable parameters
 */
/**
 *  \brief PWM instance base address.
 *
 *  Note: If changed to other instance, PRCM and pinmux changes needs to be
 *  taken care in the application.
 */
#define APP_EHRPWM_INST_BASE_ADDR       (CSL_EPWM0_EPWM_BASE)

/**
 *  \brief Output channel - A or B.
 *
 *  Note: If changed to channel B, pinmux changes needs to be taken care
 *  in the application.
 */
#define APP_EHRPWM_OUTPUT_CH            (CSL_EPWM_OUTPUT_CH_A)

/** \brief Frequency of PWM output signal in Hz - 2 KHz is selected */
#define APP_EHRPWM_OUT_FREQ             (1L * 2000L)

/** \brief Duty Cycle of PWM output signal in % - give value from 0 to 100 */
#define APP_EHRPWM_DUTY_CYCLE           (25U)

/** \brief APP run time in seconds */
#define APP_RUN_TIME                    (10L)

/** \brief APP run count in event equal zero ISR count */
#define APP_RUN_TIME_ISRCOUNT           (APP_RUN_TIME * APP_EHRPWM_OUT_FREQ)

/**
 *  \brief Functional clock to the PWMSS.
 *  Fixed for the platform - can't be changed.
 */
#define SOC_EHRPWM_MODULE_FREQ          (250U * 1000U * 1000U)

/** \brief TB frequency in Hz - so that /4 divider is used */
#define APP_EHRPWM_TB_FREQ              (SOC_EHRPWM_MODULE_FREQ / 4U)

/**
 *  \brief PRD value - this determines the period
 *
 *  PRD = (TBCLK/PWM FREQ) / 2
 *  NOTE: /2 is added becasue up&down counter is selected. So period is 2 times
 */
#define APP_EHRPWM_PRD_VAL              ((APP_EHRPWM_TB_FREQ/APP_EHRPWM_OUT_FREQ)/2U)
/**
 *  \brief COMPA value - this determines the duty cycle
 *
 *  COMPA = (PRD - ((dutycycle * PRD) / 100)
 */
#define APP_EHRPWM_COMPA_VAL            (APP_EHRPWM_PRD_VAL -                  \
                                            ((APP_EHRPWM_DUTY_CYCLE *          \
                                            APP_EHRPWM_PRD_VAL) / 100U))

#define APP_EHRPWM_INT                  (32U)
#define APP_EHRPWM_XBAR_CPU             (CSL_XBAR_IRQ_CPU_ID_IPU1)
#define APP_EHRPWM_XBAR_INST            (CSL_XBAR_INST_IPU1_IRQ_32)
#define APP_EHRPWM_XBAR_INTR_SOURCE     (CSL_XBAR_PWMSS1_IRQ_ePWM0INT)

/* ------------------------------------------------------------------
 * Global variables for ADC/PWM benchmark
 * ------------------------------------------------------------------- */

 /* ADC related global variables */
adcpwm_int_stat gAdcPwmIntStat __attribute__((section(".testInData"))) = {0, 0, 0, 0, 0};;

int32_t gCountPerLoopMax = 0;
int32_t gCountPerLoopAve = 0;

/* declare the core statistic variables */
CSL_ArmR5CPUInfo cpuInfo __attribute__((section(".testInData"))) ;
core_stat gCoreStat __attribute__((section(".testInData"))) ;
core_stat_rcv gCoreStatRcv __attribute__((section(".testInData"))) ;
uint16_t gCoreStatRcvSize __attribute__((section(".testInData")))  = 0;
uint32_t gAppSelect __attribute__((section(".testInData")))  = APP_SEL_FIR;
uint32_t gOptionSelect __attribute__((section(".testInData")))  = RUN_FREQ_SEL_1K;
uint32_t gOption[NUM_OPTIONS] __attribute__((section(".testInData")))  = {
  RUN_FREQ_8K,
  RUN_FREQ_16K,
  RUN_FREQ_32K,
  RUN_FREQ_50K  
};
uint32_t gAppRunFreq __attribute__((section(".testInData")))  = RUN_FREQ_1K;
uint32_t dCacheMissNum __attribute__((section(".testInData"))) = 0;
uint32_t iCacheMissNum __attribute__((section(".testInData"))) = 0;

/* EPWM related variables */
/** \brief IP default configuration */
static CSL_AppEpwmObj_t gAppPwmObj =
{
    APP_EHRPWM_OUTPUT_CH,                       /* pwmCh */
    APP_EHRPWM_INST_BASE_ADDR,                  /* instAddr */
    SOC_EHRPWM_MODULE_FREQ,                     /* funcClk */
    FALSE,                                      /* enableDeadband */
    FALSE,                                      /* enableChopper */
    FALSE,                                      /* enableTripzone */
    TRUE,                                       /* enableEventTrigger */
    FALSE,                                      /* enableHighResolution */
    /* CSL_AppEpwmCfg_t*/
    {
        /* CSL_EpwmTimebaseCfg_t */
        {
            APP_EHRPWM_TB_FREQ,                 /* tbClk */
            APP_EHRPWM_OUT_FREQ,                /* pwmtbCounterFreqPrd */
            CSL_EPWM_TB_COUNTER_DIR_UP_DOWN,    /* tbCntrDirection */
            FALSE,                              /* enableSynchronization */
            PWMSS_EPWM_TBCTL_PHSDIR_COUNT_DOWN, /* cntDirAfterSync */
            0U,                                 /* phsCountAfterSync */
            PWMSS_EPWM_TBCTL_SYNCOSEL_EPWMXSYNC /* syncOutSrc */
        },
        /* CSL_EpwmCounterCmpCfg_t */
        {
            APP_EHRPWM_COMPA_VAL,               /* cmpAValue */
            APP_EHRPWM_COMPA_VAL                /* cmpBValue */
        },
        /* CSL_EpwmAqActionCfg_t */
        {
            CSL_EPWM_AQ_ACTION_DONOTHING,       /* zeroAction */
            CSL_EPWM_AQ_ACTION_DONOTHING,       /* prdAction */
            CSL_EPWM_AQ_ACTION_HIGH,            /* cmpAUpAction */
            CSL_EPWM_AQ_ACTION_LOW,             /* cmpADownAction */
            CSL_EPWM_AQ_ACTION_LOW,             /* cmpBUpAction */
            CSL_EPWM_AQ_ACTION_HIGH             /* cmpBDownAction */
        },
        /* CSL_EpwmDeadbandCfg_t */
        {
            CSL_EPWM_DB_IN_MODE_A_RED_A_FED,    /* inputMode */
            CSL_EPWM_DB_OUT_MODE_BYPASS,        /* outputMode */
            CSL_EPWM_DB_POL_SEL_ACTV_HIGH,      /* polaritySelect */
            0U,                                 /* risingEdgeDelay */
            0U                                  /* fallingEdgeDelay */
        },
        /* CSL_EpwmChopperCfg_t */
        {
            CSL_EPWM_CHP_DUTY_CYCLE_PERC_12PNT5,    /* dutyCycle */
            CSL_EPWM_CHP_CLK_FREQ_DIV_BY_1,         /* clkFrequency */
            CSL_EPWM_CHP_OSHT_WIDTH_1XSYSOUT_BY_8   /* oneShotPulseWidth */
        },
        /* CSL_EpwmTripzoneCfg_t */
        {
            CSL_EPWM_TZ_TRIP_ACTION_DO_NOTHING, /* tripAction */
            CSL_EPWM_TZ_EVENT_ONE_SHOT,         /* tripEvtType */
            0U,                                 /* tripPin */
            FALSE                               /* enableTripIntr */
        },
        /* CSL_EpwmEtCfg_t */
        {
            CSL_EPWM_ET_INTR_EVT_CNT_EQ_ZRO,    /* intrEvtSource */
            CSL_EPWM_ET_INTR_PERIOD_FIRST_EVT   /* intrPrd */
        }
    }
};

void appADCIntrISR(void *handle) __attribute__((aligned(8), section(".testInCode")));
void appADCIntrISR(void *handle)
{
    uint32_t status;

    status = ADCGetIntrStatus(APP_ADC_MODULE);
    ADCClearIntrStatus(APP_ADC_MODULE, status);

    gAdcPwmIntStat.adcIsrCnt++;
    ADCWriteEOI(APP_ADC_MODULE);
}

static int32_t appADCInit(void)
{
    adcRevisionId_t revId;
    int32_t         configStatus = STW_EFAIL;

    /* Get ADC information */
    ADCGetRevisionId(APP_ADC_MODULE, &revId);
    MCBENCH_log("ADC Revision ID:\n");
    MCBENCH_log("Scheme            :%d\n", revId.scheme);
    MCBENCH_log("Functional number :%d\n", revId.func);
    MCBENCH_log("RTL revision      :%d\n", revId.rtlRev);
    MCBENCH_log("Major revision    :%d\n", revId.major);
    MCBENCH_log("Minor revision    :%d\n", revId.minor);
    MCBENCH_log("Custom revision   :%d\n", revId.custom);
    /* Clear All interrupt status */
    ADCClearIntrStatus(APP_ADC_MODULE, ADC_INTR_STATUS_ALL);
    /* Power up AFE */
    ADCPowerUp(APP_ADC_MODULE, TRUE);
    /* Wait for 4us at least */
    appADCWait();
    /* Do the internal calibration */
    ADCInit(APP_ADC_MODULE, FALSE, 0U, 0U);
    /* Configure ADC divider*/
    configStatus = ADCSetClkDivider(APP_ADC_MODULE, APP_ADC_DIV);

    return configStatus;
}

static void appADCStart(void)
{
    adcSequencerStatus_t status;

    /* Check if FSM is idle */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while ((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }
    /* Start ADC conversion */
    ADCStart(APP_ADC_MODULE, TRUE);
}

static void appADCStop(void)
{
    adcSequencerStatus_t status;

    /* Disable all/enabled steps */
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_2, FALSE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_3, FALSE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_4, FALSE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_5, FALSE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_6, FALSE);
    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while ((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }
    /* Stop ADC */
    ADCStart(APP_ADC_MODULE, FALSE);
    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while ((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }
}

static void appADCWait(void)
{
    Osal_delay(10U);
}

static void appADCConfigureInterrupt(void)
{
    OsalRegisterIntrParams_t intrPrms;
    OsalInterruptRetCode_e osalRetVal;
    HwiP_Handle hwiHandle;

    Osal_RegisterInterrupt_initParams(&intrPrms);
    intrPrms.corepacConfig.arg          = (uintptr_t)0;
    intrPrms.corepacConfig.priority     = 1U;
    intrPrms.corepacConfig.corepacEventNum = 0U; /* NOT USED ? */
    intrPrms.corepacConfig.intVecNum = CSLR_R5FSS0_CORE0_INTR_ADC0_GEN_LEVEL_0;
    intrPrms.corepacConfig.isrRoutine   = (void (*)(uintptr_t)) (&appADCIntrISR) ;
    osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);
    if(OSAL_INT_SUCCESS != osalRetVal)
    {
        MCBENCH_log("Error Could not register ISR !!!\n");
    }
}

static void appADCModuleEnable(void)
{
    /* Enable ADC module */
    Sciclient_pmSetModuleState(TISCI_DEV_ADC0,
        TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
        TISCI_MSG_FLAG_AOP |
        TISCI_MSG_FLAG_DEVICE_EXCLUSIVE |
        TISCI_MSG_FLAG_DEVICE_RESET_ISO,
        SCICLIENT_SERVICE_WAIT_FOREVER);
}

static void appADCModuleDisable(void)
{
    /* Disable ADC module */
    Sciclient_pmSetModuleState(TISCI_DEV_ADC0,
        TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
        TISCI_MSG_FLAG_AOP |
        TISCI_MSG_FLAG_DEVICE_EXCLUSIVE |
        TISCI_MSG_FLAG_DEVICE_RESET_ISO,
        SCICLIENT_SERVICE_WAIT_FOREVER);
}

void appADCPWMBenchInit(int32_t freq)
{
    int32_t         configStatus, testErrCount = 0;
    adcStepConfig_t adcConfig;

    MCBENCH_log("\nStarting application...\n");

    appADCConfigureInterrupt();

    /* Enable ADC module */
    appADCModuleEnable();

    /* Initialize ADC module */
    configStatus = appADCInit();
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC divider configuration.\n");
        testErrCount++;
    }
    /* Initialize ADC configuration params */
    adcConfig.mode             = ADC_OPERATION_MODE_CONTINUOUS;
    adcConfig.channel          = ADC_CHANNEL_1;
    adcConfig.openDelay        = 0;
    adcConfig.rangeCheckEnable = 0U;
    adcConfig.fifoNum          = ADC_FIFO_NUM_0;
    switch (freq)
    {
    case RUN_FREQ_8K:
      /* 25Mhz/39/5/16 = 8013hz */
      adcConfig.sampleDelay      = 22U; /* 39-15-2 = 22 */
      adcConfig.averaging        = ADC_AVERAGING_16_SAMPLES;
      break;
    case RUN_FREQ_16K:
      /* 25Mhz/39/5/8 = 16026hz */
      adcConfig.sampleDelay      = 22U; /* 39-15-2 = 22 */
      adcConfig.averaging        = ADC_AVERAGING_8_SAMPLES;
      break;
    case RUN_FREQ_32K:
      /* 25Mhz/39/5/4 = 32051hz */
      adcConfig.sampleDelay      = 22U; /* 39-15-2 = 22 */
      adcConfig.averaging        = ADC_AVERAGING_4_SAMPLES;
      break;
    case RUN_FREQ_50K:
      /* 25Mhz/25/5/4 = 50000hz */
      adcConfig.sampleDelay      = 8U; /* 25-15-2 = 8 */
      adcConfig.averaging        = ADC_AVERAGING_4_SAMPLES;
      break;
    default:
      /* 25Mhz/39/5/16 = 8013hz */
      adcConfig.sampleDelay      = 22U; /* 39-15-2 = 22 */
      adcConfig.averaging        = ADC_AVERAGING_16_SAMPLES;
    }

    /* Enable interrupts */
    ADCEnableIntr(APP_ADC_MODULE, (ADC_INTR_SRC_END_OF_SEQUENCE |
                                   ADC_INTR_SRC_FIFO0_THRESHOLD |
                                   ADC_INTR_SRC_FIFO0_OVERRUN |
                                   ADC_INTR_SRC_FIFO0_UNDERFLOW |
                                   ADC_INTR_SRC_FIFO1_THRESHOLD |
                                   ADC_INTR_SRC_FIFO1_OVERRUN |
                                   ADC_INTR_SRC_FIFO1_UNDERFLOW |
                                   ADC_INTR_SRC_OUT_OF_RANGE));
    /* Configure ADC */
    /* step 2 configuration */
    configStatus = ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_2, &adcConfig);
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC step configuration.\n");
        testErrCount++;
    }
    /* step 3 configuration */
    adcConfig.channel = ADC_CHANNEL_3;
    configStatus      = ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_3,
                                         &adcConfig);
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC step configuration.\n");
        testErrCount++;
    }
    /* step 4 configuration */
    adcConfig.channel = ADC_CHANNEL_4;
    configStatus      = ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_4,
                                         &adcConfig);
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC step configuration.\n");
        testErrCount++;
    }
    /* step 5 configuration */
    adcConfig.channel = ADC_CHANNEL_5;
    configStatus      = ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_5,
                                         &adcConfig);
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC step configuration.\n");
    }
    /* step 6 configuration */
    adcConfig.channel = ADC_CHANNEL_6;
    configStatus      = ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_6,
                                         &adcConfig);
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC step configuration.\n");
        testErrCount++;
    }

    ADCStepIdTagEnable(APP_ADC_MODULE, TRUE);
    configStatus =
        ADCSetCPUFIFOThresholdLevel(APP_ADC_MODULE, ADC_FIFO_NUM_0,
                                    40U);
    if (STW_SOK != configStatus)
    {
        MCBENCH_log("Error in ADC CPU threshold configuration.\n");
        testErrCount++;
    }
    /* step enable */
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_2, TRUE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_3, TRUE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_4, TRUE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_5, TRUE);
    ADCStepEnable(APP_ADC_MODULE, ADC_STEP_6, TRUE);

    /* EPWM initialization */
    appEpwmInit();
    /* Set EPWM to selected output frequency */
    appEpwmSetOutFreq(freq);

    /* start the ADC */
    appADCStart();

    MCBENCH_log("\nApplication is completed.\n");

    return;
}

void appADCPWMBenchDeInit(void)
{
    appADCStop();
    /* Power down ADC */
    ADCPowerUp(APP_ADC_MODULE, FALSE);
    /* Disable ADC module */
    appADCModuleDisable();

    return;
}

/* ----------------------------------------------------------------------
 * ADC/PWM benchmark
 * ------------------------------------------------------------------- */
int32_t appADCPWMBench(uint32_t *adcInData, int32_t *adcInDataSize) __attribute__((aligned(8), section(".testInCode")));
int32_t appADCPWMBench(uint32_t *adcInData, int32_t *adcInDataSize)
{
  uint32_t        loopcnt, fifoWordCnt;
  int32_t         dutyCycle;

  init_profiling();
  do {
   gStartTime = readPmu();
  } while (gStartTime==0);
  gEndTime = readPmu();
  if (gEndTime >= gStartTime)
    gOverheadTime = gEndTime - gStartTime;
  else
    gOverheadTime = 0; /* in case of PMU timer wrapped around */
  MCBENCH_log("\n %d overhead cycles\n", (uint32_t)gOverheadTime);

  /* ----------------------------------------------------------------------
  ** Call the ADC/PWM process function
  ** ------------------------------------------------------------------- */
  resetPmuEventCounters();
  gStartTime = readPmu();

  /*Get FIFO data */
  fifoWordCnt = ADCGetFIFOWordCount(APP_ADC_MODULE, ADC_FIFO_NUM_0);
  *adcInDataSize = fifoWordCnt;
  for (loopcnt = 0U; loopcnt < fifoWordCnt; loopcnt++)
  {
    adcInData[loopcnt] = HW_RD_REG32(APP_ADC_MODULE + ADC_FIFODATA(ADC_FIFO_NUM_0));
  }
  
  /* change the EPWM duty cycle base on the ADC value */
  dutyCycle = ((adcInData[0]&0xFFF)*100)/0xFFF;
  appEpwmSetDutyCycle(dutyCycle);
  
  /*********** Compute benchmark in cycles ********/
  gEndTime = readPmu();
  if (gEndTime >= (gStartTime+gOverheadTime))
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
  else
    gTotalTime = 0;

  iCacheMissNum = readPmuInstCacheMiss();
  dCacheMissNum = readPmuDataCacheMiss();

  /* Compute the average and max of count per loop */
  if (gTotalTime>(int64_t)gCountPerLoopMax)
  {
    /* Count per loop max */ 
    gCountPerLoopMax = gTotalTime;
  }
  gCountPerLoopAve = ((int64_t)gCountPerLoopAve*(gAdcPwmIntStat.adcIsrCntPrev-1)+gTotalTime)/gAdcPwmIntStat.adcIsrCntPrev;
  /* populate the core stat */
  gCoreStat.output.ave_count = gAdcPwmIntStat.adcIsrCntPrev;
  /* get Group and CPU ID */
  CSL_armR5GetCpuID(&cpuInfo);
  /* compute core number */
  gCoreStat.output.core_num = cpuInfo.grpId*2 + cpuInfo.cpuID;
  gCoreStat.output.app = gAppSelect;
  gCoreStat.output.freq = gOptionSelect;
  gCoreStat.output.ccploop.ave = gCountPerLoopAve;
  gCoreStat.output.ccploop.max = gCountPerLoopMax;
  gCoreStat.output.cload.cur = gTotalTime*gAppRunFreq*100/CPU_FREQUENCY;
  gCoreStat.output.cload.ave = (int64_t)gCountPerLoopAve*gAppRunFreq*100/CPU_FREQUENCY;
  gCoreStat.output.cload.max = (int64_t)gCountPerLoopMax*gAppRunFreq*100/CPU_FREQUENCY;
  gCoreStat.output.ilate.max = gTimerIntStat.intLatencyMax;
  gCoreStat.output.ilate.ave = gTimerIntStat.intLatencyAve;

  MCBENCH_log("\n END FIR benchmark\n");
  return 0;
}

void appEpwmIntrISR(void *handle) __attribute__((aligned(8), section(".testInCode")));
void appEpwmIntrISR(void *handle)
{
    volatile uint16_t status = CSL_epwmEtIntrStatus(APP_EHRPWM_INST_BASE_ADDR);
    status = status;
    CSL_epwmEtIntrClear(APP_EHRPWM_INST_BASE_ADDR);
    gAdcPwmIntStat.pwmIsrCnt++;
    return;
}

/**
 * \brief   This API configures the ePWM module
 *
 * \param   pObj             pointer to the ePwm object data structure.
 */
static void appEpwmPwmCfg(CSL_AppEpwmObj_t *pObj)
{
    uint32_t baseAddr = pObj->instAddr;
    uint32_t pwmCh    = pObj->pwmCh;
    uint32_t pwmFuncClk = pObj->funcClk;
    CSL_AppEpwmCfg_t *pPwmCfg = &pObj->pwmCfg;

    /* Configure the Time base Sub-Module */
    appEpwmTimebaseModuleCfg(baseAddr, pwmFuncClk, &pPwmCfg->tbCfg);

    /* Counter-Comparator Sub-Module Configuration */
    appEpwmCounterComparatorCfg(baseAddr, &pPwmCfg->ccCfg);

    /* Configure Action Qualifier */
    CSL_epwmAqActionOnOutputCfg(baseAddr, pwmCh, &pPwmCfg->aqCfg);

    /* Dead band sub-module configuration */
    if (TRUE == pObj->enableDeadband)
    {
       /* Enable and configure dead band sub module */
       CSL_epwmDeadbandCfg(baseAddr, &pPwmCfg->dbCfg);
    }
    else
    {
        /* Bypass dead band sub module */
        CSL_epwmDeadbandBypass(baseAddr);
    }

    /* Chopper sub-module configuration */
    if (TRUE == pObj->enableChopper)
    {
        /* Configure chopper sub - module */
        CSL_epwmChopperCfg(baseAddr, &pPwmCfg->chpCfg);

        /* Enable Chopper */
        CSL_epwmChopperEnable(baseAddr, TRUE);
    }
    else
    {
        /* Disable Chopper */
        CSL_epwmChopperEnable(baseAddr, FALSE);
    }

    /* Trip Zone Sub-Module Configuration */
    if (TRUE == pObj->enableTripZone)
    {
        /* Configure the Trip action */
        CSL_epwmTzTriggerTripAction(
            baseAddr, CSL_EPWM_TZ_TRIP_ACTION_HIGH, pwmCh);

        /* Enable the Trip event */
        CSL_epwmTzTripEventEnable(
            baseAddr, pPwmCfg->tzCfg.tripEvtType, pPwmCfg->tzCfg.tripPin);
    }
    else
    {
        /* Disable trip zone event handling and ignore all trip zone events */
        CSL_epwmTzTripEventDisable(
            baseAddr, CSL_EPWM_TZ_EVENT_ONE_SHOT, pPwmCfg->tzCfg.tripPin);
        CSL_epwmTzTripEventDisable(
            baseAddr, CSL_EPWM_TZ_EVENT_CYCLE_BY_CYCLE, pPwmCfg->tzCfg.tripPin);
    }

    /* Event trigger sub - module configuration */
    if (TRUE == pObj->enableEventTrigger)
    {
        /* Configure the Event trigger processing */
        CSL_epwmEtIntrCfg(
            baseAddr, pPwmCfg->etCfg.intrEvtSource, pPwmCfg->etCfg.intrPrd);
        CSL_epwmEtIntrEnable(baseAddr);
    }
    else
    {
        /* Disable Event trigger interrupts */
        CSL_epwmEtIntrDisable(baseAddr);
    }

    return;
}

/**
 * \brief   This API configures the Timebase Sub-module.
 *
 * \param   baseAddr        Base address of PWMSS instance used
 * \param   pwmFuncClk      PWM functional clock value in Hz
 * \param   pTbCfg          Pointer to the Time base sub-module configuration
 *                          data structure
 */
static void appEpwmTimebaseModuleCfg(uint32_t baseAddr,
                                     uint32_t pwmFuncClk,
                                     CSL_EpwmTimebaseCfg_t *pTbCfg)
{
    /* Configure Time base clock */
    CSL_epwmTbTimebaseClkCfg(baseAddr, pTbCfg->tbClk, pwmFuncClk);

    /* Configure PWM time base counter frequency and direction */
    CSL_epwmTbPwmFreqCfg(
        baseAddr,
        pTbCfg->tbClk,
        pTbCfg->pwmtbCounterFreqPrd,
        pTbCfg->tbCntrDirection,
        CSL_EPWM_SHADOW_REG_CTRL_ENABLE);

    if (TRUE == pTbCfg->enableSynchronization)
    {
        /* Enable Synchronization */
        CSL_epwmTbSyncEnable(
            baseAddr, pTbCfg->phsCountAfterSync, pTbCfg->cntDirAfterSync);
    }
    else
    {
        /* Disable Synchronization */
        CSL_epwmTbSyncDisable(baseAddr);
    }

    /* Configure Sync out signal */
    CSL_epwmTbSetSyncOutMode(baseAddr, pTbCfg->syncOutSrc);

    /* Configure the emulation behaviour */
    CSL_epwmTbSetEmulationMode(baseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    return;
}

uint32_t cslEpwmCounterComparatorCfg(uint32_t baseAddr,
                                      uint32_t cmpValA,
                                      uint32_t cmpValB,
                                      uint32_t enableShadowWrite,
                                      uint32_t shadowToActiveLoadTrigger,
                                      uint32_t overwriteShadow) __attribute__((aligned(8), section(".testInCode")));
uint32_t cslEpwmCounterComparatorCfg(uint32_t baseAddr,
                                      uint32_t cmpValA,
                                      uint32_t cmpValB,
                                      uint32_t enableShadowWrite,
                                      uint32_t shadowToActiveLoadTrigger,
                                      uint32_t overwriteShadow)
{
    uint32_t status = FALSE;
    uint32_t regVal =
        HW_RD_REG16((baseAddr + PWMSS_EPWM_OFFSET) + PWMSS_EPWM_CMPCTL);

    if ((TRUE == overwriteShadow) ||
        (PWMSS_EPWM_CMPCTL_SHDWAFULL_FIFO_NOT_FULL ==
            HW_GET_FIELD(regVal, PWMSS_EPWM_CMPCTL_SHDWAFULL)))
    {
        HW_SET_FIELD32(regVal,
            PWMSS_EPWM_CMPCTL_SHDWAMODE, enableShadowWrite);
        HW_SET_FIELD32(regVal, PWMSS_EPWM_CMPCTL_LOADAMODE,
            shadowToActiveLoadTrigger);
        HW_WR_REG16(((baseAddr + PWMSS_EPWM_OFFSET) + PWMSS_EPWM_CMPCTL),
            (uint16_t)regVal);

        HW_WR_FIELD16(((baseAddr + PWMSS_EPWM_OFFSET) + PWMSS_EPWM_CMPA),
            PWMSS_EPWM_CMPA, (uint16_t)cmpValA);

        HW_WR_FIELD16(((baseAddr + PWMSS_EPWM_OFFSET) + PWMSS_EPWM_CMPB),
            PWMSS_EPWM_CMPB, (uint16_t)cmpValB);

            status = TRUE;
    }

    return status;
}

/**
 * \brief   This API configures the Counter-Comparator Sub-module.
 *
 * \param   baseAddr    Base address of PWMSS instance used
 * \param   pCcCfg      Pointer to the Counter-Comparator Sub-module
 *                      configuration data structure
 */

void appEpwmCounterComparatorCfg(uint32_t baseAddr,
                                        CSL_EpwmCounterCmpCfg_t *pCcCfg) __attribute__((aligned(8), section(".testInCode")));
void appEpwmCounterComparatorCfg(uint32_t baseAddr,
                                        CSL_EpwmCounterCmpCfg_t *pCcCfg)
{
    /* Counter Comparator A configuration */
    cslEpwmCounterComparatorCfg(
        baseAddr,
        pCcCfg->cmpAValue,
        pCcCfg->cmpBValue,
        CSL_EPWM_SHADOW_REG_CTRL_ENABLE,
        CSL_EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO,
        TRUE);

    return;
}

static int32_t padConfig_prcmEnable(void)
{
    /* Configure ePWM pads */
    appEpwmCfgEPwmPads();

    /* Enable EPWM module time base clock */
    appEpwmTbClockEnable(APP_EPWM0_INST_ID);

    /* Register & enable interrupt */

    Intc_Init();
    Intc_IntSetSrcType(CSLR_R5FSS0_CORE0_INTR_EPWM0_EPWM_ETINT_0, 1);
    Intc_IntPrioritySet(CSLR_R5FSS0_CORE0_INTR_EPWM0_EPWM_ETINT_0, 1U, 0U);
    Intc_IntRegister(CSLR_R5FSS0_CORE0_INTR_EPWM0_EPWM_ETINT_0,
                    (IntrFuncPtr) appEpwmIntrISR,
                    0U);
    Intc_IntEnable(CSLR_R5FSS0_CORE0_INTR_EPWM0_EPWM_ETINT_0);
    Intc_SystemEnable();

    return APP_ERR_SOK;
}

/* Configure EPWM pads */
static void appEpwmCfgEPwmPads(void)
{
    /* Unlock all MMR */
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_LOCK0_KICK0, KICK0_UNLOCK_VAL);
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_LOCK0_KICK1, KICK1_UNLOCK_VAL);

    /* Configure PADCONFIG 15-19 to mux mode 3 */
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG15, 0x250003);
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG16, 0x10003);
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG17, 0x250003);
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG18, 0x50003);
    HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG19, 0x50003);
}

/* Enable ePWM time base clock */
static void appEpwmTbClockEnable(uint32_t pwmId)
{
    uint32_t regVal;
    
    CSL_main_ctrl_mmr_cfg0Regs *pMainCtrlMmrCfg0Regs;
    volatile uint32_t *pMainCtrlMmrCfg0Reg;

    pMainCtrlMmrCfg0Regs = (CSL_main_ctrl_mmr_cfg0Regs *)CSL_CTRL_MMR0_CFG0_BASE;
    
    pMainCtrlMmrCfg0Reg = &pMainCtrlMmrCfg0Regs->LOCK1_KICK0;
    regVal = HW_RD_REG32(pMainCtrlMmrCfg0Reg);
    if (!(regVal & 0x1))
    {
        /* Unlock CTLR_MMR0 registers */
        pMainCtrlMmrCfg0Reg = &pMainCtrlMmrCfg0Regs->LOCK1_KICK0;
        HW_WR_REG32(pMainCtrlMmrCfg0Reg, KICK0_UNLOCK_VAL);
        pMainCtrlMmrCfg0Reg = &pMainCtrlMmrCfg0Regs->LOCK1_KICK1;
        HW_WR_REG32(pMainCtrlMmrCfg0Reg, KICK1_UNLOCK_VAL);
        pMainCtrlMmrCfg0Reg = &pMainCtrlMmrCfg0Regs->LOCK1_KICK0;
        do {
            regVal = HW_RD_REG32(pMainCtrlMmrCfg0Reg);
        } while (!(regVal & 0x1));
    }
    
    HW_WR_REG32(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN,
        APP_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_TB_CLKEN_EN);
}

/* Get ePWM module functional clock */
static int32_t appEpwmGetPwmFuncClock(CSL_AppEpwmObj_t *pObj)
{
    return APP_ERR_SOK;
}

int32_t appEpwmInit(void)
{
    CSL_AppEpwmObj_t *pObj = &gAppPwmObj;
    int32_t status;

    /* Do PRCM enable and pad config for PWM */
    status = padConfig_prcmEnable();
    if (status < 0)
    {
        MCBENCH_log("\nError: padConfig_prcmEnable() fail.\n");

        return status;
    }

    MCBENCH_log("\nStarting EPWM duty cycle test application...\n");
    MCBENCH_log("Probe the PWM signal to verify...\n");
    MCBENCH_log("App will wait for 10 seconds (using  PWM period ISR)...\n");
    MCBENCH_log("Probe ");
    MCBENCH_log("R154 ");
    MCBENCH_log("for 1KHz @ 25 percent duty cycle waveform...\n");

    /* Enable clocks for EPWM module inside the PWM sub system. */
    CSL_epwmClockEnable(pObj->instAddr);

    /* Get ePWM module functional clock */
    status = appEpwmGetPwmFuncClock(pObj);
    if (status < 0)
    {
        MCBENCH_log("\nError: appEpwmGetPwmFuncClock() fail.\n");

        return status;
    }

    /* EPWM channel configuration */
    appEpwmPwmCfg(pObj);

    return 0;
}

int32_t appEpwmSetOutFreq(int32_t freq)
{
    CSL_AppEpwmObj_t *pObj = &gAppPwmObj;
    uint32_t prdVal;

    /* set EPWM_TBPRD */
    switch (freq)
    {
    case RUN_FREQ_8K:
      pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd = RUN_FREQ_8K;
      break;
    case RUN_FREQ_16K:
      pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd = RUN_FREQ_16K;
      break;
    case RUN_FREQ_32K:
      pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd = RUN_FREQ_32K;
      break;
    case RUN_FREQ_50K:
      pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd = RUN_FREQ_50K;
      break;
    default:
      pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd = RUN_FREQ_8K;
    }

    /* Update object based on ePWM module frequency */
    pObj->pwmCfg.tbCfg.tbClk = pObj->funcClk / 4;
    prdVal = (pObj->pwmCfg.tbCfg.tbClk / pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd) / 2;
    pObj->pwmCfg.ccCfg.cmpAValue = prdVal - ((APP_EHRPWM_DUTY_CYCLE * prdVal)/ 100U);    
    pObj->pwmCfg.ccCfg.cmpBValue = pObj->pwmCfg.ccCfg.cmpAValue;

    /* EPWM channel configuration */
    appEpwmPwmCfg(pObj);

    return 0;
}

int32_t appEpwmSetDutyCycle(int32_t dc) __attribute__((aligned(8), section(".testInCode")));
int32_t appEpwmSetDutyCycle(int32_t dc)
{
    CSL_AppEpwmObj_t *pObj = &gAppPwmObj;
    uint32_t prdVal;
    uint32_t baseAddr = pObj->instAddr;
    CSL_AppEpwmCfg_t *pPwmCfg = &pObj->pwmCfg;

    /* Update object based on ePWM module frequency */
    prdVal = (pObj->pwmCfg.tbCfg.tbClk / pObj->pwmCfg.tbCfg.pwmtbCounterFreqPrd) / 2;
    pObj->pwmCfg.ccCfg.cmpAValue = prdVal - ((dc * prdVal)/ 100U);    
    pObj->pwmCfg.ccCfg.cmpBValue = pObj->pwmCfg.ccCfg.cmpAValue;

    /* Counter-Comparator Sub-Module Configuration */
    appEpwmCounterComparatorCfg(baseAddr, &pPwmCfg->ccCfg);

    return 0;
}

/** \endlink */
