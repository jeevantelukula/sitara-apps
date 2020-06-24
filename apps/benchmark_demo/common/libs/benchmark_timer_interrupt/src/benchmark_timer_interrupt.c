/**
 *  \file   benchmark_timer_interrupt.c
 *
 *  \brief  This file contains the benchmark timer interrupt functions and
 *          macros.
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

#include <stdint.h>

#include "benchmark_log.h"
#include "benchmark_stat.h"
#include "benchmark_timer_interrupt.h"

/* Timer handle -- Benchmark Timer interrupt */
TimerP_Handle gTimerHandle;
timer_int_stat gTimerIntStat __attribute__((section(".testInData"))) = {0, 0, 0, 0};
uint32_t reloadVal __attribute__((section(".testInData")));
uint32_t curVal __attribute__((section(".testInData")));

/* Timer tick function -- benchmark interrupt */
void benchmarkTimerTickFxn(void *arg) __attribute__((aligned(8), section(".testInCode")));
void benchmarkTimerTickFxn(void *arg)
{
   uint32_t latency;

   /* compute the timer interrupt latency */
   curVal = TIMERCounterGet(CSL_MCU_TIMER2_CFG_BASE);
   reloadVal = TIMERReloadGet(CSL_MCU_TIMER2_CFG_BASE);
   latency = (curVal-reloadVal)*BENCHMARK_TIMER_TICK_PRD;
   if (latency>gTimerIntStat.intLatencyMax)
   {
      /* timer interrupt latency in ns */ 
      gTimerIntStat.intLatencyMax = latency;
   }
   /* compute average timer interrupt latency */
   gTimerIntStat.intLatencyAve = (gTimerIntStat.intLatencyAve*gTimerIntStat.isrCnt+latency)/(gTimerIntStat.isrCnt+1);

   /* increase the interrupt counter */
   gTimerIntStat.isrCnt++;
}

/* Timer initialization */
void benchmarkTimerInit(void)
{
   TimerP_Params timerParams;

   /*
     Set up timer -- benchmark timer interrupt
   */
   gTimerIntStat.isrCnt = 0;
   gTimerIntStat.intLatencyMax = 0;

   /* Timer parameters */
   TimerP_Params_init(&timerParams);
   timerParams.name = "benchmarkBeatTimer";
   timerParams.periodType = TimerP_PeriodType_MICROSECS;
   timerParams.extfreqLo = BENCHMARK_TIMER_FREQ_HZ;
   timerParams.extfreqHi = 0;
   timerParams.startMode = TimerP_StartMode_USER;
   timerParams.runMode = TimerP_RunMode_CONTINUOUS;
   timerParams.period = BENCHMARK_TIMER_PERIOD_USEC;
   timerParams.arg = 0;
   timerParams.intNum = BENCHMARK_TIMER_INTNUM;
 
   /* Create timer -- benchmark timer interrupt */
   gTimerHandle = TimerP_create(BENCHMARK_TIMER_ID, (TimerP_Fxn)&benchmarkTimerTickFxn, &timerParams);
   if (gTimerHandle == NULL)
   {
      MCBENCH_log("\n TimerP_create failed\n");
      return;
   }
   MCBENCH_log("\n TimerP_create OK\n");

   /* Start timer */
   TimerP_start(gTimerHandle);
   MCBENCH_log("APP: Timer started !!!\n");	
   return;
}

/* set the timer interrupt to designated frequency */
int32_t benchmarkTimerSetFreq(Run_Freq_Sel sel)
{
   int32_t microSecs = 0;
   int32_t status = 0;

   /* Stop timer */
   if (gTimerHandle==NULL)
   {
      MCBENCH_log("APP: Invalid Timer Handle !!!\n");
      return -1;
   }
   status = TimerP_stop(gTimerHandle);
   if (status!=TimerP_OK)
   {
      MCBENCH_log("APP: Timer Timer stopped failed !!!\n");
      return status;
   }
   else
      MCBENCH_log("APP: Timer stopped !!!\n");

   switch (sel)
   {
      case RUN_FREQ_SEL_1K:
      microSecs = 1000000/RUN_FREQ_1K;
      break;
      
      case RUN_FREQ_SEL_2K:
      microSecs = 1000000/RUN_FREQ_2K;
      break;
      
      case RUN_FREQ_SEL_4K:
      microSecs = 1000000/RUN_FREQ_4K;
      break;

      case RUN_FREQ_SEL_8K:
      microSecs = 1000000/RUN_FREQ_8K;
      break;

      case RUN_FREQ_SEL_16K:
      microSecs = 1000000/RUN_FREQ_16K;
      break;

      case RUN_FREQ_SEL_32K:
      microSecs = 1000000/RUN_FREQ_32K;
      break;

      case RUN_FREQ_SEL_50K:
      microSecs = 1000000/RUN_FREQ_50K;
      break;

      default:
      microSecs = 1000000/RUN_FREQ_1K;
      break;
   }

   status = TimerP_setPeriodMicroSecs(gTimerHandle, microSecs);
   if (status!=TimerP_OK)
   {
      MCBENCH_log("APP: Timer TimerP_setPeriodMicroSecs failed !!!\n");
      return status;
   }
   else
      MCBENCH_log("APP: Timer TimerP_setPeriodMicroSecs OK !!!\n");   

   status = TimerP_start(gTimerHandle);
   if (status!=TimerP_OK)
   {
      MCBENCH_log("APP: Timer TimerP_start failed !!!\n");
      return status;
   }
   else
      MCBENCH_log("APP: Timer started !!!\n");
  return 0;
}
