/* ----------------------------------------------------------------------
 * Copyright (C) 2010-2020 ARM Limited. All rights reserved.
 *
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
 * Title:        arm_fir_example_f32.c
 *
 * Description:  Example code demonstrating how an FIR filter can be used
 *               as a low pass filter.
 *
 * Target Processor: Cortex-M4/Cortex-M3
 *
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

/**
 * @ingroup groupExamples
 */

/**
 * @defgroup PID Example
 *
 * \par Description:
 * \par
 * Removes high frequency signal components from the input using an FIR lowpass filter.
 * The example demonstrates how to configure an FIR filter and then pass data through
 * it in a block-by-block fashion.
 * \image html FIRLPF_signalflow.gif
 *
 * \par Algorithm:
 * \par
 * The input signal is a sum of two sine waves:  1 kHz and 15 kHz.
 * This is processed by an FIR lowpass filter with cutoff frequency 6 kHz.
 * The lowpass filter eliminates the 15 kHz signal leaving only the 1 kHz sine wave at the output.
 * \par
 * The lowpass filter was designed using MATLAB with a sample rate of 48 kHz and
 * a length of 29 points.
 * The MATLAB code to generate the filter coefficients is shown below:
 * <pre>
 *     h = fir1(28, 6/24);
 * </pre>
 * The first argument is the "order" of the filter and is always one less than the desired length.
 * The second argument is the normalized cutoff frequency.  This is in the range 0 (DC) to 1.0 (Nyquist).
 * A 6 kHz cutoff with a Nyquist frequency of 24 kHz lies at a normalized frequency of 6/24 = 0.25.
 * The CMSIS FIR filter function requires the coefficients to be in time reversed order.
 * <pre>
 *     fliplr(h)
 * </pre>
 * The resulting filter coefficients and are shown below.
 * Note that the filter is symmetric (a property of linear phase FIR filters)
 * and the point of symmetry is sample 14.  Thus the filter will have a delay of
 * 14 samples for all frequencies.
 * \par
 * \image html FIRLPF_coeffs.gif
 * \par
 * The frequency response of the filter is shown next.
 * The passband gain of the filter is 1.0 and it reaches 0.5 at the cutoff frequency 6 kHz.
 * \par
 * \image html FIRLPF_response.gif
 * \par
 * The input signal is shown below.
 * The left hand side shows the signal in the time domain while the right hand side is a frequency domain representation.
 * The two sine wave components can be clearly seen.
 * \par
 * \image html FIRLPF_input.gif
 * \par
 * The output of the filter is shown below.  The 15 kHz component has been eliminated.
 * \par
 * \image html FIRLPF_output.gif
 *
 * \par Variables Description:
 * \par
 * \li \c testInput_f32_1kHz_15kHz points to the input data
 * \li \c refOutput points to the reference output data
 * \li \c testOutput points to the test output data
 * \li \c firStateF32 points to state buffer
 * \li \c firCoeffs32 points to coefficient buffer
 * \li \c blockSize number of samples processed at a time
 * \li \c numBlocks number of frames
 *
 * \par CMSIS DSP Software Library Functions Used:
 * \par
 * - arm_fir_init_f32()
 * - arm_fir_f32()
 *
 * <b> Refer  </b>
 * \link arm_fir_example_f32.c \endlink
 *
 */


/** \example arm_fir_example_f32.c
 */

/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */

#include <stdint.h>

#ifndef IO_CONSOLE
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#endif

#include <ti/csl/arch/csl_arch.h>

#include "benchmark_log.h"
#include "profile.h"
#include "benchmark_stat.h"
#include "ipc_setup.h"
#include "benchmark_timer_interrupt.h"

#include "arm_math.h"
#include "math_helper.h"

/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */

#define NUM_PID_LOOP 100

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */

/* -------------------------------------------------------------------
 * Declare Test output buffer
 * ------------------------------------------------------------------- */

/* ------------------------------------------------------------------
 * Global variables for PID Example
 * ------------------------------------------------------------------- */

uint32_t gCoreId __attribute__((section(".testInData")));
arm_pid_instance_f32 myPIDInstance __attribute__((section(".testInData")));
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

int32_t gCountPerLoopMax __attribute__((aligned(8), section(".testInData")))= 0;
int32_t gCountPerLoopAve __attribute__((aligned(8), section(".testInData")))= 0;

/* ----------------------------------------------------------------------
 * PID loop
 * ------------------------------------------------------------------- */
/* execute PID loop */
void pidLoop(uint16_t loopCnt) __attribute__((section(".testInCode")));
void pidLoop(uint16_t loopCnt)
{
  /* use volatile to prevent optimizer from optimizing out calling to 
     arm_pid_f32 due to no subsequent dependances on its results 
  */
  volatile uint32_t i, j;
  float pidOutput;

  init_profiling();
  do {
   gStartTime = readPmu();
  } while (gStartTime==0);
  gEndTime = readPmu();
  if (gEndTime >= gStartTime)
    gOverheadTime = gEndTime - gStartTime;
  else
    gOverheadTime = 0; /* in case of PMU timer wrapped around */

  /* Initialize PID instance */
  myPIDInstance.Kp = 350;
  myPIDInstance.Ki = 300;
  myPIDInstance.Kd = 50;
  
  /* Call PID init function to initialize the instance structure. */
  arm_pid_init_f32(&myPIDInstance, 1);

  resetPmuEventCounters();
  gStartTime = readPmu();
  for (j=0; j<loopCnt; j++)
    for (i=0; i<NUM_PID_LOOP; i++)
      pidOutput = arm_pid_f32(&myPIDInstance, 0.1);

  /*********** Compute benchmark in cycles ********/
  gEndTime = readPmu();
  if (gEndTime >= (gStartTime+gOverheadTime))
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
  else
    gTotalTime = 0;

  iCacheMissNum = readPmuInstCacheMiss();
  dCacheMissNum = readPmuDataCacheMiss();

  /* Compute the average and max of count per loop */
  if (gTotalTime>(uint64_t)gCountPerLoopMax)
  {
    /* Count per loop max */ 
    gCountPerLoopMax = gTotalTime;
  }
  gCountPerLoopAve = ((uint64_t)gCountPerLoopAve*(gTimerIntStat.isrCnt-1)+gTotalTime)/gTimerIntStat.isrCnt;
  /* populate the core stat */
  gCoreStat.output.ave_count = gTimerIntStat.isrCnt;
  /* get Group and CPU ID */
  CSL_armR5GetCpuID(&cpuInfo);
  /* compute core number */
  gCoreStat.output.core_num = cpuInfo.grpId*2 + cpuInfo.cpuID;
  gCoreStat.output.app = gAppSelect;
  gCoreStat.output.freq = gOptionSelect;
  gCoreStat.output.ccploop.ave = gCountPerLoopAve;
  gCoreStat.output.ccploop.max = gCountPerLoopMax;
  gCoreStat.output.cload.cur = gTotalTime*gAppRunFreq*100/CPU_FREQUENCY;
  gCoreStat.output.cload.ave = (uint64_t)gCountPerLoopAve*gAppRunFreq*100/CPU_FREQUENCY;
  gCoreStat.output.cload.max = (uint64_t)gCountPerLoopMax*gAppRunFreq*100/CPU_FREQUENCY;
  gCoreStat.output.ilate.max = gTimerIntStat.intLatencyMax;
  gCoreStat.output.ilate.ave = gTimerIntStat.intLatencyAve;

  /* MCBENCH_log is blank, if the DEBUG_PRINT is not defined in benchmark_log.h */ 
  /* supress the unused variable build warning */
  (void) pidOutput;
}

int32_t main(void)
{
#ifndef IO_CONSOLE
  Board_initCfg boardCfg;

  boardCfg = BOARD_INIT_PINMUX_CONFIG |
             BOARD_INIT_MODULE_CLOCK  |
             BOARD_INIT_UART_STDIO;
  Board_init(boardCfg);
#endif

/* define ENABLE_IPC_RPMSG_CHAR to enable    */
/* the IPC RPMSG_char between A53 and R5     */
/* In the case of R5 only test,              */
/* ENABLE_IPC_RPMSG_CHAR should be undefined */

#ifdef ENABLE_IPC_RPMSG_CHAR
   /* Initializes the SCI Client driver */
   MCBENCH_log("\n Initializes the SCI Client driver\n");
   ipc_initSciclient();
   MCBENCH_log("\n Set up the IPC RPMsg\n");
   ipc_rpmsg_init();
#endif

   /* Set up the timer interrupt */
   CSL_armR5GetCpuID(&cpuInfo);
   /* compute core number */
   gCoreId = cpuInfo.grpId*2 + cpuInfo.cpuID;
   benchmarkTimerInit(gCoreId);

   /* set to RUN_FREQ_1K */
   benchmarkTimerSetFreq(gCoreId, RUN_FREQ_SEL_50K);
   gAppRunFreq = RUN_FREQ_50K;

   MCBENCH_log("\n START PID benchmark\n");
   while (1)
   {
      /* Check for new timer interrupt */
      if (gTimerIntStat.isrCnt>gTimerIntStat.isrCntPrev)
      {
        /* Execute FOC loop 1 time */
        pidLoop(1);
        gTimerIntStat.isrCntPrev++;
      }

#ifdef ENABLE_IPC_RPMSG_CHAR
     /* Check for new RPMsg arriving */
     gCoreStatRcvSize = 0;
     ipc_rpmsg_receive((char *)&gCoreStatRcv, &gCoreStatRcvSize);
     if (gCoreStatRcvSize>0)
     {
       /* has to match the PID */
       if (gCoreStatRcv.input.app==APP_SEL_PID)
       {
         gOptionSelect = gCoreStatRcv.input.freq;
         /* set the running frequency to the selected one */
         if ((gOptionSelect>0)&&(gOptionSelect<=NUM_OPTIONS))
         {
           if (gAppRunFreq!=gOption[gOptionSelect-1])
           {
             /* Set up the timer interrupt */
             CSL_armR5GetCpuID(&cpuInfo);
             /* compute core number */
             gCoreId = cpuInfo.grpId*2 + cpuInfo.cpuID;
             /* set to selected frequency */
             benchmarkTimerSetFreq(gCoreId, (Run_Freq_Sel)gOptionSelect);
             gAppRunFreq = gOption[gOptionSelect-1];
             gTimerIntStat.isrCnt = 0L;
             gTimerIntStat.isrCntPrev = 0L;
             gTimerIntStat.intLatencyMax = 0;
             gTimerIntStat.intLatencyAve = 0;
             gTimerIntStat.intLatencyTotal = 0L;
             gCountPerLoopAve = 0;
             gCountPerLoopMax = 0;
           }
         }
      }
      /* Send the gCoreStat to the A53 */
      ipc_rpmsg_send((char *)&gCoreStat, (uint16_t)sizeof(gCoreStat));
    }
#endif

    /* Execute a WFI */
    asm volatile (" wfi");
  }
}

/** \endlink */
