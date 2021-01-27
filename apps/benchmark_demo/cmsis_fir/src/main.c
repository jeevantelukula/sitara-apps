/**
 *  \file   main.c
 *
 *  \brief  This file contains main function and macros for fir benchmark.
 *
 */

/*
 * Copyright (C) 2014 - 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef IO_CONSOLE
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#endif

#include "benchmark_log.h"
#include "profile.h"
#include "math_helper.h"
#include "arm_math.h"
#include "fir.h"
#include "ipc_setup.h"
#include "benchmark_timer_interrupt.h"

extern CSL_ArmR5CPUInfo cpuInfo;
uint32_t gCoreId __attribute__((section(".testInData")));;

void main(void) 
{
#ifndef IO_CONSOLE
   Board_initCfg boardCfg;
#endif
    
/* define ENABLE_IPC_RPMSG_CHAR to enable    */
/* the IPC RPMSG_char between A53 and R5     */
/* In the case of R5 only test,              */
/* ENABLE_IPC_RPMSG_CHAR should be undefined */

#ifndef IO_CONSOLE
   boardCfg = BOARD_INIT_PINMUX_CONFIG |
              BOARD_INIT_MODULE_CLOCK  |
              BOARD_INIT_UART_STDIO;
   Board_init(boardCfg);
#endif
   
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
   benchmarkTimerSetFreq(gCoreId, RUN_FREQ_1K);
   gAppRunFreq = RUN_FREQ_1K;

   MCBENCH_log("\n START FIR benchmark\n");
   while (1)
   {
      /* Check for new timer interrupt */
      if (gTimerIntStat.isrCnt>gTimerIntStat.isrCntPrev)
      {
         /* Execute CFFT loop */
         fir_bench(320);
         gTimerIntStat.isrCntPrev++;
      }

#ifdef ENABLE_IPC_RPMSG_CHAR
      /* Check for new RPMsg arriving */
      gCoreStatRcvSize = 0;
      ipc_rpmsg_receive((char *)&gCoreStatRcv, &gCoreStatRcvSize);
      if (gCoreStatRcvSize>0)
      {
         /* has to match the FIR */
         if (gCoreStatRcv.input.app==APP_SEL_FIR)
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
                 gAppRunFreq = gOption[gOptionSelect-1];
                 benchmarkTimerSetFreq(gCoreId, gAppRunFreq);
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
