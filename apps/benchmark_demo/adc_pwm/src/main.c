/**
 *  \file   main.c
 *
 *  \brief  This file contains main function and macros for ADC/PWM benchmark.
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

#ifndef IO_CONSOLE
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#endif

#include "benchmark_log.h"
#include "profile.h"
#include "app_adc_epwm.h"
#include "ipc_setup.h"

uint32_t gADCInputBuffer[32] __attribute__((section(".testInData")));
int32_t  gADCInputBufferSize __attribute__((section(".testInData"))) = 0;

uint32_t gCoreId = 0;

void main(void) 
{
#ifndef IO_CONSOLE
   Board_initCfg boardCfg;
#endif
    
#ifndef IO_CONSOLE
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

   /* Call ADC/PWM init function to initialize the instance structure. */
   gAppRunFreq = RUN_FREQ_8K;
   /* set the ADC smapling freqency to the selected frequency */
   appADCPWMBenchInit(gAppRunFreq);

   MCBENCH_log("\n START ADC/PWM benchmark\n");
   while (1)
   {
      /* Check for new ADC interrupt */
      if (gAdcPwmIntStat.adcIsrCnt>gAdcPwmIntStat.adcIsrCntPrev)
      {
         /* Execute ADC loop */
         appADCPWMBench(gADCInputBuffer, &gADCInputBufferSize);
         gAdcPwmIntStat.adcIsrCntPrev++;
      }

#ifdef ENABLE_IPC_RPMSG_CHAR
      /* Check for new RPMsg arriving */
      gCoreStatRcvSize = 0;
      ipc_rpmsg_receive((char *)&gCoreStatRcv, &gCoreStatRcvSize);
      if (gCoreStatRcvSize>0)
      {
         /* has to match the ADC */
         if (gCoreStatRcv.input.app==APP_SEL_ADC)
         {
            gOptionSelect = gCoreStatRcv.input.freq;
            /* set the running frequency to the selected one */
            if ((gOptionSelect>0)&&(gOptionSelect<=NUM_OPTIONS))
            {
               if (gAppRunFreq!=gOption[gOptionSelect-1])
               {
                 /* set to selected frequency */
                 gAppRunFreq = gOption[gOptionSelect-1];
                 /* de-initialize the existing ADC instance */
                 appADCPWMBenchDeInit();
                 gCountPerLoopAve = 0;
                 gCountPerLoopMax = 0;
                 gAdcPwmIntStat.adcIsrCnt = 0;
                 gAdcPwmIntStat.adcIsrCntPrev = 0;
                 /* create and set the ADC instance to selected frequency */
                 appADCPWMBenchInit(gAppRunFreq);
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
