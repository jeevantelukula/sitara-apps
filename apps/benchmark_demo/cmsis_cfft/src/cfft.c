/**
 *  \file   cfft.c
 *
 *  \brief  This file contains the cfft benchmark specific functions and
 *          macros.
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

/* CMSIS-DSPLIB */
#include "arm_math.h"
#include "arm_const_structs.h"

#include "cfft.h"

#include "benchmark_log.h"
#include "profile.h"
#include "benchmark_stat.h"
#include "benchmark_timer_interrupt.h"
#include <ti/csl/arch/csl_arch.h>

/* declare the core statistic variables */
CSL_ArmR5CPUInfo cpuInfo __attribute__((section(".testInData"))) ;
core_stat gCoreStat __attribute__((section(".testInData"))) ;
core_stat_rcv gCoreStatRcv __attribute__((section(".testInData"))) ;
uint16_t gCoreStatRcvSize __attribute__((section(".testInData")))  = 0;
uint32_t gAppSelect __attribute__((section(".testInData")))  = APP_SEL_CFFT;
uint32_t gOptionSelect __attribute__((section(".testInData")))  = CFFT_SIZE_SEL_1024;
uint32_t gOption[NUM_CFFT_SIZE] __attribute__((section(".testInData")))  = {
  CFFT_SIZE_128,
  CFFT_SIZE_256,
  CFFT_SIZE_512,
  CFFT_SIZE_1024
};
uint32_t gAppRunFreq __attribute__((section(".testInData")))  = RUN_FREQ_1K;

float32_t cfftInData[2048] __attribute__((aligned(128), section(".testInData")));
float32_t cfftMagData[1024] __attribute__((aligned(128), section(".testInData")));

int32_t gCountPerLoopMax __attribute__((aligned(8), section(".testInData")))= 0;
int32_t gCountPerLoopAve __attribute__((aligned(8), section(".testInData")))= 0;

/* initialize FFT complex array */
void cfftInit(void)
{
    uint16_t i;
    float sin;
    float cos;

    for(i = 0; i < 2048; i+=2) {
        arm_sin_cos_f32((float)i*7.5, &sin, &cos);
        cfftInData[i] = sin;
        cfftInData[i+1] = 0;
        cfftMagData[i/2] = 0;
    }
    memset(&gCoreStat, 0, sizeof(gCoreStat));    
}

/* execute Complex FFT */
int32_t cfft_bench(int32_t fftSize) __attribute__((aligned(8), section(".testInCode")));
int32_t cfft_bench(int32_t fftSize)
{
    const static arm_cfft_instance_f32 *S;
    uint32_t refIndex1 = 43, refIndex2 = 981;
    float refMax = 422.244476;

    switch (fftSize) {
        case 128:
            S = &arm_cfft_sR_f32_len128;
            break;
        case 256:
            S = &arm_cfft_sR_f32_len256;
            break;
        case 512:
            S = &arm_cfft_sR_f32_len512;
            break;

        default:
        case 1024:
            S = &arm_cfft_sR_f32_len1024;
            break;
    }

    init_profiling();
    gStartTime = readPmu(); /* two initial reads are necessary for correct overhead time */
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;
    MCBENCH_log("\n %d overhead cycles\n", (uint32_t)gOverheadTime);

    cfftInit();

    gStartTime = readPmu();

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_f32(S, cfftInData, 0, 1);

    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;

    /* Compute the average and max of count per loop */
    if (gTotalTime>(int64_t)gCountPerLoopMax)
    {
      /* Count per loop max */
      gCountPerLoopMax = gTotalTime;
    }
    gCountPerLoopAve = ((int64_t)gCountPerLoopAve*(gTimerIntStat.isrCnt-1)+gTotalTime)/gTimerIntStat.isrCnt;
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
    gCoreStat.output.cload.ave = (int64_t)gCountPerLoopAve*gAppRunFreq*100/CPU_FREQUENCY;
    gCoreStat.output.cload.max = (int64_t)gCountPerLoopMax*gAppRunFreq*100/CPU_FREQUENCY;
    gCoreStat.output.ilate.max = gTimerIntStat.intLatencyMax;
    gCoreStat.output.ilate.ave = gTimerIntStat.intLatencyAve;

    /* Process the data through the Complex Magnitude Module for 
       calculating the magnitude at each bin */
    arm_cmplx_mag_f32(cfftInData, cfftMagData, fftSize);
    /* The two max BIN values have to match */
    if ((cfftMagData[refIndex1]!=refMax)||(cfftMagData[refIndex2]!=refMax))
    {
       MCBENCH_log("\n %d point CFFT failed\n", fftSize);
       return 0;
    } else
    {
       MCBENCH_log("\n %d point CFFT %d cycles\n", fftSize, (uint32_t)gTotalTime);
       return -1;
    }
}
