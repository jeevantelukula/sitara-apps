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
#include "test_data.h"

float32_t cfftInData[2048] __attribute__((aligned(128), section(".testInData")));
float32_t cfftMagData[1024] __attribute__((aligned(128), section(".testInData")));

float a __attribute__((section(".testInData"))) = 5.3; 
float b __attribute__((section(".testInData"))) = 9.2; 
float c __attribute__((section(".testInData"))) = 8.1; 
float d __attribute__((section(".testInData"))) = 9.5; 
float e __attribute__((section(".testInData"))) = 7.6; 
float f __attribute__((section(".testInData"))) = 120.5; 
volatile float g __attribute__((section(".testInData"))) = 283.185;  	

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
}

/* execute Complex FFT */
void cfft_bench(uint16_t fftSize) __attribute__((aligned(8), section(".testInCode")));
void cfft_bench(uint16_t fftSize)
{
    const static arm_cfft_instance_f32 *S;
    uint32_t refIndex1 = 43, refIndex2 = 981;
	float refMax = 422.244476;
	
    switch (fftSize) {
        case 1024:
            S = &arm_cfft_sR_f32_len1024;
            break;
    }

#if PROFILE == COMPONENTS
    init_profiling();
    gStartTime = readPmu(); // two initial reads are necessary for correct overhead time
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;
    MCBENCH_log("\n %d overhead cycles\n", (uint32_t)gOverheadTime);
#endif

#if PROFILE == COMPONENTS
    cfftInit();
    gStartTime = readPmu();
	/* Process the data through the CFFT/CIFFT module */
    arm_cfft_f32(S, cfftInData, 0, 1);
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
	/* Process the data through the Complex Magnitude Module for 
	   calculating the magnitude at each bin */
    arm_cmplx_mag_f32(cfftInData, cfftMagData, fftSize);
	/* The two max BIN values have to match */    
	if ((cfftMagData[refIndex1]!=refMax)||(cfftMagData[refIndex2]!=refMax))  
	{    
       MCBENCH_log("\n %d point CFFT failed\n", fftSize);
	   return;
	} else
	{
       MCBENCH_log("\n %d point CFFT %d cycles\n", fftSize, (uint32_t)gTotalTime);
	}
#endif

#if PROFILE == COMPONENTS
    gStartTime = readPmu();
    //c = a / b;
    //f = d / e;
    g = (a / b) / (d / e);
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n Division: %d cycles\n", (uint32_t)gTotalTime);
    a += 0.001;
    b += 0.02;
    d += .105;
    e += 1.0023;
#endif
}
