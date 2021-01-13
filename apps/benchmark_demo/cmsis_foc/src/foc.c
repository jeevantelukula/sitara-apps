/**
 *  \file   foc.c
 *
 *  \brief  This file contains the FOC benchmark functions.
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

#include <ti/csl/arch/csl_arch.h>
/* C2000 Control Suite */
#include "dmctype.h"
#include "IQmathLib.h"
#include "rmp_cntl.h"
#include "rampgen.h"
#include "speed_fr.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "pi.h"
#include "svgen_dq.h"

#include "foc.h"

#include "benchmark_log.h"
#include "benchmark_stat.h"
#include "profile.h"
#include "test_data.h"
#include "profile_data.h"
#include "benchmark_timer_interrupt.h"

/*
 * FOC variable and instance definitions
*/
#define PI_SPD_KP       ( 50.0 )
#define PI_SPD_KI       ( T * gSpeedLoopPrescaler / 0.2 )
#define PI_SPD_UMIN     ( -1.0 )
#define PI_SPD_UMAX     ( 1.0 )

#define PI_ID_KP        ( 0.3 )
#define PI_ID_KI        ( 0.003 )
#define PI_ID_UMIN      ( -0.95 )
#define PI_ID_UMAX      ( 0.95 )

#define PI_IQ_KP        ( 0.3 )
#define PI_IQ_KI        ( 0.003 )
#define PI_IQ_UMIN      ( -0.95 )
#define PI_IQ_UMAX      ( 0.95 )

float T __attribute__((section(".testInData"))) = 0.001 / ISR_FREQUENCY;    /* sampling period in sec for 10 kz T=0.0001. */
uint16_t gSpeedLoopPrescaler __attribute__((section(".testInData"))) = 10;  /* speed loop pre-scaler */

/* Input test data */
_iq gIdRef __attribute__((section(".testInData"))) = _IQ(0.0);              /* Id reference (per-unit) */
_iq  gSpeedRef __attribute__((section(".testInData"))) = _IQ(0.15);         /* for Open Loop tests */
_iq gInData[4] __attribute__((section(".testInData")));             /* 0- Input Current Phase A; 1- Input Current Phase B; 2- Input Current Phase C; 3- Voltage of one of the Phases */
_iq gElecTheta __attribute__((section(".testInData"))) = _IQ(0.0);  /* single_turn Theta, Electrical: converted from mechanical */

/* Calculated data */
_iq gSinElecTheta __attribute__((section(".testInData"))); /* sin of electrical angle */
_iq gCosElecTheta __attribute__((section(".testInData"))); /* cos of electrical angle */

RMPCNTL gRmpCntl __attribute__((section(".testInData"))) = RMPCNTL_DEFAULTS;    /* Ramp controller to smoothly ramp the speed */
RAMPGEN gRampGen __attribute__((section(".testInData"))) = RAMPGEN_DEFAULTS;    /* Ramp generator to simulate an electrical angle */
SPEED_MEAS_QEP gSpeedMeasQep __attribute__((section(".testInData"))) = SPEED_MEAS_QEP_DEFAULTS;
CLARKE gClarke __attribute__((section(".testInData"))) = CLARKE_DEFAULTS;
PARK gPark __attribute__((section(".testInData"))) = PARK_DEFAULTS;
IPARK gIPark __attribute__((section(".testInData"))) = IPARK_DEFAULTS;
PI_CONTROLLER gPiSpd __attribute__((section(".testInData"))) = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER gPiId __attribute__((section(".testInData"))) = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER gPiIq __attribute__((section(".testInData"))) = PI_CONTROLLER_DEFAULTS;
SVGENDQ gSvgenDq __attribute__((section(".testInData"))) = SVGENDQ_DEFAULTS;

static uint32_t gAdcSampCnt __attribute__((section(".testInData")));        /* current ADC sample count */
static void readAdcSamps(_iq inData[4]);
static uint32_t gSvGenOutCnt __attribute__((section(".testInData")));       /* current SVGen output count */
static void writeSvGenOut(SVGENDQ svGenDq);
static uint32_t gInvClarkeOutCnt __attribute__((section(".testInData")));   /* current Inverse Clarke output count */
static void writeInvClarkeOut(float32_t Ia, float32_t Ib);

/* CMSIS Clarke transform output */
float32_t gCmsisClarkeAlphaOut __attribute__((section(".testInData")));
float32_t gCmsisClarkeBetaOut __attribute__((section(".testInData")));

/* CMSIS sin/cos */
float32_t gCmsisElecTheta __attribute__((section(".testInData")));          /* electrical theta expressed degrees for CMSIS */
float32_t gCmsisSinElecThetaOut __attribute__((section(".testInData")));    /* CMSIS sin output */
float32_t gCmsisCosElecThetaOut __attribute__((section(".testInData")));    /* CMSIS cos output */

/* CMSIS Park transform output */
float32_t gCmsisParkDsOut __attribute__((section(".testInData"))); 
float32_t gCmsisParkQsOut __attribute__((section(".testInData")));

/* CMSIS PID (PI) instances */
arm_pid_instance_f32 gCmsisPiSpd __attribute__((section(".testInData")));   /* Speed */
arm_pid_instance_f32 gCmsisPiId __attribute__((section(".testInData")));    /* ID */
arm_pid_instance_f32 gCmsisPiIq __attribute__((section(".testInData")));    /* IQ */
float32_t gCmsisPiSpdOut __attribute__((section(".testInData")));           /* Speed output */
float32_t gCmsisPiIdOut __attribute__((section(".testInData")));            /* ID output */
float32_t gCmsisPiIqOut __attribute__((section(".testInData")));            /* IQ output */

/* CMSIS Inverse Park transform output */
float32_t gCmsisParkAlphaOut __attribute__((section(".testInData")));
float32_t gCmsisParkBetaOut __attribute__((section(".testInData")));

/* CMSIS Inverse Clarke transform output */
float32_t gCmsisInvClarkeIaOut __attribute__((section(".testInData")));
float32_t gCmsisInvClarkeIbOut __attribute__((section(".testInData")));
 
int32_t gCountPerLoopMax = 0;
int32_t gCountPerLoopAve = 0;

/* declare the core statistic variables */
CSL_ArmR5CPUInfo cpuInfo __attribute__((section(".testInData"))) ;
core_stat gCoreStat __attribute__((section(".testInData"))) ;
core_stat_rcv gCoreStatRcv __attribute__((section(".testInData"))) ;
uint16_t gCoreStatRcvSize __attribute__((section(".testInData")))  = 0;
uint32_t gAppSelect __attribute__((section(".testInData")))  = APP_SEL_FIR;
uint32_t gOptionSelect __attribute__((section(".testInData")))  = RUN_FREQ_SEL_1K;
uint32_t gOption[NUM_RUN_FREQS] __attribute__((section(".testInData")))  = {
  RUN_FREQ_1K,
  RUN_FREQ_2K,
  RUN_FREQ_4K,
  RUN_FREQ_8K,
  RUN_FREQ_16K,
  RUN_FREQ_32K,
  RUN_FREQ_50K  
};
uint32_t gAppRunFreq __attribute__((section(".testInData")))  = RUN_FREQ_1K;
/* initialize FOC loop */
void focLoopInit(void)
{
    gAdcSampCnt = 0;      /* initialize ADC sample count */
    gSvGenOutCnt = 0;     /* initialize  SVGen output count */ 
    gInvClarkeOutCnt = 0; /* initialize Inverse Clarke count */

    gRampGen.StepAngleMax = _IQ(BASE_FREQ * T);

    /* Initialize speed estimator */
    gSpeedMeasQep.K1 = _IQ21(1 / (BASE_FREQ * T));
    gSpeedMeasQep.K2 = _IQ(1 / (1 + T * 2 * PI * 5));  /* low-pass filter cutoff frequency */
    gSpeedMeasQep.K3 = _IQ(1) - gSpeedMeasQep.K2;
    gSpeedMeasQep.BaseRpm = BASE_FREQ * 60 * (2 / NUM_POLES);

    /* Initialize PI Speed */
    gPiSpd.Kp = _IQ(PI_SPD_KP);
    gPiSpd.Ki = _IQ(PI_SPD_KI);
    gPiSpd.Umin = _IQ(PI_SPD_UMIN);
    gPiSpd.Umax = _IQ(PI_SPD_UMAX);

    /* Initialize PI Id */
    gPiId.Kp = _IQ(PI_ID_KP);
    gPiId.Ki = _IQ(PI_ID_KI);
    gPiId.Umin = _IQ(PI_ID_UMIN);
    gPiId.Umax = _IQ(PI_ID_UMAX);

    /* Initialize PI Iq */
    gPiIq.Kp = _IQ(PI_IQ_KP);
    gPiIq.Ki = _IQ(PI_IQ_KI);
    gPiIq.Umin = _IQ(PI_IQ_UMIN);
    gPiIq.Umax = _IQ(PI_IQ_UMAX);

    /* Initialize CMSIS PI Speed */
    gCmsisPiSpd.Kp = PI_SPD_KP;
    gCmsisPiSpd.Ki = PI_SPD_KI;
    gCmsisPiSpd.Kd = 0;
    arm_pid_init_f32(&gCmsisPiSpd, 1);

    /* Initialize CMSIS PI Id */
    gCmsisPiId.Kp = PI_ID_KP;
    gCmsisPiId.Ki = PI_ID_KI;
    gCmsisPiId.Kd = 0;
    arm_pid_init_f32(&gCmsisPiId, 1);

    /* Initialize CMSIS PI Iq */
    gCmsisPiIq.Kp = PI_IQ_KP;
    gCmsisPiIq.Ki = PI_IQ_KI;
    gCmsisPiIq.Kd = 0;
    arm_pid_init_f32(&gCmsisPiIq, 1);
}

/* execute FOC loop */
void focLoop(uint16_t loopCnt) __attribute__((section(".testInCode")));
void focLoop(uint16_t loopCnt)
{
    /* Initialize FOC loop */
    focLoopInit();

    init_profiling();
    do {
     gStartTime = readPmu();
    } while (gStartTime==0);
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;    

    /* Get ADC samples */
    readAdcSamps(gInData);

    resetPmuCnt();
    /* Ramp controller smoothly ramps speed to SpeedRef */
    gStartTime = readPmu();
    gRmpCntl.TargetValue = gSpeedRef;
    RC_MACRO(gRmpCntl);

    /* Calculate electrical angle based on EnDat position feedback */
    /* calcElecTheta(&gElecTheta); */
    /* Ramp generator simulates electrical angle output from encoder */
    gRampGen.Freq = gRmpCntl.SetpointValue;
    RG_MACRO(gRampGen);
    gElecTheta = gRampGen.Out;

    /* Connect inputs to speed calculation macro, 
       call speed calculation macro */
    gSpeedMeasQep.ElecTheta = gElecTheta;
    SPEED_FR_MACRO(gSpeedMeasQep);

    /* Connect inputs to Clarke transform macro, call macro */
    gClarke.As = gInData[0];
    gClarke.Bs = gInData[1];    
    /* CMSIS library call clarke */
    arm_clarke_f32((float32_t)gInData[0], (float32_t)gInData[1], &gCmsisClarkeAlphaOut, &gCmsisClarkeBetaOut);

    /* Compute sin/cos of electrical angle using CMSIS-DSPLIB */
    gCmsisElecTheta = gElecTheta*360;
    if (gCmsisElecTheta > 180.0)
    {
        gCmsisElecTheta = gCmsisElecTheta - 360.0;
    }

    /* CMSIS library call sin_cos */
    arm_sin_cos_f32(gCmsisElecTheta, &gCmsisSinElecThetaOut, &gCmsisCosElecThetaOut);

    /* Connect inputs to Park transform macro, call macro */
    gPark.Alpha = gClarke.Alpha;
    gPark.Beta = gClarke.Beta;
    gPark.Angle = gElecTheta; /* unused */
    gPark.Sine = gSinElecTheta;
    gPark.Cosine = gCosElecTheta;

    /* CMSIS library call park */
    arm_park_f32(gCmsisClarkeAlphaOut, gCmsisClarkeBetaOut, &gCmsisParkDsOut, &gCmsisParkQsOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);

    /* CMSIS library call pid */
    gCmsisPiSpdOut = arm_pid_f32(&gCmsisPiSpd, gSpeedRef-gSpeedMeasQep.Speed);

    /* CMSIS library call pid */
    gCmsisPiIqOut = arm_pid_f32(&gCmsisPiIq, gCmsisPiSpdOut-gCmsisParkQsOut);

    /* CMSIS library call pid */
    gCmsisPiIdOut = arm_pid_f32(&gCmsisPiId, gIdRef-gCmsisParkDsOut);

    /* CMSIS library call inv-park */
    arm_inv_park_f32(gCmsisPiIdOut, gCmsisPiIqOut, &gCmsisParkAlphaOut, &gCmsisParkBetaOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);

    /* CMSIS library call inv-clarke */
    /* Connect inputs to Inverse Clark transform */
    /* Note: only purpose is benchmarking inverse Clarke transform in CMSIS-DSPLIB.
       This is because no SVGen macro exists in CMSIS-DSPLIB */
    arm_inv_clarke_f32(gCmsisParkAlphaOut, gCmsisParkBetaOut, &gCmsisInvClarkeIaOut, &gCmsisInvClarkeIbOut);

    writeSvGenOut(gSvgenDq);
    writeInvClarkeOut(gCmsisInvClarkeIaOut, gCmsisInvClarkeIbOut);

    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      

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
}

/* Read ADC samples */
static void readAdcSamps(_iq inData[4])
{
    uint32_t sampU;
    int32_t sampI;
    float sampF;

    /* Input Phase Current A */
    sampU = (Uint32)gTestInAdcDataA[gAdcSampCnt];
    sampI = sampU - 32768;              /* unsigned to signed */
    sampI = sampI - ADC_SAMP_OFFSET;    /* remove DC offset */
    sampF = (float)sampI/32768.0 * ADC_SAMP_SCALEF;
    inData[0] = _IQ(sampF);

    /* Input Phase Current B */
    sampU = (uint32_t)gTestInAdcDataB[gAdcSampCnt];
    sampI = sampU - 32768;              /* unsigned to signed */
    sampI = sampI - ADC_SAMP_OFFSET;    /* remove DC offset */
    sampF = (float)sampI/32768.0 * ADC_SAMP_SCALEF;
    inData[1] = _IQ(sampF);

    gAdcSampCnt++;
}

/* Write Space Vector Generation outputs */
static void writeSvGenOut(SVGENDQ svGenDq)
{
    gTestOutSvGenTa[gSvGenOutCnt] = svGenDq.Ta;
    gTestOutSvGenTb[gSvGenOutCnt] = svGenDq.Tb;
    gTestOutSvGenTc[gSvGenOutCnt] = svGenDq.Tc;

    gSvGenOutCnt++;
}

/* Write Inverse Clarke outputs */
static void writeInvClarkeOut(float32_t Ia, float32_t Ib)
{
    gTestOutInvClarkeIa[gInvClarkeOutCnt] = Ia;
    gTestOutInvClarkeIb[gInvClarkeOutCnt] = Ib;

    gInvClarkeOutCnt++;
}
