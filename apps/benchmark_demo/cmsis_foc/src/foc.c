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
#include "profile.h"
#include "test_data.h"
#include "profile_data.h"

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

float T __attribute__((section(".testInData"))) = 0.001 / ISR_FREQUENCY;    // sampling period in sec for 10 kz T=0.0001.          
uint16_t gSpeedLoopPrescaler __attribute__((section(".testInData"))) = 10;  // speed loop pre-scaler

/* Input test data */
_iq gIdRef __attribute__((section(".testInData"))) = _IQ(0.0);              // Id reference (per-unit)
_iq  gSpeedRef __attribute__((section(".testInData"))) = _IQ(0.15);         // for Open Loop tests
_iq gInData[4] __attribute__((section(".testInData")));             //  0- Input Current Phase A; 1- Input Current Phase B; 2- Input Current Phase C; 3- Voltage of one of the Phases
_iq gElecTheta __attribute__((section(".testInData"))) = _IQ(0.0);  // single_turn Theta, Electrical: converted from mechanical

/* Calculated data */
_iq gSinElecTheta __attribute__((section(".testInData"))); // sin of electrical angle
_iq gCosElecTheta __attribute__((section(".testInData"))); // cos of electrical angle

RMPCNTL gRmpCntl __attribute__((section(".testInData"))) = RMPCNTL_DEFAULTS;    // Ramp controller to smoothly ramp the speed
RAMPGEN gRampGen __attribute__((section(".testInData"))) = RAMPGEN_DEFAULTS;    // Ramp generator to simulate an electrical angle
SPEED_MEAS_QEP gSpeedMeasQep __attribute__((section(".testInData"))) = SPEED_MEAS_QEP_DEFAULTS;
CLARKE gClarke __attribute__((section(".testInData"))) = CLARKE_DEFAULTS;
PARK gPark __attribute__((section(".testInData"))) = PARK_DEFAULTS;
IPARK gIPark __attribute__((section(".testInData"))) = IPARK_DEFAULTS;
PI_CONTROLLER gPiSpd __attribute__((section(".testInData"))) = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER gPiId __attribute__((section(".testInData"))) = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER gPiIq __attribute__((section(".testInData"))) = PI_CONTROLLER_DEFAULTS;
SVGENDQ gSvgenDq __attribute__((section(".testInData"))) = SVGENDQ_DEFAULTS;

static uint32_t gAdcSampCnt __attribute__((section(".testInData")));        // current ADC sample count
static void readAdcSamps(_iq inData[4]);
static uint32_t gSvGenOutCnt __attribute__((section(".testInData")));       // current SVGen output count
static void writeSvGenOut(SVGENDQ svGenDq);
static uint32_t gInvClarkeOutCnt __attribute__((section(".testInData")));   // current Inverse Clarke output count
static void writeInvClarkeOut(float32_t Ia, float32_t Ib);

// CMSIS Clarke transform output
float32_t gCmsisClarkeAlphaOut __attribute__((section(".testInData")));
float32_t gCmsisClarkeBetaOut __attribute__((section(".testInData")));

// CMSIS sin/cos
float32_t gCmsisElecTheta __attribute__((section(".testInData")));          // electrical theta expressed degrees for CMSIS
float32_t gCmsisSinElecThetaOut __attribute__((section(".testInData")));    // CMSIS sin output
float32_t gCmsisCosElecThetaOut __attribute__((section(".testInData")));    // CMSIS cos output

// CMSIS Park transform output
float32_t gCmsisParkDsOut __attribute__((section(".testInData"))); 
float32_t gCmsisParkQsOut __attribute__((section(".testInData")));

// CMSIS PID (PI) instances
arm_pid_instance_f32 gCmsisPiSpd __attribute__((section(".testInData")));   // Speed
arm_pid_instance_f32 gCmsisPiId __attribute__((section(".testInData")));    // ID
arm_pid_instance_f32 gCmsisPiIq __attribute__((section(".testInData")));    // IQ
float32_t gCmsisPiSpdOut __attribute__((section(".testInData")));           // Speed output
float32_t gCmsisPiIdOut __attribute__((section(".testInData")));            // ID output
float32_t gCmsisPiIqOut __attribute__((section(".testInData")));            // IQ output

// CMSIS Inverse Park transform output
float32_t gCmsisParkAlphaOut __attribute__((section(".testInData")));
float32_t gCmsisParkBetaOut __attribute__((section(".testInData")));

// CMSIS Inverse Clarke transform output
float32_t gCmsisInvClarkeIaOut __attribute__((section(".testInData")));
float32_t gCmsisInvClarkeIbOut __attribute__((section(".testInData")));
 
/* initialize FOC loop */
void focLoopInit(void)
{
    gAdcSampCnt = 0; // initialize ADC sample count
    
    gRampGen.StepAngleMax = _IQ(BASE_FREQ * T);
    
    /* Initialize speed estimator */
    gSpeedMeasQep.K1 = _IQ21(1 / (BASE_FREQ * T));
    gSpeedMeasQep.K2 = _IQ(1 / (1 + T * 2 * PI * 5));  // low-pass filter cutoff frequency
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
#if PROFILE == COMPONENTS
    int32_t i;
    init_profiling();
    gStartTime = readPmu(); // two initial reads are necessary for correct overhead time
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;
    MCBENCH_log("\n %d overhead cycles\n", (uint32_t)gOverheadTime);
#endif

    /* Get ADC samples */
    readAdcSamps(gInData);
    
    /* Ramp controller smoothly ramps speed to SpeedRef */
    gRmpCntl.TargetValue = gSpeedRef;
#if PROFILE == COMPONENTS
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       RC_MACRO(gRmpCntl);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n RC_MACRO %d cycles\n", (uint32_t)gTotalTime);
#else
    RC_MACRO(gRmpCntl);
#endif
       
    /* Calculate electrical angle based on EnDat position feedback */
    //calcElecTheta(&gElecTheta);
    /* Ramp generator simulates electrical angle output from encoder */
    gRampGen.Freq = gRmpCntl.SetpointValue;
#if PROFILE == COMPONENTS
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       RG_MACRO(gRampGen);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n RG_MACRO %d cycles\n", (uint32_t)gTotalTime);
#else
    RG_MACRO(gRampGen);
#endif
    gElecTheta = gRampGen.Out;
    
    /* Connect inputs to speed calculation macro, 
       call speed calculation macro */
    gSpeedMeasQep.ElecTheta = gElecTheta;
#if PROFILE == COMPONENTS
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       SPEED_FR_MACRO(gSpeedMeasQep);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n SPEED_FR_MACRO %d cycles\n", (uint32_t)gTotalTime);
#else    
    SPEED_FR_MACRO(gSpeedMeasQep);
#endif
    
    /* Connect inputs to Clarke transform macro, call macro */
    gClarke.As = gInData[0];
    gClarke.Bs = gInData[1];
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       CLARKE_MACRO(gClarke);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n CLARKE_MACRO %d cycles\n", (uint32_t)gTotalTime);
    
    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       arm_clarke_f32((float32_t)gInData[0], (float32_t)gInData[1], &gCmsisClarkeAlphaOut, &gCmsisClarkeBetaOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n arm_clarke_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_clarke_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    CLARKE_MACRO(gClarke);

    // CMSIS library call
    arm_clarke_f32((float32_t)gInData[0], (float32_t)gInData[1], &gCmsisClarkeAlphaOut, &gCmsisClarkeBetaOut);
#endif
    
    /* Compute sin of electrical angle */
#if PROFILE == COMPONENTS
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       gSinElecTheta = _IQsinPU(gElecTheta);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n _IQsinPU (sinf) %d cycles\n", (uint32_t)gTotalTime);
#else
    gSinElecTheta = _IQsinPU(gElecTheta);
#endif    
    
    /* Compute cosine of electrical angle */
#if PROFILE == COMPONENTS
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       gCosElecTheta = _IQcosPU(gElecTheta);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n _IQcosPU (cosf) %d cycles\n", (uint32_t)gTotalTime);
#else
    gCosElecTheta = _IQcosPU(gElecTheta);
#endif    
    
    /* Compute sin/cos of electrical angle using CMSIS-DSPLIB */
    gCmsisElecTheta = gElecTheta*360;
    if (gCmsisElecTheta > 180.0)
    {
        gCmsisElecTheta = gCmsisElecTheta - 360.0;
    }
#if PROFILE == COMPONENTS
    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       arm_sin_cos_f32(gCmsisElecTheta, &gCmsisSinElecThetaOut, &gCmsisCosElecThetaOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n arm_sin_cos_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_sin_cos_cyc[loopCnt] = gTotalTime;
#else
    // CMSIS library call
    arm_sin_cos_f32(gCmsisElecTheta, &gCmsisSinElecThetaOut, &gCmsisCosElecThetaOut);
#endif
    
    /* Connect inputs to Park transform macro, call macro */
    gPark.Alpha = gClarke.Alpha;
    gPark.Beta = gClarke.Beta;
    gPark.Angle = gElecTheta; // unused
    gPark.Sine = gSinElecTheta;
    gPark.Cosine = gCosElecTheta;
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       PARK_MACRO(gPark);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n PARK_MACRO %d cycles\n", (uint32_t)gTotalTime);    
    
    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       arm_park_f32(gCmsisClarkeAlphaOut, gCmsisClarkeBetaOut, &gCmsisParkDsOut, &gCmsisParkQsOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n arm_park_f32 %d cycles\n", (uint32_t)gTotalTime);    
    gCmsis_park_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    PARK_MACRO(gPark);

    // CMSIS library call
    arm_park_f32(gCmsisClarkeAlphaOut, gCmsisClarkeBetaOut, &gCmsisParkDsOut, &gCmsisParkQsOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);
#endif
    
    /* Connect inputs to Speed PI macro, call macro */
    gPiSpd.Ref = gSpeedRef;
    gPiSpd.Fbk = gSpeedMeasQep.Speed;
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       PI_MACRO(gPiSpd);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;  
    MCBENCH_log("\n SPEED PI_MACRO %d cycles\n", (uint32_t)gTotalTime);
    
    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       gCmsisPiSpdOut = arm_pid_f32(&gCmsisPiSpd, gSpeedRef-gSpeedMeasQep.Speed);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;  
    MCBENCH_log("\n SPEED arm_pid_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_pid_speed_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    PI_MACRO(gPiSpd);

    // CMSIS library call
    gCmsisPiSpdOut = arm_pid_f32(&gCmsisPiSpd, gSpeedRef-gSpeedMeasQep.Speed);
#endif
    
    /* Connect inputs to Iq PI macro, call macro */
    gPiIq.Ref = gPiSpd.Out;
    gPiIq.Fbk = gPark.Qs;
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       PI_MACRO(gPiIq);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n IQ PI_MACRO %d cycles\n", (uint32_t)gTotalTime);

    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       gCmsisPiIqOut = arm_pid_f32(&gCmsisPiIq, gCmsisPiSpdOut-gCmsisParkQsOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n IQ arm_pid_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_pid_iq_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    PI_MACRO(gPiIq);
    
    // CMSIS library call
    gCmsisPiIqOut = arm_pid_f32(&gCmsisPiIq, gCmsisPiSpdOut-gCmsisParkQsOut);
#endif
    
    /* Connect inputs to Id PI macro, call macro */
    gPiId.Ref = gIdRef;
    gPiId.Fbk = gPark.Ds;
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       PI_MACRO(gPiId);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n ID PI_MACRO %d cycles\n", (uint32_t)gTotalTime);
    
    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       gCmsisPiIdOut = arm_pid_f32(&gCmsisPiId, gIdRef-gCmsisParkDsOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n ID arm_pid_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_pid_id_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    PI_MACRO(gPiId);
    
    // CMSIS library call
    gCmsisPiIdOut = arm_pid_f32(&gCmsisPiId, gIdRef-gCmsisParkDsOut);
#endif
    
    /* Connect inputs to Inverse Park macro, call macro */
    gIPark.Sine = gSinElecTheta;
    gIPark.Cosine = gCosElecTheta;
    gIPark.Ds = gPiId.Out;
    gIPark.Qs = gPiIq.Out;
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       IPARK_MACRO(gIPark);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n IPARKE_MACRO %d cycles\n", (uint32_t)gTotalTime);

    // CMSIS library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       arm_inv_park_f32(gCmsisPiIdOut, gCmsisPiIqOut, &gCmsisParkAlphaOut, &gCmsisParkBetaOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n arm_inv_park_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_inv_park_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    IPARK_MACRO(gIPark);
    
    // CMSIS library call
    arm_inv_park_f32(gCmsisPiIdOut, gCmsisPiIqOut, &gCmsisParkAlphaOut, &gCmsisParkBetaOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);
#endif
    
    /* Connect inputs to Space Vector Generation macro, call macro */
    gSvgenDq.Ualpha = gIPark.Alpha;
    gSvgenDq.Ubeta = gIPark.Beta;
#if PROFILE == COMPONENTS
    // Control Suite library call
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       SVGEN_MACRO(gSvgenDq);
	}
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n SVGEN_MACRO %d cycles\n", (uint32_t)gTotalTime);
    
    // CMSIS library call
    /* Connect inputs to Inverse Clark transform */
    /* Note: only purpose is benchmarking inverse Clarke transform in CMSIS-DSPLIB.
       This is because no SVGen macro exists in CMSIS-DSPLIB */
    gStartTime = readPmu();
	for (i=0; i<NUM_LOOP; i++)
	{
       arm_inv_clarke_f32(gCmsisParkAlphaOut, gCmsisParkBetaOut, &gCmsisInvClarkeIaOut, &gCmsisInvClarkeIbOut);
    }
	gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;      
    MCBENCH_log("\n arm_inv_clarke_f32 %d cycles\n", (uint32_t)gTotalTime);
    gCmsis_inv_clarke_cyc[loopCnt] = gTotalTime;
#else
    // Control Suite library call
    SVGEN_MACRO(gSvgenDq);
    
    // CMSIS library call
    /* Connect inputs to Inverse Clark transform */
    /* Note: only purpose is benchmarking inverse Clarke transform in CMSIS-DSPLIB.
       This is because no SVGen macro exists in CMSIS-DSPLIB */
    arm_inv_clarke_f32(gCmsisParkAlphaOut, gCmsisParkBetaOut, &gCmsisInvClarkeIaOut, &gCmsisInvClarkeIbOut);
#endif
    writeSvGenOut(gSvgenDq);
    writeInvClarkeOut(gCmsisInvClarkeIaOut, gCmsisInvClarkeIbOut);
}

/* Read ADC samples */
static void readAdcSamps(_iq inData[4])
{
    uint32_t sampU;
    int32_t sampI;
    float sampF;

    // Input Phase Current A
    sampU = (Uint32)gTestInAdcDataA[gAdcSampCnt];
    sampI = sampU - 32768;              // unsigned to signed
    sampI = sampI - ADC_SAMP_OFFSET;    // remove DC offset
    sampF = (float)sampI/32768.0 * ADC_SAMP_SCALEF;
    inData[0] = _IQ(sampF);

    // Input Phase Current B
    sampU = (uint32_t)gTestInAdcDataB[gAdcSampCnt];
    sampI = sampU - 32768;              // unsigned to signed
    sampI = sampI - ADC_SAMP_OFFSET;    // remove DC offset
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
