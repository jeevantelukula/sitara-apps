/*
 * Copyright (C) 2017-2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

#ifndef MOTOR_CTRL_USER_H
#define MOTOR_CTRL_USER_H

#include "motor_ctrl_settings.h"

/* FCL Computation time predetermined from library */
#define M_FCL_COMPUTATION_TIME  (1.00)  /* in uS */

/*
 * NOTE:-
 * Base voltage and base current information from TIDA-00909 doc is
 * based off of an ADC that works at 3.3V reference.
 * The base current = 16.5A (for a spread of 3.3V - 1.65V = 1.65V)
 * The base voltage  = 81.5 / sqrt(3)=
 * Define the base quantites
*/
#define M_BASE_VOLTAGE          236.14 /* Base peak phase voltage (volt), */
                                       /* maximum measurable Vdc/sqrt(3) */
#define M_BASE_SHUNT_CURRENT    9.95   /* Base peak phase current (amp), */
                                       /* maximum measurable peak curr. */
#define M_BASE_LEM_CURRENT      12.0   /* Base peak phase current (amp), */
                                       /* maximum measurable peak current */
#define M_BASE_CURRENT          M_BASE_LEM_CURRENT
#define M_BASE_TORQUE           NULL    /* Base torque (N.m) */
#define M_BASE_FLUX             NULL    /* Base flux linkage (volt.sec/rad) */
#define M_BASE_FREQ             250     /* Base electrical frequency (Hz) */
#define M_MAXIMUM_CURRENT       8.0     /* Motor maximum torque current (amp) */

#define M_SPEED_REF     0.05            /* reference speed (pu) */
#define M_ID_START      0.1             /* alignment reference d-axis current */
#define M_ID_RUN        0.0             /* running d-axis current */
#define M_IQ_START      0.05            /* startup q-axis current */

/* Current sensors scaling */
/* 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647 */
#define M_CURRENT_SCALE(A)             (uint16_t)(2048 * A / M_BASE_CURRENT)

/* Analog scaling with ADC */
#define M_ADC_PU_SCALE_FACTOR          0.000244140625     /* 1/2^12 */
#define M_ADC_PU_PPB_SCALE_FACTOR      0.000488281250     /* 1/2^11 */

/* Current Scale */
#define M_MAXIMUM_SCALE_CURRENT        M_BASE_CURRENT * 2.0
#define M_CURRENT_SENSE_SCALE          (M_MAXIMUM_SCALE_CURRENT / 4096.0)

/* Voltage Scale */
#define M_MAXIMUM_SCALE_VOLATGE        M_BASE_VOLTAGE * 1.732050808
#define M_VOLTAGE_SENSE_SCALE          (M_MAXIMUM_SCALE_VOLATGE / 4096.0)

#endif  /* end of MOTOR_CTRL_USER_H definition */
