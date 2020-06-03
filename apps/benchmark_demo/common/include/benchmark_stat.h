/**
 * benchmark_stat.h
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/* =================================================================================
File name:       benchmark_stat.h
===================================================================================*/

typedef struct
{
    int32_t app;      /* Input: application number (0-5: 0: none, 1:cfft, 2:fir, 3:foc, 4:pid, 5:ADC/ePWM) */
    int32_t freq;     /* Input: application running frequency (0-4: 0: none, 1:8Khz, 2:16Khz, 3:32Khz, 4:50Khz) */
    int32_t mod_flag; /* Input: modification flag (0/1) */
} core_input;

typedef struct
{
    int32_t cur;  /* Output: current CPU load (0-100) */
    int32_t ave;  /* Output: average CPU laod (0-100) */
    int32_t max;  /* Output: max CPU laod (0-100) */
} cpu_load;

typedef struct
{
    int32_t ave;  /* Output: average timer interrupt latency (0-10000 us) */
    int32_t max;  /* Output: max timer interrupt latency (0-10000 us) */
} int_latency;

typedef struct
{
    int32_t ave;  /* Output: average cycle count per loop (0-2^32) */
    int32_t max;  /* Output: max cycle count per loop (0-2^32) */
} ccp_loop;

typedef struct
{
    cpu_load cload;      /* Output: CPU load struct */
    int_latency ilate;   /* Output: INT latency struct */
    ccp_loop ccploop;    /* Output: circle count per loop struct */
    int32_t ave_count;       /* Output: counter for average period  */
    int32_t sram_pcnt;       /* Output: SRAM usage in percent (0-100) */
    int32_t core_num;        /* Output: current core ID (0-3) */
    int32_t app;             /* Output: current app (0-3) */
    int32_t freq;            /* Output: current freq (0-3) */
} core_output;

typedef struct
{
    int64_t payload_num; /* rpmsg payload number */
    int64_t payload_size;/* rpmsg payload size */
    core_input input;    /* core input parameters */
    core_output output;  /* core input parameters */
} core_stat;

#define APP_RUN_FREQUNCY 8  /* in Khz */
#define CPU_FREQUNCY 800000  /* in Khz */
