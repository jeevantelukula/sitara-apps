/**
 *  \file   main.c
 *
 *  \brief  This file contains main function and macros for cfft menchmark.
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
#include "test_data.h"
#include "cfft.h"
#include "ipc_setup.h"
#include "benchmark_stat.h"
#include <ti/csl/arch/csl_arch.h>

/* declare the core statistic variables */
core_stat gCoreStat;
CSL_ArmR5CPUInfo cpuInfo;

void main(void) 
{
    uint16_t i, j;

#ifndef IO_CONSOLE
	Board_initCfg boardCfg;
#endif
    
#ifndef IO_CONSOLE
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_MODULE_CLOCK  |
               BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);
#endif

#if PROFILE != COMPONENTS
    init_profiling();
    gStartTime = readPmu(); // two initial reads are necessary for correct overhead time
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;    
    MCBENCH_log("\n %d overhead cycles\n", (uint32_t)gOverheadTime);
#endif    
	
	/* Initializes the SCI Client driver */
    MCBENCH_log("\n Initializes the SCI Client driver\n");
	ipc_initSciclient();
	
	/* Initialize Sine wave input array */
	cfftInit();

    MCBENCH_log("\n START CFFT benchmark\n");
    /* Iterate through all FFT size for 1024 */
    for (i = 1024; i <= 1024; i *= 2)
    {
        for (j = 0; j < NUM_CFFT_LOOP_ITER; j++)
        {
            MCBENCH_log("\n ITERATION %d\n", (uint32_t)j);
            
#if PROFILE != COMPONENTS
            gStartTime = readPmu();
#endif        
            /* Execute FOC loop */
            cfft_bench(i);
#if PROFILE != COMPONENTS
            /*********** Compute benchmark in cycles ********/
            gEndTime = readPmu();
            gTotalTime = gEndTime - gStartTime - gOverheadTime;
            MCBENCH_log("\n Test used %d cycles\n", (uint32_t)gTotalTime);
#endif        
        }
    }
    MCBENCH_log("\n END CFFT benchmark\n");

    /* Set up the IPC RPMsg */
    /* Start the IPC RPMsg loop */
    MCBENCH_log("\n Set up the IPC RPMsg\n");
    ipc_rpmsg_func(&gCoreStat, sizeof(gCoreStat));
}
