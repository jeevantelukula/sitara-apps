/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/board/board.h>
#include <app_sciclient.h>
#include <app_log.h>
#include <profile.h>

/* Added for debug purpose when load and run via SBL.
 * set enableDebug = 1 and build for debug.
 * Once started running connect CCS and reset enableDebug=0
 * to proceed with single-step from the beginning
 */
void StartupEmulatorWaitFxn (void)
{
    volatile uint32_t enableDebug = 0;
    do
    {
    }while (enableDebug);
}

/*
 * Implements Hello World R5F main().
 * Control comes here immediately after the core boots-up
 */
int main(void)
{
    int32_t status;
    uint32_t i_cache_miss_count, d_cache_miss_count = 0;
    
    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    /* Use PDK Board library to initialize clocks, EVM pinmux, and UART output */
    Board_init(BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_UART_STDIO);

    /* Output build time to UART */
    UART_printf("\r\nBuild Timestamp: %s %s\n", __DATE__, __TIME__);

    /* Sciclient initialization */
    status = appSciclientInit();
    if(status) {
        UART_printf("\r\nSciclient Init Failed!\n");
    }
    else {
        UART_printf("\r\nSciclient Init Passed!\n");
    }

    /* Initialize the R5F PMU */
    init_profiling();
    /* Take initial PMU cycle count readings to calculate overhead */
    /* Two initial reads are necessary for correct overhead time */
    gStartTime = readPmu(); // two initial reads are necessary for correct overhead time
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;
    UART_printf("\nProfiling Overhead Cycles:%d\n", (uint32_t)gOverheadTime);

    /* Reset PMU counters (cycle and cache miss counters) and take initial reading */
    resetPmuEventCounters();
    gStartTime = readPmu();

    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /* Perform function(s) to be profiled here */
    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    UART_printf("\r\nUART Hello World!\n\n");
    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /* End profiled function(s)                */
    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

    /* Take final cycle count reading and calculate total cycle count*/
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    /* Read instruction/data cache miss counters */
    i_cache_miss_count = readPmuInstCacheMiss();
    d_cache_miss_count = readPmuDataCacheMiss();
    
    /* Print results to UART */
    UART_printf("UART Output Benchmark\n");
    UART_printf("---------------------\n");
    UART_printf("Total Cycles:%d\n", (uint32_t)gTotalTime);
    /* UART_printf does not natively print float numbers */
    UART_printf("Instruction Cache Misses:%d\n", i_cache_miss_count);
    UART_printf("Data Cache Misses: %d\n\n", d_cache_miss_count);

    /* Reset PMU counters (cycle and cache miss counters) and take initial reading */
    resetPmuEventCounters();
    gStartTime = readPmu();

    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /* Perform function(s) to be profiled here */
    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    appLogPrintf("appLogPrintf CCS Console Hello World!\n\n");
    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /* End profiled function(s)                */
    /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

    /* Take final cycle count reading and calculate total cycle count*/
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    /* Read instruction/data cache miss counters */
    i_cache_miss_count = readPmuInstCacheMiss();
    d_cache_miss_count = readPmuDataCacheMiss();

    /* Print results to CCS Console */
    appLogPrintf("appLogPrintf CCS Console Output Benchmark\n");
    appLogPrintf("----------------------------\n");
    appLogPrintf("Total Cycles:%d\n", (uint32_t)gTotalTime);
    appLogPrintf("Total Time in ms:%f\n", (float)gTotalTime * 0.00000125);
    appLogPrintf("Instruction Cache Misses:%d\n", i_cache_miss_count);
    appLogPrintf("Data Cache Misses: %d\n\n", d_cache_miss_count);
}
