/*
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

 /* SYSBIOS includes */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>

/* PRSDK includes */
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/board/board.h>
#include <ti/osal/osal.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#include "cfg_host_intr.h"
#include "timesyncDrv_utils.h"          /* TS driver utilities */
#include "app_timesync.h"
#include "test_utils.h"
#include "cfg_soc.h"

/* Task priorities */
#define TASK_SYSINIT_PRI                ( 3 )

/* Test TS CMP periods */
/* IEP frequency = 200 MHz, time factor = 5, TS frequency = 32 kHz */
/* Note: 
 * When using an IEP frequency of 250 MHz or 333 MHz
 * use a time factor of 4 or 3 respectively */
#define TEST_PRD_COUNT0         ( 5*200000000u/1000u   )    /* 1ms     (1KHz) - sim sync0 */
#define TEST_PRD_COUNT1         ( 5*200000000u/100000u )    /* 10us    (100Khz) (CMP7)    */
#define TEST_PRD_COUNT2         ( 5*200000000u/32000u  )    /* 31.25us (32Khz)  (CMP8)    */
#define TEST_PRD_COUNT3         ( 5*200000000u/8000u   )    /* 125us   (8Khz)   (CMP9)    */
#define TEST_PRD_COUNT4         ( 5*200000000u/1000u   )    /* 1ms     (1Khz)   (CMP10)   */

/* Test TS CMP offsets */
#define TEST_PRD_OFFSET1        (      0 )  	/* No offset */
#define TEST_PRD_OFFSET2        (      0 )  	/* No offset */
#define TEST_PRD_OFFSET3        ( -10000 )  	/* 10us before sync0 */
#define TEST_PRD_OFFSET4        (  20000 )  	/* 20us after sync0 */

/* PRU TS IRQ handler */
void pruTsIrqHandler(
    uintptr_t foobar
);

/* ------------------------------------------------------------------------- *
 *                                Globals                                    *
 * ------------------------------------------------------------------------- */
/* Test PRUSS DRV handle */
PRUICSS_Handle gPruIcssHandle;
/* Test TS object */
TsObj gTestTs;

/* Task handles */
Task_Handle gHSysInitTask;

/* Interrupt counter */
volatile uint32_t interrupts = 0;
/* Interrupt stats */
volatile uint32_t minDelay, maxDelay, totDelay, numDelay, lastCmp, errorDelay, resetDelay=1;

/*
 *  ======== taskSysInitFxn ========
 */
Void taskSysInitFxn(
    UArg a0, 
    UArg a1
)
{
    Board_IDInfo boardInfo;
    TsPrmsObj tsPrms;
    Error_Block eb;
    int32_t status;
    uint32_t seconds = 0, last_seconds = 0;

    System_printf("enter taskSysInitFxn()\n");

    /* Put error block initial state */
    Error_init(&eb);

    /* Initialize board */
    Board_init(BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_UART_STDIO);

    /* Output board & chip information */
    Board_getIDInfo(&boardInfo);
    UART_printf("\nBoard name \t: ");
    UART_printf(boardInfo.boardName);
    UART_printf("\n\rChip Revision \t: ");
    UART_printf(boardInfo.version);

    /* Output build time */
    UART_printf("\r\nBuild Timestamp      : %s %s", __DATE__, __TIME__);

    /* Configure ICSSG clock selection */
    status = cfgIcssgClkSel(TEST_ICSSG_INST_ID, APP_TS_CORE_CLKSEL_CPSWHSDIV_CLKOUT2);
    if (status != APP_TS_ERR_NERR) {
        UART_printf("\n\rtaskSysInitFxn: Error=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Initialize ICSSG */
    status = initIcss(TEST_ICSSG_INST_ID, &gPruIcssHandle);
    if (status != APP_TS_ERR_NERR) {
        UART_printf("\n\rtaskSysInitFxn: Error=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Configure Interrupts */
    status = configureInterrupts();
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = APP_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Register interrupt */
    status = registerIntrOnCmpEvent(TS_CMPEVT_INTRTR_R5, &pruTsIrqHandler);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = APP_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Initialize PRU for Timesync */
    status = initPruTimesync(gPruIcssHandle, TEST_PRU_INST_ID);
    if (status != APP_TS_ERR_NERR) {
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Initialize ICSSG TS DRV */
    /* Initialize PRU for TS */
    tsPrms.icssInstId = TEST_ICSSG_INST_ID;
    tsPrms.pruInstId = TEST_PRU_INST_ID;
    tsPrms.prdCount[0] = TEST_PRD_COUNT1;
    tsPrms.prdCount[1] = TEST_PRD_COUNT2;
    tsPrms.prdCount[2] = TEST_PRD_COUNT3;
    tsPrms.prdCount[3] = TEST_PRD_COUNT4;
    tsPrms.prdOffset[0] = TEST_PRD_OFFSET1;
    tsPrms.prdOffset[1] = TEST_PRD_OFFSET2;
    tsPrms.prdOffset[2] = TEST_PRD_OFFSET3;
    tsPrms.prdOffset[3] = TEST_PRD_OFFSET4;
    tsPrms.cfgMask = TS_CFG_CMP_ALL;
    tsPrms.testTsPrdCount = TEST_PRD_COUNT0;
    status = initIcssgTsDrv(gPruIcssHandle, &tsPrms, &gTestTs);
    if (status != APP_TS_ERR_NERR) {
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Enable interrupt for event from PRU */
    enableIntrOnPruEvent(TS_CMPEVT_INTRTR_R5);

    /* Start TSs */
    status = startTs(&gTestTs);
    if (status != APP_TS_ERR_NERR) {
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    while (seconds < 30) {
        seconds = interrupts/100000;
        if (seconds != last_seconds)
        {
            last_seconds = seconds;
            /* There is minor race here where numDelay might not match totDelay by 1 or min/max includes an extra interrupt */
            UART_printf("s=%d, min=%d, max=%d, n=%d, tot=%d, er=%d\n",seconds, minDelay, maxDelay, numDelay, totDelay, errorDelay);
            resetDelay = 1;
        }
    }

    System_printf("exit taskSysInitFxn()\n");
}

/*
 *  ======== main ========
 */
Int main()
{
    Task_Params taskParams;
    Error_Block eb;

    /* Create System Initialization Task */
    Task_Params_init(&taskParams);
    taskParams.priority = TASK_SYSINIT_PRI;
    Error_init(&eb);
    gHSysInitTask = Task_create(taskSysInitFxn, &taskParams, &eb);
    if (gHSysInitTask == NULL)
    {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    /* Initialize profiling */
    init_profiling();
    gStartTime = readPmu(); /* two initial reads are necessary for correct overhead time */
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;

    BIOS_start();   /* does not return */
    return(0);
}

/* PRU TS IRQ handler */
void pruTsIrqHandler(uintptr_t foobar)
{
    uint32_t curIep, curCmp7, thisDelay;
    
    /* Benchmark */
    icssgTsDrv_readIepCmp(gTestTs.hTsDrv, &curIep, &curCmp7, NULL, NULL, NULL);
    
    /* Clear interrupt on Host */
    Osal_ClearInterrupt(TS_CMPEVT_INTRTR_R5, TS_CMPEVT_INTRTR_R5);

    if (resetDelay) {
        /* it is not necessary to reset min/max/tot because they are set to thisDelay next iteration */
        resetDelay = numDelay = errorDelay = 0;
    } else {
        thisDelay = curIep - lastCmp;
        if (thisDelay > 0x80000000u) {
            errorDelay++; /* Error computing delay, we read curCmp7 before it was updated by fw */
        } else {
            if (numDelay == 0) {
                minDelay = maxDelay = totDelay = thisDelay;
            } else {
                if (thisDelay > maxDelay) {
                    maxDelay = thisDelay;
                }
                if (thisDelay < minDelay) {
                    minDelay = thisDelay;
                }
                totDelay += thisDelay;
            }
            numDelay++;
        }
    }
    lastCmp = curCmp7;
    interrupts++;
}

