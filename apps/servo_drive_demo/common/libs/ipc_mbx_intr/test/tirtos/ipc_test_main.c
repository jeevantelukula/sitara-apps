/*
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
 **/

 
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/osal/HwiP.h>
#include <ti/osal/TaskP.h>
#include <ti/osal/osal.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_mailbox.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/board/board.h>

#include <app_log.h>
#include <app_mbx_ipc.h>
#include <app_sciclient.h>
#include <app_mbx_ipc_test.h>


/* Timer -- simulate ECAT interrupt */
void timerTickFxn(void *arg);   /* Timer tick function */
uint32_t gTimerIsrCnt=0;
int32_t  gStatus = 0;

/* Timer handle -- simulate ECAT interrupt */
TimerP_Handle gTimerHandle;

int32_t timerInit(void)
{
    int32_t status = 0;
    TimerP_Params timerParams;

    /* Set up timer -- simulate ECAT interrupt */
    /* Timer parameters */
    TimerP_Params_init(&timerParams);
    timerParams.name = "simEcatTimer";
    timerParams.periodType = TimerP_PeriodType_MICROSECS;
    timerParams.extfreqLo = SIM_ECAT_TIMER_FREQ_HZ;
    timerParams.extfreqHi = 0;
    timerParams.startMode = TimerP_StartMode_USER;
    timerParams.runMode = TimerP_RunMode_CONTINUOUS;
    timerParams.period = SIM_ECAT_TIMER_PERIOD_USEC;
    timerParams.arg = 0;
    timerParams.intNum = SIM_ECAT_TIMER_INTNUM;

    /* Create timer -- simulate ECAT interrupt */
    gTimerHandle = TimerP_create(SIM_ECAT_TIMER_ID, (TimerP_Fxn)&timerTickFxn, &timerParams);
    if (gTimerHandle == NULL)
    {
        status = -1;
        appLogPrintf("MBX IPC Test: Timer start Failed !!!\n");
    }
    else
    {
        /* Start timer */
        TimerP_start(gTimerHandle);
        appLogPrintf("MBX IPC Test: Timer started !!!\n");
    }

    return status;
}

/* Timer tick function -- simulate ECAT interrupt */
void timerTickFxn(void *arg)
{
    gTimerIsrCnt++;
    gStatus = ipcTestRun(gTimerIsrCnt);

    return;
}

static void taskMain(UArg arg0, UArg arg1)
{
    Board_initCfg boardCfg;
    app_mbxipc_init_prm_t mbxipc_init_prm;

    appLogPrintf("MBX-IPC: Echo Test Started ... !!!\n");

    /*Pad configurations */
    boardCfg = BOARD_INIT_UNLOCK_MMR | BOARD_INIT_UART_STDIO |
               BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_ICSS_PINMUX;
    Board_init(boardCfg);

    appSciclientInit();

    /* initialize CSL Mbx IPC */
    appMbxIpcInitPrmSetDefault(&mbxipc_init_prm);
    mbxipc_init_prm.master_cpu_id = MBXIPC_TEST_CPU_1;
    mbxipc_init_prm.self_cpu_id = MBXIPC_TEST_CPU_1;
    mbxipc_init_prm.num_cpus = 0;
    mbxipc_init_prm.enabled_cpu_id_list[mbxipc_init_prm.num_cpus] = MBXIPC_TEST_CPU_1;
    mbxipc_init_prm.num_cpus++;
    mbxipc_init_prm.enabled_cpu_id_list[mbxipc_init_prm.num_cpus] = MBXIPC_TEST_CPU_2;
    mbxipc_init_prm.num_cpus++;
    /* IPC CPU sync check works only when appMbxIpcInit() called from both R5Fs */
    appMbxIpcInit(&mbxipc_init_prm);
    /* Register Application callback to invoke on receiving a notify message */
    appMbxIpcRegisterNotifyHandler((app_mbxipc_notify_handler_f) mbxIpcMsgTestHandler);
    timerInit();

    do
    {
        if (gStatus)
        {
            break;
        }
        TaskP_sleep(1);
    }while(MAX_ITERATION_COUNT > gTimerIsrCnt);

    if (gStatus)
    {
        appLogPrintf("MBX-IPC: Echo Test Failed \n");
    }
    else
    {
        appLogPrintf("MBX-IPC: Echo Test Passed \n");
    }

    /* Stop timer */
    TimerP_stop(gTimerHandle);
    appLogPrintf("MBX IPC Test: Timer Stopped !!!\n");

    appMbxIpcDeInit();
    appSciclientDeInit();

    return;
}

static uint8_t gTskStackMain[8*1024]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;
    
int main(void)
{
    Task_Params tskParams;
    Error_Block eb;
    Task_Handle task;

    Error_init(&eb);
    Task_Params_init(&tskParams);

    tskParams.arg0 = (UArg) NULL;
    tskParams.arg1 = (UArg) NULL;
    tskParams.priority = 8u;
    tskParams.stack = gTskStackMain;
    tskParams.stackSize = sizeof (gTskStackMain);
    task = Task_create(taskMain, &tskParams, &eb);
    if(NULL == task)
    {
        BIOS_exit(0);
    }
    BIOS_start();

    return 0;
}

/********************************* End of file ******************************/

