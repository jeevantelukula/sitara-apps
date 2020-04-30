/**
 * tiescutils.c: Implements all required Tasks and SWi for EtherCAT Slave.
 *               main() implemented in Beckhoff stack cia402appl.c file.
*/
/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
#include "ecat_def.h"

#include <TaskP.h>
#include <ti/osal/HwiP.h>
#include <OSP.h>

#include <tiescutils.h>
#include <tiesc_soc.h>
#include <applInterface.h>
#include <ecatslv.h>

#include <version.h>
/* TI-RTOS Header files */
#include <ti/osal/osal.h>
#ifndef DISABLE_UART_PRINT
#include <ti/drv/uart/UART_stdio.h>
#endif
#include <ti/board/board.h>

#include <board_gpioLed.h>
#include <board_i2cLed.h>
#include <board_spi.h>

/* Please note: Baremetal mode is not validated on AM6xx devices */
#ifdef BARE_METAL
#include <tiesc_baremetal.h>

#ifdef ENABLE_SPIA_TASK
#include <board_mcspi.h>
#include <ti/drv/spi/soc/SPI_v1.h>
#include <ti/drv/spi/src/SPI_osal.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#endif

#ifdef ENABLE_GPMC_TASK
#include <board_gpmc.h>
#include <ti/csl/src/ip/gpmc/V1/gpmc.h>
#include <ti/drv/gpmc/GPMC.h>
#include <ti/drv/gpmc/soc/GPMC_soc.h>
#endif

#endif //BARE_METAL

//#define PROFILE_ECAT_STACK
#ifdef PROFILE_ECAT_STACK
#include "tiescbsp.h"
uint32_t mainloop_delta, mainloop_max, pdi_delta, pdi_max, sync_delta, sync_max;
#endif

#if CiA402_DEVICE
#include <CiA402_eeprom.h> // header equivalent of ESI bin file
#elif TIESC_APPLICATION
#ifdef ECAT_LIMITED_DEMO
#include <demo_tiesc_eeprom.h> // header equivalent of ESI bin file
#else
#include <tiesc_eeprom.h> // header equivalent of ESI bin file
#endif
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Global Structure pointer holding PRUSS0 memory Map. */
PRUICSS_Handle pruIcss1Handle;
/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle pruIcss0Handle;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#ifdef ENABLE_SPIA_TASK
uint16_t SPIA_txBuffer[16];
uint16_t SPIA_rxBuffer[16];
#endif

#ifdef ENABLE_GPMC_TASK
uint8_t GPMC_txBuffer[32];
uint8_t GPMC_rxBuffer[32];
#endif

#if CiA402_DEVICE
extern UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize);
extern UINT16 CiA402_Init();
#endif


TaskP_Handle tsk1; // ECAT mainloop
#ifdef ENABLE_PDI_TASK
TaskP_Handle pdiTask; // ECAT sw ISR
#else
#ifdef ENABLE_PDI_SWI
void PDIswi(uint32_t arg1, uint32_t arg2);
SwiP_Handle pdiSwi; // ECAT sw ISR
#endif
#endif
#ifndef DISABLE_UART_PRINT
TaskP_Handle uartTask;                   // UART processing
#endif
TaskP_Handle ledTaskHndl;                // LED Control Task

#ifdef ENABLE_SYNC_TASK
TaskP_Handle sync0Task; // ECAT SYNC0 ISR
void Sync0task(uint32_t arg0, uint32_t arg1);
TaskP_Handle sync1Task; // ECAT SYNC1 ISR
void Sync1task(uint32_t arg0, uint32_t arg1);
#endif
uint32_t appState = 0;

uint8_t task1_init()
{
    uint8_t u8Err = 0;
    TaskP_Params taskParams;
#ifdef ENABLE_PDI_SWI
    SwiP_Params swiParams;
#endif

#ifndef DISABLE_UART_PRINT
    UART_printf("\nVersion - ");
    UART_printf(IND_PKG_VERSION);

    Board_IDInfo boardInfo;
    Board_getIDInfo(&boardInfo);

    UART_printf("\nBoard name \t: ");
    UART_printf(boardInfo.boardName);

    UART_printf("\nBoard Revision \t: ");
    UART_printf(boardInfo.version);

    if(isEtherCATDevice())
    {
        UART_printf("\n\rEtherCAT Device");
    }

    else
    {
        UART_printf("\n\rNon-EtherCAT Device");
    }

    UART_printf("\n\rSYS/BIOS EtherCAT Internal application ");
    UART_printf(APPL_BUILD_VER);
#endif

    bsp_soc_evm_init();

    /* initialize the Hardware and the EtherCAT Slave Controller */
    HW_Init();
    u8Err = MainInit(); // EtherCAT stack init

#if CiA402_DEVICE
    /*Initialize Axes structures*/
    CiA402_Init();
    /*Create basic mapping*/
    APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize);
#endif

    /* Create tasks */
    /* Create tree tasks that share a resource*/
#ifdef ENABLE_PDI_TASK
    TaskP_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stacksize = 1152*TIESC_TASK_STACK_SIZE_MUL;
    taskParams.arg0 = (void *)pruIcss1Handle;
    pdiTask = TaskP_create(PDItask, &taskParams);
#else
#ifdef ENABLE_PDI_SWI
    SwiP_Params_init(&swiParams);
    swiParams.priority = 6;
    pdiSwi = SwiP_create(PDIswi, &swiParams);
#endif
#endif
#ifdef ENABLE_SYNC_TASK
    TaskP_Params_init(&taskParams);
    taskParams.priority = 8;
    taskParams.stacksize = 1152*TIESC_TASK_STACK_SIZE_MUL;
    taskParams.arg0 = (void *)pruIcss1Handle;
    sync0Task = TaskP_create(Sync0task, &taskParams);

    TaskP_Params_init(&taskParams);
    taskParams.priority = 8;
    taskParams.stacksize = 1152*TIESC_TASK_STACK_SIZE_MUL;
    taskParams.arg0 = (void *)pruIcss1Handle;
    sync1Task = TaskP_create(Sync1task, &taskParams);
#endif

#ifdef ENABLE_SPIA_TASK
    TaskP_Params_init(&taskParams);
    taskParams.priority = 4;
    TaskP_create(SPIAMaster_statusTask, &taskParams);
#endif

#ifdef ENABLE_GPMC_TASK
    TaskP_Params_init(&taskParams);
    taskParams.priority = 4;
    TaskP_create(GPMC_statusTask, &taskParams);
#endif

    TaskP_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stacksize = 1512*TIESC_TASK_STACK_SIZE_MUL;
    ledTaskHndl = TaskP_create(LEDtask, &taskParams);

    return u8Err;
}

void task1(uint32_t arg0, uint32_t arg1)
{
    static int task1init = FALSE;
    if(!task1init)
    {
        task1_init();
        task1init = TRUE;
    }
    bRunApplication = TRUE;
#ifndef BARE_METAL
        do
        {
#endif
#ifdef PROFILE_ECAT_STACK
            {
                uint32_t mainloop_start, mainloop_stop;
                bsp_get_local_sys_time(&mainloop_start, 0);
#endif

                MainLoop();
#ifdef PROFILE_ECAT_STACK
                bsp_get_local_sys_time(&mainloop_stop,  0);

                if(mainloop_stop >= mainloop_start)
                {
                    mainloop_delta = mainloop_stop - mainloop_start;
                }

                else
                {
                    mainloop_delta = 0xFFFFFFFFF - mainloop_start + mainloop_stop;
                }

                if(mainloop_delta > mainloop_max)
                {
                    mainloop_max = mainloop_delta;
                }
            }
#endif
#ifndef BARE_METAL
            TaskP_yield();
        }
        while(bRunApplication == TRUE);
#else
        if(bRunApplication == TRUE)
        {
            return;
        }
#endif //BARE_METAL

    HW_Release();

    OSAL_OS_exit(0);
}



#ifdef ENABLE_PDI_TASK
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief Interrupt service routine for the interrupts from the EtherCAT Slave Controller
*////////////////////////////////////////////////////////////////////////////////////////
#if AL_EVENT_ENABLED
void HW_EcatIsr(void)
{
#ifdef PROFILE_ECAT_STACK
    uint32_t pdi_start, pdi_stop;
    /* get the AL event register */
    bsp_get_local_sys_time(&pdi_start,  0);
#endif
    PDI_Isr();
#ifdef PROFILE_ECAT_STACK
    bsp_get_local_sys_time(&pdi_stop,   0);
    pdi_delta = pdi_stop - pdi_start;

    if(pdi_delta > pdi_max)
    {
        pdi_max = pdi_delta;
    }

#endif
}
#endif

void PDItask(uint32_t arg1, uint32_t arg2)
{
    TaskP_sleep(10 * OS_TICKS_IN_MILLI_SEC);
#if AL_EVENT_ENABLED
    uint32_t evtOutNum = HOST_AL_EVENT - 20;
#ifndef BARE_METAL
    while(1)
    {
        PRUICSS_pruWaitEvent((PRUICSS_Handle)arg1, evtOutNum);
#else
    {
        if(baremetal_PRUICSS_pruWaitEvent((PRUICSS_Handle)pruIcss1Handle, evtOutNum) != SemaphoreP_OK)
        {
            return;
        }
#endif //BARE_METAL
        /* ISR processing */
        HW_EcatIsr();
    }
#endif
}
#else
#ifdef ENABLE_PDI_SWI
void PDIswi(uint32_t arg1, uint32_t arg2)
{
#ifdef PROFILE_ECAT_STACK
    uint32_t pdi_start, pdi_stop;
    /* get the AL event register */
    bsp_get_local_sys_time(&pdi_start,  0);
#endif
    PDI_Isr();
#ifdef PROFILE_ECAT_STACK
    bsp_get_local_sys_time(&pdi_stop,   0);
    pdi_delta = pdi_stop - pdi_start;

    if(pdi_delta > pdi_max)
    {
        pdi_max = pdi_delta;
    }

#endif
}
void PDI_Swi()
{
    SwiP_post(pdiSwi);
}
#endif
#endif

void LEDtask(uint32_t arg0, uint32_t arg1)
#ifndef BARE_METAL
{

    VARVOLATILE uint8_t state = STATE_INIT;
#else
{
    static bool LEDinit = FALSE;
    static VARVOLATILE uint8_t state;
    if(!LEDinit)
    {
        LEDinit = TRUE;
        state = STATE_INIT;
#endif //BARE_METAL
#ifdef ENABLE_STARTUP_DIGOUT_ANIMATION
        VARVOLATILE uint8_t state = STATE_INIT;
        static uint16_t digout_led_mask = 1;

        while(STATE_INIT == state)
        {
            I2CSetLed(&gI2CObj, digout_led_mask);
            digout_led_mask <<= 1;

            if(digout_led_mask >= 256)
            {
                digout_led_mask = 1;
            }

            HW_EscReadByte(state, ESC_ADDR_ALSTATUS);
            TaskP_sleep(200 * OS_TICKS_IN_MILLI_SEC);
        }

#else
        TaskP_sleep(200 * OS_TICKS_IN_MILLI_SEC);

        Board_setDigOutput(0x6a);
#endif
#ifdef BARE_METAL
    }
#endif //BARE_METAL

#ifndef BARE_METAL
    while(1)
#endif
    {
        TaskP_sleep(50 * OS_TICKS_IN_MILLI_SEC);

        uint32_t reset_reg_val;
        //ReadHVS(&gMcSPIObj);// trigger a read of hardware inputs

#if ESC_EEPROM_EMULATION
        HW_EscReadByte(state, ESC_ADDR_ALSTATUS);
        state &= 0xF;

		if(bsp_get_eeprom_update_status())
		{
			uint32_t t_cur_time;
#ifdef USE_ECAT_TIMER
			bsp_get_local_sys_time(&t_cur_time, NULL);
#else
			t_cur_time = Timestamp_get32();
#endif
			uint32_t t_last_time = bsp_get_eeprom_updated_time();
			t_cur_time = ((t_cur_time >= t_last_time) ? (t_cur_time - t_last_time) :
						  t_cur_time + (0xFFFFFFFF - t_last_time));

			if(t_cur_time >= (ECAT_TIMER_INC_P_MS * 100))
			{
				bsp_set_eeprom_update_status(0);
				bsp_eeprom_emulation_flush();
			}
		}

#endif
        reset_reg_val = bsp_read_dword(pruIcss1Handle, ESC_ADDR_TI_ESC_RESET);

        if((reset_reg_val == TI_ESC_RST_CMD_U) ||
                (reset_reg_val == TI_ESC_RST_CMD_L))
        {
            //EtherCAT master has requested S/W RESET
            HW_RestartTarget();
        }
    }
}

#ifdef ENABLE_SPIA_TASK
void SPIAMaster_statusTask(uint32_t arg0, uint32_t arg1)
{
    static int SPIAMaster_init = 0;
    static SPI_Transaction transaction;
    static SPI_Handle masterASpi;
    if (!SPIAMaster_init)
    {
        masterASpi = SPIAMaster_open();
        SPIAMaster_init = 1;
    }

    transaction.count = 16; /* Number of frames */
    transaction.txBuf = &SPIA_txBuffer[0];
    transaction.rxBuf = &SPIA_rxBuffer[0];
    SPI_transfer(masterASpi, &transaction);
}
#endif

#ifdef ENABLE_GPMC_TASK
void GPMC_statusTask(uint32_t arg0, uint32_t arg1)
{
    static int gpmc_init = 0;
    static GPMC_Transaction g_transaction;
    static GPMC_Params     gpmcParams;  /* GPMC params structure */
    static GPMC_Handle     gpmcHandle;  /* GPMC handle */

    if (!gpmc_init)
    {
        GPMC_initConfig();
        GPMC_init();

        /* Use default GPMC config params if no params provided */
        GPMC_Params_init(&gpmcParams);
        gpmcHandle = (GPMC_Handle)GPMC_open(BOARD_GPMC_INSTANCE, &gpmcParams);
        gpmc_init = 1;
    }

    g_transaction.offset = 0x200;
    g_transaction.txBuf  = (void *)&GPMC_txBuffer[0];
    g_transaction.rxBuf  = NULL;
    g_transaction.count  = 32; /* Number of bytes */
    GPMC_transfer(gpmcHandle, &g_transaction);
}
#endif


#ifdef ENABLE_SYNC_TASK
void Sync0task(uint32_t arg1, uint32_t arg2)
{
#ifdef PROFILE_ECAT_STACK
    uint32_t sync_start, sync_stop;
#endif
#ifndef DISABLE_UART_PRINT
    UART_printf("SYNC0 task started\n\r");
#endif
    uint32_t evtOutNum = HOST_SYNC0_EVENT - 20;
#ifndef BARE_METAL
    while(1)
    {
        PRUICSS_pruWaitEvent((PRUICSS_Handle)arg1, evtOutNum);
#else
    {
        int32_t Sync0_state;
        Sync0_state = baremetal_PRUICSS_pruWaitEvent((PRUICSS_Handle)pruIcss1Handle, evtOutNum);
        if(Sync0_state != SemaphoreP_OK)
        {
            return;
        }
#endif //BARE_METAL
        //Do sync0 event handling
        DISABLE_ESC_INT();
#ifdef PROFILE_ECAT_STACK
        bsp_get_local_sys_time(&sync_start, 0);
#endif
        Sync0_Isr();

        if(!bsp_read_word((PRUICSS_Handle)arg1, ESC_ADDR_SYNC_PULSE_LENGTH))
        {
            bsp_read_byte((PRUICSS_Handle)arg1, ESC_ADDR_SYNC_STATUS);
        }

#ifdef PROFILE_ECAT_STACK
        bsp_get_local_sys_time(&sync_stop,  0);
        sync_delta = sync_stop - sync_start;

        if(sync_delta > sync_max)
        {
            sync_max = sync_delta;
        }

#endif
        ENABLE_ESC_INT();
    }
}

void Sync1task(uint32_t arg1, uint32_t arg2)
{
#ifdef PROFILE_ECAT_STACK
    uint32_t sync_start, sync_stop;
#endif
#ifndef DISABLE_UART_PRINT
    UART_printf("SYNC1 task started\n\r");
#endif
    uint32_t evtOutNum = HOST_SYNC1_EVENT - 20;
#ifndef BARE_METAL
    while(1)
    {
        PRUICSS_pruWaitEvent((PRUICSS_Handle)arg1, evtOutNum);
#else
    {
        int32_t Sync1_state;
        Sync1_state = baremetal_PRUICSS_pruWaitEvent((PRUICSS_Handle)pruIcss1Handle, evtOutNum);
        if(Sync1_state != SemaphoreP_OK)
        {
            return;
        }
#endif //BARE_METAL
        //Do sync1 event handling
        DISABLE_ESC_INT();
        Sync1_Isr();

        if(!bsp_read_word((PRUICSS_Handle)arg1, ESC_ADDR_SYNC_PULSE_LENGTH))
        {
            bsp_read_byte((PRUICSS_Handle)arg1, ESC_ADDR_SYNC_STATUS + 1);
        }

        ENABLE_ESC_INT();
    }
}
#endif

void StartupEmulatorWaitFxn (void)
{
    volatile uint32_t enableDebug = 0;
    do
    {
    }while (enableDebug);
}

void common_main()
{
    TaskP_Params taskParams;
	
    StartupEmulatorWaitFxn();

    Board_init(BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_MODULE_CLOCK |
               BOARD_INIT_ICSS_PINMUX | BOARD_INIT_UART_STDIO);
    TaskP_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stacksize = 2048*TIESC_TASK_STACK_SIZE_MUL;
    tsk1 = TaskP_create(task1, &taskParams);
    OSAL_OS_start();
}

