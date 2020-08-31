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
#include <ti/osal/CacheP.h>
#include <OSP.h>

#include <app_sciclient.h>
#include <tiescipc.h>

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
#include <ti/osal/HwiP.h>

#include <board_gpioLed.h>
#include <board_i2cLed.h>
#include <board_spi.h>

/* Please note: Baremetal mode is not validated on AM6xx devices */
#ifdef BARE_METAL
#include <tiesc_baremetal.h>
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

/* Usage of #ifdef required here until Time Sync is supported on AM64x.*/
#ifdef SOC_AM65XX
/* Time Sync */
#include "app_ts.h"
#include "tiesctscfg.h"
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Global Structure pointer holding PRUSS0 memory Map. */
PRUICSS_Handle pruIcss1Handle;
/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle pruIcss0Handle;

#define ICSS0_RESET_ISOLATION_MEMORYREAD_ADDRESS (CSL_PRU_ICSSG0_RAM_SLV_RAM_BASE+0xF00) // 0xB010F00
#define ICSS1_RESET_ISOLATION_MEMORYREAD_ADDRESS (CSL_PRU_ICSSG1_RAM_SLV_RAM_BASE+0xF00) // 0xB110F00
#define ICSS_RESET_ISOLATION_VALUE              0x89ABCDEF

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


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
bool icssgResetIsolated = FALSE;

/* Global data MSG object used in IPC communication */
static app_ipc_mc_obj_t gAppIpcMsgObj
__attribute__ ((section(".bss:ipcMCBuffSection")))
__attribute__ ((aligned(128)))={0}
    ;
/* Usage of #ifdef required here until Time Sync is supported on AM64x.*/
#ifdef SOC_AM65XX
/* Time Sync object */
TsObj gTs;
#endif

uint8_t task1_init()
{
    uint8_t u8Err = 0;
    TaskP_Params taskParams;
#ifdef ENABLE_PDI_SWI
    SwiP_Params swiParams;
#endif
/* Usage of #ifdef required here until Time Sync is supported on AM64x.*/
#ifdef SOC_AM65XX
    TsPrmsObj tsPrms;
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
#endif

    bsp_soc_evm_init();

    /* initialize the Hardware and the EtherCAT Slave Controller */
    HW_Init();

    if((PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE) &&
       ((*((volatile uint32_t *)ICSS0_RESET_ISOLATION_MEMORYREAD_ADDRESS)) != ICSS_RESET_ISOLATION_VALUE))
    {
        /*Write a known value to 0xB010F00. */
        (*((volatile uint32_t *)(ICSS0_RESET_ISOLATION_MEMORYREAD_ADDRESS))) = ICSS_RESET_ISOLATION_VALUE;
    }
    else if((PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO) &&
       ((*((volatile uint32_t *)ICSS1_RESET_ISOLATION_MEMORYREAD_ADDRESS)) != ICSS_RESET_ISOLATION_VALUE))
    {
        /*Write a known value to 0xB110F00. */
        (*((volatile uint32_t *)(ICSS1_RESET_ISOLATION_MEMORYREAD_ADDRESS))) = ICSS_RESET_ISOLATION_VALUE;
    }

    u8Err = MainInit(); // EtherCAT stack init

#if CiA402_DEVICE
/* Usage of #ifdef required here until Time Sync is supported on AM64x.*/
#ifdef SOC_AM65XX
	/* Initialize Time Sync */
    memset(&tsPrms, 0, sizeof(tsPrms));
    tsPrms.icssInstId = TS_ICSSG_INST_ID;
    tsPrms.pruInstId = TS_PRU_INST_ID;
    tsPrms.prdCount[0] = TS_PRD_COUNT1;
    tsPrms.prdOffset[0] = TS_PRD_OFFSET1;
    tsPrms.cmpEvtRtrInIntNum[0] = TS_CMPEVT_INTRTR_IN0;
    tsPrms.cmpEvtRtrOutIntNum[0] = TS_CMPEVT_INTRTR_OUT0;
    tsPrms.cmpEvtRtrHostId[0] = TS_CMPEVT_INTRTR_HOST_ID0;
    tsPrms.prdCfgMask = TS_CFG_CMP7;
    u8Err |= appTs_initTs(pruIcss1Handle, &tsPrms, &gTs);
    /* Start Time Sync */
    u8Err |= appTs_startTs(&gTs);
#endif    
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

    TaskP_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stacksize = 1512*TIESC_TASK_STACK_SIZE_MUL;
    ledTaskHndl = TaskP_create(LEDtask, &taskParams);

    return u8Err;
}

void task1(uint32_t arg0, uint32_t arg1)
{
    int32_t i;
    uint8_t u8Err = 0;
    static int task1init = FALSE;
    app_mbxipc_init_prm_t mbxipc_init_prm;

    /* initialize SCICLIENT */
    u8Err = appSciclientInit();
    
    /* Perform PAD configuration */

    if(icssgResetIsolated==FALSE)
    {
		/*This configuration is required only after poweron reset. Is not required after a warm reset.*/
		tiesc_mii_pinmuxConfig();    
    }
    
    /* initialize CSL Mbx IPC */
    appMbxIpcInitPrmSetDefault(&mbxipc_init_prm);
    mbxipc_init_prm.master_cpu_id = IPC_ETHERCAT_CPU_ID;
    mbxipc_init_prm.self_cpu_id = IPC_ETHERCAT_CPU_ID;
    mbxipc_init_prm.num_cpus = 0;
    mbxipc_init_prm.enabled_cpu_id_list[mbxipc_init_prm.num_cpus] = IPC_ETHERCAT_CPU_ID;
    mbxipc_init_prm.num_cpus++;
    mbxipc_init_prm.enabled_cpu_id_list[mbxipc_init_prm.num_cpus] = IPC_PSL_MC_CPU_ID;
    mbxipc_init_prm.num_cpus++;
    /* IPC cpu sync check works only when appMbxIpcInit() called from both R5Fs */
    u8Err |= appMbxIpcInit(&mbxipc_init_prm);
    /* Register Application callback to invoke on receiving a notify message */
    appMbxIpcRegisterNotifyHandler((app_mbxipc_notify_handler_f) appMbxIpcMsgHandler);
    for (i=0; i< MAX_NUM_AXES; i++){
        gAppIpcMsgObj.axisObj[i].isMsgReceived = 0;
    }

    if(!task1init)
    {
        u8Err |= task1_init();
        task1init = TRUE;
        /* If task1_init() fails, better loop here as system will not work */
        while (u8Err);
    }

    /* Start Time Sync */
    //u8Err |= appTs_startTs(&gTs);
    //while (u8Err);
    
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

    appMbxIpcDeInit();

    appSciclientDeInit();
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

#ifndef TIESC_EMULATION_PLATFORM
        Board_setDigOutput(0x6a);
#endif
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

void common_main()
{
    TaskP_Params taskParams;
	
    if(((PRUICSS_INSTANCE == PRUICSS_INSTANCE_ONE) &&
    /*Read a known value from ICSS to know if it's reset isolated. */
    ((*((volatile uint32_t *)ICSS0_RESET_ISOLATION_MEMORYREAD_ADDRESS)) == ICSS_RESET_ISOLATION_VALUE))
    ||
    ((PRUICSS_INSTANCE == PRUICSS_INSTANCE_TWO) &&
    /*Read a known value from ICSSG to know if it's reset isolated. */
    ((*((volatile uint32_t *)ICSS1_RESET_ISOLATION_MEMORYREAD_ADDRESS)) == ICSS_RESET_ISOLATION_VALUE)))
    {
        icssgResetIsolated = TRUE;
    }

    /*Enable ICSSG0 Reset Isolation by setting RESETISO and BLKCHIP1RST bits in PSC0_MDCTL30 register. */
	/*This needs to be moved to ICSSG1(PSC0_MDCTL31 0xA7C) when EtherCAT support is moved to ICSSG1 by PSW team. */
    (*((volatile uint32_t *)(CSL_PSC0_BASE+0xA7C))) |= 0x1800;
	
    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    Board_init( BOARD_INIT_MODULE_CLOCK |
               BOARD_INIT_UART_STDIO );

    if(icssgResetIsolated==FALSE)
    {
        /*These initializations are not required after a warm reset, as pinmux does not change for a warm reset. */
		Board_init(BOARD_INIT_ICSS_PINMUX | BOARD_INIT_PINMUX_CONFIG);
    }

    TaskP_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stacksize = 2048*TIESC_TASK_STACK_SIZE_MUL;
    tsk1 = TaskP_create(task1, &taskParams);
    OSAL_OS_start();
}

/* ISR callback that is invoke when current CPU receives a MSG */
void appMbxIpcMsgHandler (uint32_t src_cpu_id, uint32_t payload)
{
    uint16_t axisIndex;
    mc2ecat_msg_obj_t *rxobj;
    mc2ecat_msg_obj_t *payload_ptr = (mc2ecat_msg_obj_t*)payload;

    axisIndex = payload_ptr->u16AxisIndex;
    if (axisIndex < MAX_NUM_AXES)
    {
        rxobj = &gAppIpcMsgObj.axisObj[axisIndex].receiveObj;
        if (src_cpu_id==IPC_PSL_MC_CPU_ID)
        {
            *rxobj = *payload_ptr;
            gAppIpcMsgObj.axisObj[axisIndex].isMsgReceived = 1;
        }
    }
}

#ifdef TI_CiA402_3AXIS_MOTOR_CONTROL
void TI_CiA402_3axisMotionControl(TCiA402Axis *pCiA402Axis)
{
    uintptr_t key;
    uint32_t payload;
    ecat2mc_msg_obj_t *txobj;
    mc2ecat_msg_obj_t *rxobj;	
	static uint8_t msgsent=0U;
	uint16_t axisIndex = pCiA402Axis->axisIndex;
	
	if(msgsent == 0U)
	{
		/* Send a message to Partner Core that R5F is up and running. */
		Send_BootComplete_Message_To_Partner();
		msgsent = 1U;
	}
 
	axisIndex = pCiA402Axis->axisIndex;
    if (axisIndex < MAX_NUM_AXES)
    {
        txobj = &gAppIpcMsgObj.axisObj[axisIndex].sendObj;
        rxobj = &gAppIpcMsgObj.axisObj[axisIndex].receiveObj;
        payload = (uint32_t)txobj;

        /* In EthCAT IPC loopback application, send Actual values calculated */
        /* by CiA402_DummyMotionControl() to Motor Control R5F and receiving */
        /* back on EtherCAT R5F.  This will be later modified to make use of */
        /* Position-speed MC algo on MC R5F to calculate the Actual Values.  */
        CiA402_DummyMotionControl(pCiA402Axis);
        txobj->i32TargetPosition = pCiA402Axis->Objects.objPositionActualValue;
        txobj->i32TargetVelocity = pCiA402Axis->Objects.objVelocityActualValue;
        txobj->i16ModesOfOperation = pCiA402Axis->Objects.objModesOfOperation;
        txobj->i16State = pCiA402Axis->i16State;
        txobj->u16AxisIndex = axisIndex;

        if (appMbxIpcGetSelfCpuId()==IPC_ETHERCAT_CPU_ID)
        {
            appMbxIpcSendNotify(IPC_PSL_MC_CPU_ID, CPU0_ATCM_SOCVIEW(payload));
        }

        /* Wait for IPC MSG from MC R5F with updated actual MC parameters */
        do
        {
            TaskP_yield();
        } while (1!=gAppIpcMsgObj.axisObj[axisIndex].isMsgReceived);

        key = HwiP_disable();
        gAppIpcMsgObj.axisObj[axisIndex].isMsgReceived = 0;
        HwiP_restore(key);
        /* Copy updated actual MC parameters to CiA402 Axis data object */
        if (axisIndex==rxobj->u16AxisIndex)
        {
            pCiA402Axis->Objects.objPositionActualValue = rxobj->i32PositionActual;
            pCiA402Axis->Objects.objVelocityActualValue = rxobj->i32VelocityActual;
        }
        else
        {
            APP_ASSERT_SUCCESS(1);
        }
    }
    else
    {
        APP_ASSERT_SUCCESS(1);
    }

}
#endif


