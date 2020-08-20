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

#include <ti/csl/tistdtypes.h>
#include <xdc/runtime/Error.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_mailbox.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/osal/osal.h>
#include <ti/board/board.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <app_log.h>
#include <app_sciclient.h>
#include <app_misc.h>
#include <app_misc_soc.h>

#include "cia402appl.h"

#include <app_pslctrl_cfg.h>
#include "app_pslctrl_mbxipc.h"
#include "app_pslctrl_cfg_mcu_intr.h"
#include "app_pslctrl_timesync.h"
#include "app_pslctrl_esc_sim.h"
#include "app_pslctrl_save_data.h"

/* Status codes */
#define PSL_CTRL_INIT_SOK                   (  0 )
#define PSL_CTRL_INIT_SERR_SCICLIENT_INIT   ( -1 )
#define PSL_CTRL_INIT_SERR_MBXIPC_INIT      ( -2 )
#define PSL_CTRL_INIT_SERR_TS_INIT          ( -3 )

/* Function prototypes */
int32_t pslCtrlInit(
    pinmuxPerCfg_t *pPinmuxPerCfg
);
void StartupEmulatorWaitFxn(void);

/* Task stack */
uint8_t gTskStackMain[8*1024]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;

/* Task handle */
Task_Handle hTaskPslCtrl;

/* Time Sync object */
TsObj gTs;

/* MC parameters -- set via JTAG, sent to PSL via IPC */
volatile int32_t    gVelocityTarget = 0.2*10000;
volatile int32_t    gPositionTarget = 0*10000;
/* cia402appl.h:
    CYCLIC_SYNC_POSITION_MODE:  8, Cyclic Synchronous Position mode
    CYCLIC_SYNC_VELOCITY_MODE:  9, Cyclic Synchronous Velocity mode
*/
volatile int16_t    gModesOfOperation = CYCLIC_SYNC_VELOCITY_MODE;
/* cia402appl.h:
    STATE_READY_TO_SWITCH_ON:   0x0004, Ready to switch on (mandatory)
    STATE_OPERATION_ENABLED:    0x0010, Operation enabled (mandatory)
*/
volatile int16_t    gCtrlState = STATE_READY_TO_SWITCH_ON;

/* Indicates whether to continue sending/receiving IPC data */
volatile bool gRunState = TRUE;

uint32_t gSimSync0IsrCnt=0; /* Simulated SYNC0 ISR count */

/* Simulated SYNC0 ISR -- IPC gen traffic to Pos/Speed Loop */
void simSync0IrqHandler(uintptr_t arg)
{
    ecat2mc_msg_obj_t *txobj;
    uint32_t payload;
    uint16_t axisIdx;

    gSimSync0IsrCnt++;
    
    if (appMbxIpcGetSelfCpuId() == IPC_ETHERCAT_CPU_ID)
    {
        for (axisIdx = 0; axisIdx < MAX_NUM_AXES; axisIdx++)
        {
            /* Fill in Tx data */
            txobj = &gAppPslCtrlTxMsgAxes[axisIdx].sendObj;
            txobj->i32TargetVelocity = gVelocityTarget;
            txobj->i32TargetPosition = gPositionTarget;
            txobj->i16ModesOfOperation = gModesOfOperation;
            txobj->i16State = gCtrlState;
            txobj->u16AxisIndex = axisIdx;

            /* Translate the ATCM local view addr to SoC view addr */
            payload = (uint32_t)txobj;
            payload = CPU0_ATCM_SOCVIEW(payload);
            /* Tx address of payload */
            appMbxIpcSendNotify(IPC_PSL_MC_CPU_ID, payload);
        }
    }
}

/* Initialize PSL Control */
int32_t pslCtrlInit(
    pinmuxPerCfg_t *pPinmuxPerCfg
)
{
    int32_t status;

#ifdef ENABLE_BOARD
    {
	Board_initCfg boardCfg;

	/* Pad configurations */
	boardCfg = BOARD_INIT_UNLOCK_MMR | BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG;
	Board_init(boardCfg);

	/* PINMUX configurations */
	appSetPinmux(pPinmuxPerCfg);
    }
#endif

    /* Initialize Sciclient */
    status = appSciclientInit();
    if (status != 0)
    {
        appLogPrintf("pslCtrlInit: appSciclientInit() failed.\n");
        return PSL_CTRL_INIT_SERR_SCICLIENT_INIT;
    }

    return PSL_CTRL_INIT_SOK;
}

/* Task PSL Control function */
void taskPslCtrl(uint32_t arg0, uint32_t arg1)
{
    appPslCtrlMbxIpcCfg_t appPslCtrlMbxIpcCfg;
    TsPrmsObj tsPrms;
    mc2ecat_msg_obj_t *rxobj;
    bool isMsgFound = FALSE;
    uint16_t i;
    uint16_t axisIdx;
    uint32_t saveDataIdx;
    int32_t status;
    uintptr_t key;

    /* Initialize PSL Control */
    status = pslCtrlInit(&gFsiPinCfg);
    if (status != PSL_CTRL_INIT_SOK)
    {
        appLogPrintf("taskPslCtrl: pslCtrlInit() failed.\n");
        Task_exit();
    }

    /* Initialize MBX IPC */
    appPslCtrlMbxIpcCfg.appMbxIpcInitPrm.self_cpu_id = IPC_ETHERCAT_CPU_ID;
    appPslCtrlMbxIpcCfg.appMbxIpcInitPrm.master_cpu_id = IPC_ETHERCAT_CPU_ID;
    appPslCtrlMbxIpcCfg.appMbxIpcInitPrm.num_cpus = 2;
    appPslCtrlMbxIpcCfg.appMbxIpcInitPrm.enabled_cpu_id_list[0] = IPC_ETHERCAT_CPU_ID;
    appPslCtrlMbxIpcCfg.appMbxIpcInitPrm.enabled_cpu_id_list[1] = IPC_PSL_MC_CPU_ID;
    appPslCtrlMbxIpcCfg.appMbxIpcMsgHandler = appMbxIpcMsgHandler;
    status = appPslCtrlMbxIpcInit(&appPslCtrlMbxIpcCfg);
    if (status != APP_PSLCTRL_MBXIPC_SOK)
    {
        appLogPrintf("taskPslCtrl: appPslCtrlMbxIpcInit() failed.\n");
        Task_exit();
    }

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
    tsPrms.simSync0PrdCount = SIM_SYNC0_TS_PRD_COUNT0;
    tsPrms.simSync0CmpEvtRtrInIntNum = SIM_SYNC0_CMPEVT_INTRTR_IN;
    tsPrms.simSync0CmpEvtRtrOutIntNum = SIM_SYNC0_CMPEVT_INTRTR_OUT;
    tsPrms.simSync0CmpEvtRtrHostId = SIM_SYNC0_CMPEVT_INTRTR_HOST_ID;
    tsPrms.simSync0IntrNum = SIM_SYNC0_INTR_NUM;
    tsPrms.simSync0IsrRoutine = simSync0IrqHandler;
    status = appPslCtrlTsInit(&tsPrms, &gTs);
    if (status != APP_PSLCTRL_TS_SOK)
    {
        appLogPrintf("taskPslCtrl: appPslCtrlTsInit() failed.\n");
        Task_exit();
    }
    
    /* Initialze ESC firmware regs */
    status = escFwRegsInit(TS_ICSSG_INST_ID, STATE_SAFEOP, 
        SIM_SYNC0_TS_PRD_COUNT0);
    if (status != APP_PSLCTRL_ESC_SIM_SOK) {
        appLogPrintf("taskPslCtrl: escFwRegsInit() failed.\n");
        Task_exit();
    }

    /* Start Time Sync */
    status = startTs(&gTs);
    if (status != APP_PSLCTRL_TS_SOK) {
        appLogPrintf("taskPslCtrl: startTs() failed.\n");
        Task_exit();
    }

    do
    {
        /* Wait for IPC MSG from PSL R5F with actual MC parameters */
        do
        {
            for (i = 0; i < MAX_NUM_AXES; i++)
            {
                /* Enter critical section (Rx mailbox message receive flag), disable interrupts */
                key = HwiP_disable();
                if (gAppPslCtrlRxMsgAxes[i].isMsgReceived == 1) {
                    gAppPslCtrlRxMsgAxes[i].isMsgReceived = 0;
                    /* Exit critical section (Rx mailbox message receive flag), restore interrupt setting */
                    HwiP_restore(key);
                    
                    axisIdx = i;
                    isMsgFound = TRUE;
                    break;
                }
                else {
                    /* Exit critical section (Rx mailbox message receive flag), restore interrupt setting */
                    HwiP_restore(key);
                }
            }
        } while (isMsgFound == FALSE);
        isMsgFound = FALSE;

        rxobj = &gAppPslCtrlRxMsgAxes[i].receiveObj;
        if (axisIdx != rxobj->u16AxisIndex)
        {
            APP_ASSERT_SUCCESS(1);
        }
        
        /* Save received data to circular buffer */
        saveDataIdx = gSaveDataIdx[axisIdx];
        gVelocityActual[axisIdx][saveDataIdx] = rxobj->i32VelocityActual;  
        gPositionActual[axisIdx][saveDataIdx] = rxobj->i32PositionActual;
        saveDataIdx++;
        if (saveDataIdx >= APP_PSL_CTRL_SAVE_DATA_SZ)
        {
            saveDataIdx = 0;
        }
        gSaveDataIdx[axisIdx] = saveDataIdx;
        
    } while (gRunState == TRUE);

}

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
    } while (enableDebug);
}

int main(void)
{
    Task_Params tskParams;
    Error_Block eb;
	
    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    /* Create PSL Control Task */
    Error_init(&eb);
    Task_Params_init(&tskParams);
    tskParams.arg0 = (UArg) NULL;
    tskParams.arg1 = (UArg) NULL;
    tskParams.priority = TASK_PSL_CTRL_PRI;
    tskParams.stack = gTskStackMain;
    tskParams.stackSize = sizeof(gTskStackMain);
    hTaskPslCtrl = Task_create(taskPslCtrl, &tskParams, &eb);
    if (hTaskPslCtrl == NULL)
    {
        BIOS_exit(0);
    }
    BIOS_start();
}
