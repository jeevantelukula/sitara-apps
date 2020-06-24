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
#include <ti/csl/soc.h>
#include <ti/csl/csl_fsi_tx.h>
#include <ti/csl/csl_fsi_rx.h>
#include <ti/csl/cslr_icss.h>
#include <ti/osal/osal.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include <logs/include/app_log.h>
#include "motor_ctrl_settings.h"
#include "multi_axis_master_lead.h"
#include "multi_axis_master_ctrl.h"
#include "multi_axis_fsi_shared.h"
#include "cfg_icss.h"
#include "cfg_mcu_intr_soc.h"
#include "PRU_FSI_Transmit.h"		/* FSI Transmit image data */
#include "PRU_FSI_Receive.h"		/* FSI Receive image data */
#include "app_psl_mbxipc.h"
#include "position_speed_loop_if.h"

// debug
#include <ti/drv/gpio/GPIO.h>
#include "GPIO_board.h"

/* If FSI only pull speed and command for all slaves from the sysVars (CTRL_SYN_ENABLE) */
//#define _CTRL_SYN_ENABLE

/* Timer -- simulate ECAT interrupt */
void timerTickFxn(void *arg);   /* Timer tick function */
uint32_t gTimerIsrCnt=0;

/* 
 * PRU IRQ handlers
 */
__attribute__((interrupt("IRQ")))   void fsiRxInt1IrqHandler(void);
__attribute__((interrupt("IRQ")))   void fsiRxInt2IrqHandler(void);
__attribute__((interrupt("IRQ")))   void fsiTxInt1IrqHandler(void);
__attribute__((interrupt("IRQ")))   void fsiTxInt2IrqHandler(void);

// debug
uint32_t gFsiRxInt1IsrCnt=0;
uint32_t gFsiRxInt2IsrCnt=0;
uint32_t gFsiTxInt1IsrCnt=0;
uint32_t gFsiTxInt2IsrCnt=0;

/* ------------------------------------------------------------------------- *
 *                                Globals                                    *
 * ------------------------------------------------------------------------- */

/* ICSSG handle */
PRUICSS_Handle gPruIcssHandle;
/* Simulated FSI base pointer */
uint32_t gFsiTxBase = CSL_PRU_ICSSG2_DRAM0_SLV_RAM_BASE;
uint32_t gFsiRxBase = CSL_PRU_ICSSG2_DRAM1_SLV_RAM_BASE;
/* Global ping frame counter for handshake */
volatile uint8_t numPingFrames = 0;
volatile uint32_t numDataFrames = 0;
 
/* Timer handle -- simulate ECAT interrupt */
TimerP_Handle gTimerHandle;

/* Initialization function */
int32_t appPositionSpeedLoopInit(void)
{
    McuIntrRegPrms mcuIntrRegPrms;
    McuIntrRtrPrms mcuIntrRtrPrms;
    TimerP_Params timerParams;
    int32_t status;

    // initialize system parameters
#if (BUILDLEVEL >= FCL_LEVEL5 && BUILDLEVEL <= FCL_LEVEL9)
#ifndef _CTRL_SYN_ENABLE
    sysVars.ctrlSynSet = CTRL_SYN_DISABLE;
    sysVars.ecatCtrlSet = ECAT_CTRL_DISABLE;
#else
    /* If FSI only pull speed and command for all slaves from the sysVars (CTRL_SYN_ENABLE) */
    sysVars.ctrlSynSet = CTRL_SYN_ENABLE;
    sysVars.ecatCtrlSet = ECAT_CTRL_DISABLE;
#endif /* _CTRL_SYN_ENABLE */
#endif
    initSysParameters(&sysVars);
    
    {
        uint16_t nodes;

        for(nodes = SYS_NODEM; nodes< SYS_NODE_NUM; nodes++)
        {
            // initialize controller parameters for each motor
            initCtrlParameters(&ctrlVars[nodes]);

            // reset some controller variables for each motor
            resetControllerVars(&ctrlVars[nodes]);
        }
    }

#if (BUILDLEVEL >= FCL_LEVEL5)       // Control Over FSI    
    FSI_initParams();
#endif

    /* Initialize ICSSG */
    status = initIcss(FSI_ICSS_INST_ID, &gPruIcssHandle);
    if (status != CFG_ICSS_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /*
        Register ICSSG Host interrupts
    */

    /* Initialize MCU INTC */
    status = McuIntc_Init();
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }
    
    /* Configure MCU interrupt for FSI RX INT1 */
    mcuIntrRegPrms.intrNum = FSI_RX_INT1_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_RX_INT1_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_RX_INT1_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_RX_INT1_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiRxInt1IrqHandler;
    mcuIntrRtrPrms.tisciSrcId = FSI_RX_INT1_INTR_RTR_DEV_SRC_ID;
    mcuIntrRtrPrms.tisciSrcIndex = FSI_RX_INT1_INTR_RTR_DEV_SRC_IRQ_IDX;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, &mcuIntrRtrPrms, MCU_INTR_IDX(0));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Configure MCU interrupt for FSI RX INT2 */
    mcuIntrRegPrms.intrNum = FSI_RX_INT2_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_RX_INT2_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_RX_INT2_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_RX_INT2_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiRxInt2IrqHandler;
    mcuIntrRtrPrms.tisciSrcId = FSI_RX_INT2_INTR_RTR_DEV_SRC_ID;
    mcuIntrRtrPrms.tisciSrcIndex = FSI_RX_INT2_INTR_RTR_DEV_SRC_IRQ_IDX;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, &mcuIntrRtrPrms, MCU_INTR_IDX(1));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Configure MCU interrupt for FSI TX INT1 */
    mcuIntrRegPrms.intrNum = FSI_TX_INT1_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_TX_INT1_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_TX_INT1_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_TX_INT1_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiTxInt1IrqHandler;
    mcuIntrRtrPrms.tisciSrcId = FSI_TX_INT1_INTR_RTR_DEV_SRC_ID;
    mcuIntrRtrPrms.tisciSrcIndex = FSI_TX_INT1_INTR_RTR_DEV_SRC_IRQ_IDX;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, &mcuIntrRtrPrms, MCU_INTR_IDX(2));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }
    
    /* Configure MCU interrupt for FSI TX INT2 */
    mcuIntrRegPrms.intrNum = FSI_TX_INT2_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_TX_INT2_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_TX_INT2_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_TX_INT2_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiTxInt2IrqHandler;
    mcuIntrRtrPrms.tisciSrcId = FSI_TX_INT2_INTR_RTR_DEV_SRC_ID;
    mcuIntrRtrPrms.tisciSrcIndex = FSI_TX_INT2_INTR_RTR_DEV_SRC_IRQ_IDX;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, &mcuIntrRtrPrms, MCU_INTR_IDX(3));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }
       
    /* Initialize PRU0 for FSI TX */
    status = initPruFsi(gPruIcssHandle, FSI_TX_PRU_INST_ID, 
        (uint32_t *)PRU_FSI_Transmit_image_1, sizeof(PRU_FSI_Transmit_image_1), 
        (uint32_t *)PRU_FSI_Transmit_image_0, sizeof(PRU_FSI_Transmit_image_0));
    if (status != CFG_ICSS_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Initialize PRU1 for FSI RX */
    status = initPruFsi(gPruIcssHandle, FSI_RX_PRU_INST_ID, 
        (uint32_t *)PRU_FSI_Receive_image_1, sizeof(PRU_FSI_Receive_image_1), 
        (uint32_t *)PRU_FSI_Receive_image_0, sizeof(PRU_FSI_Receive_image_0));
    if (status != CFG_ICSS_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }    
           
    /* Enable Host interrupts for events from PRU */
    McuIntc_enableIntr(MCU_INTR_IDX(0), true);
    McuIntc_enableIntr(MCU_INTR_IDX(1), true);
    McuIntc_enableIntr(MCU_INTR_IDX(2), true);
    McuIntc_enableIntr(MCU_INTR_IDX(3), true);
    
    /*
        Set up timer -- simulate ECAT interrupt
    */
    
    // TODO: update to use CSL-FL DM Timer + VIM configuration
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
        return POSITION_SPEED_LOOP_SERR_INIT;
    }
    
    // debug
    GPIO_init();
    GPIO_write(TEST_GPIO_IDX, GPIO_PIN_VAL_HIGH);
    GPIO_write(TEST_GPIO_IDX, GPIO_PIN_VAL_LOW);

    GPIO_write(TEST_GPIO2_IDX, GPIO_PIN_VAL_HIGH);
    GPIO_write(TEST_GPIO2_IDX, GPIO_PIN_VAL_LOW);

    return POSITION_SPEED_LOOP_SOK;
}

/* Entry point function */
int32_t appPositionSpeedLoopStart(void)
{
    SysNode_e nodeIdx;
    volatile uint8_t run_flag = 1;

    appLogPrintf("APP: Position Speed Loop demo started !!!\n");

    FSI_handshakeLead(gFsiTxBase, gFsiRxBase);

    appLogPrintf("APP: FSI Handshake Done !!!\n");

    FSI_setupTRxFrameData(gFsiTxBase, gFsiRxBase);

    /* Start timer */
    TimerP_start(gTimerHandle);

    appLogPrintf("APP: Timer started !!!\n");

    /* move to background task */
    while (run_flag) {
        for (nodeIdx = SYS_NODE1; nodeIdx <= SYS_NODE4; nodeIdx++)
        {
            /* Rx MC message from EthCAT */
            appPslMbxIpcRxMsg(nodeIdx);
            
            /* Run controller */
            runController(nodeIdx);
            
            /* Tx MC message to EthCAT */
            appPslMbxIpcTxMsg(nodeIdx);
        }
    }
    
    return POSITION_SPEED_LOOP_SOK;
}

/* Timer tick function -- simulate ECAT interrupt */
void timerTickFxn(void *arg)
{
    // debug
    gTimerIsrCnt++;
    GPIO_write(TEST_GPIO_IDX, GPIO_PIN_VAL_LOW);

    buildLevel7_9();
    FSI_updateTransmissionData();

    FSI_setTxBufferPtr(gFsiTxBase, 0U);
    FSI_setTxFrameType(gFsiTxBase, 0x3);
    FSI_writeTxDataBuffer(gFsiTxBase, fsiTxDataBufAddr, fsiTxDataWords);

    FSI_writeTxTagUserData(gFsiTxBase, fsiTxUserDataTag);

    FSI_startTxTransmit(gFsiTxBase);
    
    /* Inform background task to transmit latest actual values to EtherCAT   */
    gAppPslTxMsgAxes[ECAT_MC_AXIS_IDX0].isMsgSend = 1;
    gAppPslTxMsgAxes[ECAT_MC_AXIS_IDX1].isMsgSend = 1;
    gAppPslTxMsgAxes[ECAT_MC_AXIS_IDX2].isMsgSend = 1;

    // debug
    GPIO_write(TEST_GPIO_IDX, GPIO_PIN_VAL_HIGH);
}

/* IRQ handler, FSI RX INT1 */
void fsiRxInt1IrqHandler(void)
{
    volatile uint32_t intNum;
    uint16_t fsiRxStatus = 0;
    int32_t status;

    // debug
    gFsiRxInt1IsrCnt++;
    GPIO_write(TEST_GPIO2_IDX, GPIO_PIN_VAL_LOW);

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if (status == CSL_PASS)
    {
        
        /* Clear interrupt at source */
        /* Write 18 to ICSSG_STATUS_CLR_INDEX_REG
            18 = 16+2, 2 is Host Interrupt Number. See AM654x TRM, Table 6-391.
        */
        PRUICSS_pruClearEvent(gPruIcssHandle, 16+2);

        FSI_getRxEventStatus(gFsiRxBase, &fsiRxStatus);

        if (fsiRxStatus & FSI_RX_EVT_DATA_FRAME)
        {
            numDataFrames++;
            FSI_readRxFrameData(gFsiRxBase);
            FSI_updateReceivedData();
        }
        else if (fsiRxStatus & FSI_RX_EVT_PING_FRAME)
        {
            numPingFrames++;
        }

        FSI_setRxBufferPtr(gFsiRxBase, 0U);

        /* Clear the interrupt flag and issue ACK */
        FSI_clearRxEvents(gFsiRxBase, fsiRxStatus);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_RX_INT1_INT_MAP );

    }

    // debug
    GPIO_write(TEST_GPIO2_IDX, GPIO_PIN_VAL_HIGH);
}

/* IRQ handler, FSI RX INT2 */
void fsiRxInt2IrqHandler(void)
{
    volatile uint32_t intNum;
    int32_t status;

    // debug
    gFsiRxInt2IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if (status == CSL_PASS)
    {
        /* Clear interrupt at source */
        PRUICSS_pruClearEvent(gPruIcssHandle, 16+3);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_RX_INT2_INT_MAP );
    }
}

/* IRQ handler, FSI TX INT1 */
void fsiTxInt1IrqHandler(void)
{
    volatile uint32_t intNum;
    int32_t status;

    // debug
    gFsiTxInt1IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if (status == CSL_PASS)
    {
        /* Clear interrupt at source */
        PRUICSS_pruClearEvent(gPruIcssHandle, 16+4);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_TX_INT1_INT_MAP );
    }
}

/* IRQ handler, FSI TX INT2 */
void fsiTxInt2IrqHandler(void)
{
    volatile uint32_t intNum;
    int32_t status;

    // debug
    gFsiTxInt2IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if (status == CSL_PASS)
    {
        /* Clear interrupt at source */
        PRUICSS_pruClearEvent(gPruIcssHandle, 16+5);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_TX_INT2_INT_MAP );
    }
}


/* Deinitialization function */
int32_t appPositionSpeedLoopDeinit(void)
{
	// TODO
    return POSITION_SPEED_LOOP_SOK;
}
