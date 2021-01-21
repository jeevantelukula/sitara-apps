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
#include <ti/osal/osal.h>
#include <ti/board/board.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <board_i2c_io_exp.h>
#include <app_log.h>
#include "motor_ctrl_settings.h"
#include "multi_axis_master_lead.h"
#include "multi_axis_master_ctrl.h"
#include "multi_axis_fsi_shared.h"
#include "cfg_mcu_intr_soc.h"
#include "app_psl_mbxipc.h"
#include "position_speed_loop_if.h"

/* If FSI only pull speed and command for all slaves from the sysVars (CTRL_SYN_ENABLE) */
/*#define _CTRL_SYN_ENABLE */

/* 
 * PRU IRQ handlers
 */
__attribute__((interrupt("IRQ")))   void fsiRxInt1IrqHandler(void);
__attribute__((interrupt("IRQ")))   void fsiRxInt2IrqHandler(void);
__attribute__((interrupt("IRQ")))   void fsiTxInt1IrqHandler(void);
__attribute__((interrupt("IRQ")))   void fsiTxInt2IrqHandler(void);

/* Time Sync IRQ handler */
__attribute__((interrupt("IRQ")))   void tsIrqHandler(void);

/* Below pragma CODE_STATE overrides the compilation state of a file,
   at the function level. File is compiled in thumb2/16bit mode, but
   we want ISR functions in that file to be compiled in 32-bit mode.
   This is an ARM architecture requirement for Thumb2 mode to work.
   The ARM C/C++ compiler supports CODE_STATE pragma.
 */
#pragma CODE_STATE (fsiRxInt1IrqHandler,32)
#pragma CODE_STATE (fsiRxInt2IrqHandler,32)
#pragma CODE_STATE (fsiTxInt1IrqHandler,32)
#pragma CODE_STATE (fsiTxInt2IrqHandler,32)
#pragma CODE_STATE (tsIrqHandler,32)

/* FSI interrupt statistics counters */
volatile uint32_t gFsiRxInt1IsrCnt=0;   /* FSI Rx INT1 ISR count */
volatile uint32_t gFsiRxInt2IsrCnt=0;   /* FSI Rx INT2 ISR count */
volatile uint32_t gFsiTxInt1IsrCnt=0;   /* FSI Tx INT1 ISR count */
volatile uint32_t gFsiTxInt2IsrCnt=0;   /* FSI Tx INT2 ISR count */
volatile uint32_t gFsiRxInt1ErrCnt=0;   /* FSI Rx INT1 ISR error count */
volatile uint32_t gFsiRxInt2ErrCnt=0;   /* FSI Rx INT2 ISR error count */
volatile uint32_t gFsiTxInt1ErrCnt=0;   /* FSI Tx INT1 ISR error count */
volatile uint32_t gFsiTxInt2ErrCnt=0;   /* FSI Tx INT2 ISR error count */

/* Time Sync interrupt statistics counters */
volatile uint32_t gTsIsrCnt=0;          /* Time Sync ISR count */
volatile uint32_t gTsIntErrCnt=0;       /* Time Sync ISR error count */

/* ------------------------------------------------------------------------- *
 *                                Globals                                    *
 * ------------------------------------------------------------------------- */

/* FSI base pointer */
uint32_t gFsiTxBase = CSL_FSITX0_CFG_BASE;
uint32_t gFsiRxBase = CSL_FSIRX0_CFG_BASE;
/* Global ping frame counter for handshake */
volatile uint8_t numPingFrames = 0;
volatile uint32_t numDataFrames = 0;

/* Initialization function */
int32_t appPositionSpeedLoopInit(void)
{
    Board_I2cInitCfg_t i2cCfg;
    McuIntrRegPrms mcuIntrRegPrms;
    int32_t status;

    i2cCfg.i2cInst   = BOARD_I2C_IOEXP_DEVICE1_INSTANCE;
    i2cCfg.socDomain = BOARD_SOC_DOMAIN_MAIN;
    Board_setI2cInitConfig(&i2cCfg);

    Board_i2cIoExpInit();
    Board_i2cIoExpSetPinDirection(BOARD_I2C_IOEXP_DEVICE1_ADDR,
                                      THREE_PORT_IOEXP,
                                      PORTNUM_0,
                                      PIN_NUM_7,
                                      PIN_DIRECTION_OUTPUT);

    Board_i2cIoExpPinLevelSet(BOARD_I2C_IOEXP_DEVICE1_ADDR,
                                  THREE_PORT_IOEXP,
                                  PORTNUM_0,
                                  PIN_NUM_7,
                                  GPIO_SIGNAL_LEVEL_HIGH);

/* compile-time check for match between IPC & FSI number of motor control axes */
#if ((MAX_NUM_AXES != SYS_NODE_NUM) || (MAX_NUM_AXES != FSI_NODES))
    #error "Mismatch between IPC and FSI number of MC axes"
#elif (MAX_NUM_AXES != (FSI_NODE_LAST-FSI_NODE_FIRST+1))
    #error "Mismatch between IPC and FSI number of MC axes"
#endif

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

        for (nodes = SYS_NODE1; nodes < SYS_NODE_NUM; nodes++)
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

    /*
        Register FSI interrupts
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
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, MCU_INTR_IDX(0));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Configure MCU interrupt for FSI RX INT2 */
    mcuIntrRegPrms.intrNum = FSI_RX_INT2_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_RX_INT2_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_RX_INT2_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_RX_INT2_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiRxInt2IrqHandler;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, MCU_INTR_IDX(1));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Configure MCU interrupt for FSI TX INT1 */
    mcuIntrRegPrms.intrNum = FSI_TX_INT1_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_TX_INT1_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_TX_INT1_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_TX_INT1_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiTxInt1IrqHandler;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, MCU_INTR_IDX(2));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Configure MCU interrupt for FSI TX INT2 */
    mcuIntrRegPrms.intrNum = FSI_TX_INT2_INT_NUM;
    mcuIntrRegPrms.intrType = FSI_TX_INT2_INT_TYPE;
    mcuIntrRegPrms.intrMap = FSI_TX_INT2_INT_MAP;
    mcuIntrRegPrms.intrPri = FSI_TX_INT2_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &fsiTxInt2IrqHandler;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, MCU_INTR_IDX(3));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    } 
    
    /* Configure MCU interrupt for Time Sync */
    mcuIntrRegPrms.intrNum = TS_INT_NUM;
    mcuIntrRegPrms.intrType = TS_INT_TYPE;
    mcuIntrRegPrms.intrMap = TS_INT_MAP;
    mcuIntrRegPrms.intrPri = TS_INT_PRI;
    mcuIntrRegPrms.isrRoutine = &tsIrqHandler;
    status = McuIntc_cfgIntr(&mcuIntrRegPrms, MCU_INTR_IDX(4));
    if (status != CFG_MCU_INTR_SOK) {
        return POSITION_SPEED_LOOP_SERR_INIT;
    }

    /* Enable Host interrupts for events from PRU */
    McuIntc_enableIntr(MCU_INTR_IDX(0), true);
    McuIntc_enableIntr(MCU_INTR_IDX(1), true);
    McuIntc_enableIntr(MCU_INTR_IDX(2), false); /* FSI Tx INT1 currently unused */
    McuIntc_enableIntr(MCU_INTR_IDX(3), false); /* FSI Tx INT2 currently unused */

    return POSITION_SPEED_LOOP_SOK;
}

volatile uint8_t gRunFlag = 1;
/* Entry point function */
int32_t appPositionSpeedLoopStart(void)
{
    SysNode_e sysNodeIdx;   /* SYSTEM MC node index:    FSI_NODE_FIRST+1...FSI_NODE_LAST+1 */
    uint16_t mcAxisIdx;     /* IPC MC axis index:       0...MAX_NUM_AXES-1 */

    FSI_handshakeLead(gFsiTxBase, gFsiRxBase);

    FSI_setupTRxFrameData(gFsiTxBase, gFsiRxBase);

    /* Clear pending Time Sync interrupts */
    /* Enable Time Sync Interrupts */
    CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, TS_INT_NUM );
    McuIntc_enableIntr(MCU_INTR_IDX(4), true);

    /* move to background task */
    while (gRunFlag) {
        /* Continuously loop over all MC axes */
        sysNodeIdx = SYS_NODE1;
        for (mcAxisIdx = 0; mcAxisIdx < MAX_NUM_AXES; mcAxisIdx++)
        {
            /* Rx MC message from EthCAT */
            appPslMbxIpcRxMsg(mcAxisIdx, sysNodeIdx);
            
            /* Run controller */
            runController(sysNodeIdx);
            
            /* Tx MC message to EthCAT */
            appPslMbxIpcTxMsg(mcAxisIdx, sysNodeIdx);
            
            sysNodeIdx++;
        }
    }
    
    return POSITION_SPEED_LOOP_SOK;
}

/* Time Sync IRQ handler -- simulate SYNC0 */
void tsIrqHandler(void)
{
    volatile uint32_t intNum;
    int32_t status;

    /* Update statistics */
    gTsIsrCnt++;
    
    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if (status == CSL_PASS && intNum == TS_INT_NUM)
    {
        /* Clear pulse-type interrupt before executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );
    
        sysVars.speedLoopCount++;
        if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
        {
            /* Loop count satisfies the defined prescaler, run control loop and trigger FSI TX */
            buildLevel7_speedLoop();
        }

        /* Check state to determine if new state is requested or if fault has occurred */
        buildLevel7_ctrlStateMachine();

        /* Update TX data and send over FSI */
        FSI_updateTransmissionData();
        FSI_setTxBufferPtr(gFsiTxBase, 0U);
        FSI_setTxFrameType(gFsiTxBase, 0x3);
        FSI_writeTxDataBuffer(gFsiTxBase, fsiTxDataBufAddr, fsiTxDataWords);
        FSI_writeTxTagUserData(gFsiTxBase, fsiTxUserDataTag);
        FSI_startTxTransmit(gFsiTxBase);

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, TS_INT_MAP );
    }
    else 
    {
        gTsIntErrCnt++;
    }
}

/* IRQ handler, FSI RX INT1 */
void fsiRxInt1IrqHandler(void)
{
    volatile uint32_t intNum;
    uint16_t fsiRxStatus = 0;
    int32_t status;

    /* Update statistics */
    gFsiRxInt1IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if ((status == CSL_PASS) && (intNum == FSI_RX_INT1_INT_NUM))
    {
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
    else 
    {
        gFsiRxInt1ErrCnt++;
    }
}

/* IRQ handler, FSI RX INT2 */
void fsiRxInt2IrqHandler(void)
{
    volatile uint32_t intNum;
    uint16_t fsiRxStatus = 0;
    int32_t status;

    /* Update statistics */
    gFsiRxInt2IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if ((status == CSL_PASS) && (intNum == FSI_RX_INT2_INT_NUM))
    {
        FSI_getRxEventStatus(gFsiRxBase, &fsiRxStatus);

        /* Clear the interrupt flag and issue ACK */
        FSI_clearRxEvents(gFsiRxBase, fsiRxStatus);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_RX_INT2_INT_MAP );
    }
    else 
    {
        gFsiRxInt2ErrCnt++;
    }
}

/* IRQ handler, FSI TX INT1 */
void fsiTxInt1IrqHandler(void)
{
    volatile uint32_t intNum;
    uint16_t fsiTxStatus = 0;
    int32_t status;

    /* Update statistics */
    gFsiTxInt1IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if ((status == CSL_PASS) && (intNum == FSI_TX_INT1_INT_NUM))
    {
        FSI_getTxEventStatus(gFsiTxBase, &fsiTxStatus);

        /* Clear the interrupt flag and issue ACK */
        FSI_clearTxEvents(gFsiTxBase, fsiTxStatus);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_TX_INT1_INT_MAP );
    }
    else 
    {
        gFsiTxInt1ErrCnt++;
    }
}

/* IRQ handler, FSI TX INT2 */
void fsiTxInt2IrqHandler(void)
{
    volatile uint32_t intNum;
    uint16_t fsiTxStatus = 0;
    int32_t status;

    /* Update statistics */
    gFsiTxInt2IsrCnt++;

    status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        CSL_VIM_INTR_MAP_IRQ, (uint32_t *)&intNum, (uint32_t *)0 );
    if ((status == CSL_PASS) && (intNum == FSI_TX_INT2_INT_NUM))
    {
        FSI_getTxEventStatus(gFsiTxBase, &fsiTxStatus);

        /* Clear the interrupt flag and issue ACK */
        FSI_clearTxEvents(gFsiTxBase, fsiTxStatus);

        /* Clear level-type interrupt after executing ISR code */
        CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, intNum );

        /* Acknowledge interrupt servicing */
        CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, FSI_TX_INT2_INT_MAP );
    }
    else 
    {
        gFsiTxInt2ErrCnt++;
    }
}


/* Deinitialization function */
int32_t appPositionSpeedLoopDeinit(void)
{
	// TODO
    return POSITION_SPEED_LOOP_SOK;
}
