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
#include "test_utils.h"
#include "timesync_array.h"             /* TS PRU FW image data */
#include "timesyncDrv_api.h"            /* TS driver API */


/* ------------------------------------------------------------------------- *
 *                          Macros & Structures                              *
 * ------------------------------------------------------------------------- */
/* ICSSG functional clock source selection options */
#define CORE_CLKSEL_PER1HSDIV_CLKOUT1   ( 0 )
#define CORE_CLKSEL_CPSWHSDIV_CLKOUT2   ( 1 )

/* Default ICSS pin mux setting */
#define PRUICSS_PINMUX_DEF              ( 0x0 )

/* Bits for timesync configuration mask */
#define TS_CFG_EN              ( 1<<0 )
#define TS_CFG_PRD_COUNT       ( 1<<1 )
#define TS_CFG_ALL \
    ( TS_CFG_EN | \
      TS_CFG_PRD_COUNT )

/* Status codes */
#define TEST_TS_ERR_NERR                (  0 )  /* no error */
#define TEST_TS_ERR_CFG_ICSSG_CLKSEL    ( -1 )  /* ICSSG clock selection error */
#define TEST_TS_ERR_CFG_HOST_INTR       ( -2 )  /* interrupt configuration error */
#define TEST_TS_ERR_INIT_ICSSG          ( -3 )  /* initialize ICSSG error */
#define TEST_TS_ERR_INIT_PRU            ( -4 )  /* initialize PRU error */
#define TEST_TS_ERR_INIT_TS_DRV         ( -5 )  /* initialize TS DRV error */
#define TEST_TS_ERR_START_TS            ( -6 )  /* start TS error */

/* Test ICSSG instance ID */
#define TEST_ICSSG_INST_ID              ( PRUICCSS_INSTANCE_ONE )
/* Test PRU instance ID */
#define TEST_PRU_INST_ID                ( PRUICCSS_RTU0 )

/* Task priorities */
#define TASK_SYSINIT_PRI                ( 3 )

/* Test TS CMP periods */
/* IEP frequency = 200 MHz, TS frequency = 32 kHz */
#define TEST_PRD_COUNT0         ( 200000000u/1000u   )    /* 1ms     (1KHz) - sim sync0 */
#define TEST_PRD_COUNT1         ( 200000000u/100000u )    /* 10us    (100Khz) (CMP3)    */
#define TEST_PRD_COUNT2         ( 200000000u/32000u  )    /* 31.25us (32Khz)  (CMP4)    */
#define TEST_PRD_COUNT3         ( 200000000u/8000u   )    /* 125us   (8Khz)   (CMP5)    */
#define TEST_PRD_COUNT4         ( 200000000u/1000u   )    /* 1ms     (1Khz)   (CMP6)    */

/* Test TS CMP offsets */
#define TEST_PRD_OFFSET1        (     0 )  /* No offset */
#define TEST_PRD_OFFSET2        (     0 )  /* No offset */
#define TEST_PRD_OFFSET3        ( -2000 )  /* 10us before sync0 */
#define TEST_PRD_OFFSET4        (  4000 )  /* 20us after sync0 */

/* TS parameters number of PRD */
#define TS_NUM_PRDS   ( 5 )

/* Compare event router i*/
#define CMPEVT_INTRTR_IN    ( 35 )  /* ICSSG_0_IEP0_CMP_TIMER3_INT */

/* Compare event router output */
#define CMPEVT_INTRTR_OUT   ( 16 )  /* COMPEVT_RTR_COMP_16_EVT */

/* TS configuration parameters */
typedef struct TsPrmsObj_s {
    PRUICSS_MaxInstances icssInstId;        /* ICSSG hardware instance ID */
    PRUSS_PruCores pruInstId;               /* PRU hardware instance ID */
    uint8_t nPrdCount;                      /* Number of PRD counts to set */
    uint32_t prdCount[TS_NUM_PRDS+1];       /* Period Count */
    int32_t prdOffset[TS_NUM_PRDS];         /* Positive/Negative offset */
    uint8_t cfgMask;                        /* Configuration mask */
} TsPrmsObj;

/* TS object */
typedef struct TsObj_s {
    PRUICSS_Handle pruIcssHandle;           /* PRUSS DRV handle */
    TsPrmsObj tsPrms;                       /* TS configuration parameters */
    uint8_t icssgId;                        /* TS DRV ICSSG hardware module ID */
    uint8_t pruId;                          /* TS DRV PRU hardware module ID */
    IcssgTsDrv_Handle hTsDrv;               /* TS DRV handle */
} TsObj;

/* ------------------------------------------------------------------------- *
 *                          Function Prototypes                              *
 * ------------------------------------------------------------------------- */

/* Configure ICSSG clock selection */
int32_t cfgIcssgClkSel(
    PRUICSS_MaxInstances icssInstId,
    uint8_t source
);

/* Initialize ICSSG */
int32_t initIcss(
    PRUICSS_MaxInstances icssInstId,
    PRUICSS_Handle *pPruIcssHandle
);

/* Initialize PRU for TS */
int32_t initPruTimesync(
    PRUICSS_Handle pruIcssHandle,
    PRUSS_PruCores pruInstId
);

/* Initialize ICSSG TS DRV */
int32_t initIcssgTsDrv(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTs
);

/* Start ICSSG TS */
int32_t startTs(
    TsObj *pTs
);

/* PRU TS IRQ handler */
void pruTsIrqHandler(uintptr_t foobar);

/* PRU TS SWI function */
void pruTsSwiFxn(UArg a0, UArg a1);

#define TEST_DC_CNT_INCR            ( 5 ) /* max resolution increment for IEP @ 200 MHz */
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
 *  ======== cfgIcssgClkSel ========
 */
/* Configure ICSSG clock selection */
int32_t cfgIcssgClkSel(
    PRUICSS_MaxInstances icssInstId,
    uint8_t source
)
{
    CSL_main_ctrl_mmr_cfg0Regs *pCtrlMmrCfg0Regs = (CSL_main_ctrl_mmr_cfg0Regs *)CSL_CTRL_MMR0_CFG0_BASE;
    uint32_t regVal;

    if (icssInstId == PRUICCSS_INSTANCE_ONE) {
        regVal = HW_RD_REG32(&pCtrlMmrCfg0Regs->ICSSG0_CLKSEL);
        regVal &= ~CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_CORE_CLKSEL_MASK;
        regVal |= source & 0x1;
        HW_WR_REG32(&pCtrlMmrCfg0Regs->ICSSG0_CLKSEL, regVal);
    }
    else if (icssInstId == PRUICCSS_INSTANCE_TWO) {
        regVal = HW_RD_REG32(&pCtrlMmrCfg0Regs->ICSSG1_CLKSEL);
        regVal &= ~CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_CORE_CLKSEL_MASK;
        regVal |= source & 0x1;
        HW_WR_REG32(&pCtrlMmrCfg0Regs->ICSSG1_CLKSEL, regVal);
    }
    else if (icssInstId == PRUICCSS_INSTANCE_MAX) {
        regVal = HW_RD_REG32(&pCtrlMmrCfg0Regs->ICSSG2_CLKSEL);
        regVal &= ~CSL_MAIN_CTRL_MMR_CFG0_ICSSG2_CLKSEL_CORE_CLKSEL_MASK;
        regVal |= source & 0x1;
        HW_WR_REG32(&pCtrlMmrCfg0Regs->ICSSG2_CLKSEL, regVal);
    }
    else {
        return TEST_TS_ERR_CFG_ICSSG_CLKSEL;
    }

    return TEST_TS_ERR_NERR;
}

/*
 *  ======== initIcss ========
 */
/* Initialize ICSSG */
int32_t initIcss(
    PRUICSS_MaxInstances icssInstId,
    PRUICSS_Handle *pPruIcssHandle
)
{
    PRUICSS_Config *pruIcssCfg; /* ICSS configuration */
    PRUICSS_Handle pruIcssHandle;
    uint8_t i;
    int32_t status;

    /* Get SoC level PRUICSS initial configuration */
    status = PRUICSS_socGetInitCfg(&pruIcssCfg);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return TEST_TS_ERR_INIT_ICSSG;
    }

    /* Create ICSS PRU instance */
    pruIcssHandle = PRUICSS_create(pruIcssCfg, icssInstId);
    if (pruIcssHandle == NULL) {
        return TEST_TS_ERR_INIT_ICSSG;
    }

    /* Disable PRUs & RTUs */
    for (i = 0; i < PRUICSS_MAX_PRU; i++)
    {
        status = PRUICSS_pruDisable(pruIcssHandle, i);
        if (status != PRUICSS_RETURN_SUCCESS) {
            return TEST_TS_ERR_INIT_ICSSG;
        }
    }

    /* Set ICSS pin mux to default */
    PRUICSS_pinMuxConfig(pruIcssHandle, PRUICSS_PINMUX_DEF);

    *pPruIcssHandle = pruIcssHandle;

    return TEST_TS_ERR_NERR;
}

/* Initialize PRU for timesync */
int32_t initPruTimesync(
    PRUICSS_Handle pruIcssHandle,
    PRUSS_PruCores pruInstId
)
{
    uint8_t slicePruInstId;
    uint32_t pruDMem, pruIMem;
    int32_t size;
    const uint32_t *sourceMem;    /* Source memory[ Array of uint32_t ] */
    uint32_t offset;              /* Offset at which write will happen */
    uint32_t byteLen;             /* Total number of bytes to be written */
    int32_t status;

    /* Reset PRU */
    status = PRUICSS_pruReset(pruIcssHandle, pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return TEST_TS_ERR_INIT_PRU;
    }

    /* Determine PRU ID in slice */
    slicePruInstId = pruInstId - (uint8_t)pruInstId/ICSSG_NUM_SLICE * ICSSG_NUM_SLICE;
    /* Determine PRU DMEM address */
    pruDMem = PRU_ICSS_DATARAM(slicePruInstId);
    /* Determine PRU IMEM address */
    switch (pruInstId)
    {
        case PRUICCSS_PRU0:
        case PRUICCSS_PRU1:
            pruIMem = PRU_ICSS_IRAM_PRU(slicePruInstId);
            break;
        case PRUICCSS_RTU0:
        case PRUICCSS_RTU1:
            pruIMem = PRU_ICSS_IRAM_RTU(slicePruInstId);
            break;
        case PRUICCSS_TPRU0:
        case PRUICCSS_TPRU1:
            pruIMem = PRU_ICSS_IRAM_TXPRU(slicePruInstId);
            break;
        default:
            break;
    }

    /* Initialize DMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, pruDMem);
    if (size == 0) {
        return TEST_TS_ERR_INIT_PRU;
    }

    /* Initialize IMEM */
    size = PRUICSS_pruInitMemory(pruIcssHandle, pruIMem);
    if (size == 0)
    {
        return TEST_TS_ERR_INIT_PRU;
    }

    /* Write DMEM */
    offset = ICSSG_TS_BASE_ADDR;
    sourceMem = (uint32_t *)pru_timesync_image_1;
    byteLen = sizeof(pru_timesync_image_1);
    size = PRUICSS_pruWriteMemory(pruIcssHandle, pruDMem, offset, sourceMem, byteLen);
    if (size == 0)
    {
        return TEST_TS_ERR_INIT_PRU;
    }

    /* Write IMEM */
    offset = 0;
    sourceMem = pru_timesync_image_0;
    byteLen = sizeof(pru_timesync_image_0);
    size = PRUICSS_pruWriteMemory(pruIcssHandle, pruIMem, offset, sourceMem, byteLen);
    if (size == 0)
    {
        return TEST_TS_ERR_INIT_PRU;
    }

    return TEST_TS_ERR_NERR;
}

/* Initialize ICSSG timesync DRV */
int32_t initIcssgTsDrv(
    PRUICSS_Handle pruIcssHandle,
    TsPrmsObj *pTsPrms,
    TsObj *pTs
)
{
    IcssgTsDrv_Handle hTsDrv;
    uint8_t cfgMask;
    uint32_t recfgBf;
    int32_t status;

    /* Copy TS parameters to TS object */
    pTs->tsPrms = *pTsPrms;
    /* Copy ICSS handle to TS object */
    pTs->pruIcssHandle = pruIcssHandle;

    /* Translate ICSSG & PRU hardware module IDs to TS API */
    status = getIcssgId(pTsPrms->icssInstId, &pTs->icssgId);
    if (status != TEST_UTILS_ERR_NERR) {
        return TEST_TS_ERR_INIT_TS_DRV;
    }
    getPruId(pTsPrms->pruInstId, &pTs->pruId);
    if (status != TEST_UTILS_ERR_NERR) {
        return TEST_TS_ERR_INIT_TS_DRV;
    }

    /* Initialize TS DRV instance */
    hTsDrv = icssgTsDrv_initDrv(pTs->icssgId, pTs->pruId);
    if (hTsDrv == NULL) {
        return TEST_TS_ERR_INIT_TS_DRV;
    }

    /* Set TS Global Enable */
    status = icssgTsDrv_setTsGblEn(hTsDrv, ICSSG_TS_DRV__IEP_TS_GBL_EN_ENABLE);
    if (status != ICSSG_TS_DRV__STS_NERR) {
        return TEST_TS_ERR_INIT_TS_DRV;
    }

    /* Set non-default configuration */
    cfgMask = pTs->tsPrms.cfgMask;

    /* Configure TS Period Count */
    if (cfgMask & TS_CFG_PRD_COUNT) {
        /* Configure IEP0 Period Count */
        status = icssgTsDrv_prepRecfgTsPrdCount(hTsDrv, pTs->tsPrms.prdCount, pTs->tsPrms.prdOffset, pTs->tsPrms.nPrdCount, &recfgBf);
        if (status != ICSSG_TS_DRV__STS_NERR) {
            return TEST_TS_ERR_INIT_TS_DRV;
        }
    }

    /* Store TS DRV handle to TS object */
    pTs->hTsDrv = hTsDrv;    

    return TEST_TS_ERR_NERR;
}

/* Start ICSSG Timesync */
int32_t startTs(
    TsObj *pTs
)
{
    int32_t status;

    /* Start IEP0 counter */
    icssgTsDrv_startIepCount(pTs->hTsDrv);

    /* Enable PRU */
    status = PRUICSS_pruEnable(pTs->pruIcssHandle, pTs->tsPrms.pruInstId);
    if (status != PRUICSS_RETURN_SUCCESS) {
        return TEST_TS_ERR_START_TS;
    }

    /* Wait for PRU FW initialization complete */
    status = icssgTsDrv_waitFwInit(pTs->hTsDrv);
    if (status != ICSSG_TS_DRV__STS_NERR)
    {
        return TEST_TS_ERR_START_TS;
    }

    return TEST_TS_ERR_NERR;
}

/*
 *  ======== taskSysInitFxn ========
 */
Void taskSysInitFxn(UArg a0, UArg a1)
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
    status = cfgIcssgClkSel(TEST_ICSSG_INST_ID, CORE_CLKSEL_CPSWHSDIV_CLKOUT2);
    if (status != TEST_TS_ERR_NERR) {
        UART_printf("\n\rtaskSysInitFxn: Error=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Initialize ICSSG */
    status = initIcss(TEST_ICSSG_INST_ID, &gPruIcssHandle);
    if (status != TEST_TS_ERR_NERR) {
        UART_printf("\n\rtaskSysInitFxn: Error=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Configure CompareEvent Interrupt Router */
    status = configureCmpEventInterruptRouter(CMPEVT_INTRTR_IN, CMPEVT_INTRTR_OUT);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Register interrupt */
    status = registerIntrOnCmpEvent(224+32, &pruTsIrqHandler);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Initialize PRU for Timesync */
    status = initPruTimesync(gPruIcssHandle, TEST_PRU_INST_ID);
    if (status != TEST_TS_ERR_NERR) {
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Initialize ICSSG TS DRV */
    /* Initialize PRU for TS */
    tsPrms.icssInstId = TEST_ICSSG_INST_ID;
    tsPrms.pruInstId = TEST_PRU_INST_ID;
    tsPrms.nPrdCount = TS_NUM_PRDS;
    tsPrms.prdCount[0] = TEST_PRD_COUNT0;
    tsPrms.prdCount[1] = TEST_PRD_COUNT1;
    tsPrms.prdCount[2] = TEST_PRD_COUNT2;
    tsPrms.prdCount[3] = TEST_PRD_COUNT3;
    tsPrms.prdCount[4] = TEST_PRD_COUNT4;

    tsPrms.prdOffset[0] = TEST_PRD_OFFSET1;
    tsPrms.prdOffset[1] = TEST_PRD_OFFSET2;
    tsPrms.prdOffset[2] = TEST_PRD_OFFSET3;
    tsPrms.prdOffset[3] = TEST_PRD_OFFSET4;

    tsPrms.cfgMask = TS_CFG_ALL;
    status = initIcssgTsDrv(gPruIcssHandle, &tsPrms, &gTestTs);
    if (status != TEST_TS_ERR_NERR) {
        UART_printf("\n\rError=%d: ", status);
        System_printf("taskSysInitFxn: Error=%d: ", status);
        System_exit(-1);
    }

    /* Enable interrupt for event from PRU */
    enableIntrOnPruEvent(224+32);

    /* Start TSs */
    status = startTs(&gTestTs);
    if (status != TEST_TS_ERR_NERR) {
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
    uint32_t curIep, curCmp3, thisDelay;
    
    /* Benchmark */
    icssgTsDrv_readIepCmp(gTestTs.hTsDrv, &curIep, &curCmp3, NULL, NULL, NULL);
    
    /* Clear interrupt on Host */
    Osal_ClearInterrupt(224+32, 224+32);

    if (resetDelay) {
        /* it is not necessary to reset min/max/tot because they are set to thisDelay next iteration */
        resetDelay = numDelay = errorDelay = 0;
    } else {
        thisDelay = curIep - lastCmp;
        if (thisDelay > 0x80000000u) {
            errorDelay++; /* Error computing delay, we read curCmp3 before it was updated by fw */
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
    lastCmp = curCmp3;
    interrupts++;
}

