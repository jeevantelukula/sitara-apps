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
#include <xdc/runtime/System.h>
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

/* PRSDK includes */
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/board/board.h>
#include <ti/osal/osal.h>
#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

/* Status codes */
#define CFG_HOST_INTR_ERR_NERR                  (  0 )  /* no error */
#define CFG_HOST_INTR_ERR_CFG_INTR_RTR          ( -1 )  /* interrupt router configuration error */
#define CFG_HOST_INTR_ERR_REG_INTR              ( -2 )  /* interrupt registration error */
/* Status codes */
#define TEST_TS_ERR_NERR               (  0 )  /* no error */
#define TEST_TS_ERR_CFG_ICSSG_CLKSEL   ( -2 )  /* ICSSG clock selection error */
#define TEST_TS_ERR_CFG_HOST_INTR      ( -3 )  /* interrupt configuration error */
#define TEST_TS_ERR_INIT_ICSSG         ( -4 )  /* initialize ICSSG error */
#define TEST_TS_ERR_INIT_PRU           ( -5 )  /* initialize PRU error */
#define TEST_TS_ERR_INIT_TS_DRV       ( -6 )  /* initialize TS DRV error */
#define TEST_TS_ERR_INIT_TS_DRV_HL    ( -7 )  /* initialize TS DRV HL error */
#define TEST_TS_ERR_INIT_TS_DRV_LL    ( -8 )  /* initialize TS DRV LL error */
#define TEST_TS_ERR_START_TS          ( -9 )  /* start TS error */

HwiP_Handle hwiHandle = NULL;

/* Interrupt counter */
volatile uint32_t interrupts = 0;

/*
 * Registers an interrupt for event from compare event router
 * CMPEVENT_INTRTR0 interrupt router must be configured before this function is called.
 */
int32_t registerIntrOnCmpEvent(
    int32_t intrNum,
    void (*isrRoutine)(uintptr_t) /* The ISR routine to hook the corepacEventNum to */
)
{
    /* Register interrupt from PRU */
    OsalInterruptRetCode_e retCode;

    OsalRegisterIntrParams_t interruptRegParams;
    /* Initialize interrupt with defaults */
    Osal_RegisterInterrupt_initParams(&interruptRegParams);
    interruptRegParams.corepacConfig.triggerSensitivity = OSAL_ARM_GIC_TRIG_TYPE_EDGE;
    interruptRegParams.corepacConfig.isrRoutine = isrRoutine;
    interruptRegParams.corepacConfig.priority = 0x20U;
    interruptRegParams.corepacConfig.name = NULL;
    interruptRegParams.corepacConfig.corepacEventNum = intrNum;
    interruptRegParams.corepacConfig.intVecNum = intrNum; /* Host Interrupt vector */
    /* Register interrupt */
    retCode = Osal_RegisterInterrupt(&interruptRegParams, &(hwiHandle));
    if (retCode != osal_OK)
    {
        return CFG_HOST_INTR_ERR_REG_INTR;
    }

    /* Disable and clear interrupts */
    Osal_DisableInterrupt(intrNum, intrNum);
    Osal_ClearInterrupt(intrNum, intrNum);

    return CFG_HOST_INTR_ERR_NERR;
}


/* TS IRQ handler */
void tsIrqHandler(uintptr_t foobar)
{
    interrupts++;

    /* Clear interrupt on Host */
    Osal_ClearInterrupt(CSLR_GICSS0_SPI_CMP_EVENT_INTROUTER0_OUTP_0, CSLR_GICSS0_SPI_CMP_EVENT_INTROUTER0_OUTP_0);
}

static Void appMain(UArg arg0, UArg arg1)
{
    int32_t status;
    
    /* Register interrupt */
    status = registerIntrOnCmpEvent(CSLR_GICSS0_SPI_CMP_EVENT_INTROUTER0_OUTP_0, &tsIrqHandler);
    if (status != CFG_HOST_INTR_ERR_NERR) {
        status = TEST_TS_ERR_CFG_HOST_INTR;
        System_exit(-1);
    }
    /* Enable the interrupt */
    Osal_EnableInterrupt(CSLR_GICSS0_SPI_CMP_EVENT_INTROUTER0_OUTP_0, CSLR_GICSS0_SPI_CMP_EVENT_INTROUTER0_OUTP_0);

    while(1)
    {
        TaskP_sleepInMsecs(100u);
    }
}

void StartupEmulatorWaitFxn (void)
{
    volatile uint32_t enableDebug = 0;
    do
    {
    }while (enableDebug);
}


static uint8_t gTskStackMain[32*1024] 
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;

int main(void)
{
    Error_Block eb;
    
    Board_initCfg boardCfg;
    Task_Params tskParams;
    Task_Handle task;

    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    Error_init(&eb);
    Task_Params_init(&tskParams);

    tskParams.arg0 = (UArg) NULL;
    tskParams.arg1 = (UArg) NULL;
    tskParams.priority = 8u;
    tskParams.stack = gTskStackMain;
    tskParams.stackSize = sizeof (gTskStackMain);
    task = Task_create(appMain, &tskParams, &eb);
    if(NULL == task)
    {
        BIOS_exit(0);
    }
    BIOS_start();

    return 0;
}

#include <ti/sysbios/family/arm/v8a/Mmu.h>

volatile int emuwait_mmu=1;
void InitMmu(void)
{
    Bool retVal= false;
    Mmu_MapAttrs attrs;

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = 0;

    retVal = Mmu_map(0x00000000, 0x00000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x20000000, 0x20000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x40000000, 0x40000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x60000000, 0x60000000, 0x10000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = 7;

    retVal = Mmu_map(0x80000000, 0x80000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0xA0000000, 0xA0000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x70000000, 0x70000000, 0x10000000, &attrs); /* msmc        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = 7;

mmu_exit:
    if(retVal==false) {
		 printf("ERROR: Mmu_map returned error %d",retVal);
		 while(emuwait_mmu);
	 }

    return;
}

#include <ti/sysbios/hal/Cache.h>

Void ti_sysbios_hal_Cache_wb__E(Ptr blockPtr, SizeT byteCnt, Bits16 type, Bool wait)
{
    ti_sysbios_hal_Cache_CacheProxy_wb__E(blockPtr, byteCnt, type, wait);
}

Void ti_sysbios_hal_Cache_wbInv__E(Ptr blockPtr, SizeT byteCnt, Bits16 type, Bool wait)
{
    ti_sysbios_hal_Cache_CacheProxy_wbInv__E(blockPtr, byteCnt, type, wait);
}

Void ti_sysbios_hal_Cache_inv__E(Ptr blockPtr, SizeT byteCnt, Bits16 type, Bool wait)
{
    ti_sysbios_hal_Cache_CacheProxy_inv__E(blockPtr, byteCnt, type, wait);
}



