/**
 *  \file   benchmark_timer_interrupt.c
 *
 *  \brief  This file contains the benchmark timer interrupt functions and
 *          macros.
 *
 */

/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 *
 */

#include <stdint.h>

#include "benchmark_log.h"
#include "benchmark_stat.h"
#include "benchmark_timer_interrupt.h"

extern CSL_ArmR5CPUInfo cpuInfo;
extern uint32_t gCoreId;

timer_int_stat gTimerIntStat __attribute__((section(".testInData"))) = {0, 0, 0, 0};
uint32_t reloadVal __attribute__((section(".testInData")));
uint32_t curVal __attribute__((section(".testInData")));

#pragma DATA_SECTION (gVimRegsBaseAddr, ".testInData")
/* VIM registers base address */
uint32_t gVimRegsBaseAddr = UINIT_VIM_REGS_BASE_ADDR;

/* Timer allocation for each core */
McuTimerCfg gMcuTimerCfg[NUM_MCU_CORE] = {
    {1, CSL_TIMER1_CFG_BASE, CSLR_R5FSS0_CORE0_INTR_TIMER1_INTR_PEND_0},
    {2, CSL_TIMER2_CFG_BASE, CSLR_R5FSS0_CORE0_INTR_TIMER2_INTR_PEND_0},
    {3, CSL_TIMER3_CFG_BASE, CSLR_R5FSS0_CORE0_INTR_TIMER3_INTR_PEND_0},
    {4, CSL_TIMER4_CFG_BASE, CSLR_R5FSS0_CORE0_INTR_TIMER4_INTR_PEND_0},
};

/* MCU interrupt configurations */
static McuIntrCfg gMcuIntrCfg[NUM_MCU_INTR] = {
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, false},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, false},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, false},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, false},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, false}
};

#pragma CODE_STATE (benchmarkTimerISR,32) /* SCTLR:TE=0 on entry, so exceptions are taken in ARM state */
#pragma CODE_SECTION (benchmarkTimerISR, ".textHwiIsr")
__attribute__((interrupt("IRQ"))) void benchmarkTimerISR(void);

/* Initialize MCU INTC */
int32_t McuIntc_Init(void)
{
    CSL_ArmR5CPUInfo info;

    CSL_armR5GetCpuID(&info);
    if (info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0 || info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)
    {
        /* Initialize VIM base address */
        gVimRegsBaseAddr = CSL_MAIN_DOMAIN_VIM_BASE_ADDR;

        return CSL_PASS;
    }
    else
    {
        /* Invalid Pulsar R5 SS */

        gVimRegsBaseAddr = UINIT_VIM_REGS_BASE_ADDR;
        return CSL_EFAIL;
    }
}

/* Configure MCU interrupt */
int32_t McuIntc_cfgIntr(
    McuIntrRegPrms *pMcuIntrRegPrms,    /* MCU interrupt registration parameters */
    uint8_t mcuIntrIdx                  /* MCU interrupt index */
)
{
    McuIntrCfg *pMcuIntrCfg;
    int32_t status;

    if (mcuIntrIdx > NUM_MCU_INTR)
    {
        return CSL_EFAIL;
    }

    pMcuIntrCfg = &gMcuIntrCfg[mcuIntrIdx]; /* get MCU interrupt configuration */

    pMcuIntrCfg->intrCfgValid = false;

    if (pMcuIntrRegPrms != NULL) {
        /* Register VIM Interrupt */
        status = CSL_vimCfgIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr,
            pMcuIntrRegPrms->intrNum, 
            pMcuIntrRegPrms->intrPri, 
            pMcuIntrRegPrms->intrMap, 
            pMcuIntrRegPrms->intrType, 
            (uint32_t)pMcuIntrRegPrms->isrRoutine );
        if (status != CSL_PASS) {
            return CSL_EFAIL;
        }

        /* Copy interrupt parameters to MCU interrupt configuration */
        pMcuIntrCfg->mcuIntrRegPrms = *pMcuIntrRegPrms;
        pMcuIntrCfg->intrCfgValid = true;
    }

    return CSL_PASS;
}

/* MCU INTC, Enable MCU interrupt */
int32_t McuIntc_enableIntr(
    uint8_t mcuIntrIdx,             /* MCU interrupt index (0..NUM_MCU_INTR) */
    bool bEnable                    /* whether to enable/disable interrupt (true/false: enable/disable) */
)
{
    McuIntrCfg *pMcuIntrCfg;
    
    if (mcuIntrIdx > NUM_MCU_INTR)
    {
        return CSL_EFAIL;
    }

    pMcuIntrCfg = &gMcuIntrCfg[mcuIntrIdx]; /* get pointer to config structure */

    /* Enable the interrupt */
    CSL_vimSetIntrEnable( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr,
        pMcuIntrCfg->mcuIntrRegPrms.intrNum, bEnable );

    return CSL_PASS;
}

int32_t csldmTimer_registerTimerInt(uint32_t  coreId)
{
   McuIntrRegPrms mcuIntrRegPrms;
   int32_t status = CSL_PASS;

   /* Configure MCU interrupt for gMcuTimerCfg[coreId].timerId */
   mcuIntrRegPrms.intrNum = gMcuTimerCfg[coreId].intrNum;
   mcuIntrRegPrms.intrType = BENCHMARK_TIMER_INT_TYPE;
   mcuIntrRegPrms.intrMap = BENCHMARK_TIMER_INT_MAP;
   mcuIntrRegPrms.intrPri = BENCHMARK_TIMER_INT_PRI;
   mcuIntrRegPrms.isrRoutine = &benchmarkTimerISR;
   status = McuIntc_cfgIntr(&mcuIntrRegPrms, MCU_INTR_IDX(0));
   if (status != CSL_PASS) {
      MCBENCH_log("\nError: McuIntc_cfgIntr() fail.\n");
      return status;
   }

    return (status);
}

int32_t csldmTimer_initObj(uint32_t coreId)
{
    int32_t cslRet = CSL_EFAIL, testStatus;
    uint32_t status, baseAddr = gMcuTimerCfg[coreId].timerBaseAddr;

    /* Create the Timer ISR hook before enabling the timer */
    testStatus = csldmTimer_registerTimerInt(coreId);

    /* Reset the timer */
    if (testStatus == CSL_PASS)
    {
        cslRet = TIMERReset(baseAddr);
    }

    if (cslRet == CSL_PASS)
    {
        cslRet = TIMEREmuModeConfigure(baseAddr, TIMER_FREE);
    }

    if (cslRet == CSL_PASS)
    {
        cslRet = TIMERIdleModeConfigure(baseAddr, TIMER_NO_IDLE);
    }

    if (cslRet == CSL_PASS)
    {
        cslRet = TIMERPostedModeConfig(baseAddr, TIMER_NONPOSTED);
    }

    if (cslRet == CSL_PASS)
    {
        cslRet = TIMERReadModeConfig(baseAddr, TIMER_READ_MODE_NONPOSTED);
    }

    if (cslRet == CSL_PASS)
    {
        uint32_t postedStatus;
        cslRet = TIMERCounterSet(baseAddr, BENCHMARK_TIMER_COUNTER_VAL);
        do {
          cslRet = TIMERWritePostedStatusGet2(baseAddr, &postedStatus);
          if (cslRet != CSL_PASS)
          {
              break;
          }
          status = (postedStatus & BENCHMARK_TIMER_DM_TWPS_W_PEND_TCRR);
        } while (status != (uint32_t) 0u);
    }

    if (cslRet == CSL_PASS)
    {
        uint32_t postedStatus;
        cslRet = TIMERReloadSet(baseAddr, BENCHMARK_TIMER_COUNTER_VAL);
        do {
          cslRet = TIMERWritePostedStatusGet2(baseAddr, &postedStatus);
          if (cslRet != CSL_PASS)
          {
              break;
          }
          status = (postedStatus & BENCHMARK_TIMER_DM_TWPS_W_PEND_TLDR);
        } while (status != (uint32_t) 0u);
    }

    if (cslRet == CSL_PASS)
    {
        /* Enable the Timer Wakeup events represented by wakeFlags */
        cslRet = TIMERWakeEnable(baseAddr, TIMER_IRQWAKEEN_OVF_WUP_ENA_MASK);
    }

    if (cslRet == CSL_PASS)
    {
        TIMERIntEnable(baseAddr, TIMER_INT_OVF_EN_FLAG);
    }
    if (cslRet == CSL_PASS)
    {
        cslRet = TIMERDisable(baseAddr);
    }

    /* Return the operation status */
    if (cslRet == CSL_PASS)
    {
        testStatus = CSL_PASS;
    }
    else
    {
        testStatus = CSL_EFAIL;
    }

    return (testStatus);
}

int32_t csldmTimer_stop(uint32_t coreId)
{
    uint32_t  tisr = 0u;
    int32_t   cslRet;
    uint32_t  baseAddr = gMcuTimerCfg[coreId].timerBaseAddr;

    /* Disable the Timer */
    cslRet = TIMERDisable(baseAddr);

    if (cslRet == CSL_PASS)
    {
        cslRet = TIMERIntStatusGet2(baseAddr, &tisr);
    }

    if (cslRet == CSL_PASS)
    {
        if(tisr) {
          /* Clear all pending interrupts */
          cslRet = TIMERIntStatusClear(baseAddr, tisr);
        }
    }
    return (cslRet);
}

int32_t csldmTimer_setMode(uint32_t coreId, uint32_t timerMode) __attribute__((aligned(8), section(".testInCode")));
int32_t csldmTimer_setMode(uint32_t coreId, uint32_t timerMode)
{
    int32_t     testStatus, cslRet;
    uint32_t    key, baseAddr = gMcuTimerCfg[gCoreId].timerBaseAddr;

    key        = Intc_SystemDisable();
    testStatus = csldmTimer_stop(coreId);

    if (testStatus == CSL_PASS)
    {
        /* Clear the timer interrupt */
        Intc_IntClrPend(gMcuTimerCfg[gCoreId].intrNum);
        Intc_IntEnable(gMcuTimerCfg[gCoreId].intrNum);

        cslRet = TIMERModeConfigure(baseAddr, timerMode);
        if (cslRet != CSL_PASS)
        {
            testStatus = CSL_EFAIL;
        }
    }
    if (testStatus == CSL_PASS)
    {
        cslRet = TIMEREnable(baseAddr);
    }
    Intc_SystemRestore(key);

    if (cslRet == CSL_PASS)
    {
        return CSL_PASS;
    }
    else
    {
        return CSL_EFAIL;
    }
}

int32_t csldmTimer_setCountVal(uint32_t coreId, uint32_t timerCount) __attribute__((aligned(8), section(".testInCode")));
int32_t csldmTimer_setCountVal(uint32_t coreId, uint32_t timerCount)
{
    int32_t     cslRet;
    uint32_t    key, countVal;
    uint32_t    status, postedStatus;
    uint32_t    baseAddr = gMcuTimerCfg[gCoreId].timerBaseAddr;
	
    key        = Intc_SystemDisable();
    cslRet = csldmTimer_stop(coreId);

    if (cslRet == CSL_PASS)
    {
        /* Clear the timer interrupt */
        Intc_IntClrPend(gMcuTimerCfg[gCoreId].intrNum);
        Intc_IntEnable(gMcuTimerCfg[gCoreId].intrNum);

        countVal = 0xffffffff-timerCount+1;
        cslRet = TIMERCounterSet(baseAddr, countVal);
        if (cslRet == CSL_PASS)
        {
           do {
              cslRet = TIMERWritePostedStatusGet2(baseAddr, &postedStatus);
              if (cslRet != CSL_PASS)
              {
                  break;
              }
              status = (postedStatus & BENCHMARK_TIMER_DM_TWPS_W_PEND_TCRR);
           } while (status != (uint32_t) 0u);

           cslRet = TIMERReloadSet(baseAddr, countVal);
           do {
              cslRet = TIMERWritePostedStatusGet2(baseAddr, &postedStatus);
              if (cslRet != CSL_PASS)
              {
                  break;
              }
              status = (postedStatus & BENCHMARK_TIMER_DM_TWPS_W_PEND_TLDR);
           } while (status != (uint32_t) 0u);
        }
    }

    if (cslRet == CSL_PASS)
    {
        cslRet = TIMEREnable(baseAddr);
    }
    Intc_SystemRestore(key);

    if (cslRet == CSL_PASS)
    {
        return CSL_PASS;
    }
    else
    {
        return CSL_EFAIL;
    }
}

/* Timer tick function -- benchmark interrupt */
void benchmarkTimerISR(void)
{
   uint32_t latency, status;
   uint32_t intNum;
   uint32_t baseAddr;

   /* compute the timer interrupt latency */
#if defined(SOC_AM65XX)
   curVal = TIMERCounterGet(CSL_MCU_TIMER2_CFG_BASE);
   reloadVal = TIMERReloadGet(CSL_MCU_TIMER2_CFG_BASE);
#else
   curVal = HW_RD_REG32(gMcuTimerCfg[cpuInfo.grpId*2 + cpuInfo.cpuID].timerBaseAddr + TIMER_TCRR);
   reloadVal = HW_RD_REG32(gMcuTimerCfg[cpuInfo.grpId*2 + cpuInfo.cpuID].timerBaseAddr + TIMER_TLDR);
#endif
   latency = (curVal-reloadVal)*BENCHMARK_TIMER_TICK_PRD;
   if (latency>gTimerIntStat.intLatencyMax)
   {
      /* timer interrupt latency in ns */ 
      gTimerIntStat.intLatencyMax = latency;
   }
   /* compute average timer interrupt latency */
   gTimerIntStat.intLatencyAve = (gTimerIntStat.intLatencyAve*gTimerIntStat.isrCnt+latency)/(gTimerIntStat.isrCnt+1);

   baseAddr = (uint32_t) (gMcuTimerCfg[cpuInfo.grpId*2 + cpuInfo.cpuID].timerBaseAddr);
   /* clear and acknowledge the timer interrupt */
   status = CSL_vimGetActivePendingIntr( (CSL_vimRegs *)(uintptr_t)CSL_MAIN_DOMAIN_VIM_BASE_ADDR, 
        BENCHMARK_TIMER_INT_MAP, (uint32_t *)&intNum, (uint32_t *)0 );
   if (status == CSL_PASS)
   {
      /* Disable the Timer interrupts */
      TIMERIntDisable(baseAddr, TIMER_INT_OVF_EN_FLAG);

      /* acknowledge the Timer interrupt */
      TIMERIntStatusClear(baseAddr, TIMER_IRQSTATUS_OVF_IT_FLAG_MASK);

      /* Enable the Timer interrupts */
      TIMERIntEnable(baseAddr, TIMER_INT_OVF_EN_FLAG);

      /* Clear interrupt after executing ISR code */
      CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)CSL_MAIN_DOMAIN_VIM_BASE_ADDR, intNum );

      /* Acknowledge interrupt servicing */
      CSL_vimAckIntr( (CSL_vimRegs *)(uintptr_t)CSL_MAIN_DOMAIN_VIM_BASE_ADDR, BENCHMARK_TIMER_INT_MAP );     
   }

   /* increase the interrupt counter */
   gTimerIntStat.isrCnt++;
}

/* Timer initialization */
void benchmarkTimerInit(uint32_t coreId)
{
   int32_t     status;

   if (coreId>NUM_MCU_CORE)
      return;

   /*
     Set up timer -- benchmark timer interrupt
   */
   gTimerIntStat.isrCnt = 0;
   gTimerIntStat.intLatencyMax = 0;

   /* Initialize MCU INTC */
   status = McuIntc_Init();
   if (status != CSL_PASS) {
      MCBENCH_log("\nError: McuIntc_Init() fail.\n");
      return;
   }

   /* Init the Timer */
   status = csldmTimer_initObj(coreId);
   if (status == CSL_PASS)
   {
      status = csldmTimer_setMode(coreId, TIMER_AUTORLD_NOCMP_ENABLE);
   }

   /* Enable R5 interrupts for events from Timer2 */
   McuIntc_enableIntr(MCU_INTR_IDX(0), true);

   /* Enable R5 IRQ */
   CSL_armR5IntrEnableIrq(1);

   return;
}

/* set the timer interrupt to designated frequency */
int32_t benchmarkTimerSetFreq(uint32_t coreId, Run_Freq_Sel sel)
{
   int32_t timerCount = 0;
   int32_t status = 0;

   switch (sel)
   {
      case RUN_FREQ_SEL_1K:
      timerCount = 25000000/RUN_FREQ_1K;
      break;
      
      case RUN_FREQ_SEL_2K:
      timerCount = 25000000/RUN_FREQ_2K;
      break;
      
      case RUN_FREQ_SEL_4K:
      timerCount = 25000000/RUN_FREQ_4K;
      break;

      case RUN_FREQ_SEL_8K:
      timerCount = 25000000/RUN_FREQ_8K;
      break;

      case RUN_FREQ_SEL_16K:
      timerCount = 25000000/RUN_FREQ_16K;
      break;

      case RUN_FREQ_SEL_32K:
      timerCount = 25000000/RUN_FREQ_32K;
      break;

      case RUN_FREQ_SEL_50K:
      timerCount = 25000000/RUN_FREQ_50K;
      break;

      default:
      timerCount = 25000000/RUN_FREQ_1K;
      break;
   }

   status = csldmTimer_setCountVal(coreId, timerCount);
   if (status != CSL_PASS)
   {
      MCBENCH_log("APP: csldmTimer_setCountVal failed !!!\n");
      return status;
   }
   else
      MCBENCH_log("APP: csldmTimer_setCountVal OK !!!\n");

  return CSL_PASS;
}
