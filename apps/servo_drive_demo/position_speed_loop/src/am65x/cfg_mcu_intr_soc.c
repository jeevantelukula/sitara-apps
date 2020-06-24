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

#include <string.h>
#include <cfg_mcu_intr_soc.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/csl_intr_router.h>
#include <ti/drv/sciclient/sciclient.h>

/* Unitialized values */
#define UINIT_MPU_CPU_ID                ( 0xFFFFFFFF )  /* MPU CPU ID */
#define UINIT_VIM_REGS_BASE_ADDR        ( 0x0 )         /* VIM base address register */

/* MCU interrupt configuration structure */
typedef struct McuIntrCfg_s {
    McuIntrRegPrms mcuIntrRegPrms;  /* interrupt configuration */
    McuIntrRtrPrms mcuIntrRtrPrms;  /* interrupt router configuration */
    bool intrRtrIntrCfgValid;       /* flag indicates whether router configuration valid */
} McuIntrCfg;

/* Configure MAIN2MCU interrupt router */
static int32_t cfgMain2McuIntrRouter(
    int32_t intrNum,                /* VIM interrupt number */
    McuIntrRtrPrms *pMcuIntrRtrPrms /* interrupt router configuration */
);

/* VIM registers base address */
uint32_t gVimRegsBaseAddr = UINIT_VIM_REGS_BASE_ADDR;
/* MCU CPU/core ID within cluster */
static volatile uint32_t gMpuCpuID = UINIT_MPU_CPU_ID;
/* MCU interrupt configurations */
static McuIntrCfg gMcuIntrCfg[NUM_MCU_INTR] = {
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, {0, 0}},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, {0, 0}},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, {0, 0}},
    {{0, CSL_VIM_INTR_TYPE_LEVEL, CSL_VIM_INTR_MAP_IRQ, 15, NULL}, {0, 0}}
};

/* Initialize MCU INTC */
int32_t McuIntc_Init(void)
{
    CSL_ArmR5CPUInfo info;

    CSL_armR5GetCpuID(&info);
    if (info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        /* MCU SS Pulsar R5 SS */
        
        /* Initialize CPU/core ID within cluster */
        gMpuCpuID = info.cpuID;
        
        /* Initialize VIM base address */
        gVimRegsBaseAddr = (info.cpuID == CSL_ARM_R5_CPU_ID_0)?
            CSL_MCU_DOMAIN_VIM_BASE_ADDR0:
            CSL_MCU_DOMAIN_VIM_BASE_ADDR1;
        
        return CFG_MCU_INTR_SOK;
    }
    else
    {
        /* Invalid Pulsar R5 SS */
        
        gMpuCpuID = UINIT_MPU_CPU_ID;
        gVimRegsBaseAddr = UINIT_VIM_REGS_BASE_ADDR;
        return CFG_MCU_INTR_SERR_INV_GROUP_ID;
    }
}

/* Configure MCU interrupt */
int32_t McuIntc_cfgIntr(
    McuIntrRegPrms *pMcuIntrRegPrms,    /* MCU interrupt registration parameters */
    McuIntrRtrPrms *pMcuIntrRtrPrms,    /* MCU interrupt MAIN2MCU interrupt router parameters */
    uint8_t mcuIntrIdx                  /* MCU interrupt index */
)
{
    McuIntrCfg *pMcuIntrCfg;
    int32_t status;   
    
    if (mcuIntrIdx > NUM_MCU_INTR)
    {
        return CFG_MCU_INTR_SERR_INV_ID;
    }
    
    pMcuIntrCfg = &gMcuIntrCfg[mcuIntrIdx]; /* get MCU interrupt configuration */
    
    /* Copy MCU interrupt parameters to MCU interrupt configuration */
    pMcuIntrCfg->mcuIntrRegPrms = *pMcuIntrRegPrms;
    pMcuIntrCfg->mcuIntrRtrPrms = *pMcuIntrRtrPrms;
    pMcuIntrCfg->intrRtrIntrCfgValid = true;
    
    /* Configure Interrupt Router */
    status = cfgMain2McuIntrRouter(pMcuIntrRegPrms->intrNum, pMcuIntrRtrPrms);
    if (status != CFG_MCU_INTR_SOK) {
        return status;
    }
    
    /* Register VIM Interrupt */    
    status = CSL_vimCfgIntr( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr,
        pMcuIntrRegPrms->intrNum, 
        pMcuIntrRegPrms->intrPri, 
        pMcuIntrRegPrms->intrMap, 
        pMcuIntrRegPrms->intrType, 
        (uint32_t)pMcuIntrRegPrms->isrRoutine );    
    if (status != CSL_PASS) {
        return CFG_MCU_INTR_SERR_REG_INTR;
    }
    
    return CFG_MCU_INTR_SOK;
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
        return CFG_MCU_INTR_SERR_INV_ID;
    }
    
    pMcuIntrCfg = &gMcuIntrCfg[mcuIntrIdx]; /* get pointer to config structure */
    
    /* Enable the interrupt */
    CSL_vimSetIntrEnable( (CSL_vimRegs *)(uintptr_t)gVimRegsBaseAddr, 
        pMcuIntrCfg->mcuIntrRegPrms.intrNum, bEnable );
    
    return CFG_MCU_INTR_SOK;
}

/* Configure MAIN2MCU interrupt router */
static int32_t cfgMain2McuIntrRouter(
    int32_t intrNum,                /* VIM interrupt number */
    McuIntrRtrPrms *pMcuIntrRtrPrms /* interrupt router configuration */
)
{    
    struct tisci_msg_rm_irq_set_req rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    int32_t status;

    memset(&rmIrqReq,0,sizeof(rmIrqReq));
    rmIrqReq.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
    rmIrqReq.src_id = pMcuIntrRtrPrms->tisciSrcId;
    rmIrqReq.src_index = pMcuIntrRtrPrms->tisciSrcIndex;
    rmIrqReq.dst_id = (gMpuCpuID == CSL_ARM_R5_CPU_ID_0) ? 
        TISCI_DEV_MCU_ARMSS0_CPU0 : TISCI_DEV_MCU_ARMSS0_CPU1;
    rmIrqReq.dst_host_irq = (uint16_t)intrNum;
    rmIrqReq.valid_params = TISCI_MSG_VALUE_RM_DST_ID_VALID | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    status = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        return CFG_MCU_INTR_SERR_REG_INTR;
    }

    return CFG_MCU_INTR_SOK;
}
