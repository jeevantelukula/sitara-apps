/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <stdbool.h>

/* TI CSL Includes */
#include <ti/csl/soc.h>
#include <ti/csl/soc/cslr_soc_ctrl_mmr.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_intaggr.h>

#include "app.h"
#include "ratcfg.h"

/* global variables */
volatile uint32_t reset_received, pru_ack_received, fsoe_data_available;
uint32_t interrupts_icssg = 0;

/* Initialize Interrupt Aggregator config structure */
CSL_IntaggrCfg iaRegs =
{
    .pCfgRegs       = (CSL_intaggr_cfgRegs *) (CSL_DMASS0_INTAGGR_CFG_BASE + MCU_RAT_OFFSET2),
    .pImapRegs      = (CSL_intaggr_imapRegs *) (CSL_DMASS0_INTAGGR_IMAP_BASE + MCU_RAT_OFFSET2),
    .pIntrRegs      = (CSL_intaggr_intrRegs *) (CSL_DMASS0_INTAGGR_INTR_BASE + MCU_RAT_OFFSET2),
    .pL2gRegs       = (CSL_intaggr_l2gRegs *) (CSL_DMASS0_INTAGGR_L2G_BASE + MCU_RAT_OFFSET2),
    .pMcastRegs     = (CSL_intaggr_mcastRegs *) (CSL_DMASS0_INTAGGR_MCAST_BASE + MCU_RAT_OFFSET2),
    .pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) (CSL_DMASS0_INTAGGR_GCNTCFG_BASE + MCU_RAT_OFFSET2),
    .pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) (CSL_DMASS0_INTAGGR_GCNTRTI_BASE + MCU_RAT_OFFSET2),
    .srcEventCnt    = 1536U,
    .virtIntrCnt    = 184U,
    .localEventCnt  = 32U,
    .globalEventCnt = 256U,
    .mcastEventCnt  = 128U
};

/* MPU region 5 must be added to accomodate the DMA register address translation */
/* All other MPU regions are the same as default from ti/csl/arch/m4/src/startup/startup.c */
const CSL_ARMM4MpuRegionCfg_t gCslM4MpuCfg[] =
{
    {
        /* Region 0 configuration: IRAM+DRAM */
        .regionId         = 0U,
        .enable           = 1U,
        .baseAddr         = 0x0,
        .size             = CSL_ARM_M4_MPU_REGION_SIZE_256KB,
        .subRegionEnable  = CSL_ARM_M4_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_M4_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)FALSE,
        .cachePolicy      = CSL_ARM_M4_CACHE_POLICY_NON_CACHEABLE,
        .memAttr          = 0U
    },
    {
        /* Region 1 configuration: 8k RAT Cfg Regs */
        .regionId         = 1U,
        .enable           = 1U,
        .baseAddr         = 0x44200000,
        .size             = CSL_ARM_M4_MPU_REGION_SIZE_8KB,
        .subRegionEnable  = CSL_ARM_M4_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 1U,
        .accessPermission = CSL_ARM_M4_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)FALSE,
        .cachePolicy      = 0U,
        .memAttr          = 0U
    },
    {
        /* Region 2 configuration: 256 MB IP */
        .regionId         = 2U,
        .enable           = 1U,
        .baseAddr         = 0x60000000,
        .size             = CSL_ARM_M4_MPU_REGION_SIZE_256MB,
        .subRegionEnable  = CSL_ARM_M4_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 1U,
        .accessPermission = CSL_ARM_M4_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)FALSE,
        .cachePolicy      = 0U,
        .memAttr          = 0U
    },
    {
        /* Region 3 configuration: 2 MB of banked OCMSRAM (Banks 0-7) */
        .regionId         = 3U,
        .enable           = 1U,
        .baseAddr         = 0x70000000,
        .size             = CSL_ARM_M4_MPU_REGION_SIZE_2MB,
        .subRegionEnable  = CSL_ARM_M4_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_M4_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_M4_CACHE_POLICY_WB_WA,
        .memAttr          = 0U,
    },
    {
        /* Region 4 configuration: 1 GB DDR RAM */
        .regionId         = 4U,
        .enable           = 1U,
        .baseAddr         = 0x80000000,
        .size             = CSL_ARM_M4_MPU_REGION_SIZE_1GB,
        .subRegionEnable  = CSL_ARM_M4_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_M4_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_M4_CACHE_POLICY_WB_WA,
        .memAttr          = 0U,
    },
    {
        /* Region 5 configuration: 256 MB IP */
        .regionId         = 5U,
        .enable           = 1U,
        .baseAddr         = 0xA8000000,
        .size             = CSL_ARM_M4_MPU_REGION_SIZE_128MB,
        .subRegionEnable  = CSL_ARM_M4_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 1U,
        .accessPermission = CSL_ARM_M4_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)FALSE,
        .cachePolicy      = 0U,
        .memAttr          = 0U
    }
};

void timesync_isr()
{
    uint32_t vintrBitNum;

    vintrBitNum = ((TS_INTAGGR_EVT_NUM << 6) & 0x3FC0) | (0 & 0x3F);
    CSL_intaggrClrIntr(&iaRegs, vintrBitNum);

    interrupts_icssg += 1;

    Intc_IntClrPend(CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_168);
}

void configure_nvic()
{
    Intc_Init();

    Intc_IntClrPend(CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_168);
    
    Intc_IntRegister(CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_168, (IntrFuncPtr) &timesync_isr, NULL);

    Intc_SystemEnable(CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_168);

    Intc_IntEnable(0);
}

void application_loop()
{
    bool looping = true;
    uint32_t loopCounter = 0;
    uint32_t heartbeatState = GPIO_PIN_LOW;

    /* Dummy/test loop */
    while(looping) {
        loopCounter = 10000;    /* adjust so heartbeat LED toggles 1Hz */

        while(loopCounter-- > 0) {
        }

        /* toggle LED via GPIO pin every time loopCounter hits 0 */
        heartbeatState = heartbeatState == 0 ? GPIO_PIN_HIGH : GPIO_PIN_LOW;

    }
}

