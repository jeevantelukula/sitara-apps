/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 */

/**
 *  \file udma_sw_trigger_ccs_test.c
 *
 *  \brief UDMA SW trigger sample application performs 2D transfer using
 *  SW trigger method as below
 *
 *  App_init(): init UDMA and channel parameters then fill srcBuffer with data
 *  App_create(): open UDMA channel, register event, then enable channel
 *
 *
 *  Each run transfers (UDMA_TEST_1D_SIZE x UDMA_TEST_2D_SIZE) bytes of data
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/udma/udma.h>
#include <ti/drv/udma/examples/udma_apputils/udma_apputils.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * Application test config parameters
 */
/** \brief 1D Size */
#define UDMA_TEST_1D_SIZE                   (1024U)     // DMA transfers 1D byte chunks
/** \brief 2D Size */
#define UDMA_TEST_2D_SIZE                   (1024U)     // DMA transfers above 2D times

/*
 * Buffer parameters
 */
/** \brief Number of bytes to copy and buffer allocation */
#define UDMA_TEST_APP_NUM_BYTES             (UDMA_TEST_1D_SIZE * UDMA_TEST_2D_SIZE)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_BUF_SIZE_ALIGN        ((UDMA_TEST_APP_NUM_BYTES + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))
/** \brief Number of bytes to alloc is IRAM */

/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much memcpy operations */
#define UDMA_TEST_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_TEST_APP_RING_MEM_SIZE     (UDMA_TEST_APP_RING_ENTRIES * UDMA_TEST_APP_RING_ENTRY_SIZE)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_RING_MEM_SIZE_ALIGN ((UDMA_TEST_APP_RING_MEM_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/*
 * TRPD parameters
 */
#define UDMA_TEST_APP_TRPD_SIZE         ((sizeof(CSL_UdmapTR15) * 2U) + 4U)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_TRPD_SIZE_ALIGN   ((UDMA_TEST_APP_TRPD_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/** \brief Get GTC Timer Ticks */
#define App_getGTCTimerTicks()          CSL_REG64_RD(CSL_GTC0_GTC_CFG1_BASE + 0x8U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    struct Udma_ChObj       chObj;
    struct Udma_EventObj    trEventObj;

    Udma_ChHandle           chHandle;
    Udma_EventHandle        trEventHandle;
    Udma_EventPrms          trEventPrms;

    uint32_t                trigger;
    /**< Global0 or Global 1 Trigger - refer \ref CSL_UdmapTrFlagsTrigger. */
    uint32_t                eventSize;
    /**< Refer \ref CSL_UdmapTrFlagsEventSize. */
    uint32_t                triggerType;
    /**< Refer \ref CSL_UdmapTrFlagsTriggerType. */
    uint32_t                eolType;
    /**< Refer \ref CSL_UdmapTrFlagsEol. */
    volatile uint32_t      *pSwTriggerReg;
    uint32_t                triggerMask;

    Udma_DrvHandle          drvHandle;
    SemaphoreP_Handle       transferDoneSem;

    uint8_t                 *txRingMem;
    uint8_t                 *trpdMem;

    uint8_t                 *srcBuf;
    uint8_t                 *destBuf;
} App_UdmaChObj;

typedef struct
{
    struct Udma_DrvObj      drvObj;
    App_UdmaChObj           appChObj;
} App_UdmaObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t App_swTriggerTest(App_UdmaObj *appObj);
static int32_t App_udmaSwTrigger(App_UdmaObj *appObj);

static int32_t App_init(App_UdmaObj *appObj);
static int32_t App_deinit(App_UdmaObj *appObj);

static int32_t App_create(App_UdmaObj *appObj);
static int32_t App_delete(App_UdmaObj *appObj);

static void App_udmaTrpdInit(App_UdmaChObj *appChObj);

static void App_print(const char *str);
static void App_printNum(const char *str, uint32_t num);

/* No.of ticks taken to do a GTC Reg Read operation */
volatile uint64_t gTickDelay = 0;
volatile uint64_t gClockGTC = 0;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * UDMA driver and channel objects
 */
App_UdmaObj gUdmaAppObj;

/*
 * UDMA Memories
 */
static uint8_t gTxRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTrpdMem[UDMA_TEST_APP_TRPD_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/*
 * Application Buffers
 * User linker cmd file to place buffers in any memory mapped address
 */
static uint8_t gUdmaTestSrcBuf[UDMA_TEST_APP_BUF_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT), section(".src_buffer")));
static uint8_t gUdmaTestDestBuf[UDMA_TEST_APP_BUF_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT), section(".dest_buffer")));

/* Global test pass/fail flag */
static volatile int32_t gUdmaAppResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * UDMA SW Trigger test
 */
int32_t Udma_swTriggerTest(void)
{
    int32_t         retVal;
    App_UdmaObj    *appObj = &gUdmaAppObj;

    retVal = App_init(appObj);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App init failed!!\n");
    }

    App_print("UDMA SW trigger application started...\n");

    if(UDMA_SOK == retVal)
    {
        retVal = App_create(appObj);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App create failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = App_swTriggerTest(appObj);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App SW trigger test failed!!\n");
        }
    }

    retVal += App_delete(appObj);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App delete failed!!\n");
    }

    retVal += App_deinit(appObj);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App deinit failed!!\n");
    }

    if((UDMA_SOK == retVal) && (UDMA_SOK == gUdmaAppResult))
    {
        App_print("UDMA SW trigger Passed!!\n");
        App_print("All tests have passed!!\n");
    }
    else
    {
        App_print("UDMA SW trigger Failed!!\n");
        App_print("Some tests have failed!!\n");
    }

    return (0);
}

static int32_t App_swTriggerTest(App_UdmaObj *appObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        i;
    uint8_t        *destBuf;

    /* Reset dest buffers */
    destBuf  = &gUdmaTestDestBuf[0U];
    for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
    {
        destBuf[i] = 0U;
    }
    /* Writeback destination buffer */
    Udma_appUtilsCacheWb(destBuf, UDMA_TEST_APP_NUM_BYTES);

    /* Perform UDMA memcpy */
    retVal = App_udmaSwTrigger(appObj);

    return (retVal);
}

static int32_t App_udmaSwTrigger(App_UdmaObj *appObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t       *pTrResp, trRespStatus, i;
    uint8_t        *trpdMem;
    uint64_t        pDesc = 0;
    App_UdmaChObj  *appChObj;
    Udma_ChHandle   chHandle;
    uint8_t        *srcBuf, *destBuf;
    uint64_t        ticksGTC;

    appChObj = &appObj->appChObj;
	chHandle = appChObj->chHandle;
    trpdMem  = appChObj->trpdMem;

    /* Get SW trigger register for easy access */
    appChObj->triggerMask = ((uint32_t)1U << (appChObj->trigger - 1U));
    appChObj->pSwTriggerReg = (volatile uint32_t *) Udma_chGetSwTriggerRegister(chHandle);
    if(NULL == appChObj->pSwTriggerReg)
    {
        App_print("[Error] Channel trigger register get failed!!\n");
    }

    /* Submit TRPD to channels */
    App_udmaTrpdInit(appChObj);
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(chHandle),
                 (uint64_t) Udma_appVirtToPhyFxn(trpdMem, 0U, NULL));
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] Channel queue failed!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        /* Set channel trigger and wait for completion */
        ticksGTC = App_getGTCTimerTicks();
        CSL_REG64_WR(appChObj->pSwTriggerReg, appChObj->triggerMask);
        while(1U)
        {
            volatile uint64_t intrStatusReg;
            intrStatusReg = CSL_REG64_RD(appChObj->trEventPrms.intrStatusReg);
            if(intrStatusReg & appChObj->trEventPrms.intrMask)
            {
                /* Clear interrupt */
                CSL_REG64_WR(appChObj->trEventPrms.intrClearReg, appChObj->trEventPrms.intrMask);
                break;
            }
        }

        ticksGTC = App_getGTCTimerTicks() - ticksGTC - gTickDelay;
        App_printNum("  Transferred %d", UDMA_TEST_APP_NUM_BYTES);
        App_printNum(" bytes in %d ns\n", (uint32_t) ((ticksGTC * 1000000000U) / gClockGTC));
    }

    if(UDMA_SOK == retVal)
    {
        /* wait for response to be received in completion queue */
        while(1)
        {
            retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
            if(UDMA_SOK == retVal)
            {
                break;
            }
        }

        /*
         * Sanity check
         */
        /* Check returned descriptor pointer */
        if(((uint64_t) Udma_appPhyToVirtFxn(pDesc, 0U, NULL)) != ((uint64_t) trpdMem))
        {
            App_print("[Error] TR descriptor pointer returned doesn't "
                   "match the submitted address!!\n");
            retVal = UDMA_EFAIL;
        }

        if(UDMA_SOK == retVal)
        {
            /* Invalidate cache */
            Udma_appUtilsCacheInv(trpdMem, UDMA_TEST_APP_TRPD_SIZE_ALIGN);

            /* check TR response status */
            pTrResp = (uint32_t *) (trpdMem + (sizeof(CSL_UdmapTR15) * 2U));
            trRespStatus = CSL_FEXT(*pTrResp, UDMAP_TR_RESPONSE_STATUS_TYPE);
            if(trRespStatus != CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE)
            {
                App_print("[Error] TR Response not completed!!\n");
                retVal = UDMA_EFAIL;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Compare data */
        /* Invalidate destination buffer */
        srcBuf  = &gUdmaTestSrcBuf[0U];
        destBuf  = &gUdmaTestDestBuf[0U];
        Udma_appUtilsCacheInv(destBuf, UDMA_TEST_APP_NUM_BYTES);
        for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
        {
            if(srcBuf[i] != destBuf[i])
            {
                App_print("[Error] Data mismatch!!\n");
                retVal = UDMA_EFAIL;
            }
        }
    }

    return (retVal);
}

static int32_t App_init(App_UdmaObj *appObj)
{
    int32_t         retVal;
    Udma_InitPrms   initPrms;
    uint32_t        instId;
    App_UdmaChObj  *appChObj;
    Udma_DrvHandle  drvHandle = &appObj->drvObj;
    uint32_t        i;
    uint8_t        *srcBuf;

#if defined (SOC_AM64X)
    /* Use Block Copy DMA for AM64x */
    instId = UDMA_INST_ID_BCDMA_0;
#else
#error
#endif
    /* UDMA driver init */
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.virtToPhyFxn   = &Udma_appVirtToPhyFxn;
    initPrms.phyToVirtFxn   = &Udma_appPhyToVirtFxn;
    initPrms.printFxn       = &App_print;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA init failed!!\n");
    }

    /* Init channel parameters */
    appChObj                    = &appObj->appChObj;
    appChObj->chHandle          = &appChObj->chObj;
    appChObj->drvHandle         = drvHandle;
    appChObj->transferDoneSem   = NULL;
    appChObj->trEventHandle     = NULL;
    appChObj->trigger           = CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0;
    appChObj->eventSize         = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC;
    appChObj->triggerType       = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC;
    appChObj->eolType           = CSL_UDMAP_TR_FLAGS_EOL_ICNT0_ICNT1;
    appChObj->txRingMem         = &gTxRingMem[0U];
    appChObj->trpdMem           = &gUdmaTrpdMem[0U];
    appChObj->srcBuf            = &gUdmaTestSrcBuf[0U];
    appChObj->destBuf           = &gUdmaTestDestBuf[0U];

	srcBuf = &gUdmaTestSrcBuf[0U];
    for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
    {
        srcBuf[i] = i;
    }
    /* Writeback source destination buffer */
    Udma_appUtilsCacheWb(srcBuf, UDMA_TEST_APP_NUM_BYTES);
    
    return (retVal);
}

static int32_t App_deinit(App_UdmaObj *appObj)
{
    int32_t         retVal;
    Udma_DrvHandle  drvHandle = &appObj->drvObj;

    retVal = Udma_deinit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA deinit failed!!\n");
    }

    return (retVal);
}
static int32_t App_create(App_UdmaObj *appObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    App_UdmaChObj      *appChObj;
    Udma_ChHandle       chHandle;
    Udma_DrvHandle      drvHandle = &appObj->drvObj;

	appChObj = &appObj->appChObj;
	chHandle = appChObj->chHandle;

	if(UDMA_SOK == retVal)
	{
		/* Init channel parameters */
		chType = UDMA_CH_TYPE_TR_BLK_COPY;
		UdmaChPrms_init(&chPrms, chType);
		chPrms.fqRingPrms.ringMem       = appChObj->txRingMem;
		chPrms.fqRingPrms.ringMemSize   = UDMA_TEST_APP_RING_MEM_SIZE;
		chPrms.fqRingPrms.elemCnt       = UDMA_TEST_APP_RING_ENTRIES;

		/* Open channel for block copy */
		retVal = Udma_chOpen(drvHandle, chHandle, chType, &chPrms);
		if(UDMA_SOK != retVal)
		{
			App_print("[Error] UDMA channel open failed!!\n");
		}
	}

	if(UDMA_SOK == retVal)
	{
		/* Config TX channel */
		UdmaChTxPrms_init(&txPrms, chType);
		retVal = Udma_chConfigTx(chHandle, &txPrms);
		if(UDMA_SOK != retVal)
		{
			App_print("[Error] UDMA TX channel config failed!!\n");
		}
	}

	if(UDMA_SOK == retVal)
	{
		/* Config RX channel - which is implicitly paired to TX channel in
		 * block copy mode */
		UdmaChRxPrms_init(&rxPrms, chType);
		retVal = Udma_chConfigRx(chHandle, &rxPrms);
		if(UDMA_SOK != retVal)
		{
			App_print("[Error] UDMA RX channel config failed!!\n");
		}
	}

	if(UDMA_SOK == retVal)
	{
		/* Register TR event */
		eventHandle = &appChObj->trEventObj;
		UdmaEventPrms_init(&appChObj->trEventPrms);
		appChObj->trEventPrms.eventType         = UDMA_EVENT_TYPE_TR;
		appChObj->trEventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
		appChObj->trEventPrms.chHandle          = chHandle;
		appChObj->trEventPrms.masterEventHandle = NULL;
		appChObj->trEventPrms.eventCb           = NULL;
		appChObj->trEventPrms.appData           = appChObj;
		retVal = Udma_eventRegister(drvHandle, eventHandle, &appChObj->trEventPrms);
		if(UDMA_SOK != retVal)
		{
			App_print("[Error] UDMA TR event register failed!!\n");
		}
		else
		{
			appChObj->trEventHandle = eventHandle;
		}
	}

	if(UDMA_SOK == retVal)
	{
		/* Channel enable */
		retVal = Udma_chEnable(chHandle);
		if(UDMA_SOK != retVal)
		{
			App_print("[Error] UDMA channel enable failed!!\n");
		}
	}

    return (retVal);
}

static int32_t App_delete(App_UdmaObj *appObj)
{
    int32_t         retVal, tempRetVal;
    uint64_t        pDesc;
    App_UdmaChObj  *appChObj;
    Udma_ChHandle   chHandle;

	appChObj = &appObj->appChObj;
	chHandle = appChObj->chHandle;

	retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
	if(UDMA_SOK != retVal)
	{
		App_print("[Error] UDMA channel disable failed!!\n");
	}

	/* Flush any pending request from the free queue */
	while(1)
	{
		tempRetVal = Udma_ringFlushRaw(
						 Udma_chGetFqRingHandle(chHandle), &pDesc);
		if(UDMA_ETIMEOUT == tempRetVal)
		{
			break;
		}
	}

	/* Unregister all events */
	if(NULL != appChObj->trEventHandle)
	{
		retVal += Udma_eventUnRegister(appChObj->trEventHandle);
		if(UDMA_SOK != retVal)
		{
			App_print("[Error] UDMA event unregister failed!!\n");
		}
		appChObj->trEventHandle = NULL;
	}

	retVal += Udma_chClose(chHandle);
	if(UDMA_SOK != retVal)
	{
		App_print("[Error] UDMA channel close failed!!\n");
	}

    return (retVal);
}

static void App_udmaTrpdInit(App_UdmaChObj *appChObj)
{
    CSL_UdmapCppi5TRPD *pTrpd = (CSL_UdmapCppi5TRPD *) appChObj->trpdMem;
    CSL_UdmapTR15 *pTr = (CSL_UdmapTR15 *)(appChObj->trpdMem + sizeof(CSL_UdmapTR15));
    uint32_t *pTrResp = (uint32_t *) (appChObj->trpdMem + (sizeof(CSL_UdmapTR15) * 2U));
    uint32_t cqRingNum = Udma_chGetCqRingNum(appChObj->chHandle);

    /* Make TRPD */
    UdmaUtils_makeTrpd(pTrpd, UDMA_TR_TYPE_15, 1U, cqRingNum);

    /* Setup TR */
    pTr->flags  = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOL, appChObj->eolType);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, appChObj->eventSize);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, appChObj->trigger);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, appChObj->triggerType);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    pTr->icnt0    = UDMA_TEST_1D_SIZE;
    pTr->icnt1    = UDMA_TEST_2D_SIZE;
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->addr     = (uint64_t) Udma_appVirtToPhyFxn(appChObj->srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;        /* Linear addressing, 1 byte per elem.
                                           Replace with CSL-FL API */
    pTr->dicnt0   = UDMA_TEST_1D_SIZE;
    pTr->dicnt1   = UDMA_TEST_2D_SIZE;
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->daddr    = (uint64_t) Udma_appVirtToPhyFxn(appChObj->destBuf, 0U, NULL);

    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = pTr->icnt0 * pTr->icnt1;
    pTr->dim3     = pTr->icnt0 * pTr->icnt1 * pTr->icnt2;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = pTr->dicnt0 * pTr->dicnt1;
    pTr->ddim3    = pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2;

    /* Clear TR response memory */
    *pTrResp = 0xFFFFFFFFU;

    /* Writeback cache */
    Udma_appUtilsCacheWb(appChObj->trpdMem, UDMA_TEST_APP_TRPD_SIZE_ALIGN);

    return;
}

static void App_print(const char *str)
{
    UART_printf("%s", str);
    if(TRUE == Udma_appIsPrintSupported())
    {
        printf("%s", str);
    }

    return;
}

static void App_printNum(const char *str, uint32_t num)
{
    static char printBuf[200U];

    snprintf(printBuf, 200U, str, num);
    UART_printf("%s", printBuf);

    if(TRUE == Udma_appIsPrintSupported())
    {
        printf("%s", printBuf);
    }

    return;
}

int32_t App_getGTCClk(uint32_t moduleId,
                      uint32_t clkId)
{
    int32_t retVal;
    uint64_t retClk;

    retVal = Sciclient_pmGetModuleClkFreq(moduleId,
                                          clkId,
                                          &retClk,
                                          SCICLIENT_SERVICE_WAIT_FOREVER);

    gClockGTC = retClk;
    App_printNum("GTC Clk running at %d Hz. \n", (uint32_t) gClockGTC);

    /* Enable GTC */
    CSL_REG64_WR(CSL_GTC0_GTC_CFG1_BASE + 0x0U, 0x1);

    /* Measure and store the time spent to do a getTime operation */
    gTickDelay = App_getGTCTimerTicks();
    gTickDelay = App_getGTCTimerTicks() - gTickDelay;
    App_printNum("Time taken to read GTC Timer ticks = %d ns ",
                 (uint32_t)((gTickDelay*1000000000U)/gClockGTC));
    App_printNum("(%d ticks) \n", (uint32_t)gTickDelay);

    return (retVal);
}
