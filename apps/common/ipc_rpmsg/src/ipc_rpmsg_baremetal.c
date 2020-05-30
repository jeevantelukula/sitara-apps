/*
 *  Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *  All rights reserved.
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
 *  \file ipc_rpmsg_baremetal.c
 *
 *  \brief 2-core (Linux-to-Baremetal) IPC library performing
 *  basic echo communication using the baremetal IPC RPMsg_char driver
 *  The application after initialization of the IPC lld, waits for messages
 *  from the Linux host core and echos each message received, back to the 
 *  source.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <ti/osal/HwiP.h>
#include <ti/osal/osal.h>
/* SCI Client */
#include <ti/drv/sciclient/sciclient.h>

#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/ipcver.h>
#include <ipc_setup.h>
#include "ipc_trace.h"
#include "ipcapp_baremetal.h"
#ifndef BUILD_MPU1_0
#if defined(SOC_AM65XX)
#include "ipc_am65xx_rsctable.h"
#else
#include "ipc_rsctable.h"
#endif
#endif

#include "benchmark_stat.h"

/* Size of message */
#define MSGSIZE  256U
/* Service name to be registered for the end point */
#define SERVICE  "ti.ipc4.ping-pong"
/* End point number to be used for IPC communication */
#define ENDPT1   13U
#define NUMMSGS  10000 /* number of message sent by the sender function */
                       /* Note: The sender function is not active in this example */

uint32_t rpmsgDataSize = RPMSG_DATA_SIZE;

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define TEST_TIMEOUT_FOREVER (~(0U))

/* Number of cores used in the test */
#define CORE_IN_TEST            2

/* Define System_printf to use Ipc_Trace_printf */
#define System_printf Ipc_Trace_printf


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint8_t  gCntrlBuf[RPMSG_DATA_SIZE] __attribute__ ((section("ipc_data_buffer"), aligned (8)));
uint8_t  gSysVqBuf[VQ_BUF_SIZE]  __attribute__ ((section ("ipc_data_buffer"), aligned (8)));
uint8_t  gRspBuf[RPMSG_DATA_SIZE]  __attribute__ ((section ("ipc_data_buffer"), aligned (8)));

volatile uint32_t gMessagesReceived = 0;

extern core_stat gCoreStat;

#ifdef BUILD_MPU1_0
#define NUM_CHRDEV_SERVICES (1)
#define CHRDEV_PROC_NAME    "a53-mpu-core-0"
uint32_t selfProcId = IPC_MPU1_0;
uint32_t remoteProc[] =
{
    IPC_MCU1_0,
    IPC_MCU1_1,
};
#elif defined(BUILD_MCU1_0)
uint32_t selfProcId = IPC_MCU1_0;
uint32_t remoteProc[] =
{
    IPC_MPU1_0
};
#elif defined(BUILD_MCU1_1)
/* NOTE: all other cores are not used in this test, but must be built as part of full PDK build */
uint32_t selfProcId = IPC_MCU1_1;
uint32_t remoteProc[] =
{
    IPC_MPU1_0
};
#else
#error BUILD CORE is not defined
#endif

uint32_t *pRemoteProcArray = remoteProc;
volatile uint32_t  gNumRemoteProc = sizeof(remoteProc)/sizeof(uint32_t);

/*
 * This function initializes the SCI Client driver
 *
 */
void ipc_initSciclient()
{
    Sciclient_ConfigPrms_t        config;

    /* Now reinitialize it as default parameter */
    Sciclient_configPrmsInit(&config);

    /* Initialize SCI Client */
    Sciclient_init(&config);
}

bool g_exitRespTsk = 0;

/*
 * This function waits for a "ping" message from any processor
 * then replies with a "pong" message.
 */
void rpmsg_responderFxn(uintptr_t arg0, uintptr_t arg1)
{
    RPMessage_Handle handle;
    RPMessage_Params params;
    uint32_t		 myEndPt = 0;
    uint32_t		 remoteEndPt;
    uint32_t		 remoteProcId;
    uint16_t		 len;
    int32_t		     status = 0;
    void		     *buf;
    uint32_t         t;
    uint32_t         lastNumMessagesReceived = 0;
    uint32_t         emptyReceiveCalls = 0;

    uint32_t            bufSize = rpmsgDataSize;
    char                str[MSGSIZE], *strPtr;

    buf = gRspBuf;
    if(buf == NULL)
    {
        System_printf("RecvTask: buffer allocation failed\n");
        return;
    }

    RPMessageParams_init(&params);

    params.requestedEndpt = ENDPT1;

    params.buf = buf;
    params.bufSize = bufSize;

    handle = RPMessage_create(&params, &myEndPt);
    if(!handle)
    {
        System_printf("RecvTask: Failed to create endpoint\n");
        return;
    }
    System_printf("RPMessage_create is done\n");

    for (t = 0; t < gNumRemoteProc; t++)
    {
        Ipc_mailboxEnableNewMsgInt(selfProcId, t);
    }
    System_printf("Ipc_mailboxEnableNewMsgInt is done\n");

    status = RPMessage_announce(RPMESSAGE_ALL, myEndPt, SERVICE);
    if(status != IPC_SOK)
    {
        System_printf("RecvTask: RPMessage_announce() failed\n");
        return;
    }
    System_printf("RPMessage_announce is done\n");

    while(!g_exitRespTsk)
    {
        /* Wait for messages to show up */
        while(gMessagesReceived == lastNumMessagesReceived);
        /* NOTE: The following function may need to be replaced by a blocking
          function RPMessage_recv in later implementations */
        status = RPMessage_recvNb(handle,
                                  (Ptr)str, &len, &remoteEndPt,
                                  &remoteProcId);
        if(status != IPC_SOK)
        {
#ifdef DEBUG_PRINT
            System_printf("RecvTask: failed with code %d\n", status);
#endif
            emptyReceiveCalls++;
        }
        else
        {
            lastNumMessagesReceived++;

            /* NULL terminated string */
            str[len] = '\0';
#ifdef DEBUG_PRINT
            System_printf("RecvTask: Revcvd msg \"%s\" len %d from %s\n",
                    str, len, Ipc_mpGetName(remoteProcId));
#endif
        }
        if(status == IPC_SOK)
        {
			/* If this is not ping/pong message, just print the message */
			System_printf("%s <--> %s : %s recvd : %d:%d:%d\n",
						  Ipc_mpGetSelfName(),
						  Ipc_mpGetName(remoteProcId),
						  str,
						  gMessagesReceived,
						  lastNumMessagesReceived,
						  emptyReceiveCalls);
#ifdef DEBUG_PRINT
            System_printf("RecvTask: Sending msg \"%s\" len %d from %s to %s\n",
                          str, len, Ipc_mpGetSelfName(),
                          Ipc_mpGetName(remoteProcId));
#endif
            strPtr = (char *)&gCoreStat;
			status = RPMessage_send(handle, remoteProcId, remoteEndPt, myEndPt, strPtr, sizeof(gCoreStat));
            if (status != IPC_SOK)
            {
                System_printf("RecvTask: Sending msg \"%s\" len %d from %s to %s failed!!!\n",
                    str, len, Ipc_mpGetSelfName(),
                    Ipc_mpGetName(remoteProcId));
            }
			/* wait for 1ms  before send the next message*/
			/* Osal_delay(1000); */
        }
    }

    System_printf("%s responder task exiting ...\n",
                    Ipc_mpGetSelfName());
}

#define INTRTR_CFG_MAIN_DOMAIN_MBX_CLST0_USR1_OUT_INT_NO  (17U)
#define INTRTR_CFG_START_LEVEL_INT_NUMBER \
            (CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_0)
#define CDD_IPC_CORE_MPU1_0            (0u)
#define APP_SCICLIENT_TIMEOUT          (SCICLIENT_SERVICE_WAIT_FOREVER)
/*
 * This function calls the Ipc lld ISR handler to service the interrupt
 */
void IpcAppBaremetalIrqMbxFromMpu_10(void)
{
    Ipc_newMessageIsr(CDD_IPC_CORE_MPU1_0);
    return;
}
/*
 * This function needs to be registered as an interrupt
 * handler for IPC mailbox events
 */
void IpcAppBaremetalAppMsgFromMpu10Isr(uintptr_t notUsed)
{
    /* Invoke MPU 10 Isr handler */
    IpcAppBaremetalIrqMbxFromMpu_10();
}
/*
 * This function is the callback function the ipc lld library calls when a
 * message is received.
 */
static void IpcAppBaremetalNewMsgCb(uint32_t srcEndPt, uint32_t procId)
{
    /* Add code here to take action on any incoming messages */
    gMessagesReceived++;
    return;
}

/*
 * This function registers the interrupt handlers
 * NOTE: This code may change and may be abstracted into the lld code 
 *       or a seperate osal file in a future release.
 */
int32_t IpcAppConfigureInterruptHandlers(void)
{
    OsalRegisterIntrParams_t    intrPrms;
    OsalInterruptRetCode_e      osalRetVal;
    HwiP_Handle hwiHandle;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
    int32_t retVal;

#if defined(BUILD_MCU1_0)
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER0;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_index              = 1U; /* 0 for User 0, 1 for user 1... */
    rmIrqReq.dst_id                 = TISCI_DEV_MCU_ARMSS0_CPU0;
    rmIrqReq.dst_host_irq           =
                        (INTRTR_CFG_MAIN_DOMAIN_MBX_CLST0_USR1_OUT_INT_NO +
                         INTRTR_CFG_START_LEVEL_INT_NUMBER);

    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
#elif defined(BUILD_MCU1_1)
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER1;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_index              = 1U; /* 0 for User 0, 1 for user 1... */
    rmIrqReq.dst_id                 = TISCI_DEV_MCU_ARMSS0_CPU1;
    rmIrqReq.dst_host_irq           =
                        (INTRTR_CFG_MAIN_DOMAIN_MBX_CLST0_USR1_OUT_INT_NO +
                         INTRTR_CFG_START_LEVEL_INT_NUMBER + 32); /* add 32 to put it in the range allocated for MCU1_1 */

    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
#else
#error BUILD CORE is not defined
#endif
    retVal = Sciclient_rmIrqSet(
                 &rmIrqReq, &rmIrqResp, APP_SCICLIENT_TIMEOUT);
     if(CSL_PASS != retVal)
    {
        System_printf(": Error in SciClient Interrupt Params Configuration!!!\n");
        return -1;
    }
    System_printf("Sciclient_rmIrqSet is done\n");

    /* Interrupt hook up */
    Osal_RegisterInterrupt_initParams(&intrPrms);
    intrPrms.corepacConfig.arg          = (uintptr_t)NULL;
    intrPrms.corepacConfig.isrRoutine   = &IpcAppBaremetalAppMsgFromMpu10Isr;
    intrPrms.corepacConfig.priority     = 1U;
    intrPrms.corepacConfig.corepacEventNum = 0U;
#if defined(BUILD_MCU1_0)
    intrPrms.corepacConfig.intVecNum    =
        (INTRTR_CFG_MAIN_DOMAIN_MBX_CLST0_USR1_OUT_INT_NO +
         INTRTR_CFG_START_LEVEL_INT_NUMBER);
#elif defined(BUILD_MCU1_1)
    intrPrms.corepacConfig.intVecNum    =
        (INTRTR_CFG_MAIN_DOMAIN_MBX_CLST0_USR1_OUT_INT_NO +
         INTRTR_CFG_START_LEVEL_INT_NUMBER + 32);
#else
#error BUILD CORE is not defined
#endif
    osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);

    if(OSAL_INT_SUCCESS != osalRetVal)
    {
        System_printf( ": Error Could not register ISR to receive"
                         " from MCU 1 1 !!!\n");
        return -1;
    }
    System_printf("Osal_RegisterInterrupt is done\n");
    Osal_EnableInterrupt(0, intrPrms.corepacConfig.intVecNum);
    System_printf("Osal_EnableInterrupt is done\n");
    return 0;

}

/*
 * This is the main test function which initializes and runs the Sender and 
 * responder functions.
 * NOTE: Sender function is not actually active in this example.
 *       Only the receiver function does a echo back of incoming messages
 */
/* ==========================================*/
int32_t ipc_rpmsg_func(void)
{
    uint32_t          t;
    uint32_t          numProc = gNumRemoteProc;
    Ipc_VirtIoParams  vqParam;
    Ipc_InitPrms      initPrms;

    System_printf("\nEnter ipc_rpmsg_func\n");
    /* Step1 : Initialize the multiproc */
    if (IPC_SOK == Ipc_mpSetConfig(selfProcId, numProc, pRemoteProcArray))
    {
        System_printf("ipc_rpmsg_func (core : %s) .....\r\n%s\r\n",
                Ipc_mpGetSelfName(), IPC_DRV_VERSION_STR);

        initPrms.instId = 0U;
        initPrms.osalPrms.disableAllIntr = &IpcAppBaremetalCriticalSectionIntEnter;
        initPrms.osalPrms.restoreAllIntr = &IpcAppBaremetalCriticalSectionIntExit;

        initPrms.osalPrms.createHIsr     = &IpcAppBaremetalHIsrCreate;
        initPrms.osalPrms.deleteHIsr     = &IpcAppBaremetalHIsrDelete;
        initPrms.osalPrms.postHIsr       = &IpcAppBaremetalHIsrPost;
        initPrms.osalPrms.createHIsrGate = &IpcAppBaremetalHIsrGateCreate;
        initPrms.osalPrms.deleteHIsrGate = &IpcAppBaremetalHIsrGateDelete;
        initPrms.osalPrms.lockHIsrGate   = &IpcAppBaremetalHIsrGateEnter;
        initPrms.osalPrms.unLockHIsrGate = &IpcAppBaremetalHIsrGateExit;

        initPrms.osalPrms.createMutex    = (Ipc_OsalMutexCreateFxn) NULL_PTR;
        initPrms.osalPrms.deleteMutex    = (Ipc_OsalMutexDeleteFxn) NULL_PTR;
        initPrms.osalPrms.lockMutex      = (Ipc_OsalMutexLockFxn) NULL_PTR;
        initPrms.osalPrms.unlockMutex    = (Ipc_OsalMutexUnlockFxn) NULL_PTR;

        initPrms.newMsgFxn = &IpcAppBaremetalNewMsgCb;

        if (IPC_SOK != Ipc_init(&initPrms))
        {
            return -1;
        }
    } else
	{
        System_printf("Ipc_mpSetConfig failed!!!\n");
	}
#ifdef DEBUG_PRINT
    System_printf("Required Local memory for Virtio_Object = %d\r\n",
                  numProc * Ipc_getVqObjMemoryRequiredPerCore());
#endif
#if !defined(BUILD_MPU1_0) && defined(MPU_LINUX_OS)
    /* If MPU remote core is running Linux OS, then
     * load resource table
     */
    Ipc_loadResourceTable((void*)&ti_ipc_remoteproc_ResourceTable);
    System_printf("Ipc_loadResourceTable done!!!\n");

#if !defined(MPU_LINUX_OS_IPC_ATTACH)
    /* Wait for Linux VDev ready... */
    System_printf("\nWait for Linux VDev ready...\n");
    for(t = 0; t < numProc; t++)
    {
        while(!Ipc_isRemoteReady(pRemoteProcArray[t]))
        {
            /* Task_sleep(100); */
        }
    }
    System_printf("Linux VDEV ready now .....\n");
#endif
#endif

    /* Step2 : Initialize Virtio */
    vqParam.vqObjBaseAddr = (void*)gSysVqBuf;
    vqParam.vqBufSize     = numProc * Ipc_getVqObjMemoryRequiredPerCore();
    vqParam.vringBaseAddr = (void*)VRING_BASE_ADDRESS;
    vqParam.vringBufSize  = IPC_VRING_BUFFER_SIZE;
    vqParam.timeoutCnt    = 100;  /* Wait for counts */
    Ipc_initVirtIO(&vqParam);

    /* Step 3: Initialize RPMessage */
    RPMessage_Params cntrlParam;

#ifdef DEBUG_PRINT
    System_printf("\nRequired Local memory for RPMessage Object = %d\n",
                  RPMessage_getObjMemRequired());
#endif

    /* Initialize the param */
    RPMessageParams_init(&cntrlParam);

    System_printf("\nRPMessageParams_init is done\n");

    /* Set memory for HeapMemory for control task */
    cntrlParam.buf         = gCntrlBuf;
    cntrlParam.bufSize     = rpmsgDataSize;
    cntrlParam.stackBuffer = NULL;
    cntrlParam.stackSize   = 0U;
    RPMessage_init(&cntrlParam);
    System_printf("\nRPMessage_init is done\n");

    /* Step 4: Setup IPC interrupt handling for baremetal */
    if(IpcAppConfigureInterruptHandlers() != 0)
    {
        System_printf("IpcAppConfigureInterruptHandlers Failed!!!\n");
        return -1;
    }

    System_printf("\nSetup IPC interrupt handling is done\n");

    /* run responder function to receive and reply messages back */
    System_printf("\nStart rpmsg_responderFxn\n");
    rpmsg_responderFxn(0,0);

    return 1;
}
