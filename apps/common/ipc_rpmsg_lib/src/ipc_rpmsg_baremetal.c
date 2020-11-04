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
#include "ipc_rsctable.h"
#endif

/* Size of message */
#define MSGSIZE  256U
/* Service name to be registered for the end point */
#define SERVICE  "ti.ipc4.ping-pong"
/* End point number to be used for IPC communication */
#define ENDPT1   13U
/* Service name to be registered for chrdev end point */
#define SERVICE_CHRDEV "rpmsg_chrdev"
/* End point number to be used for chrdev end point */
#define ENDPT_CHRDEV   14U

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
uint8_t  gRspBufChar[RPMSG_DATA_SIZE]  __attribute__ ((section ("ipc_data_buffer"), aligned (8)));

volatile uint32_t gMessagesReceived = 0;
volatile uint32_t gMessagesReceivedPrev = 0;

uint32_t		  myEndPt = 0;
uint32_t		  remoteEndPt;
uint32_t		  remoteProcId =0xFFFFFFFF;
RPMessage_Handle  *responseHandle = NULL;
RPMessage_Handle  handleIpcRPMsg;
RPMessage_Params  paramsIpcRPMsg;
RPMessage_Handle handleIpcRPMsgChar;
RPMessage_Params  paramsIpcRPMsgChar;

#ifdef BUILD_MPU1_0
#define NUM_CHRDEV_SERVICES (1)
#define CHRDEV_PROC_NAME    "a53-mpu-core-0"
uint32_t selfProcId = IPC_MPU1_0;
uint32_t remoteProc[] =
{
    IPC_MCU1_0,
    IPC_MCU1_1,
	IPC_MCU2_0,
	IPC_MCU2_1,
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
#elif defined(BUILD_MCU2_0)
/* NOTE: all other cores are not used in this test, but must be built as part of full PDK build */
uint32_t selfProcId = IPC_MCU2_0;
uint32_t remoteProc[] =
{
    IPC_MPU1_0
};
#elif defined(BUILD_MCU2_1)
/* NOTE: all other cores are not used in this test, but must be built as part of full PDK build */
uint32_t selfProcId = IPC_MCU2_1;
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
 * This function check for message from any processor
 */
void ipc_rpmsg_receive(char *msg, uint16_t *msg_size)
{
   int32_t status = 0;

   /* if there is a message showed up */
   if (gMessagesReceived > gMessagesReceivedPrev)
   {
      /* NOTE: The following function may need to be replaced by a blocking
         function RPMessage_recv in later implementations */
      status = RPMessage_recvNb(handleIpcRPMsg,
         (Ptr)msg, msg_size, &remoteEndPt, &remoteProcId);
      if(status == IPC_SOK)
      {
         gMessagesReceivedPrev++;

         /* NULL terminated string */
         msg[*msg_size] = '\0';
         System_printf("ipc_rpmsg_receive 1: Revcvd msg \"%s\" len %d from %s\n",
            msg, *msg_size, Ipc_mpGetName(remoteProcId));
 	     responseHandle = &handleIpcRPMsg;
      } else
	  {
         /* NOTE: The following function may need to be replaced by a blocking
            function RPMessage_recv in later implementations */
         status = RPMessage_recvNb(handleIpcRPMsgChar,
            (Ptr)msg, msg_size, &remoteEndPt, &remoteProcId);
         if(status != IPC_SOK)
         {
            System_printf("ipc_rpmsg_receive 2: failed with code %d\n", status);
         } else 
         {
            gMessagesReceivedPrev++;

            /* NULL terminated string */
            msg[*msg_size] = '\0';
            System_printf("ipc_rpmsg_receive 2: Revcvd msg \"%s\" len %d from %s\n",
               msg, *msg_size, Ipc_mpGetName(remoteProcId));
         }
	     responseHandle = &handleIpcRPMsgChar;
	  }
   }
}

/*
 * This function send message to any processor
 */
void ipc_rpmsg_send(char *msg, uint16_t msg_size)
{
   int32_t status = 0;

   /* Got the remote Proc ID and endpoint */
   if (remoteProcId!=0xFFFFFFFF)
   {
      status = RPMessage_send(*responseHandle, remoteProcId, remoteEndPt, myEndPt, msg, msg_size);
      if (status != IPC_SOK)
      {
         System_printf("RecvTask: Sending msg \"%s\" len %d from %s to %s failed!!!\n",
            msg, msg_size, Ipc_mpGetSelfName(),Ipc_mpGetName(remoteProcId));
      }
   }
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

uint32_t IpcAppVirtToPhyFxn(const void *virtAddr)
{
    return ((uint32_t) virtAddr);
}

void *IpcAppPhyToVirtFxn(uint32_t phyAddr)
{
    uint32_t temp = phyAddr;

    return ((void *) temp);
}

/*
 * This is the function which initializes the IPC RPMsg_char channel
 */
/* ==========================================*/
int32_t ipc_rpmsg_init(void)
{
    uint32_t          t;
    uint32_t          numProc = gNumRemoteProc;
    Ipc_VirtIoParams  vqParam;
    Ipc_InitPrms      initPrms;
    int32_t		      status = 0;
    void		      *buf;
    uint32_t          bufSize = rpmsgDataSize;

    System_printf("\nEnter ipc_rpmsg_init\n");
    /* Step1 : Initialize the multiproc */
    if (IPC_SOK == Ipc_mpSetConfig(selfProcId, numProc, pRemoteProcArray))
    {
        System_printf("ipc_rpmsg_func (core : %s) .....\r\n%s\r\n",
                Ipc_mpGetSelfName(), IPC_DRV_VERSION_STR);

        /* Initialize params with defaults */
        IpcInitPrms_init(0U, &initPrms);

        initPrms.newMsgFxn = &IpcAppBaremetalNewMsgCb;
        initPrms.virtToPhyFxn = &IpcAppVirtToPhyFxn;
        initPrms.phyToVirtFxn = &IpcAppPhyToVirtFxn;

        if (IPC_SOK != Ipc_init(&initPrms))
        {
            return -1;
        }
    } else
	{
        System_printf("Ipc_mpSetConfig failed!!!\n");
	}

    System_printf("Required Local memory for Virtio_Object = %d\r\n",
                  numProc * Ipc_getVqObjMemoryRequiredPerCore());
    /* If MPU remote core is running Linux OS, then
     * load resource table
     */
    Ipc_loadResourceTable((void*)&ti_ipc_remoteproc_ResourceTable);
    System_printf("Ipc_loadResourceTable done!!!\n");

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

    /* Step2 : Initialize Virtio */
    vqParam.vqObjBaseAddr = (void*)gSysVqBuf;
    vqParam.vqBufSize     = numProc * Ipc_getVqObjMemoryRequiredPerCore();
    vqParam.vringBaseAddr = (void*)VRING_BASE_ADDRESS;
    vqParam.vringBufSize  = IPC_VRING_BUFFER_SIZE;
    vqParam.timeoutCnt    = 100;  /* Wait for counts */
    Ipc_initVirtIO(&vqParam);

    /* Step 3: Initialize RPMessage */
    RPMessage_Params cntrlParam;

    System_printf("\nRequired Local memory for RPMessage Object = %d\n",
                  RPMessage_getObjMemRequired());

    /* Initialize the param */
    RPMessageParams_init(&cntrlParam);

    System_printf("\nRPMessageParams_init is done\n");

    /* Set memory for HeapMemory for control task */
    cntrlParam.buf         = gCntrlBuf;
    cntrlParam.bufSize     = rpmsgDataSize;
    cntrlParam.stackBuffer = NULL;
    cntrlParam.stackSize   = 0U;
    RPMessage_init(&cntrlParam);
    System_printf("\nRPMessage_init 1 is done\n");

    buf = gRspBuf;
    if(buf == NULL)
    {
        System_printf("Buffer allocation failed\n");
        return -2;
    }

    RPMessageParams_init(&paramsIpcRPMsg);

    paramsIpcRPMsg.requestedEndpt = ENDPT1;

    paramsIpcRPMsg.buf = buf;
    paramsIpcRPMsg.bufSize = bufSize;

    handleIpcRPMsg = RPMessage_create(&paramsIpcRPMsg, &myEndPt);
    if(!handleIpcRPMsg)
    {
        System_printf("RPMessage_create 1 is failed\n");
        return -3;
    }
    System_printf("RPMessage_create 1 is done\n");

    status = RPMessage_announce(RPMESSAGE_ALL, myEndPt, SERVICE);
    if(status != IPC_SOK)
    {
        System_printf("RPMessage_announce 1 failed\n");
        return -4;
    }
    System_printf("RPMessage_announce 1 is done\n");

    buf = gRspBufChar;
    if(buf == NULL)
    {
        System_printf("Buffer allocation failed\n");
        return -2;
    }
    RPMessageParams_init(&paramsIpcRPMsgChar);
    System_printf("\nRPMessage_init 2 is done\n");


    paramsIpcRPMsgChar.requestedEndpt = ENDPT_CHRDEV;

    paramsIpcRPMsgChar.buf = buf;
    paramsIpcRPMsgChar.bufSize = bufSize;

    handleIpcRPMsgChar = RPMessage_create(&paramsIpcRPMsgChar, &myEndPt);
    if(!handleIpcRPMsgChar)
    {
        System_printf("RPMessage_create 2 is failed\n");
        return -3;
    }
    System_printf("RPMessage_create 2 is done\n");

    status = RPMessage_announce(RPMESSAGE_ALL, myEndPt, SERVICE_CHRDEV);
    if(status != IPC_SOK)
    {
        System_printf("RPMessage_announce 2 failed\n");
        return -4;
    }
    System_printf("RPMessage_announce 2 is done\n");

    return 0;
}
