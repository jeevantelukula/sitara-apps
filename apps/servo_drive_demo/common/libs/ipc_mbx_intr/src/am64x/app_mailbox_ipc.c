/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr_gtc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/osal/osal.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/mailbox/mailbox.h>

#include <app_log.h>
#include <app_mbx_ipc.h>
#include <app_mailbox_ipc.h>


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* IPC global data object */
static app_mbxipc_obj_t g_app_mbxipc_obj;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void appMailboxIsr(Mbox_Handle handle, Mailbox_Instance remoteEndpoint);

int appMbxIpcSync(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void appMbxIpcInitPrmSetDefault(app_mbxipc_init_prm_t *prm)
{
    uint32_t cpu_id = 0;

    prm->num_cpus = 0;

    for(cpu_id=0; cpu_id<MAILBOX_IPC_MAX_PROCS; cpu_id++)
    {
        prm->enabled_cpu_id_list[cpu_id] = APP_IPC_CPU_INVALID;
    }

    prm->self_cpu_id = APP_IPC_CPU_INVALID;
    prm->master_cpu_id = APP_IPC_CPU_INVALID;
}

int32_t appMbxIpcRegisterNotifyHandler(app_mbxipc_notify_handler_f handler)
{
    int32_t status = -1;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    if(handler)
    {
        obj->mbxipc_notify_handler = handler;
        status = 0;
    }

    return status;
}

uint32_t appMbxIpcGetMasterCpuId(void)
{
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    return obj->prm.master_cpu_id;
}

uint32_t appMbxIpcGetSelfCpuId(void)
{
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    return obj->prm.self_cpu_id;
}

uint32_t appMbxIpcIsCpuEnabled(uint32_t cpu_id)
{
    uint32_t is_enabled = 0, cur_cpu_id;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    if(cpu_id>=MAILBOX_IPC_MAX_PROCS)
    {
        is_enabled = 0;
    }
    for(cur_cpu_id=0; cur_cpu_id<obj->prm.num_cpus; cur_cpu_id++)
    {
        if(cpu_id==obj->prm.enabled_cpu_id_list[cur_cpu_id])
        {
            is_enabled = 1;
            break;
        }
    }
    return is_enabled;
}

int32_t appMbxIpcInit(app_mbxipc_init_prm_t *prm)
{
    int32_t retVal = 0;
    uint16_t remoteId;
    int32_t errCode;
    uint32_t selfId;
    Mailbox_initParams initParam;
    Mailbox_openParams openParam;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;
    obj->prm = *prm;
    obj->mbxipc_notify_handler = NULL;

    appLogPrintf("MBX-IPC: Init ... !!!\n");

    selfId = appMbxIpcGetSelfCpuId();

    /* Setup the default Mailbox init Parameters */
    Mailbox_initParams_init(&initParam);
    initParam.localEndpoint = selfId;

    /* Initialize the Mailbox */
    Mailbox_init(&initParam);

    /* Setup the default Mailbox open Parameters */
    Mailbox_openParams_init(&openParam);
    openParam.cfg.writeMode = MAILBOX_MODE_FAST;
    openParam.cfg.readMode = MAILBOX_MODE_FAST;
    openParam.cfg.readCallback = NULL;
    openParam.cfg.enableVIMDirectInterrupt = true;

    /* Enable without Interrupt call-back to perform Sync between all cores */
    for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
    {
        /* Skip self */
		if (remoteId == selfId)
		{
			continue;
		}
		if (!appMbxIpcIsCpuEnabled(remoteId))
		{
			continue;
		}

        openParam.remoteEndpoint = remoteId;

        /* Open the Instance */
        obj->handle[remoteId] = Mailbox_open(&openParam, &errCode);

        if (obj->handle[remoteId] == NULL)
        {
            appLogPrintf("MBX-IPC: Mailbox Instance open failed!! \n");
            retVal = -1;
            break;
        }
    }

    if (retVal == 0)
    {
        retVal = appMbxIpcSync();
        if (retVal != 0)
        {
            appLogPrintf("MBX-IPC: appMbxIpcSync() failed!! \r\n");
        }
    }

    /* Close all open handles to re-open with Interrupt callback mode */
    for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
    {
        /* Skip self */
		if (remoteId == selfId)
		{
			continue;
		}
		if (!appMbxIpcIsCpuEnabled(remoteId))
		{
			continue;
		}

        Mailbox_close(obj->handle[remoteId]);
    }

    /* Setup the default Mailbox open Parameters */
    Mailbox_openParams_init(&openParam);
    openParam.cfg.writeMode = MAILBOX_MODE_FAST;
    openParam.cfg.readMode = MAILBOX_MODE_FAST;
    openParam.cfg.readCallback = appMailboxIsr;
    openParam.cfg.enableVIMDirectInterrupt = true;

    /* Enable with Interrupt call-back on all cores */
    for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
    {
        /* Skip self */
		if (remoteId == selfId)
		{
			continue;
		}
		if (!appMbxIpcIsCpuEnabled(remoteId))
		{
			continue;
		}

        openParam.remoteEndpoint = remoteId;

        /* Open the Instance */
        obj->handle[remoteId] = Mailbox_open(&openParam, &errCode);

        if (obj->handle[remoteId] == NULL)
        {
            appLogPrintf("MBX-IPC: Mailbox Instance open failed!! \n");
            retVal = -1;
            break;
        }
    }

    if (retVal)
    {
        appLogPrintf("MBX-IPC: Init ... Failed !!!\n");
    }
    else
    {
        appLogPrintf("MBX-IPC: Init ... Done !!!\n");
    }

    return retVal;
}

int32_t appMbxIpcDeInit(void)
{
    uint32_t selfId;
    uint16_t remoteId;
    int32_t retVal = 0;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    appLogPrintf("MBX-IPC: DeInit ... !!!\n");

    selfId = appMbxIpcGetSelfCpuId();

    /* Close all open handles */
    for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
    {
        /* Skip self */
		if (remoteId == selfId)
		{
			continue;
		}
		if (!appMbxIpcIsCpuEnabled(remoteId))
		{
			continue;
		}

        Mailbox_close(obj->handle[remoteId]);
    }

	appMbxIpcInitPrmSetDefault(&obj->prm);
    obj->mbxipc_notify_handler = NULL;

    appLogPrintf("MBX-IPC: DeInit... Done !!!\n");

    return retVal;
}

void appMailboxIsr(Mbox_Handle handle, Mailbox_Instance remoteEndpoint)
{
    int16_t remoteId;
    uint32_t payload;
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

    remoteId = (uint16_t)remoteEndpoint;

    Mailbox_read(handle, (uint8_t *)&payload, sizeof(payload));

    if(obj->mbxipc_notify_handler)
    {
        obj->mbxipc_notify_handler(remoteId, payload);
    }
}

int appMbxIpcSync(void)
{
    int retVal = 0;
	uint32_t payload;
    uint16_t remoteId;
	uint32_t msgStatus = MESSAGE_INVALID;
	uint32_t selfId = appMbxIpcGetSelfCpuId();
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

	if (selfId == appMbxIpcGetMasterCpuId())
	{
		/* Send sync message to all cores */
		for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
		{
			/* Skip self */
			if (remoteId == selfId)
			{
				continue;
			}
			if (!appMbxIpcIsCpuEnabled(remoteId))
			{
				continue;
			}
			payload = MAILBOX_APP_SYNC_MESSAGE;
			/* Send initial Sync message */
			Mailbox_write(obj->handle[remoteId], (uint8_t *)&payload, sizeof(payload));
		}

		/* Receive Ack message back for all cores */
		for (remoteId = 0; remoteId < MAILBOX_IPC_MAX_PROCS; remoteId++)
		{
			/* Skip self */
			if (remoteId == selfId)
			{
				continue;
			}
			if (!appMbxIpcIsCpuEnabled(remoteId))
			{
				continue;
			}
			do
			{
                msgStatus = Mailbox_read(obj->handle[remoteId], (uint8_t *)&payload, sizeof(payload));
			} while (msgStatus != MAILBOX_SOK || payload != MAILBOX_APP_ACK_MESSAGE);
			if (payload != MAILBOX_APP_ACK_MESSAGE)
			{
				retVal = -1;
			}
		}
	}
	else
	{
		remoteId = appMbxIpcGetMasterCpuId();
		do
		{
			msgStatus = Mailbox_read(obj->handle[remoteId], (uint8_t *)&payload, sizeof(payload));
		} while (msgStatus != MAILBOX_SOK || payload != MAILBOX_APP_SYNC_MESSAGE);
		if (payload != MAILBOX_APP_SYNC_MESSAGE)
		{
			retVal = -1;
		}
		if ( retVal == 0)
		{
	        payload = MAILBOX_APP_ACK_MESSAGE;
			/* Send initial Sync message */
			Mailbox_write(obj->handle[remoteId], (uint8_t *)&payload, sizeof(payload));
		}
	}

    return retVal;
}

void appMbxIpcSendNotify(uint32_t remoteId, uint32_t payload)
{
    app_mbxipc_obj_t *obj = &g_app_mbxipc_obj;

	Mailbox_write(obj->handle[remoteId], (uint8_t *)&payload, sizeof(payload));

    return;
}


/********************************* End of file ******************************/
