/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file ipcapp_baremetal.c
 *
 *  \brief IPC baremetal functions
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ti/osal/osal.h>
#include <ti/drv/ipc/ipc.h>

#include "ipcapp_baremetal.h"

/* ==========================================*/
/* CDD functions */
/** \brief Disabled interrupt context */
static uintptr_t IpcAppBaremetalIntKey;
static int32_t IpcAppBaremetalSchMCount = 0;

uintptr_t IpcAppBaremetalCriticalSectionIntEnter(void)
{
    IpcAppBaremetalIntKey = HwiP_disable();
    IpcAppBaremetalSchMCount++;
    return (uintptr_t)1U;
}

void IpcAppBaremetalCriticalSectionIntExit(uintptr_t keyNotUsed)
{
    HwiP_restore(IpcAppBaremetalIntKey);
    IpcAppBaremetalSchMCount--;

}

int32_t IpcAppBaremetalHIsrCreate(Ipc_OsalHIsrHandle *handle,
                                    Ipc_OsalHIsrFxn fxn, void *arg)
{
    int32_t rtnVal = IPC_EBADARGS;

    /* No HISR, HISR would be invoked in ISR context it self */
    if ((NULL != handle) && ((NULL != fxn) && (NULL != arg)))
    {
        handle->arg0    = (uintptr_t) arg;
        handle->arg1    = (uintptr_t) NULL;
        handle->hIsrFxn = fxn;
        handle->hLosHisrHandle = NULL;
        rtnVal = IPC_SOK;
    }

    return (rtnVal);
}

void IpcAppBaremetalHIsrDelete(Ipc_OsalHIsrHandle *handle)
{
    if (NULL != handle)
    {
        handle->arg0    = NULL;
        handle->arg1    = NULL;
        handle->hIsrFxn = NULL;
        handle->hLosHisrHandle = NULL;
    }

    return;
}

int32_t IpcAppBaremetalHIsrPost(Ipc_OsalHIsrHandle *handle)
{
    int32_t rtnVal = IPC_EBADARGS;

    if (NULL != handle)
    {
        handle->hIsrFxn(handle->arg0, NULL);
        rtnVal = IPC_SOK;
    }
    return (rtnVal);
}

void *IpcAppBaremetalHIsrGateCreate(void)
{
    /* Really!!, nothing required as rely on interrupt locking for exclusive
        access. */
    return (void *) 1U;
}

void IpcAppBaremetalHIsrGateDelete(Ipc_OsalHIsrGateHandle *handle)
{
    /* Really!!, nothing required as rely on interrupt locking for exclusive
        access. */
    return;
}

int32_t IpcAppBaremetalHIsrGateEnter(Ipc_OsalHIsrGateHandle handle)
{
    /* Really!!, nothing required as rely on interrupt locking for exclusive
        access. */
    return ((int32_t)IpcAppBaremetalCriticalSectionIntEnter());
}

void IpcAppBaremetalHIsrGateExit(Ipc_OsalHIsrGateHandle handle, int32_t key)
{
    /* Really!!, nothing required as rely on interrupt locking for exclusive
        access. */
    IpcAppBaremetalCriticalSectionIntExit(key);
    return;
}
