/*
 * Copyright (C) 2018-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include <logs/include/app_log.h>
#include <ti/drv/sciclient/sciclient.h>
#include <stdio.h>

int32_t appSciclientDmscGetVersion(char *version_str, uint32_t version_str_size)
{
    int32_t retVal = 0;

    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *)&request,
        sizeof(request),
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    retVal = Sciclient_service(&reqPrm, &respPrm);
    if (0 == retVal)
    {
        if (respPrm.flags == TISCI_MSG_FLAG_ACK)
        {
            if(version_str == NULL)
            {
                appLogPrintf("SCICLIENT: DMSC FW version [%s]\n", (char *) response.str);
                appLogPrintf("SCICLIENT: DMSC FW revision 0x%x  \n", response.version);
                appLogPrintf("SCICLIENT: DMSC FW ABI revision %d.%d\n",
                    response.abi_major, response.abi_minor);
            }
            else
            {
                snprintf(version_str, version_str_size, "version %s, revision 0x%x, ABI %d.%d",
                    (char *) response.str,
                    response.version,
                    response.abi_major, response.abi_minor
                    );
            }
        }
        else
        {
            retVal = -1;
        }
    }
    if(retVal!=0)
    {
        appLogPrintf("SCICLIENT: ERROR: DMSC Firmware Get Version failed !!!\n");
    }

    return (retVal);
}

int32_t appSciclientInit()
{
    int32_t retVal = 0;
    Sciclient_ConfigPrms_t  sciClientCfg;

    appLogPrintf("SCICLIENT: Init ... !!!\n");

    Sciclient_configPrmsInit(&sciClientCfg);

    #ifdef C71
    sciClientCfg.isSecureMode = 1;
    #endif

    retVal = Sciclient_init(&sciClientCfg);
    if(retVal!=0)
    {
        appLogPrintf("SCICLIENT: ERROR: Sciclient init failed !!!\n");
    }
    if(retVal==0)
    {
        appSciclientDmscGetVersion(NULL, 0);
    }

    appLogPrintf("SCICLIENT: Init ... Done !!!\n");

    return retVal;
}

int32_t appSciclientDeInit()
{
    int32_t retVal = 0;

    retVal = Sciclient_deinit();
    if(retVal!=0)
    {
        appLogPrintf("SCICLIENT: ERROR: Sciclient deinit failed !!!\n");
    }

    return retVal;
}
