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

#ifndef APP_SCICLIENT_WRAPPER_API_H_
#define APP_SCICLIENT_WRAPPER_API_H_

#include <ti/drv/sciclient/sciclient.h>
#include <utils/logs/include/app_log.h>

/* make below 0 to disable debug print's for these macro APIs */
#define APP_DEBUG_SCICLIENT 1

#define SET_CLOCK_PARENT(MOD, CLK, PARENT) do { \
    int32_t status = 0; \
    if(status == 0) { \
        if(APP_DEBUG_SCICLIENT) \
            appLogPrintf("SCICLIENT: Sciclient_pmSetModuleClkParent module=%u clk=%u parent=%u\n", MOD, CLK, PARENT); \
	    status = Sciclient_pmSetModuleClkParent(MOD, CLK, PARENT, SCICLIENT_SERVICE_WAIT_FOREVER); \
	    if(status != 0) appLogPrintf("SCICLIENT: ERROR: Sciclient_pmSetModuleClkParent failed\n"); \
	    else \
        { \
            if(APP_DEBUG_SCICLIENT) \
                appLogPrintf("SCICLIENT: Sciclient_pmSetModuleClkParent success\n"); \
        } \
    } \
} while(0)

#define SET_DEVICE_STATE(MOD, STATE) do { \
    int32_t status = 0; \
    if(status == 0) { \
        if(APP_DEBUG_SCICLIENT) \
            appLogPrintf("SCICLIENT: Sciclient_pmSetModuleState module=%u state=%u\n", MOD, STATE); \
        status = Sciclient_pmSetModuleState(MOD, STATE, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER); \
        if(status != 0) appLogPrintf("SCICLIENT: ERROR: Sciclient_pmSetModuleState failed\n"); \
	    else \
        { \
            if(APP_DEBUG_SCICLIENT) \
                appLogPrintf("SCICLIENT: Sciclient_pmSetModuleState success\n"); \
        } \
    } \
} while(0)

#define SET_DEVICE_STATE_ON(MOD) SET_DEVICE_STATE(MOD,TISCI_MSG_VALUE_DEVICE_SW_STATE_ON)

#define SET_CLOCK_STATE(MOD, CLK, FLAG, STATE) do { \
    int32_t status = 0; \
    if(status == 0) { \
        if(APP_DEBUG_SCICLIENT) \
            appLogPrintf("SCICLIENT: Sciclient_pmModuleClkRequest module=%u clk=%u state=%u flag=%u\n", MOD, CLK, STATE, FLAG); \
        status = Sciclient_pmModuleClkRequest(MOD, CLK, STATE, FLAG, SCICLIENT_SERVICE_WAIT_FOREVER); \
        if(status != 0) appLogPrintf("SCICLIENT: ERROR: Sciclient_pmModuleClkRequest failed\n"); \
   	    else \
        { \
            if(APP_DEBUG_SCICLIENT) \
                appLogPrintf("SCICLIENT: Sciclient_pmModuleClkRequest success\n"); \
        } \
    } \
} while(0)

#define QUERY_CLOCK_FREQ(MOD, CLK, FREQ) do { \
    int32_t status = 0; \
    uint64_t freq; \
    if(status == 0) { \
        if(APP_DEBUG_SCICLIENT) \
            appLogPrintf("SCICLIENT: Sciclient_pmQueryModuleClkFreq module=%u clk=%u freq=%u%06u\n", MOD, CLK, (uint32_t)(FREQ / 1000000), (uint32_t)(FREQ % 1000000)); \
        status = Sciclient_pmQueryModuleClkFreq(MOD, CLK, FREQ, &freq, SCICLIENT_SERVICE_WAIT_FOREVER); \
        if(status != 0) appLogPrintf("SCICLIENT: ERROR: Sciclient_pmQueryModuleClkFreq failed\n"); \
	    else \
        { \
            if(APP_DEBUG_SCICLIENT) \
                appLogPrintf("SCICLIENT: Sciclient_pmQueryModuleClkFreq freq=%u%06u\n", (uint32_t)(freq / 1000000), (uint32_t)(freq % 1000000)); \
        } \
    } \
} while(0)

#define SET_CLOCK_FREQ(MOD, CLK, FREQ) do { \
    int32_t status = 0; \
    if(status == 0) { \
        if(APP_DEBUG_SCICLIENT) \
            appLogPrintf("SCICLIENT: Sciclient_pmSetModuleClkFreq module=%u clk=%u freq=%u%06u\n", MOD, CLK, (uint32_t)(FREQ / 1000000), (uint32_t)(FREQ % 1000000)); \
        status = Sciclient_pmSetModuleClkFreq(MOD, CLK, FREQ, 0, SCICLIENT_SERVICE_WAIT_FOREVER); \
        if(status != 0) appLogPrintf("SCICLIENT: ERROR: Sciclient_pmSetModuleClkFreq failed\n"); \
        else \
        { \
            if(APP_DEBUG_SCICLIENT) \
                appLogPrintf("SCICLIENT: Sciclient_pmSetModuleClkFreq success\n"); \
        } \
    } \
} while(0)

#endif
