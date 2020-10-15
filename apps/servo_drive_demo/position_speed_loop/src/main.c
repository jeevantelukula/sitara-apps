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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <logs/include/app_log.h>
#include "app_init.h"
#include "position_speed_loop_if.h"

/*
 * Implements Position Speed Loop R5F main().
 * Control comes here immediately after the core boots-up
 * All Motor control related APIs shall be called from this here.
 */

/* Added for debug purpose when load and run via SBL.
 * set enableDebug = 1 and build for debug.
 * Once started running connect CCS and reset enableDebug=0
 * to proceed with single-step from the beginning
 */
void StartupEmulatorWaitFxn (void)
{
    volatile uint32_t enableDebug = 0;
    do
    {
    }while (enableDebug);
}

int main(void)
{
    int32_t status;
    
    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    appLogPrintf("MCU-SS core0 is up !!!! \n");

    status = appInit();
    if (status != 0)
    {
        appLogPrintf("ERROR: appInit() failed\n");        
        return -1;
    }
    
    status = appPositionSpeedLoopInit();
    if (status != POSITION_SPEED_LOOP_SOK)
    {
        appLogPrintf("ERROR: appPositionSpeedLoopInit() failed\n");
        return -1;
    }

    appPositionSpeedLoopStart();

}
