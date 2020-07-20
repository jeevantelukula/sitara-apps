/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the
 *        distribution.
 *
 *      * Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
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

#ifndef _TIMESYNC_DRV_UTILS_H_
#define _TIMESYNC_DRV_UTILS_H_

#include <ti/csl/tistdtypes.h>
#include "timesyncDrv_api.h"

/**
 *  @name   icssgTsDrv_startIepCount
 *  @brief  Start IEP0 counter
 *
 *  @param[in]  handle          TS DRV instance handle
 *
 *  @retval none
 *
 */
void icssgTsDrv_startIepCount(
    IcssgTsDrv_Handle handle
);

/**
 *  @name   icssgTsDrv_readIepCmp
 *  @brief  Read IEP and comparators
 *
 *  @param[in]  handle          TS DRV instance handle
 *  @param[in]  curIep          Address of returned IEP value (call by reference)
 *  @param[in]  curCmp7         Address of returned CMP7 value (call by reference)
 *  @param[in]  curCmp8         Address of returned CMP8 value (call by reference)
 *  @param[in]  curCmp9         Address of returned CMP9 value (call by reference)
 *  @param[in]  curCmp10        Address of returned CMP10 value (call by reference)
 *
 *  @retval none
 *
 */
void icssgTsDrv_readIepCmp(
    IcssgTsDrv_Handle handle,
    uint32_t   *curIep,
    uint32_t   *curCmp7,
    uint32_t   *curCmp8,
    uint32_t   *curCmp9,
    uint32_t   *curCmp10
);

#endif /* _TIMESYNC_DRV_UTILS_H_ */
