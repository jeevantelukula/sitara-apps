/*
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _TIMESYNC_H_
#define _TIMESYNC_H_

#include <stdint.h>
#include <ti/csl/cslr_icss.h>
#include "timesyncHwRegs.h"

/* Return status codes */
#define IEP_STS_RECFG_PRD_COUNT     ( 1 )   /* no error, IEP Period Count reconfiguration */
#define IEP_STS_NERR                ( 0 )   /* no error */
#define IEP_STS_ERR_INV_IEP_ID      ( -1 )  /* error, invalid IEP ID */

/* IEPs per ICSSG */
#define ICSSG_NUM_IEP               ( 2 )

/* TS sets per IEP */
#define IEP_NUM_TS_SET             ( 4 )

#define DEF_COUNT_INC_PER_CLK       ( 5 )   /* IEP counter default increments per tick */

#define IEP_CMP_STATUS_CMP0_MASK    ( 0x1 )
#define IEP_CMP_STATUS_CMP1_12_MASK ( 0x1FFE )
#define IEP_CMP_STATUS_CMP0_12_MASK ( IEP_CMP_STATUS_CMP0_MASK | IEP_CMP_STATUS_CMP1_12_MASK )


/* IEP IDs */
typedef enum IepId_e
{
    IEP_ID_0 = 0,   /* IEP 0 ID */
    IEP_ID_1 = 1    /* IEP 1 ID */
} IepId;

/* ICSSG IEP TS control object */
typedef struct IcssgTsCtrlObj_s
{
    Bool                        iepTsGblEn[ICSSG_NUM_IEP];             /* IEP TS global disable/enable flags */
    CSL_IcssCfgRegs             *pIcssgCfgHwRegs;                       /* ICSSG CFG registers, init to CSL_ICSS_CFG_BASE */
} IcssgTsCtrlObj;

/* ICSSG IEP TS object */
typedef struct IcssgTsObj_s
{
    IepId                        iepId;                                 /* IEP ID, init to 0 or 1 */
    uint32_t                     iepTsMode;                             /* IEP mode */
    uint32_t                     iepTsEn;                               /* IEP enable */
    uint32_t                     iepTsPeriodCount[5];                   /* IEP period count */
    int32_t                      iepTsPeriodOffset[4];                  /* IEP period count offset */
    CSL_icss_g_pr1_iep1_slvRegs *pIepHwRegs;                            /* IEP hardware registers base address, init to CSL_ICSS_IEP_CFG_BASE */

} IcssgTsObj;

extern IcssgTsCtrlObj gIcssgTsCtrlObj;  /* IEP TS control object */
extern IcssgTsObj gIcssgIep0TsObj;      /* IEP 0 TS object */
extern IcssgTsObj gIcssgIep1TsObj;      /* IEP 1 TS object */

/* Reset TS FW control object */
int32_t resetTsCtrlObj(
    IcssgTsCtrlObj *pIcssgTsCtrlObj
);

/* Initialize TS FW control */
int32_t initTsCtrl(
    IcssgTsCtrlObj *pIcssgTsCtrlObj
);

/* Reset IEP TS object */
int32_t resetIepTsObj(
    IcssgTsObj *pIcssgTsObj,
    IepId iepId
);

/* Initialize IEP TS object
 *
 *  Initial Configuration is located in Host I/F FW Regs.
 *  Default Initial Configuration is loaded in DMEM load (static data).
 *  Host can overwrite Default Initial Configuration *before* FW execution.
 *
 *  Initial Configuration Applied whether Host_RECFG != 0 or not, i.e. Host_RECFG not checked.
 *  Initial Configuration is only place TS MODE is configured.
*/
void initIepTs(
    IcssgTsObj *pIcssgTsObj
);

/* Set TS FW initialization flag.
   Flag indicates to Host FW initialization is complete. */
int32_t setTsFwInitFlag(
    IcssgTsCtrlObj *pIcssgTsCtrlObj
);

void readIepCmp(
    IcssgTsObj *pIcssgTsObj,
    uint32_t   *curIep,
    uint32_t   *curCmp3,
    uint32_t   *curCmp4,
    uint32_t   *curCmp5,
    uint32_t   *curCmp6
);

#endif /* _TIMESYNC_H_ */
