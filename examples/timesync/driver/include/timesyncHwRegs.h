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

#ifndef _TIMESYNC_HW_REGS_H_
#define _TIMESYNC_HW_REGS_H_

#include <stdint.h>
#include <ti/csl/soc.h>

#if defined(BUILD_PRU0) || defined(BUILD_PRU1)
/* ICSS HW configuration registers base address */
#define ICSS_CFG_BASE       ( CSL_ICSS_CFG_BASE )

/* IEP0/1 HW register base addresses */
#define ICSS_IEP0_CFG_BASE  ( CSL_ICSS_IEP_CFG_BASE )
#define ICSS_IEP1_CFG_BASE  ( ICSS_IEP0_CFG_BASE + CSL_ICSS_IEP_CFG_SIZE )

#elif defined(BUILD_MCU1_0) || defined(BUILD_MCU1_1)
/* ICSS HW configuration registers base address */
#define ICSS_CFG_BASE       ( CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE ) /* Hard-coded for ICSSG0 */
//#define ICSS_CFG_BASE       ( CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE ) /* Hard-coded for ICSSG1 */

/* IEP0/1 HW register base addresses */
#define ICSS_IEP0_CFG_BASE  ( CSL_PRU_ICSSG0_IEP0_BASE )
#define ICSS_IEP1_CFG_BASE  ( CSL_PRU_ICSSG0_IEP1_BASE )
//#define ICSS_IEP0_CFG_BASE  ( CSL_PRU_ICSSG1_IEP0_BASE )
//#define ICSS_IEP1_CFG_BASE  ( CSL_PRU_ICSSG1_IEP1_BASE )

#endif
#endif /* #define _TIMESYNC_HW_REGS_H_ */
