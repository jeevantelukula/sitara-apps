/**
 * @file interruptroute.h
 *
 * @brief Interrupt Routing Interface using Sciclient APIs
* \par
* Copyright (C) 2020 Texas Instruments Incorporated
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* \par
 *
*/
#ifndef INTERRUPTROUTE_H_
#define INTERRUPTROUTE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/csl/csl_types.h>

/**
 * @def NUM_ICSS_INTERRUPTS
 *  Number of ICSS events
 */
#define NUM_ICSS_INTERRUPTS         (8)

/**
 * @def INTERRUPT_ROUTE_ERROR
 *  Error code for failure of interrupt routing
 */
#define INTERRUPT_ROUTE_ERROR       (0xD0D0D0D0)

/**
 * @def ICSSG_INTERRUPT_SRC_INDEX
 *  The source index to be used for first ICSSG event
 */
#define ICSSG_INTERRUPT_SRC_INDEX   (294)

/**
 * @def I2C0_INTERRUPT_SRC_INDEX
 *  The source index to be used for I2C interrupt
 */
#define I2C0_INTERRUPT_SRC_INDEX    (0)

/**
 * @def ICSS_INSTANCE_ONE
 *  PRU ICSS instance 1
 */
#define ICSS_INSTANCE_ONE        (1)

/**
 * @def ICSS_INSTANCE_TWO
 *  PRU ICSS instance 2
 */
#define ICSS_INSTANCE_TWO        (2)

/**
 * @def ICSS_INSTANCE_THREE
 *  PRU ICSS instance 3
 */
#define ICSS_INSTANCE_THREE      (3)

/**
 * @brief  Function to route ICSS events to R5F using interrupt mux
 *
 * NOTE: Only supported for AM65xx
 *
 * On certain devices, ICSS events are not directly mapped to the R5F core. So
 * we need to configure the interrupt mux to enable the routing of interrupts
 * from ICSS to processor.
 *
 * This function routes all the 8 events of ICSS to R5F.
 *
 * @param[in] icss_instance     ICSS instance for which the interrupts need to be routed
 * @param[out] interrupt_offset The interrupt index to be used by application for the first interrupt
 *
 */
extern uint32_t route_icss_interrupts_to_r5f(uint32_t icss_instance);

/**
 * @brief  Function to route I2C interrupt to R5F using interrupt mux
 *
 * NOTE: Only supported for AM65xx
 *
 * On certain devices, I2C interrupt not directly mapped to the R5F core. So
 * we need to configure the interrupt mux to enable the routing of interrupts
 * from ICSS to processor.
 *
 * This function routes the I2C interrupt to R5F.
 *
 * @param[out] interrupt_offset The interrupt index to be used by application for the I2C interrupt
 *
 */
extern uint32_t route_i2c_interrupts_to_r5f();

#endif /*INTERRUPTROUTE_H_*/
