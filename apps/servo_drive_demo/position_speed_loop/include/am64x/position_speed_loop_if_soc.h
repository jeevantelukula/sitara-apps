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

#ifndef _POSITION_SPEED_LOOP_IF_SOC_H_
#define _POSITION_SPEED_LOOP_IF_SOC_H_

/* 
 * Definitions for FSI interrupts 
 */
/* FSI Rx Int1 */
#define FSI_RX_INT1_INT_NUM         ( 16 ) 
#define FSI_RX_INT1_INT_TYPE        ( CSL_VIM_INTR_TYPE_LEVEL )
#define FSI_RX_INT1_INT_MAP         ( CSL_VIM_INTR_MAP_IRQ )
#define FSI_RX_INT1_INT_PRI         ( 0 ) /* 0(highest)..15(lowest) */

/* FSI Rx Int2 */
#define FSI_RX_INT2_INT_NUM         ( 17 )
#define FSI_RX_INT2_INT_TYPE        ( CSL_VIM_INTR_TYPE_LEVEL )
#define FSI_RX_INT2_INT_MAP         ( CSL_VIM_INTR_MAP_IRQ )
#define FSI_RX_INT2_INT_PRI         ( 0 ) /* 0(highest)..15(lowest) */
    
/* FSI Tx Int1 */
#define FSI_TX_INT1_INT_NUM         ( 28 )
#define FSI_TX_INT1_INT_TYPE        ( CSL_VIM_INTR_TYPE_LEVEL )
#define FSI_TX_INT1_INT_MAP         ( CSL_VIM_INTR_MAP_IRQ )
#define FSI_TX_INT1_INT_PRI         ( 0 ) /* 0(highest)..15(lowest) */

/* FSI Tx Int2 */
#define FSI_TX_INT2_INT_NUM         ( 29 )
#define FSI_TX_INT2_INT_TYPE        ( CSL_VIM_INTR_TYPE_LEVEL )
#define FSI_TX_INT2_INT_MAP         ( CSL_VIM_INTR_MAP_IRQ )
#define FSI_TX_INT2_INT_PRI         ( 0 ) /* 0(highest)..15(lowest) */

#endif /* _POSITION_SPEED_LOOP_IF_SOC_H_ */
