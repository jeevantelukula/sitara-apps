/*
 * Copyright (C) 2017-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef MULTI_AXIS_MASTER_LEAD_H
#define MULTI_AXIS_MASTER_LEAD_H

#include <ti/csl/tistdtypes.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_fsi_tx.h>
#include <ti/csl/csl_fsi_rx.h>
#include "multi_axis_fsi_shared.h"
#include <ti/csl/src/ip/fsi_tx/V0/cslr_fsi_tx.h>

//
//! \defgroup MASTER_DRIVE
//! @{
//

#define FSI_NODE_ACTIVE     FSI_SLAVE_N1

//
// Global variables for FSI
//
extern uint16_t fsiTxStatus;
extern uint16_t fsiRxStatus;

extern uint16_t fsiTxIndex;
extern uint16_t fsiRxIndex;
extern uint16_t fsiSlaveNodeFirst;
extern uint16_t fsiSlaveNodeLast;
extern uint16_t fsiSlaveNodeActive;
extern uint16_t fsiSlaveNodeReceived;
extern uint16_t fsiSlaveNodeNext;

extern uint16_t *fsiTxDataBufAddr;  // FSI TX data buffer address
extern uint16_t *fsiRxDataBufAddr;  // FSI RX data buffer address
extern uint16_t *fsiTxTagBufAddr;   // FSI TX Tag & user data buffer address
extern uint16_t *fsiRxTagBufAddr;   // FSI RX Tag & user data buffer address

extern uint16_t *fsiTxTagAddr;      // FSI TX frame tag and user data address
extern uint16_t *fsiRxTagAddr;      // FSI RX frame tag and user data address

extern uint16_t fsiTxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];   // TX data array
extern uint16_t fsiRxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];   // RX data array

// Frame tag used with Data/Ping transfers
extern FSI_FrameTag fsiFrameTag[FSI_NODE_NUM];
extern FSI_FrameTag fsiPingTag0;
extern FSI_FrameTag fsiPingTag1;

extern uint16_t fsiTxFrameTag;
extern uint16_t fsiRxFrameTag;

extern uint16_t fsiTxDataWords;
extern uint16_t fsiRxDataWords;

extern FSI_DataWidth fsiTxLanes;
extern FSI_DataWidth fsiRxLanes;

extern volatile uint16_t fsiTxUserData[FSI_NODE_NUM];
extern volatile uint16_t fsiRxUserData[FSI_NODE_NUM];

// User data to be sent with Data frame(for CPU control)
extern volatile uint16_t fsiTxUserDataTag;
extern volatile uint16_t fsiRxUserDataTag;

extern volatile FSI_HandShake_e fsiHandShakeState;

extern volatile FSI_TRxState_e fsiTxInt1Received;
extern volatile FSI_TRxState_e fsiRxInt1Received;

extern volatile uint32_t fsiTxInt2Received;
extern volatile uint32_t fsiRxInt2Received;

extern volatile uint32_t fsiError;

extern uint32_t fsiTxTimeOutCntr;
extern uint32_t fsiRxTimeOutCntr;

extern uint16_t fsiTxTimeWaitCntr;
extern uint16_t fsiTxTimeWaitSet;

extern uint32_t fsiTxFrameCntr;
extern uint32_t fsiRxFrameCntr;
extern uint32_t fsiFrameResetCntr;

extern uint16_t fsiStateFlag;

extern uint16_t frameDataRX[FSI_TSF_WORDS];
extern uint16_t frameDataTX[FSI_TSF_WORDS];

extern uint16_t fsienableCrcChk;

extern uint16_t dataCrcCalc;
extern uint16_t dataCrcRX;
extern uint16_t dataCrcTX;

//
// FSI initialize
//
extern void FSI_initParams(void);

//! \brief     Writes user defined data and frame tag for transmission
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] array is the address of the array of words to be transmitted.
//! \param[in] length is the number of words in the array to be transmitted.
static inline void
FSI_writeTxTagUserData(uint32_t base, uint16_t userDataTag)
{
    //HAL_Obj *obj = (HAL_Obj *)handle;
    //HWREGH(obj->fsiTxHandle + FSI_O_TX_FRAME_TAG_UDATA) = userDataTag;
    
    //HWREGH(base + FSI_O_TX_FRAME_TAG_UDATA) = userDataTag;
    HWREGH(base + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA) = userDataTag;
    
    return;
}

//! \brief     Writes data in FSI Tx buffer
//! \param[in] handle is the hardware abstraction layer (HAL) handle
//! \param[in] array is the address of the array of words to be transmitted.
//! \param[in] length is the number of words in the array to be transmitted.
static inline void
FSI_writeTxDataBuffer(uint32_t base, uint16_t *pTxData, uint16_t length)
{
    //HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t i;

    for(i = 0U; i < length; i++)
    {
        // Write one 16 bit word, increment buffer pointer
        //
        //HWREGH(obj->fsiTxHandle + FSI_O_TX_BUF_BASE(i)) = *(pTxData+i);
        //HWREGH(base + FSI_O_TX_BUF_BASE(i)) = *(pTxData+i);
        HWREGH(base + CSL_FSI_TX_CFG_TX_BUF_BASE(i)) = *(pTxData+i);
    }

    return;
}

//! \brief      Reads user defined data and frame tag from received
//! \param[in]  handle is the hardware abstraction layer (HAL) handle
static inline uint16_t FSI_readRxFrameTag(uint32_t base) // JR: change prototype
{
    //HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t userDataTag = HWREGH(base + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA);
    uint16_t frameTag = (userDataTag>>1) & 0x000F;

    return(frameTag);
}

//! \brief      Reads user defined data and frame tag from received
//! \param[in]  handle is the hardware abstraction layer (HAL) handle
static inline uint16_t FSI_readRxUserData(uint32_t base) // JR: change prototype
{
    //HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t userDataTag = HWREGH(base + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA);
    uint16_t userData = (userDataTag>>8) & 0x00FF;

    return(userData);
}

//! \brief      Reads data from FSI Rx buffer
//! \param[in]  handle is the hardware abstraction layer (HAL) handle
//! \param[out] array is the address of the array of words to receive the data
//! \param[in]  length is the number of words in the array to be received
static inline void
FSI_readRxDataBuffer(uint32_t base, uint16_t *pRxData, uint16_t length) // JR: change prototype
{
    //HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t i;

    for(i = 0U; i < length; i++)
    {
        //
        // Read one 16 bit word, increment buffer pointer
        //
        //*(pRxData+i) = HWREGH(obj->fsiRxHandle + FSI_O_RX_BUF_BASE(i));
        *(pRxData+i) = HWREGH(base + CSL_FSI_RX_CFG_RX_BUF_BASE(i));
    }

    return;
}

//
// read new data for frame received
//
static inline void FSI_readRxFrameData(uint32_t base) // JR: change prototype
{
    //
    // Reads user defined data and frame tag from received
    //
    fsiRxFrameTag = FSI_readRxFrameTag(base);

    if(fsiRxFrameTag == fsiFrameTag[fsiSlaveNodeActive])
    {
        fsiSlaveNodeReceived = fsiSlaveNodeActive;
        fsiTxTimeWaitCntr = 0;
    }
    else
    {
        fsiSlaveNodeReceived = fsiRxFrameTag - FSI_NODE_TAG_OFFSET;
    }

    fsiSlaveNodeActive++;

    if(fsiSlaveNodeActive > fsiSlaveNodeLast)
    {
        fsiSlaveNodeActive = fsiSlaveNodeFirst;
    }

    fsiSlaveNodeNext = fsiSlaveNodeActive + 1;

    if(fsiSlaveNodeNext > fsiSlaveNodeLast)
    {
        fsiSlaveNodeNext = fsiSlaveNodeFirst;
    }

    fsiRxUserData[fsiSlaveNodeReceived] = FSI_readRxUserData(base);

    fsiTxUserDataTag = ((fsiTxUserData[fsiSlaveNodeActive]<<8) & 0xFF00) +
                       fsiFrameTag[fsiSlaveNodeActive];

    fsiRxDataBufAddr = (uint16_t *)(&fsiRxDataBuf[fsiSlaveNodeReceived][0]);
    fsiTxDataBufAddr = (uint16_t *)(&fsiTxDataBuf[fsiSlaveNodeActive][0]);

    // read RX data buffer
    FSI_readRxDataBuffer(base, fsiRxDataBufAddr, fsiRxDataWords);
}

//
// FSI handshake
//
extern void FSI_handshakeLead(uint32_t gFsiTxBase, uint32_t gFsiRxBase);

//
// setup Tx/Rx frame data
//
extern void FSI_setupTRxFrameData(uint32_t gFsiTxBase, uint32_t gFsiRxBase);

//
// update the received data
//
extern void FSI_updateReceivedData(void);

//
// update the transmission data
//
extern void FSI_updateTransmissionData(void);

//
// Close the Doxygen group.
//! @} //defgroup MASTER_DRIVE
//

#endif  // end of MULTI_AXIS_MASTER_LEAD_H definition

