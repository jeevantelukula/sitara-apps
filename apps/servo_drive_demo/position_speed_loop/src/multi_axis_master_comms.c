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

#include <ti/csl/tistdtypes.h>
#include <ti/csl/csl_fsi_tx.h>
#include <ti/csl/csl_fsi_rx.h>
#include "multi_axis_fsi_shared.h"
#include "multi_axis_master_ctrl_main.h"
#include "app_psl_mbxipc.h"

extern uint32_t gFsiTxBase;
extern uint32_t gFsiRxBase;

uint16_t fsiTxIndex;
uint16_t fsiRxIndex;
uint16_t fsiSlaveNodeFirst;
uint16_t fsiSlaveNodeLast;
uint16_t fsiSlaveNodeActive;
uint16_t fsiSlaveNodeReceived;
uint16_t fsiSlaveNodeNext;

uint16_t *fsiTxDataBufAddr;     /* FSI TX data buffer address */
uint16_t *fsiRxDataBufAddr;     /* FSI RX data buffer address */
uint16_t *fsiTxTagBufAddr;      /* FSI TX Tag & user data buffer address */
uint16_t *fsiRxTagBufAddr;      /* FSI RX Tag & user data buffer address */

uint16_t *fsiTxTagAddr;         /* FSI TX frame tag and user data address */
uint16_t *fsiRxTagAddr;         /* FSI RX frame tag and user data address */

uint16_t fsiTxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];      /* FSI TX data array */
uint16_t fsiRxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];      /* FSI RX data array */

/* Frame tag used with Data/Ping transfers */
FSI_FrameTag fsiFrameTag[FSI_NODE_NUM];     /* FSI frame Tag array for ping */
FSI_FrameTag fsiPingTag0;
FSI_FrameTag fsiPingTag1;

uint16_t fsiTxFrameTag;
uint16_t fsiRxFrameTag;

uint16_t fsiTxDataWords;
uint16_t fsiRxDataWords;

FSI_DataWidth fsiTxLanes;
FSI_DataWidth fsiRxLanes;

volatile uint16_t fsiTxUserData[FSI_NODE_NUM];
volatile uint16_t fsiRxUserData[FSI_NODE_NUM];

/* User data to be sent with Data frame(for CPU control) */
volatile uint16_t fsiTxUserDataTag;
volatile uint16_t fsiRxUserDataTag;

volatile FSI_HandShake_e fsiHandShakeState;

volatile FSI_TRxState_e fsiTxInt1Received;
volatile FSI_TRxState_e fsiRxInt1Received;

volatile uint32_t fsiTxInt2Received;
volatile uint32_t fsiRxInt2Received;

volatile uint32_t fsiError;

uint32_t fsiTxTimeOutCntr;
uint32_t fsiRxTimeOutCntr;
uint16_t fsiTxTimeWaitCntr;
uint16_t fsiTxTimeWaitSet;

uint32_t fsiTxFrameCntr;
uint32_t fsiRxFrameCntr;
uint32_t fsiFrameResetCntr;

uint16_t fsiStateFlag;

uint16_t frameDataRX[FSI_TSF_WORDS];
uint16_t frameDataTX[FSI_TSF_WORDS];

uint16_t fsienableCrcChk;

uint16_t dataCrcCalc;
uint16_t dataCrcRX;
uint16_t dataCrcTX;

/* FSI initialize */
void FSI_initParams(void)
{
    fsiTxIndex = 0;
    fsiRxIndex = 0;

    fsiSlaveNodeFirst = FSI_NODE_FIRST;
    fsiSlaveNodeLast = FSI_NODE_LAST;
    fsiSlaveNodeActive = FSI_NODE_ACTIVE;

    fsiTxDataWords = FSI_TX_WORDS;
    fsiRxDataWords = FSI_RX_WORDS;
    fsiTxLanes = (FSI_DataWidth)FSI_TX_LANES;
    fsiRxLanes = (FSI_DataWidth)FSI_RX_LANES;

    fsiError = 0;
    fsiTxInt2Received = 0;
    fsiRxInt2Received = 0;

    fsienableCrcChk = 0;

    fsiTxFrameCntr = 0;
    fsiRxFrameCntr = 0;
    fsiFrameResetCntr = 0;

    fsiStateFlag = 0;

    fsiFrameTag[FSI_SLAVE_N1] = FSI_FRAME_TAG_NODE1;
    fsiFrameTag[FSI_SLAVE_N2] = FSI_FRAME_TAG_NODE2;
    fsiFrameTag[FSI_SLAVE_N3] = FSI_FRAME_TAG_NODE3;

    fsiPingTag0 = FSI_FRAME_TAG0;
    fsiPingTag1 = FSI_FRAME_TAG1;

    fsiRxFrameTag = fsiFrameTag[fsiSlaveNodeActive];
    fsiTxFrameTag = fsiFrameTag[fsiSlaveNodeActive];

    fsiTxUserDataTag = fsiTxUserData[fsiSlaveNodeActive] + fsiFrameTag[fsiSlaveNodeActive];
    fsiRxUserDataTag = fsiRxUserData[fsiSlaveNodeActive] + fsiFrameTag[fsiSlaveNodeActive];

#if FSI_LOOPBACK
    FSI_enableRxInternalLoopback(gFsiRxBase);
#endif
    FSI_performTxInitialization(gFsiTxBase, PRESCALER_VAL);
    FSI_performRxInitialization(gFsiRxBase);

    fsiTxDataBufAddr = (uint16_t *)(&fsiTxDataBuf[0][0]);
    fsiRxDataBufAddr = (uint16_t *)(&fsiRxDataBuf[0][0]);
    fsiTxTagBufAddr  = (uint16_t *)(&fsiTxUserDataTag);
    fsiRxTagBufAddr  = (uint16_t *)(&fsiRxUserDataTag);

    fsiTxTagAddr  = (uint16_t *)(gFsiTxBase + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA);
    fsiRxTagAddr  = (uint16_t *)(gFsiRxBase + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA);

    return;
}

/* FSI handshake */
void FSI_handshakeLead(uint32_t gFsiTxBase, uint32_t gFsiRxBase)
{
    extern volatile uint8_t numPingFrames;

    /* Allow PING frame event to generate RX INT1 */
    FSI_enableRxInterrupt(gFsiRxBase, FSI_INT1, FSI_RX_EVT_PING_FRAME);

    /* Transmit flush sequence */
    FSI_executeTxFlushSequence(gFsiTxBase, PRESCALER_VAL);

    /* Send PING0 frame */
    FSI_setTxFrameTag(gFsiTxBase, FSI_FRAME_TAG0);
    FSI_setTxFrameType(gFsiTxBase, FSI_FRAME_TYPE_PING);
    FSI_startTxTransmit(gFsiTxBase);

    /* Wait to receive a PING0 frame */
    while(numPingFrames == 0);

    /* Send PING1 frame */
    FSI_setTxFrameTag(gFsiTxBase, FSI_FRAME_TAG1);
    FSI_setTxFrameType(gFsiTxBase, FSI_FRAME_TYPE_PING);
    FSI_startTxTransmit(gFsiTxBase);

    /* Wait to receive a PING1 frame */
    while(numPingFrames == 1);

    return;
}

/* setup Tx/Rx frame data */
void FSI_setupTRxFrameData(uint32_t gFsiTxBase, uint32_t gFsiRxBase)
{
    /* Setting the requested nWords and nLanes for application */
    FSI_setTxFrameType(gFsiTxBase, FSI_FRAME_TYPE_NWORD_DATA);
    FSI_setTxSoftwareFrameSize(gFsiTxBase, fsiTxDataWords);
    FSI_setTxDataWidth(gFsiTxBase, (FSI_DataWidth)FSI_TX_LANES);

    FSI_setRxSoftwareFrameSize(gFsiRxBase, fsiRxDataWords);
    FSI_setRxDataWidth(gFsiRxBase, (FSI_DataWidth)FSI_RX_LANES);

    /* Set RX INT2 to CRC, EOF, and TYPE errors */
    FSI_enableRxInterrupt(gFsiRxBase, FSI_INT2, (FSI_RX_EVT_CRC_ERR | FSI_RX_EVT_EOF_ERR | FSI_RX_EVT_TYPE_ERR));
    /* Set RX INT1 to only data frame events (no interrupt on PING) */
    FSI_disableRxInterrupt(gFsiRxBase, FSI_INT1, FSI_RX_EVT_PING_FRAME);
    FSI_enableRxInterrupt(gFsiRxBase, FSI_INT1, FSI_RX_EVT_DATA_FRAME);
    /* Set TX INT1 to FRAME_DONE events */
    FSI_enableTxInterrupt(gFsiTxBase, FSI_INT1, FSI_TX_EVT_FRAME_DONE);

    /* Set the TX and RX buffer pointers to 0 */
    FSI_setTxBufferPtr(gFsiTxBase, 0);
    FSI_setRxBufferPtr(gFsiRxBase, 0);
}

/* update the received data */
void FSI_updateReceivedData(void)
{
    int32_t tempData;
    uint16_t fsiNode, ctrlNode;
    uint16_t ni;
    uint16_t dataType;

    /*
    //DINT;          // Disable Global interrupt INTM
    */

    fsiNode = fsiSlaveNodeReceived;
    ctrlNode = fsiNode + 1;
    dataCrcRX = fsiRxUserData[fsiNode];

    for(ni = 0; ni< FSI_RX_WORDS; ni++)
    {
        frameDataRX[ni] = fsiRxDataBuf[fsiNode][ni];
    }

    /*
    //EINT;          // Enable Global interrupt INTM
    */

    if(((frameDataRX[0]>>12) & 0x000F) != fsiFrameTag[fsiNode])
    {
        return;
    }

    if(fsienableCrcChk == 0)
    {
        dataCrcCalc = FSI_USERTAG_CHK - fsiFrameTag[fsiNode];
    }
    else
    {
        #ifdef SW_CRC_EN
        dataCrcCalc = getCRC8Value(&frameDataRX[0], FSI_RX_WORDS);
        #else
        dataCrcCalc = getChkSumValue(&frameDataRX[0], FSI_RX_WORDS);
        #endif
    }

    if(dataCrcRX != dataCrcCalc)
    {
        return;
    }

    dataType = (frameDataRX[0]>>8) & 0x000F;

    switch(dataType)
    {
        case FSI_UDATA_PS_SP_N:
            ctrlVars[ctrlNode].ctrlStateFdb = (CtrlState_e)(frameDataRX[0] & 0x00FF);

            tempData = (int32_t)frameDataRX[1];
            tempData = (tempData<<16) + frameDataRX[2];
            ctrlVars[ctrlNode].speedWe = FSI_convertPUToFloat(tempData);

            tempData = (int32_t)frameDataRX[3];
            tempData = (tempData<<16) + frameDataRX[4];
            ctrlVars[ctrlNode].posMechTheta = FSI_convertPUToFloat(tempData);

            #if(BUILDLEVEL == FCL_LEVEL5) /* Verify FSI */
            if(fabsf(ctrlVars[ctrlNode].speedWe -  ctrlVars[ctrlNode].speedWePrev) > ctrlVars[ctrlNode].speedWeDelta)
            {
                ctrlVars[ctrlNode].speedWeError = ctrlVars[ctrlNode].speedWe;
            }

            if(fabsf(ctrlVars[ctrlNode].posMechTheta - ctrlVars[ctrlNode].posMechThetaPrev) > ctrlVars[ctrlNode].posMechThetaDelta)
            {
                ctrlVars[ctrlNode].posMechThetaError = ctrlVars[ctrlNode].posMechTheta;
            }
            #else
            if(fabsf(ctrlVars[ctrlNode].speedWe - ctrlVars[ctrlNode].speedWePrev) > ctrlVars[ctrlNode].speedWeDelta)
            {
                ctrlVars[ctrlNode].speedWeError = ctrlVars[ctrlNode].speedWe;
                ctrlVars[ctrlNode].speedWe = ctrlVars[ctrlNode].speedWe * 0.4 + ctrlVars[ctrlNode].speedWePrev * 0.6;
            }

            ctrlVars[ctrlNode].speedWePrev = ctrlVars[ctrlNode].speedWe;

            if(fabsf(ctrlVars[ctrlNode].posMechTheta - ctrlVars[ctrlNode].posMechThetaPrev) > ctrlVars[ctrlNode].posMechThetaDelta)
            {
                ctrlVars[ctrlNode].posMechThetaError = ctrlVars[ctrlNode].posMechTheta;
            }
            #endif  /* (BUILDLEVEL != FCL_LEVEL5) */

            /* Inform background task to transmit latest actual values to EtherCAT */
            gAppPslTxMsgAxes[fsiSlaveNodeReceived].isMsgSend = 1;

            break;

        case FSI_UDATA_PS_SP_F:

            break;

        case FSI_UDATA_IS_FDB:

            break;

        case FSI_UDATA_TQ_POW:

            break;

        case FSI_UDATA_DV_MDT:

            break;
    }

    return;
}

/* update the transmission data */
void FSI_updateTransmissionData(void)
{
    int32_t tempData;
    uint16_t fsiNode, ctrlNode;
    uint16_t ni;
    uint16_t dataType;

    fsiNode = fsiSlaveNodeActive;
    ctrlNode = fsiNode + 1;

    dataType = FSI_UDATA_IS_REF;
    frameDataTX[0]  = (fsiFrameTag[fsiNode]<<12) & 0xF000;
    frameDataTX[0] += (dataType<<8) &0x0F00;

    switch(dataType)
    {
        case FSI_UDATA_IS_REF:
            frameDataTX[0] += ctrlVars[ctrlNode].ctrlStateCom & 0x00FF;

            tempData = FSI_convertFloatToPU(ctrlVars[ctrlNode].IqRef);
#if FSI_LOOPBACK
            /* if LOOPBACK just send the target speed reference to get looped back as actual */
            tempData = FSI_floatToPU(ctrlVars[ctrlNode].speedSet);
#endif
            frameDataTX[1] = (uint16_t)((tempData>>16) & 0x0000FFFF);
            frameDataTX[2] = (uint16_t)(tempData & 0x0000FFFF);

            tempData = FSI_convertFloatToPU(ctrlVars[ctrlNode].IdRef);
#if FSI_LOOPBACK
            /* if LOOPBACK just send the target position reference to get looped back as actual */
            tempData = FSI_floatToPU(ctrlVars[ctrlNode].positionSet);
#endif
            frameDataTX[3] = (uint16_t)((tempData>>16) & 0x0000FFFF);
            frameDataTX[4] = (uint16_t)(tempData & 0x0000FFFF);
            break;

        case FSI_UDATA_GAIN_ID:

            break;

        case FSI_UDATA_GAIN_IQ:

            break;

        case FSI_UDATA_UMN_ID:

            break;

        case FSI_UDATA_UMN_IQ:

            break;
    }

    if(fsienableCrcChk == 0)
    {
        dataCrcTX = FSI_USERTAG_CHK - fsiFrameTag[fsiNode];
    }
    else
    {
        #ifdef SW_CRC_EN
        dataCrcTX = getCRC8Value(&frameDataTX[0], FSI_TX_WORDS);
        #else
        dataCrcTX = getChkSumValue(&frameDataTX[0], FSI_TX_WORDS);
        #endif
    }

    /*
    //DINT;          // Disable Global interrupt INTM
    */
    fsiTxUserData[fsiNode] = dataCrcTX;

    fsiTxUserDataTag = (dataCrcTX << 8) | fsiFrameTag[fsiNode];

    for(ni = 0; ni< FSI_TX_WORDS; ni++)
    {
        fsiTxDataBuf[fsiNode][ni] = frameDataTX[ni];
    }
    /*
    //EINT;          // Enable Global interrupt INTM
    */
    return;
}
