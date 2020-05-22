;
; Copyright (c) 2019-2020 Texas Instruments Incorporated
; All rights reserved.
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;
;  *  Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;
;  *  Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;
;  *  Neither the name of Texas Instruments Incorporated nor the names of
;     its contributors may be used to endorse or promote products derived
;     from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
;  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
;  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
;  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
;  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
;  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
;  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
;  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
;  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;

        .include "timesync.h"

        .def    TIMESYNC_ENTRY  ; global entry point

        .sect   ".text"
        .retain ".text"
        .retainrefs ".text"
TIMESYNC_ENTRY:
        ; Clear registers R0-R30
        ZERO    &R0, 124

;
; Get IEPx TS global enable
;
get_iepx_ts_gbl_en:
        ; Load TR0.b0 <- FW_REG_TS_CTRL
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CTRL, FW_REG_TS_CTRL_SZ
        ; Mask all but IEP0 & IEP1 global enable bits
        AND     TREG0.b0, TREG0.b0, (TS_CTRL_IEP0_TS_GBL_EN_MASK | TS_CTRL_IEP1_TS_GBL_EN_MASK)

;
; Determine operating mode
;
;       QBBC    iep_gbl_en_x0, TREG0.b0, IEP0_TS_GBL_EN_LSBN
iep_gbl_en_x1:
;       QBBC    iep_gbl_en_01, TREG0.b0, IEP1_TS_GBL_EN_LSBN
iep_gbl_en_11:
        ; IEP1_TS_GBL_EN==1 && IEP0_TS_GBL_EN==1: IEP0 & 1 TSs enable, continuous clear of IEP0 & 1 CMP1-12
;       LDI     TREG1.w0, $CODE(tsCmpxClearIep0_1)
;       QBA     set_ack_flags
iep_gbl_en_01:
        ; IEP1_TS_GBL_EN==0 && IEP0_TS_GBL_EN==1: IEP0 TSs enable, continuous clear of IEP0 CMP1-12
        LDI     TREG1.w0, $CODE(tsIep0MainLoop)
        QBA     set_ack_flags
iep_gbl_en_x0:
        QBBC    iep_gbl_en_00, TREG0.b0, IEP1_TS_GBL_EN_LSBN
iep_gbl_en_10:
        ; IEP1_TS_GBL_EN==1 && IEP0_TS_GBL_EN==0: IEP1 TSs enable, continuous clear of IEP0 CMP1-12
;       LDI     TREG1.w0, $CODE(tsCmpxClearIep1)
;       QBA     set_ack_flags
iep_gbl_en_00:
        ; IEP1_TS_GBL_EN==0 && IEP0_TS_GBL_EN==0: no IEP TSs enabled, drop into exit loop
        LDI     TREG1.w0, $CODE(exitLoop)

;
; Set IEPx global enable ACK & FW initialization flag
;

set_ack_flags:
        ; Set FW initialization flag
        SET     TREG0.b0, TREG0.b0, FW_INIT_LSBN
        ; Store FW_REG_TS_CTRL -> TR0.b0
        SBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_STAT, FW_REG_TS_STAT_SZ

;
; Enter operating mode
;
        JMP     TREG1.w0

;
; Continuous clear IEP0 TS CMPx
;
tsIep0MainLoop:
; Put -1 in NEG1
        XOR     NEG1, NEG1, NEG1
        SUB     NEG1, NEG1, 1
        XOR        MASK, MASK, MASK

; Set up CMP1 for test purposes
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP1_COUNT, 4
        QBEQ    iep0NoCmp1, TREG0, 0
        SET     MASK, MASK, 1
        LBCO    &TREG1.b0, CT_PRU_ICSSG_IEP0, ICSSG_COUNT_REG0, 8
        ADD     TREG1, TREG1, TREG0     ; COUNT_LO + PERIOD
        ADC     TREG2, TREG2, 0         ; Carry the 1
        MOV     LAST_CMP1_LO, TREG1     ; stash it
        MOV     LAST_CMP1_HI, TREG2     ; stash it
        SBCO    &TREG1.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP1_REG0, 8

iep0NoCmp1:
; Now init CMP3
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP3_COUNT, 4
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP3_OFFSET, 4
        QBEQ    iep0NoCmp3Init, TREG0, 0
        SET     MASK, MASK, 3
        ADD     LAST_CMP3_LO, LAST_CMP1_LO, TREG0     ; COUNT_LO + PERIOD
        ADC     LAST_CMP3_HI, LAST_CMP1_HI, 0         ; Carry the 1
        LDI     TREG0, 0                              ; Assume no sign extend
        QBBC    iep0Cmp3NoSE, TREG1, 31
        MOV     TREG0, NEG1                           ; Sign extend needed
iep0Cmp3NoSE:
        ADD     LAST_CMP3_LO, LAST_CMP3_LO, TREG1     ; COUNT_LO + OFFSET
        ADC     LAST_CMP3_HI, LAST_CMP3_HI, TREG0     ; Carry the 1 with sign extension
        SBCO    &LAST_CMP3_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP3_REG0, 8

iep0NoCmp3Init:
; Now init CMP4
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP4_COUNT, 4
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP4_OFFSET, 4
        QBEQ    iep0NoCmp4Init, TREG0, 0
        SET     MASK, MASK, 4
        ADD     LAST_CMP4_LO, LAST_CMP1_LO, TREG0     ; COUNT_LO + PERIOD
        ADC     LAST_CMP4_HI, LAST_CMP1_HI, 0         ; Carry the 1
        LDI     TREG0, 0                              ; Assume no sign extend
        QBBC    iep0Cmp4NoSE, TREG1, 31
        MOV     TREG0, NEG1                           ; Sign extend needed
iep0Cmp4NoSE:
        ADD     LAST_CMP4_LO, LAST_CMP4_LO, TREG1     ; COUNT_LO + OFFSET
        ADC     LAST_CMP4_HI, LAST_CMP1_HI, TREG0     ; Carry the 1 with sign extension
        SBCO    &LAST_CMP4_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 8

iep0NoCmp4Init:
; Now init CMP5
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP5_COUNT, 4
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP5_OFFSET, 4
        QBEQ    iep0NoCmp5Init, TREG0, 0
        SET     MASK, MASK, 5
        ADD     LAST_CMP5_LO, LAST_CMP1_LO, TREG0     ; COUNT_LO + PERIOD
        ADC     LAST_CMP5_HI, LAST_CMP1_HI, 0         ; Carry the 1
        LDI     TREG0, 0                              ; Assume no sign extend
        QBBC    iep0Cmp5NoSE, TREG1, 31
        MOV     TREG0, NEG1                           ; Sign extend needed
iep0Cmp5NoSE:
        ADD     LAST_CMP5_LO, LAST_CMP5_LO, TREG1     ; COUNT_LO + OFFSET
        ADC     LAST_CMP5_HI, LAST_CMP5_HI, TREG0     ; Carry the 1 with sign extension
        SBCO    &LAST_CMP5_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP5_REG0, 8

iep0NoCmp5Init:
; Now init CMP6
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP6_COUNT, 4
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP6_OFFSET, 4
        QBEQ    iep0NoCmp6Init, TREG0, 0
        SET     MASK, MASK, 6
        ADD     LAST_CMP6_LO, LAST_CMP1_LO, TREG0     ; COUNT_LO + PERIOD
        ADC     LAST_CMP6_HI, LAST_CMP1_HI, 0         ; Carry the 1
        LDI     TREG0, 0                              ; Assume no sign extend
        QBBC    iep0Cmp6NoSE, TREG1, 31
        MOV     TREG0, NEG1                           ; Sign extend needed
iep0Cmp6NoSE:
        ADD     LAST_CMP6_LO, LAST_CMP6_LO, TREG1     ; COUNT_LO + OFFSET
        ADC     LAST_CMP6_HI, LAST_CMP6_HI, TREG0     ; Carry the 1 with sign extension
        SBCO    &LAST_CMP6_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP6_REG0, 8

iep0NoCmp6Init:
; Clear pending events on CMP1, 3,4,5,6
        SBCO    &MASK.b0, CT_PRU_ICSSG_IEP0, ICSSG_CMP_STATUS_REG, 4
; Enable compares
        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_CMP_CFG_REG, 4
        LSL	TREG1, MASK, 1
        OR      TREG0, TREG1, TREG0
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_CMP_CFG_REG, 4


iep0Loop:

; Read status
        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_CMP_STATUS_REG, 4
        AND     TREG0, TREG0, 0x79 ; only keep CMP1,3,4,5,6

; Reload CMP1
        QBBC    iep0NoTrgCmp1, TREG0, 1
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP1_COUNT, 4
        ADD     LAST_CMP1_LO, LAST_CMP1_LO, TREG1     ; COUNT_LO + PERIOD
        ADC     LAST_CMP1_HI, LAST_CMP1_HI, 0         ; Carry the 1
        SBCO    &LAST_CMP1_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP1_REG0, 8

; Reload CMP3
iep0NoTrgCmp1:
        QBBC    iep0NoTrgCmp3, TREG0, 3
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP3_COUNT, 4
        ADD     LAST_CMP3_LO, LAST_CMP3_LO, TREG1     ; COUNT_LO + PERIOD
        ADC     LAST_CMP3_HI, LAST_CMP3_HI, 0         ; Carry the 1
        SBCO    &LAST_CMP3_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP3_REG0, 8

; Reload CMP4
iep0NoTrgCmp3:
        QBBC    iep0NoTrgCmp4, TREG0, 4
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP4_COUNT, 4
        ADD     LAST_CMP4_LO, LAST_CMP4_LO, TREG1     ; COUNT_LO + PERIOD
        ADC     LAST_CMP4_HI, LAST_CMP4_HI, 0         ; Carry the 1
        SBCO    &LAST_CMP4_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 8

; Reload CMP5
iep0NoTrgCmp4:
        QBBC    iep0NoTrgCmp5, TREG0, 5
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP5_COUNT, 4
        ADD     LAST_CMP5_LO, LAST_CMP5_LO, TREG1     ; COUNT_LO + PERIOD
        ADC     LAST_CMP5_HI, LAST_CMP5_HI, 0         ; Carry the 1
        SBCO    &LAST_CMP5_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP5_REG0, 8

; Reload CMP6
iep0NoTrgCmp5:
        QBBC    iep0NoTrgCmp6, TREG0, 6
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_TS_CMP6_COUNT, 4
        ADD     LAST_CMP6_LO, LAST_CMP6_LO, TREG1     ; COUNT_LO + PERIOD
        ADC     LAST_CMP6_HI, LAST_CMP6_HI, 0         ; Carry the 1
        SBCO    &LAST_CMP6_LO.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP6_REG0, 8

iep0NoTrgCmp6:
; Clear status
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_CMP_STATUS_REG, 4
        QBA     iep0Loop

;
; Exit loop
;
exitLoop:
        JMP     exitLoop
