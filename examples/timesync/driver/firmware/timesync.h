;
; Copyright (c) 2019-2020, Texas Instruments Incorporated
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

        .if !$defined("__timesync_h")
__timesync_h    .set    1

        .cdecls C,NOLIST
%{
        #include "icssg_timesync.h"
%}

*        .include "icss_regs.h"
*         .include "icss_event_defs.h"
*
* ; ICSSM/L -> ICSSG, ICSSM macro
* ; PRU_ICSS_INTC_HIPIR0      -> ICSSG_PRI_HINT_REG0,         ICSS_INTC_HIPIR0
* ; PRU_ICSS_INTC_SICR        -> ICSSG_STATUS_CLR_INDEX_REG,  ICSS_INTC_SICR
* ; PRU_ICSS_IEP_TMR_CMP_STS  -> ICSSG_CMP_STATUS_REG,        ICSS_IEP_CMP_STATUS_REG
* ;
*
*
* ;
* ; Substitution symbols
* ;
*         .asg    C0, CT_PRU_ICSSG_INTC           ; Constant Able, PRU_ICSSG INTC
        .asg    C1, CT_PRU_ICSSG_IEP1           ; Constant Table, PRU_ICSSG IEP1
        .asg    C11, CT_PRU_ICSSG_CTRL          ; Constant Table, PRU Control
        .asg    C24, CT_PRU_ICSSG_LOC_DMEM      ; Constant Table, local PRU DMEM
        .asg    C26, CT_PRU_ICSSG_IEP0          ; Constant Table, PRU_ICSSG IEP0

        .asg    R0, TREG0                       ; temporary register 0
        .asg    R1, TREG1                       ; temporary register 1
        .asg    R2, TREG2                       ; temporary register 2
        .asg    R3, TREG3                       ; temporary register 3

        .asg    R4, LAST_LATCH_CMP1_LO          ; Last seen value of CMP1
        .asg    R5, LAST_LATCH_CMP1_HI

        .asg    R6, LAST_CMP3_LO                ; Last set value of CMP3
        .asg    R7, LAST_CMP3_HI

        .asg    R8, LAST_CMP4_LO                ; Last set value of CMP4
        .asg    R9, LAST_CMP4_HI

        .asg    R10, LAST_CMP5_LO                ; Last set value of CMP5
        .asg    R11, LAST_CMP5_HI

        .asg    R12, LAST_CMP6_LO                ; Last set value of CMP6
        .asg    R13, LAST_CMP6_HI

        .asg    R14, LAST_CMP1_LO                ; Last set value of CMP1
        .asg    R15, LAST_CMP1_HI

        .asg    R16, NEG1                        ; Contains -1
        .asg    R17, MASK                        ; Mask of enabled compares

; Symbolic constant for FW registers
;

; FW_REG_TS_CTRL & FW_REG_TS_STAT
IEP0_TS_GBL_EN_LSBN            .set TS_CTRL_IEP0_TS_GBL_EN_SHIFT ; IEP0 TS Global Enable bit number
IEP1_TS_GBL_EN_LSBN            .set TS_CTRL_IEP1_TS_GBL_EN_SHIFT ; IEP1 TS Global Enable bit number
FW_INIT_LSBN                    .set TS_STAT_FW_INIT_SHIFT         ; FW Initialized flag bit number

;
; Symbolic constants for HW registers
;

; ICSSG INTC events
;
; R31 event interface mapping, add pru<n>_r31_vec_valid to system event number

* ; PRU_ICSSG_INTC registers
* ;
* ICSS_INTC_HIPIR                 .set ICSS_INTC_HIPIR1   ; using Host Interrupt 1
* NONE_HINT_BIT                   .set 31                 ; ICSSG_PRI_HINT_REG:NONE_HINT_0 bit number
*
; PRU_ICSSG PRU_CTRL registers
;
; ICSSG_PRU_CTBIR0:C24_BLK_INDEX, PRU Constant Entry C24 Block Index
C24_BLK_INDEX_FW_REGS_VAL       .set 0

; PRU_ICSSG_IEP registers
;
* ICSSG_GLOBAL_CFG_REG            .set 0x0000 ; Global Configuration Register
ICSSG_COUNT_REG0                .set 0x0010 ; 64-bit Count Value Low Register
* ICSSG_COUNT_REG1                .set 0x0014 ; 64-bit Count Value High Register
ICSSG_CMP_CFG_REG               .set 0x0070 ; Compare Configuration Register
ICSSG_CMP_STATUS_REG            .set 0x0074 ; Compare Status Register
* ICSSG_IEP_CMP0_REG0             .set 0x0078 ; Compare 0 Low Register
* ICSSG_IEP_CMP0_REG1             .set 0x007C ; Compare 0 High Register
ICSSG_IEP_CMP1_REG0             .set 0x0080 ; Compare 1 Low Register
* ICSSG_IEP_CMP1_REG1             .set 0x0084 ; Compare 1 High Register
* ICSSG_IEP_CMP2_REG0             .set 0x0088 ; Compare 2 Low Register
* ICSSG_IEP_CMP2_REG1             .set 0x008C ; Compare 2 High Register
ICSSG_IEP_CMP3_REG0             .set 0x0090 ; Compare 3 Low Register
* ICSSG_IEP_CMP3_REG1             .set 0x0094 ; Compare 3 High Register
ICSSG_IEP_CMP4_REG0             .set 0x0098 ; Compare 4 Low Register
* ICSSG_IEP_CMP4_REG1             .set 0x009C ; Compare 4 High Register
ICSSG_IEP_CMP5_REG0             .set 0x00A0 ; Compare 5 Low Register
* ICSSG_IEP_CMP5_REG1             .set 0x00A4 ; Compare 5 High Register
ICSSG_IEP_CMP6_REG0             .set 0x00A8 ; Compare 6 Low Register
* ICSSG_IEP_CMP6_REG1             .set 0x00AC ; Compare 6 High Register
* ICSSG_IEP_CMP7_REG0             .set 0x00B0 ; Compare 7 Low Register
* ICSSG_IEP_CMP7_REG1             .set 0x00B4 ; Compare 7 High Register

* ; ICSSG_CMP_CFG_REG:CMP_EN
* CMP_EN_SHIFT                    .set 1
* CMP_EN_CMP1_BN                  .set 1<<CMP_EN_SHIFT
* ; ICSSG_CMP_CFG_REG:CNT_ENABLE
* CNT_ENABLE_BN                   .set 0
* ; ICSSG_CMP_STATUS_REG:CMP_STATUS
* CMP_STATUS_CMP1_BN              .set 1

;
; Misc symbolic constants
;
* ; IEP used for timing synchronization
* CT_PRU_ICSSG_IEP                .set CT_PRU_ICSSG_IEP0
*


    .endif  ; __timesync_h
