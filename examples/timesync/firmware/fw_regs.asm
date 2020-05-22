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

        .cdecls C,NOLIST
%{
        #include "icssg_timesync.h"
%}

; Default FW register values
DEF_FW_MAGIC_NUMBER             .set    ( 0x4D575074 )  ; FwMagicNumber - TS
DEF_FW_TYPE                     .set    ( 0x00000000 )  ; FwType - TBD
DEF_FW_VERSION                  .set    ( 0x00000000 )  ; FwVersion - TBD
DEF_FW_FEATURE                  .set    ( 0x00000000 )  ; FwFeature - TBD
DEF_FW_EXTENDED_FEATURE         .set    ( 0x00000000 )  ; FwExtendedFeature - Reserved for future use

; IEP0_TS_GBL_EN==0 => disabled, IEP1_TS_GBL_EN==0 => disabled
DEF_TS_CTRL                    .set    (1b<<1) | (1b<<0)
DEF_TS_STAT                    .set    (0b<<2) | (0b<<1) | (0b<<0)

                                ; Firmware Registers
                                .sect   ".fwRegs"
                                .retain ".fwRegs"
                                .retainrefs ".fwRegs"
                                .space  ICSSG_TS_FW_MAGIC_NUMBER_ADDR
                                .word   DEF_FW_MAGIC_NUMBER
                                .word   DEF_FW_TYPE
                                .word   DEF_FW_VERSION
                                .word   DEF_FW_FEATURE
                                .word   DEF_FW_EXTENDED_FEATURE
                                .word   DEF_TS_CTRL
                                .word   DEF_TS_STAT
