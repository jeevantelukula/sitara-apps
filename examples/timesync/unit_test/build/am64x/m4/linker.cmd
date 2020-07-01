/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
/* Linker Settings */
--retain="*(.bootCode)"
--retain="*(.startupCode)"
--retain="*(.startupData)"
--retain="*(.intvecs)"
--retain="*(.intc_text)"
--retain="*(.rstvectors)"
--retain="*(.irqStack)"
--retain="*(.fiqStack)"
--retain="*(.abortStack)"
--retain="*(.undStack)"
--retain="*(.svcStack)"
--fill_value=0
--stack_size=0x2000
--heap_size=0x1000
-u _c_int00
-stack  0x2000                              /* SOFTWARE STACK SIZE           */
-heap   0x2000                              /* HEAP AREA SIZE                */

/* Stack Sizes for various modes */
__IRQ_STACK_SIZE = 0x1000;
__FIQ_STACK_SIZE = 0x1000;
__ABORT_STACK_SIZE = 0x1000;
__UND_STACK_SIZE = 0x1000;
__SVC_STACK_SIZE = 0x1000;

/* SPECIFY THE SECTIONS ALLOCATION INTO MEMORY */

SECTIONS
{

    .intvecs : load > VECS_M4F_MEM
    .intc_text : load > IRAM_M4F_INTC_MEM
    .TI.noinit : load > IRAM_M4F_VTBL

    .bootCode      : {} palign(8)         > IRAM_M4F_MEM
    .startupCode   : {} palign(8)      > IRAM_M4F_MEM
    .startupData   : {} palign(8)      > DRAM_M4F_MEM, type = NOINIT
    .text          : {} palign(8)      > IRAM_M4F_MEM
    .const         : {} palign(8)      > DRAM_M4F_MEM
    .cinit         : {} palign(8)      > DRAM_M4F_MEM
    .pinit         : {} palign(8)      > IRAM_M4F_MEM
    .bss           : {} align(4)       > DRAM_M4F_MEM
    .far           : {} align(4)       > DRAM_M4F_MEM
    .data          : {} palign(128)    > DRAM_M4F_MEM
    .boardcfg_data : {} palign(128)    > DRAM_M4F_MEM
    .sysmem        : {}                > DRAM_M4F_MEM
    .safedata      : {} palign(8)      > OCSRAM5

    .stack      : {} align(4)       > DRAM_M4F_MEM
    .irqStack   : {. = . + __IRQ_STACK_SIZE;} align(4)      > DRAM_M4F_MEM
    RUN_START(__IRQ_STACK_START)
    RUN_END(__IRQ_STACK_END)
    .fiqStack   : {. = . + __FIQ_STACK_SIZE;} align(4)      > DRAM_M4F_MEM
    RUN_START(__FIQ_STACK_START)
    RUN_END(__FIQ_STACK_END)
    .abortStack : {. = . + __ABORT_STACK_SIZE;} align(4)    > DRAM_M4F_MEM
    RUN_START(__ABORT_STACK_START)
    RUN_END(__ABORT_STACK_END)
    .undStack   : {. = . + __UND_STACK_SIZE;} align(4)      > DRAM_M4F_MEM
    RUN_START(__UND_STACK_START)
    RUN_END(__UND_STACK_END)
    .svcStack   : {. = . + __SVC_STACK_SIZE;} align(4)      > DRAM_M4F_MEM
    RUN_START(__SVC_STACK_START)
    RUN_END(__SVC_STACK_END)
}
