/*
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
--entry_point=_resetvectors     /* Default C RTS boot.asm   */

-stack  0x2000                              /* SOFTWARE STACK SIZE           */
-heap   0x2000                              /* HEAP AREA SIZE                */

/* Stack Sizes for various modes */
__IRQ_STACK_SIZE = 0x1000;
__FIQ_STACK_SIZE = 0x1000;
__ABORT_STACK_SIZE = 0x1000;
__UND_STACK_SIZE = 0x1000;
__SVC_STACK_SIZE = 0x1000;

/* Memory Map */
MEMORY
{
    /* Reserved for SBL code and data (OC-SRAM bank 0) */
    SBL_RSVD (X)            : origin=0x70000000 length=0x40000
    OCSRAM1  (X)            : origin=0x70040100 length=0x40000 - 0x1100
    OCSRAM2  (X)            : origin=0x70080000 length=0x40000
    OCSRAM3  (X)            : origin=0x700C0000 length=0x40000
    OCSRAM4  (X)            : origin=0x70100000 length=0x40000
    OCSRAM5  (X)            : origin=0x70140000 length=0x40000
    /* Reserved by ROM for SYSFW (OC-SRAM bank 6 & 7) */
    SYSFW_RSVD_1 (X)        : origin=0x70180000 length=0x40000
    SYSFW_RSVD_2 (X)        : origin=0x701C0000 length=0x40000

    /*  Reset Vectors base address(RESET_VECTORS) should be 64 bytes aligned  */
    RESET_VECTORS (X)       : origin=0x70040000 length=0x100	/* start of bank 1 */
    VECTORS (X)             : origin=0x7007F000 length=0x1000	/* end of bank 1 */

    DDR0    (RWIX)          : origin=0x80000000 length=0x80000000  /* 2GB */
}

/* Section Configuration */
SECTIONS
{
    /* 'intvecs' and 'intc_text' sections shall be placed within */
    /* a range of +\- 16 MB */
    .intvecs       : {} palign(8)      > VECTORS
    .intc_text     : {} palign(8)      > VECTORS
    .rstvectors    : {} palign(8)      > RESET_VECTORS
    .bootCode      : {} palign(8)      > OCSRAM1
    .startupCode   : {} palign(8)      > OCSRAM1
    .startupData   : {} palign(8)      > OCSRAM1, type = NOINIT
    .text          : {} palign(8)      > OCSRAM1
    .const         : {} palign(8)      > OCSRAM1
    .cinit         : {} palign(8)      > OCSRAM1
    .pinit         : {} palign(8)      > OCSRAM1
    .bss           : {} align(4)       > OCSRAM1
    .far           : {} align(4)       > OCSRAM1
    .data          : {} palign(128)    > OCSRAM1
    .boardcfg_data : {} palign(128)    > OCSRAM1
    .sysmem        : {}                > OCSRAM1
    .write_buffer  : {} palign(128)    > OCSRAM1
    .read_buffer   : {} palign(128)    > OCSRAM1

    /* USB or any other LLD buffer for benchmarking */
    .benchmark_buffer (NOLOAD) {} ALIGN (8) > DDR0

    .stack      : {} align(4)       > OCSRAM1
    .irqStack   : {. = . + __IRQ_STACK_SIZE;} align(4)      > OCSRAM1
    RUN_START(__IRQ_STACK_START)
    RUN_END(__IRQ_STACK_END)
    .fiqStack   : {. = . + __FIQ_STACK_SIZE;} align(4)      > OCSRAM1
    RUN_START(__FIQ_STACK_START)
    RUN_END(__FIQ_STACK_END)
    .abortStack : {. = . + __ABORT_STACK_SIZE;} align(4)    > OCSRAM1
    RUN_START(__ABORT_STACK_START)
    RUN_END(__ABORT_STACK_END)
    .undStack   : {. = . + __UND_STACK_SIZE;} align(4)      > OCSRAM1
    RUN_START(__UND_STACK_START)
    RUN_END(__UND_STACK_END)
    .svcStack   : {. = . + __SVC_STACK_SIZE;} align(4)      > OCSRAM1
    RUN_START(__SVC_STACK_START)
    RUN_END(__SVC_STACK_END)
}

