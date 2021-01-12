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
    .safedata      : {} palign(8)      > OCSRAM6

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
