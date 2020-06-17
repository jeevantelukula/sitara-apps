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

/* linker options */
--fill_value=0
--stack_size=0x2000
--heap_size=0x1000
--retain="*(.utilsCopyVecsToAtcm)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
/* memory sections inherited from appropriate linker_mem_map.cmd              */

SECTIONS
{
    .vecs       : {
         *(.vecs)
    } palign(8) > MCU0_R5F0_BTCM
    .text_boot {
        *boot.aer5f*<*boot.o*>(.text)
     }  palign(8)   > MCU0_R5F0_BTCM
    .text:xdc_runtime_Startup_reset__I     : {} palign(8) > MCU0_R5F0_BTCM
    .text:ti_sysbios_family_arm_v7r_Cache* : {} palign(8) > MCU0_R5F0_BTCM
    .text:ti_sysbios_family_arm_MPU*       : {} palign(8) > MCU0_R5F0_BTCM
    .utilsCopyVecsToAtcm                   : {} palign(8) > MCU0_R5F0_BTCM
    
    .text       : {} palign(8)   > OCSRAM4
    .cinit      : {} palign(8)   > OCSRAM4
    .bss        : {} align(8)    > DDR_MCU1_0
    .far        : {} align(8)    > OCSRAM4
    .const      : {} palign(8)   > OCSRAM4
    .data       : {} palign(128) > OCSRAM5
    .sysmem     : {} align(8)    > OCSRAM5
    .stack      : {} align(4)    > OCSRAM5

    .safedata   : {} palign(8)  > OCSRAM6

    .bss:ipcMCBuffSection > R5F_ATCM_IPC_RSVD
    .bss:taskStackSection > OCSRAM5
    .resource_table : {
        __RESOURCE_TABLE = .;
    } > DDR_MCU1_0_RESOURCE_TABLE

    .bss:l3mem              (NOLOAD)(NOINIT) : {} > OCSRAM5
    .bss:ddr_shared_mem     (NOLOAD) : {} > DDR_MCU1_0_SHARED_MEM
    .bss:ipc_vring_mem      (NOLOAD) : {} > IPC_VRING_MEM

}
