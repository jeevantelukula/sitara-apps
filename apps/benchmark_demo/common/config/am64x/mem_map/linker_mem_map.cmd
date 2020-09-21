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

MEMORY
{
    /*******************************************************/
    /*                     R5F Memory                      */
    /*******************************************************/
    /* local view vs soc view is present as aliases. Note that when using  */
    /* an A core, ARM prohibits having physical address aliases (because   */
    /* it breaks the coherence needed to make multicore work). SoC view to */
    /* load/run usecase or dma usecase for SoC, and otherwise use local.   */
    /* MCU0/1_R5F local view */
#ifdef R5F
    R5F_ATCM_SBL_RSVD		(    X )	: ORIGIN = 0x00000000 , LENGTH = 0x00000100
    R5F_ATCM_IPC_RSVD		(    X )	: ORIGIN = 0x00000100 , LENGTH = 0x00000100
    R5F_ATCM			(    X ) 	: ORIGIN = 0x00000200 , LENGTH = 0x00008000 - 0x200
    R5F_BTCM            (    X )    : ORIGIN = 0x41010000 , LENGTH = 0x00008000

#endif

    /* MCU0_R5F_0 SoC view */
    MCU0_R5F0_ATCM 		( RWIX )	: ORIGIN = 0x78000000, LENGTH = 0x8000
    MCU0_R5F0_BTCM		( RWIX )	: ORIGIN = 0x78100000, LENGTH = 0x8000

    /* MCU0_R5F_1 SoC view */
    MCU0_R5F1_ATCM 		( RWIX )	: ORIGIN = 0x78200000, LENGTH = 0x8000
    MCU0_R5F1_BTCM 		( RWIX )	: ORIGIN = 0x78300000, LENGTH = 0x8000

    /* MCU1_R5F_0 SoC view */
    MCU1_R5F0_ATCM		( RWIX )	: ORIGIN = 0x78400000, LENGTH = 0x8000
    MCU1_R5F0_BTCM		( RWIX ) 	: ORIGIN = 0x78500000, LENGTH = 0x8000

    /* MCU1_R5F_1 SoC view */
    MCU1_R5F1_ATCM		( RWIX )	: ORIGIN = 0x78600000, LENGTH = 0x8000
    MCU1_R5F1_BTCM 		( RWIX )	: ORIGIN = 0x78700000, LENGTH = 0x8000

#ifdef M4F
    /*******************************************************/
    /*                     M4F Memory                      */
    /*******************************************************/
    VECS_M4F_MEM				: ORIGIN = 0x00000000 , LENGTH = 0x040
    IRAM_M4F_INTC_MEM				: ORIGIN = 0x00000040 , LENGTH = 0x400 - 0x040
    /* Memory assigned to move vector table for M4F core */
    IRAM_M4F_VTBL				: ORIGIN = 0x00000400 , LENGTH = 0x800
    /* M4F internal memory locations */
    IRAM_M4F_MEM		( RWIX )	: ORIGIN = 0x00000C00 , LENGTH = 0x30000 - 0xC00
    DRAM_M4F_MEM		( RWIX )	: ORIGIN = 0x00030000 , LENGTH = 0x10000
#endif

    /*******************************************************/
    /*               On-chip SRAM  Memory                  */
    /*******************************************************/
    /* Will have to split OCMRAM into banks due to DMA spanning issue.         */
    /* DMA can't read contiguous buffer acrosss bank boundary so by splitting  */
    /* OCRAM into banks in *.cmd it prevents the problem (and also allows      */
    /* placement into banks for performance reasons).                          */
    /* 2MB On-chip SRAM broken into 8x 256KB banks                             */
    /* Note: still need to create space for ATF, SYSFW, SBL usage during boot  */
    /* Note: may need to create 64 byte aligned space for R5F reset vectors    */
    OCSRAM0			( RWIX )	: ORIGIN = 0x70000000 , LENGTH = 0x40000
    OCSRAM1			( RWIX )	: ORIGIN = 0x70040000 , LENGTH = 0x40000
    OCSRAM2			( RWIX )	: ORIGIN = 0x70080000 , LENGTH = 0x40000
    OCSRAM3			( RWIX )	: ORIGIN = 0x700C0000 , LENGTH = 0x40000
    OCSRAM4			( RWIX )	: ORIGIN = 0x70100000 , LENGTH = 0x40000
    OCSRAM5			( RWIX )	: ORIGIN = 0x70140000 , LENGTH = 0x40000
    OCSRAM6			( RWIX )	: ORIGIN = 0x70180000 , LENGTH = 0x40000
    OCSRAM7			( RWIX )	: ORIGIN = 0x701C0000 , LENGTH = 0x40000

    /*******************************************************/
    /*                      DDR Memory                     */
    /*******************************************************/
    /* DDR for MCU1_0 for Linux IPC [ size 1024.00 KB ] */
    DDR_MCU1_0_IPC		( RWIX )	: ORIGIN = 0xA0000000 , LENGTH = 0x00100000
    /* DDR for MCU1_0 for Linux resource table [ size 1024 B ] */
    DDR_MCU1_0_RESOURCE_TABLE	( RWIX )	: ORIGIN = 0xA0100000 , LENGTH = 0x00000400
    /* DDR for MCU1_0 for code/data [ size ~15.00 MB ] */
    DDR_MCU1_0			( RWIX )	: ORIGIN = 0xA0100400 , LENGTH = 0x00EFFC00

    /* DDR for MCU1_1 for Linux IPC [ size 1024.00 KB ] */
    DDR_MCU1_1_IPC		( RWIX )	: ORIGIN = 0xA1000000 , LENGTH = 0x00100000
    /* DDR for MCU1_1 for Linux resource table [ size 1024 B ] */
    DDR_MCU1_1_RESOURCE_TABLE	( RWIX )	: ORIGIN = 0xA1100000 , LENGTH = 0x00000400
    /* DDR for MCU1_1 for code/data [ size ~15.00 MB ] */
    DDR_MCU1_1			( RWIX )	: ORIGIN = 0xA1100400 , LENGTH = 0x00EFFC00

    /* Memory for MCU1_0 shared memory buffers in DDR [ size 16.00 MB ] */
    DDR_MCU1_0_SHARED_MEM			: ORIGIN = 0xA3000000 , LENGTH = 0x01000000
    /* Memory for MCU1_1 shared memory buffers in DDR [ size 16.00 MB ] */
    DDR_MCU1_1_SHARED_MEM			: ORIGIN = 0xA4000000 , LENGTH = 0x01000000

    /* Memory for IPC Vring's. MUST be non-cached or cache-coherent [ size 32.00 MB ] */
    IPC_VRING_MEM				: ORIGIN = 0xBA000000 , LENGTH = 0x02000000
}
