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
    /*  Vectors base address(VECTORS) should be 64 bytes aligned  */
    MCU1_0_VECTORS (X)  			: ORIGIN = 0x41C7E000, LENGTH = 0x1000
    MCU1_1_VECTORS (X)  			: ORIGIN = 0x41C7F000, LENGTH = 0x1000
    /*  Reset Vectors base address(RESET_VECTORS) should be 64 bytes aligned  */
    MCU1_0_RESET_VECTORS (X)  		: ORIGIN = 0x41C00000, LENGTH = 0x200
    MCU1_1_RESET_VECTORS (X)  		: ORIGIN = 0x41C00200, LENGTH = 0x200

	/* local view vs soc view is present as aliases. Note that when using  */
	/* an A core, ARM prohibits having physical address aliases (because   */
	/* it breaks the coherence needed to make multicore work). SoC view to */
	/* load/run usecase or dma usecase for SoC, and otherwise use local.   */
    /* MCU0_R5F local view */
    R5F_ATCM_SBL_RSVD        (    X ) : ORIGIN = 0x00000000 , LENGTH = 0x00000100
    R5F_ATCM                 (    X ) : ORIGIN = 0x00000100 , LENGTH = 0x00008000 - 0x100

    /* MCU0_R5F_0 SoC view */
    R5F0_ATCM (RWIX) 		: ORIGIN = 0x41000100, LENGTH = 0x8000 - 0x100
    R5F0_BTCM (RWIX) 		: ORIGIN = 0x41010000, LENGTH = 0x8000

    /* MCU0_R5F_1 SoC view */
    R5F1_ATCM (RWIX)		: ORIGIN = 0x41400100, LENGTH = 0x8000 - 0x100
    R5F1_BTCM (RWIX)		: ORIGIN = 0x41410000, LENGTH = 0x8000

    /* The OCRAM is split into its component banks as defined by the hardware. */
    /* The BCDMA cannot span banks with a single TR or a single TR descriptor. */
    /* The separate banks prevent this from occuring, and allows data to be    */
    /* explicitly placed into separate banks to improve performance as each    */
    /* bank can service a memory request every memory cycle.                   */
    OCMRAM_MCU1_0 	(RWIX) 	: ORIGIN = 0x41C00400, LENGTH = 0x40000-0x400
    /* MCU0 memory used for SBL. Avaiable after boot for app starts for dynamic use */
    OCMRAM_MCU1_1 	(RWIX) 	: ORIGIN = 0x41C40000, LENGTH = 0x40000-0x2000

    /* Internal Memory for MCU1_0 for shared memory buffers [ size 640 KB ] */
    MSMC_MCU1_0              ( RWIX ) : ORIGIN = 0x70000000 , LENGTH = 0xA0000
    /* Internal Memory for MCU1_1 for shared memory buffers [ size 384 KB ] */
    MSMC_MCU1_1              ( RWIX ) : ORIGIN = 0x700A0000 , LENGTH = 0x60000

    /* DDR for MCU1_0 for Linux IPC [ size 1024.00 KB ] */
    DDR_MCU1_0_IPC           ( RWIX ) : ORIGIN = 0xA0000000 , LENGTH = 0x00100000
    /* DDR for MCU1_0 for Linux resource table [ size 1024 B ] */
    DDR_MCU1_0_RESOURCE_TABLE ( RWIX ): ORIGIN = 0xA0100000 , LENGTH = 0x00000400
    /* DDR for MCU1_0 for code/data [ size ~15.00 MB ] */
    DDR_MCU1_0               ( RWIX ) : ORIGIN = 0xA0100400 , LENGTH = 0x00EFFC00

    /* DDR for MCU1_1 for Linux IPC [ size 1024.00 KB ] */
    DDR_MCU1_1_IPC           ( RWIX ) : ORIGIN = 0xA1000000 , LENGTH = 0x00100000
    /* DDR for MCU1_1 for Linux resource table [ size 1024 B ] */
    DDR_MCU1_1_RESOURCE_TABLE ( RWIX ): ORIGIN = 0xA1100000 , LENGTH = 0x00000400
    /* DDR for MCU1_1 for code/data [ size ~15.00 MB ] */
    DDR_MCU1_1               ( RWIX ) : ORIGIN = 0xA1100400 , LENGTH = 0x00EFFC00

    /* Memory for MCU1_0 shared memory buffers in DDR [ size 16.00 MB ] */
    DDR_MCU1_0_SHARED_MEM             : ORIGIN = 0xA3000000 , LENGTH = 0x01000000
    /* Memory for MCU1_1 shared memory buffers in DDR [ size 16.00 MB ] */
    DDR_MCU1_1_SHARED_MEM             : ORIGIN = 0xA4000000 , LENGTH = 0x01000000

    /* Memory for IPC Vring's. MUST be non-cached or cache-coherent [ size 32.00 MB ] */
    IPC_VRING_MEM                     : ORIGIN = 0xBA000000 , LENGTH = 0x02000000
}

