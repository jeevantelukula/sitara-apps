/**
* tiesc_soc_am64x.c: Implements a mechanism to send message from R5F to M4F.
*/
/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <tiesc_soc_am64x.h>

/* "safedata" should be defined as a section in linker command file and should be placed in OCSRAM5.*/
#pragma DATA_SECTION(BlackChannel, ".safedata")
blackChannel_t BlackChannel;

void Configure_Rat(void)
{
    CSL_ratRegs * R5F_RAT_MMR = (CSL_ratRegs *) CSL_R5FSS0_RAT_CFG_BASE;
	
    CSL_ratDisableRegionTranslation(R5F_RAT_MMR, 0U);

    /* sizeInBytes, baseAddress, translatedAddress */
    /* 2MB, 32-bit local memory map, up to 64-bit SoC memory map */
    CSL_RatTranslationCfgInfo ratCfg0 = { OCSRAM_SIZE_INBYTES, OCSRAM_BASE_ADDRESS, OCSRAM_TRANSLATE_ADDRESS }; /* needed for OC-RAM */

    CSL_ratConfigRegionTranslation(R5F_RAT_MMR, 0U, &ratCfg0);

    CSL_ratEnableRegionTranslation(R5F_RAT_MMR, 1U);
}

void Configure_Mbox(void)
{
    MailboxReset(MAILBOX_BASE_ADDRESS);
    MailboxDisableQueueNotFullInt(MAILBOX_BASE_ADDRESS, MAILBOX_SOURCE_CPUID, MAILBOX_QUEUE_0);
    MailboxEnableNewMsgInt(MAILBOX_BASE_ADDRESS, MAILBOX_TARGET_CPUID, MAILBOX_QUEUE_0);
}

void Send_BootComplete_Message_To_Partner(void)
{
	uint32_t msg_status;
	
    Configure_Rat();
    Configure_Mbox();

	/*Sending this data to test Black Channel data exchange only. This will not be used until FSoE is available on M4F. */
    BlackChannel.num_bytes = 3U;    /* data size */
    BlackChannel.data[0] = 0xA0U;   /* data[0] */
    BlackChannel.data[1] = 0x55U;   /* data[1] */
    BlackChannel.data[2] = 0x0AU;   /* data[size-1] */

    /* Send Message to M4F. */
    msg_status = MailboxSendMessage(MAILBOX_BASE_ADDRESS, MAILBOX_QUEUE_0, (uint32_t) CMD_MAILBOX_MSG_BOOT_COMPLETE);
}
