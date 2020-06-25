/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the
 *        distribution.
 *
 *      * Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
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
 */

#ifndef _ICSSG_TIMESYNC_H_
#define _ICSSG_TIMESYNC_H_

/*
 * Firmware registers
 */

/* FW register base address */
#define ICSSG_TS_FW_REGS_BASE                           ( 0x1E00 )

/* Firmware Information Registers */
#define FW_REG_MAGIC_NUMBER_ADDR                        ( ICSSG_TS_FW_REGS_BASE + 0x0000 )
#define FW_REG_TYPE_ADDR                                ( ICSSG_TS_FW_REGS_BASE + 0x0004 )
#define FW_REG_VERSION_ADDR                             ( ICSSG_TS_FW_REGS_BASE + 0x0008 )
#define FW_REG_FEATURE_ADDR                             ( ICSSG_TS_FW_REGS_BASE + 0x000C )
#define FW_REG_EXT_FEATURE_ADDR                         ( ICSSG_TS_FW_REGS_BASE + 0x0010 )

/* FW register sizes (in bytes) */
#define FW_REG_TS_CTRL_SZ                               ( 4 )
#define FW_REG_TS_STAT_SZ                               ( 4 )

/* FW register offsets from base (in bytes) */
#define FW_REG_TS_CTRL_OFFSET                           ( ICSSG_TS_FW_REGS_BASE + 0x0014 )
#define FW_REG_TS_STAT_OFFSET                           ( ICSSG_TS_FW_REGS_BASE + 0x0018 )

/* FW register addresses */
#define FW_REG_TS_CTRL                                  ( ICSSG_TS_FW_REGS_BASE + FW_REG_TS_CTRL_OFFSET )
#define FW_REG_TS_STAT                                  ( ICSSG_TS_FW_REGS_BASE + FW_REG_TS_STAT_OFFSET )

/* FW register addresses */
#define FW_REG_TS_CMP1_COUNT                            ( 0x0028 )
#define FW_REG_TS_CMP3_COUNT                            ( 0x002C )
#define FW_REG_TS_CMP4_COUNT                            ( 0x0030 )
#define FW_REG_TS_CMP5_COUNT                            ( 0x0034 )
#define FW_REG_TS_CMP6_COUNT                            ( 0x0038 )

#define FW_REG_TS_CMP3_OFFSET                           ( 0x003C )
#define FW_REG_TS_CMP4_OFFSET                           ( 0x0040 )
#define FW_REG_TS_CMP5_OFFSET                           ( 0x0044 )
#define FW_REG_TS_CMP6_OFFSET                           ( 0x0048 )

/*
 * Firmware register bit fields
 */
/* Firmware Magic Number */
#define FW_MAGIC_NUM_BYTE0_MASK                         ( 0xF )
#define FW_MAGIC_NUM_BYTE1_MASK                         ( 0xF )
#define FW_MAGIC_NUM_BYTE2_MASK                         ( 0xF )
#define FW_MAGIC_NUM_BYTE3_MASK                         ( 0xF )
#define FW_MAGIC_NUM_MN_BYTE0_SHIFT                     (  0 )
#define FW_MAGIC_NUM_MN_BYTE0_MASK                      ( FW_MAGIC_NUM_BYTE0_MASK << FW_MAGIC_NUM_MN_BYTE0_SHIFT )
#define FW_MAGIC_NUM_MN_BYTE1_SHIFT                     (  8 )
#define FW_MAGIC_NUM_MN_BYTE1_MASK                      ( FW_MAGIC_NUM_BYTE1_MASK << FW_MAGIC_NUM_MN_BYTE1_SHIFT )
#define FW_MAGIC_NUM_MN_BYTE2_SHIFT                     ( 16 )
#define FW_MAGIC_NUM_MN_BYTE2_MASK                      ( FW_MAGIC_NUM_BYTE2_MASK << FW_MAGIC_NUM_MN_BYTE2_SHIFT )
#define FW_MAGIC_NUM_MN_BYTE3_SHIFT                     ( 24 )
#define FW_MAGIC_NUM_MN_BYTE3_MASK                      ( FW_MAGIC_NUM_BYTE3_MASK << FW_MAGIC_NUM_MN_BYTE3_SHIFT )

/* Firmware Type */
#define FW_PROTOCOL_TYPE_VERSION_MASK                   ( 0xF )
#define FW_PROTOCOL_TYPE_MASK                           ( 0xFF )
#define FW_ICSS_VERSION_MASK                            ( 0xF )
#define FW_TYPE_FW_PROTOCOL_TYPE_VERSION_SHIFT          ( 0 )
#define FW_TYPE_FW_PROTOCOL_TYPE_VERSION_MASK           ( FW_PROTOCOL_TYPE_VERSION_MASK << FW_TYPE_FW_PROTOCOL_TYPE_VERSION_SHIFT )
#define FW_TYPE_FW_PROTOCOL_TYPE_SHIFT                  ( 8 )
#define FW_TYPE_FW_PROTOCOL_TYPE_MASK                   ( FW_PROTOCOL_TYPE_MASK << FW_TYPE_FW_PROTOCOL_TYPE_SHIFT )
#define FW_TYPE_FW_ICSS_VERSION_SHIFT                   ( 24 )
#define FW_TYPE_FW_ICSS_VERSION_MASK                    ( FW_ICSS_VERSION_MASK << FW_TYPE_FW_ICSS_VERSION_SHIFT )

/* Firmware Version */
#define FW_VER_BUILD_MASK                               ( 0xF )
#define FW_VER_MINOR_MASK                               ( 0xFF )
#define FW_VER_MAJOR_MASK                               ( 0x7F )
#define FW_REL_OR_INT_VER_MASK                          ( 0x1 )
#define FW_VERSION_FW_VER_BUILD_SHIFT                   (  0 )
#define FW_VERSION_FW_VER_BUILD_MASK                    ( FW_VER_BUILD_MASK << FW_VERSION_FW_VER_BUILD_SHIFT )
#define FW_VERSION_FW_VER_MINOR_SHIFT                   (  8 )
#define FW_VERSION_FW_VER_MINOR_MASK                    ( FW_VER_MINOR_MASK << FW_VERSION_FW_VER_MINOR_SHIFT )
#define FW_VERSION_FW_VER_MAJOR_SHIFT                   ( 24 )
#define FW_VERSION_FW_VER_MAJOR_MASK                    ( FW_VER_MAJOR_MASK << FW_VERSION_FW_VER_MAJOR_SHIFT )
#define FW_VERSION_FW_REL_OR_INT_VER_SHIFT              ( 31 )
#define FW_VERSION_FW_REL_OR_INT_VER_MASK               ( FW_REL_OR_INT_VER_MASK << FW_VERSION_FW_REL_OR_INT_VER_SHIFT )

/* Firmware Feature */
#define FW_NUM_TSS_MASK                                 ( 0x1F )
#define FW_IEP0_NUM_TSS_MASK                            ( 0xF )
#define FW_IEP1_NUM_TSS_MASK                            ( 0xF )
#define FW_FEATURE_FW_NUM_TSS_SHIFT                     (  0 )
#define FW_FEATURE_FW_NUM_TSS_MASK                      ( FW_NUM_TSS_MASK << FW_FEATURE_FW_NUM_TSS_SHIFT )
#define FW_FEATURE_FW_IEP0_NUM_TSS_SHIFT                (  5 )
#define FW_FEATURE_FW_IEP0_NUM_TSS_MASK                 ( FW_IEP0_NUM_TSS_MASK << FW_FEATURE_FW_IEP0_NUM_TSS_SHIFT )
#define FW_FEATURE_FW_IEP1_NUM_TSS_SHIFT                (  9 )
#define FW_FEATURE_FW_IEP1_NUM_TSS_MASK                 ( FW_IEP1_NUM_TSS_MASK << FW_FEATURE_FW_IEP1_NUM_TSS_SHIFT )

/* TS_CTRL */
#define BF_TS_GBL_EN_DISABLE                            ( 0 )   /* Global Enable bit field disabled setting */
#define BF_TS_GBL_EN_ENABLE                             ( 1 )   /* Global Enable bit field enabled setting */
#define IEP_TS_GBL_EN_MASK                              ( 0x1 )
#define TS_CTRL_IEP0_TS_GBL_EN_SHIFT                    (  0 )
#define TS_CTRL_IEP0_TS_GBL_EN_MASK                     ( IEP_TS_GBL_EN_MASK << TS_CTRL_IEP0_TS_GBL_EN_SHIFT )
#define TS_CTRL_IEP1_TS_GBL_EN_SHIFT                    (  1 )
#define TS_CTRL_IEP1_TS_GBL_EN_MASK                     ( IEP_TS_GBL_EN_MASK << TS_CTRL_IEP1_TS_GBL_EN_SHIFT )

/* TS_STAT */
#define BF_TS_GBL_EN_ACK_DISABLE                        ( 0 )   /* Global Enable ACK bit field disabled setting */
#define BF_TS_GBL_EN_ACK_ENABLE                         ( 1 )   /* Global Enable ACK bit field enabled setting */
#define BF_TS_FW_INIT_UNINIT                            ( 0 )   /* FW initialized bit field uninitialized setting */
#define BF_TS_FW_INIT_INIT                              ( 1 )   /* FW initialized bit field initialized setting */
#define IEP_TS_GBL_EN_ACK_MASK                          ( 0x1 )
#define FW_INIT_MASK                                    ( 0x1 )
#define TS_STAT_IEP0_TS_GBL_EN_ACK_SHIFT                (  0 )
#define TS_STAT_IEP0_TS_GBL_EN_ACK_MASK                 ( IEP_TS_GBL_EN_ACK_MASK << TS_STAT_IEP0_TS_GBL_EN_ACK_SHIFT )
#define TS_STAT_IEP1_TS_GBL_EN_ACK_SHIFT                (  1 )
#define TS_STAT_IEP1_TS_GBL_EN_ACK_MASK                 ( IEP_TS_GBL_EN_ACK_MASK << TS_STAT_IEP1_TS_GBL_EN_ACK_SHIFT )
#define TS_STAT_FW_INIT_SHIFT                           (  2 )
#define TS_STAT_FW_INIT_MASK                            ( FW_INIT_MASK << TS_STAT_FW_INIT_SHIFT )

#endif /* _ICSSG_TIMESYNC_H_ */
