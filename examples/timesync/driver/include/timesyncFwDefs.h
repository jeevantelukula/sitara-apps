/*
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _TIMESYNC_FW_DEFS_H_
#define _TIMESYNC_FW_DEFS_H_

/* ICSSG INTC events */
/* Compile-time Host event for IEP0 CMP0 event, pr0_pru_mst_intr[2]_intr_req,
   ideally Host would provide this to FW */
#define TRIGGER_HOST_EVT   ( 18 ) /* 2+16 */

/*
 * Firmware global registers
 */

/* TS Registers base address */
#define ICSSG_TS_BASE_ADDR                              ( 0x1E00 )

/* Firmware Information Registers */
#define ICSSG_TS_FW_MAGIC_NUMBER_ADDR                   ( ICSSG_TS_BASE_ADDR + 0x0000 )
#define ICSSG_TS_FW_TYPE_ADDR                           ( ICSSG_TS_BASE_ADDR + 0x0004 )
#define ICSSG_TS_FW_VERSION_ADDR                        ( ICSSG_TS_BASE_ADDR + 0x0008 )
#define ICSSG_TS_FW_FEATURE_ADDR                        ( ICSSG_TS_BASE_ADDR + 0x000C )
#define ICSSG_TS_FW_EXT_FEATURE_ADDR                    ( ICSSG_TS_BASE_ADDR + 0x0010 )

/* TS Control/Status Registers */
#define ICSSG_TS_TS_CTRL_ADDR                           ( ICSSG_TS_BASE_ADDR + 0x0014 )
#define ICSSG_TS_TS_STAT_ADDR                           ( ICSSG_TS_BASE_ADDR + 0x0018 )

/*
 * Firmware global register bit fields
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
#define FW_INIT_MASK                                     ( 0x1 )
#define TS_STAT_FW_INIT_SHIFT                           (  2 )
#define TS_STAT_FW_INIT_MASK                            ( FW_INIT_MASK << TS_STAT_FW_INIT_SHIFT )

/* IEP0 TS register addresses */
#define ICSSG_TS_IEP0_TS_BASE_ADDR                      ( ICSSG_TS_BASE_ADDR + 0x001C )

/* IEP1 TS register addresses */
#define ICSSG_TS_IEP1_TS_BASE_ADDR                      ( ICSSG_TS_BASE_ADDR + 0x0068 )
/*
 * Firmware IEP TS instance register bit fields
 */

/* IEPx_TS_RECFG */
#define RECFG_IEP_TS_EN_MASK                            ( 0x1 )
#define RECFG_IEP_TS_PRD_COUNT_MASK                     ( 0x1 )
#define IEP_TS_RECFG_RECFG_IEP_TS_EN_SHIFT              (  0 )
#define IEP_TS_RECFG_RECFG_IEP_TS_EN_MASK               ( RECFG_IEP_TS_EN_MASK << IEP_TS_RECFG_RECFG_IEP_TS_EN_SHIFT )
#define IEP_TS_RECFG_RECFG_IEP_TS_PRD_COUNT_SHIFT       (  1 )
#define IEP_TS_RECFG_RECFG_IEP_TS_PRD_COUNT_MASK        ( RECFG_IEP_TS_PRD_COUNT_MASK << IEP_TS_RECFG_RECFG_IEP_TS_PRD_COUNT_SHIFT )
/* Aggregate Reconfiguration bit field mask */
#define IEP_TS_RECFG_MASK \
    ( IEP_TS_RECFG_RECFG_IEP_TS_EN_MASK | \
      IEP_TS_RECFG_RECFG_IEP_TS_PRD_COUNT_MASK )

/* IEPx_TS_EN */
#define BF_TS_EN_DISABLE                                ( 0 )   /* TS enable bit field disabled setting */
#define BF_TS_EN_ENABLE                                 ( 1 )   /* TS enable bit field enabled setting */
#define TS_EN_MASK                                      ( 0x1 )
#define IEP_TS_EN_TS0_EN_SHIFT                          ( 0 )
#define IEP_TS_EN_TS0_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS0_EN_SHIFT )
#define IEP_TS_EN_TS1_EN_SHIFT                          ( 1 )
#define IEP_TS_EN_TS1_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS1_EN_SHIFT )
#define IEP_TS_EN_TS2_EN_SHIFT                          ( 2 )
#define IEP_TS_EN_TS2_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS2_EN_SHIFT )
#define IEP_TS_EN_TS3_EN_SHIFT                          ( 3 )
#define IEP_TS_EN_TS3_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS3_EN_SHIFT )
#define IEP_TS_EN_TS4_EN_SHIFT                          ( 4 )
#define IEP_TS_EN_TS4_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS4_EN_SHIFT )
#define IEP_TS_EN_TS5_EN_SHIFT                          ( 5 )
#define IEP_TS_EN_TS5_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS5_EN_SHIFT )
#define IEP_TS_EN_TS6_EN_SHIFT                          ( 6 )
#define IEP_TS_EN_TS6_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS6_EN_SHIFT )
#define IEP_TS_EN_TS7_EN_SHIFT                          ( 7 )
#define IEP_TS_EN_TS7_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS7_EN_SHIFT )
#define IEP_TS_EN_TS8_EN_SHIFT                          ( 8 )
#define IEP_TS_EN_TS8_EN_MASK                           ( TS_EN_MASK << IEP_TS_EN_TS8_EN_SHIFT )
#define IEP_TS_EN_TS9_EN_SHIFT                          ( 9 )
#define IEP_TS_EN_TS9_EN_MASK                           ( TS_EN_MASK  << IEP_TS_EN_TS9_EN_SHIFT )
#define IEP_TS_EN_TS10_EN_SHIFT                         ( 10 )
#define IEP_TS_EN_TS10_EN_MASK                          ( TS_EN_MASK << IEP_TS_EN_TS10_EN_SHIFT )
#define IEP_TS_EN_TS11_EN_SHIFT                         ( 11 )
#define IEP_TS_EN_TS11_EN_MASK                          ( TS_EN_MASK << IEP_TS_EN_TS11_EN_SHIFT )

/* IEPx_TS_PRD_COUNT */
#define PRD_COUNT_MASK                                  ( 0xFFFFFFFF )
#define IEP_TS_PRD_COUNT_SHIFT                          (  0 )
#define IEP_TS_PRD_COUNT_MASK                           ( PRD_COUNT_MASK << IEP_TS_PRD_COUNT_SHIFT )

#endif /* _TIMESYNC_FW_DEFS_H_ */

