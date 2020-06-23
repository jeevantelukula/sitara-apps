/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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


#ifndef APP_MBX_IPC_H_
#define APP_MBX_IPC_H_

#include <stdint.h>
#include <app_mbx_ipc_soc.h>

/**
 * \defgroup group_apps_utils_ipc Inter-processor communication (IPC) APIs
 *
 * \brief This section contains APIs for Inter-processor communication (IPC)
 *
 * \ingroup group_apps_utils
 *
 * @{
 */

/**
 * \brief Callback that is invoke when current CPU receives a IPC notify
 *
 * \param remoteId [in] source CPU which generated this callback, see APP_IPC_CPU_*
 * \param payload  [in] payload or message received from source CPU to current CPU
 *
 */
typedef void (*app_mbxipc_notify_handler_f)(uint32_t remoteId, uint32_t payload);

/**
 * \brief IPC initialization parameters
 */
typedef struct {
    /**< Number of CPUs included for IPC */
    uint32_t num_cpus;
    /**< Self CPU ID for application to know which core currently it runs */
    uint32_t self_cpu_id;
    /**< Master CPU ID, who perform the CPU-sync logic between all other CPUs */
    uint32_t master_cpu_id;
    /**< List of CPU IDs enabled for IPC */
    uint32_t enabled_cpu_id_list[MAILBOX_IPC_MAX_PROCS];
} app_mbxipc_init_prm_t;

/**
 * \brief Set IPC init parameters to default state
 *
 * Recommend to call this API before calling appIpcInit.
 *
 * \param prm [out] Parameters set to default
 */
void appMbxIpcInitPrmSetDefault(app_mbxipc_init_prm_t *prm);

/**
 * \brief Initialize IPC module
 *
 * \param prm [in] Initialization parameters
 *
 * \return 0 on success, else failure
 */
int32_t appMbxIpcInit(app_mbxipc_init_prm_t *prm);

/**
 * \brief De-Initialize IPC module
 *
 * \return 0 on success, else failure
 */
int32_t appMbxIpcDeInit(void);

/**
 * \brief Register callback to invoke on receiving a notify message
 *
 * \param handler [in] Notify handler
 *
 * \return 0 on success, else failure
 */
int32_t appMbxIpcRegisterNotifyHandler(app_mbxipc_notify_handler_f handler);

/**
 * \brief Send a notify message to a given CPU
 *
 * \param remoteId [in] Destination CPU ID, see APP_IPC_CPU_*
 * \param payload [in] payload to send as part of notify
 *
 * \return 0 on success, else failure
 */
void appMbxIpcSendNotify(uint32_t remoteId, uint32_t payload);

/**
 * \brief Get current CPU ID
 *
 * \return current CPU ID, see APP_IPC_CPU_*
 */
uint32_t appMbxIpcGetSelfCpuId(void);

/**
 * \brief Get master CPU ID
 *
 * \return master CPU ID, see APP_IPC_CPU_*
 */
uint32_t appMbxIpcGetMasterCpuId(void);

/**
 * \brief Check if a CPU is enabled in current system for IPC
 *
 * \param cpu_id [in] CPU ID, see APP_IPC_CPU_*
 *
 * \return 1 if CPU is enabled, 0 if CPU is disabled
 */
uint32_t appMbxIpcIsCpuEnabled(uint32_t cpu_id);

/* @} */

#endif

