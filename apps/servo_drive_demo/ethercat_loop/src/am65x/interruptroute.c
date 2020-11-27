/**
 * interruptroute.c
 *
*/
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

#include <interruptroute.h>

int32_t route_icss_interrupts_to_r5f(uint32_t icss_instance)
{
    /* Route ICSS Interrupts to R5F. */
    int32_t retVal = CSL_PASS;
    struct tisci_msg_rm_get_resource_range_resp resp;
    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    uint16_t intStartNum;
    uint16_t intNum, dstInput, baseOffset;
    uint32_t i;

    if ( (icss_instance != ICSS_INSTANCE_ONE) && (icss_instance != ICSS_INSTANCE_TWO) && (icss_instance != ICSS_INSTANCE_THREE))
    {
        retVal = INTERRUPT_ROUTE_ERROR;
    }

    if(CSL_PASS == retVal)
    {
        req.type           = TISCI_DEV_MAIN2MCU_LVL_INTRTR0;
        req.subtype        = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
        req.secondary_host = (uint8_t)TISCI_HOST_ID_R5_0;

        /* Get interrupt number range */
        retVal =  Sciclient_rmGetResourceRange(&req, &resp, SCICLIENT_SERVICE_WAIT_FOREVER);

        if(CSL_PASS == retVal)
        {
            intStartNum = resp.range_start;

            rmIrqReq.dst_id         = TISCI_DEV_MCU_ARMSS0_CPU0;
            rmIrqReq.secondary_host = TISCI_HOST_ID_R5_0;

            for(i = 0; NUM_ICSS_INTERRUPTS != i; i++)
            {
                intNum = intStartNum + i;

                retVal = Sciclient_rmIrqTranslateIrOutput(TISCI_DEV_MAIN2MCU_LVL_INTRTR0, intNum, TISCI_DEV_MCU_ARMSS0_CPU0, &dstInput);

                if(i == 0)
                    baseOffset = dstInput;

                if (CSL_PASS == retVal)
                {
                    rmIrqReq.ia_id                  = 0U;
                    rmIrqReq.vint                   = 0U;
                    rmIrqReq.global_event           = 0U;
                    rmIrqReq.vint_status_bit_index  = 0U;

                    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID
                                                      | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID
                                                      | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
                    if (icss_instance == ICSS_INSTANCE_ONE)
                    {
                        rmIrqReq.src_id             = TISCI_DEV_PRU_ICSSG0;
                    }
                    else if (icss_instance == ICSS_INSTANCE_TWO)
                    {
                        rmIrqReq.src_id             = TISCI_DEV_PRU_ICSSG1;
                    }
                    else if (icss_instance == ICSS_INSTANCE_THREE)
                    {
                        rmIrqReq.src_id             = TISCI_DEV_PRU_ICSSG2;
                    }

                    rmIrqReq.src_index              = ICSSG_INTERRUPT_SRC_INDEX + i;
                    rmIrqReq.dst_host_irq           = dstInput;

                    /* Config event */
                    retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);
                    if ( CSL_PASS != retVal )
                    {
                        break;
                    }
                }
            }
        }
    }

    if(CSL_PASS == retVal)
    {
        retVal = baseOffset;
    }
    else
    {
        retVal = INTERRUPT_ROUTE_ERROR;
        UART_printf("Error in mapping ICSS interrupts to R5F\n");
    }

    return retVal;
}


int32_t route_i2c_interrupts_to_r5f()
{
    /* Route I2C Interrupts to R5F. */
    int32_t retVal;
    struct tisci_msg_rm_get_resource_range_resp resp;
    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    uint16_t intStartNum;
    uint16_t dstInput;

    req.type           = TISCI_DEV_MAIN2MCU_LVL_INTRTR0;
    req.subtype        = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
    req.secondary_host = (uint8_t)TISCI_HOST_ID_R5_0;

    /* Get interrupt number range */
    retVal =  Sciclient_rmGetResourceRange(&req, &resp, SCICLIENT_SERVICE_WAIT_FOREVER);

    if(CSL_PASS == retVal)
    {
        /*ASSUMPTION: First few interrupt lines are used for mapping ICSS events*/
        intStartNum = resp.range_start + NUM_ICSS_INTERRUPTS;

        rmIrqReq.dst_id         = TISCI_DEV_MCU_ARMSS0_CPU0;
        rmIrqReq.secondary_host = TISCI_HOST_ID_R5_0;

        retVal = Sciclient_rmIrqTranslateIrOutput(TISCI_DEV_MAIN2MCU_LVL_INTRTR0, intStartNum, TISCI_DEV_MCU_ARMSS0_CPU0, &dstInput);

        if (CSL_PASS == retVal)
        {
            rmIrqReq.ia_id                  = 0U;
            rmIrqReq.vint                   = 0U;
            rmIrqReq.global_event           = 0U;
            rmIrqReq.vint_status_bit_index  = 0U;

            rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID
                                      | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID
                                      | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
            rmIrqReq.src_id         = TISCI_DEV_I2C0;
            rmIrqReq.src_index      = I2C0_INTERRUPT_SRC_INDEX;
            rmIrqReq.dst_host_irq   = dstInput;
        }

        /* Config event */
        retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);
    }

    if(CSL_PASS == retVal)
    {
        retVal = dstInput;
    }
    else
    {
        retVal = INTERRUPT_ROUTE_ERROR;
        UART_printf("Error in mapping I2C interrupt to R5F\n");
    }

    return retVal;
}

int32_t unroute_icss_interrupts_to_r5f(uint32_t icss_instance, int32_t interrupt_offset)
{
    int32_t retVal;
    uint32_t i;
    struct tisci_msg_rm_irq_release_req req;

    req.dst_id                 = TISCI_DEV_MCU_ARMSS0_CPU0;
    req.secondary_host         = TISCI_HOST_ID_R5_0;
    req.ia_id                  = 0U;
    req.vint                   = 0U;
    req.global_event           = 0U;
    req.vint_status_bit_index  = 0U;

    req.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID
                                      | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID
                                      | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    if (icss_instance == ICSS_INSTANCE_ONE)
    {
        req.src_id             = TISCI_DEV_PRU_ICSSG0;
    }
    else if (icss_instance == ICSS_INSTANCE_TWO)
    {
        req.src_id             = TISCI_DEV_PRU_ICSSG1;
    }
    else if (icss_instance == ICSS_INSTANCE_THREE)
    {
        req.src_id             = TISCI_DEV_PRU_ICSSG2;
    }

    for(i = 0; NUM_ICSS_INTERRUPTS != i; i++)
    {
        req.src_index      = ICSSG_INTERRUPT_SRC_INDEX + i;
        req.dst_host_irq   = interrupt_offset + i;

        retVal =  Sciclient_rmIrqRelease(&req, SCICLIENT_SERVICE_WAIT_FOREVER);
        if ( CSL_PASS != retVal )
        {
           break;
        }
    }

    if(CSL_PASS != retVal)
    {
        retVal = INTERRUPT_ROUTE_ERROR;
        UART_printf("Error in releasing ICSS interrupt routes to R5F\n");
    }


    return retVal;
}

int32_t unroute_i2c_interrupts_to_r5f(int32_t interrupt_offset)
{
    int32_t retVal;
    struct tisci_msg_rm_irq_release_req req;

    req.dst_id         = TISCI_DEV_MCU_ARMSS0_CPU0;
    req.secondary_host = TISCI_HOST_ID_R5_0;
    req.ia_id                  = 0U;
    req.vint                   = 0U;
    req.global_event           = 0U;
    req.vint_status_bit_index  = 0U;
    req.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID
                              | TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID
                              | TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    req.src_id         = TISCI_DEV_I2C0;
    req.src_index      = I2C0_INTERRUPT_SRC_INDEX;
    req.dst_host_irq   = interrupt_offset;

    retVal =  Sciclient_rmIrqRelease(&req, SCICLIENT_SERVICE_WAIT_FOREVER);

    if(CSL_PASS != retVal)
    {
        retVal = INTERRUPT_ROUTE_ERROR;
        UART_printf("Error in releasing I2C interrupt route to R5F\n");
    }

    return retVal;
}


