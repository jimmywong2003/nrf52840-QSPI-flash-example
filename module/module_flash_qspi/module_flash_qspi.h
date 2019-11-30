/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *
 * @defgroup nrf_fstorage_nvmc NVMC implementation
 * @ingroup nrf_fstorage
 * @{
 *
 * @brief API implementation of fstorage that uses the non-volatile memory controller (NVMC).
 */

#ifndef MODULE_FLASH_QSPI_H__
#define MODULE_FLASH_QSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "nrf_drv_qspi.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

#define QSPI_STD_CMD_WRSR   0x01
#define QSPI_STD_CMD_RSTEN  0x66
#define QSPI_STD_CMD_RST    0x99

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif
    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

typedef enum flash_qspi_evt_s
{
        eflash_QSPI_IDLE,
        eflash_QSPI_INIT,
        eflash_QSPI_UNINIT,
        eflash_QSPI_READ_REQ,
        eflash_QSPI_READ_DONE,
        eflash_QSPI_WRITE_REQ,
        eflash_QSPI_WRITE_DONE,
        eflash_QSPI_ERASE_REQ,
        eflash_QSPI_ERASE_DONE,
} flash_qspi_evt_t;

typedef PACKED ( struct module_flash_qspi_report_s {
        flash_qspi_evt_t evt_type;
}) module_flash_qspi_report_t;

typedef void (* module_flash_qspi_evt_handler_t)(module_flash_qspi_report_t * p_qspi_event);

typedef PACKED (struct
{
        uint32_t base_address;
        module_flash_qspi_evt_handler_t evt_handler;
}) module_flash_qspi_init_t;

void module_flash_qspi_uninit(void);

ret_code_t module_flash_qspi_init(module_flash_qspi_init_t * p_qspi_init);


/**
 * @brief Function for Read data from QSPI memory.
 * @param[out] p_rx_buffer      Pointer to the receive buffer.
 * @param[in]  rx_buffer_length Size of the data to read.
 * @param[in]  src_address      Address in memory to read from.
 */
ret_code_t flash_qspi_read(uint32_t * p_rx_buffer, uint32_t const src_address, uint32_t rx_buffer_length);

/**
 * @brief Function for writing data to QSPI memory.
 *
 *@param[in] p_tx_buffer Pointer to the writing buffer.
 *@param[in] tx_buffer_length Size of the data to write.
 *@param[in] dst_address Address in memory to write to.
 * @retval NRF_SUCCESS            If the operation was successful (blocking mode) or operation
 *                                was commissioned (handler mode).
 * @retval NRF_ERROR_BUSY         If the driver currently handles other operation.
 * @retval NRF_ERROR_INVALID_ADDR If the provided buffer is not placed in the Data RAM region.
 */
ret_code_t flash_qspi_write(uint32_t * p_tx_buffer, uint32_t const dst_address, uint32_t tx_buffer_length);



ret_code_t flash_qspi_page_erase(uint32_t page_number);

 #ifdef __cplusplus
}
 #endif

 #endif // MODULE_FLASH_QSPI_H__
/** @} */
