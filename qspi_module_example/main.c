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
/** @file
 * @defgroup qspi_example_main main.c
 * @{
 * @ingroup qspi_example
 *
 * @brief QSPI Example Application main file.
 *
 * This file contains the source code for a sample application using QSPI.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// #include "nrf_drv_qspi.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"


#include "module_flash_qspi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "sdk_config.h"

#define QSPI_BASE_ADDRESS 0x0FF000

#define QSPI_TEST_DATA_SIZE 0x08

static uint8_t __ALIGN(4) m_buffer_tx[QSPI_TEST_DATA_SIZE];
static uint8_t __ALIGN(4) m_buffer_rx[QSPI_TEST_DATA_SIZE];

static module_flash_qspi_init_t m_flash_qspi_init;

static module_flash_qspi_evt_handler_t module_flash_qspi_handler(module_flash_qspi_report_t * p_qspi_report)
{
        switch(p_qspi_report->evt_type)
        {
        case eflash_QSPI_READ_REQ:
                NRF_LOG_INFO("eflash_QSPI_READ_REQ");
                break;
        case eflash_QSPI_READ_DONE:
                NRF_LOG_INFO("eflash_QSPI_READ_DONE");
                break;
        case eflash_QSPI_WRITE_REQ:
                NRF_LOG_INFO("eflash_QSPI_WRITE_REQ");
                break;
        case eflash_QSPI_WRITE_DONE:
                NRF_LOG_INFO("eflash_QSPI_WRITE_DONE");
                break;
        case eflash_QSPI_ERASE_REQ:
                NRF_LOG_INFO("eflash_QSPI_ERASE_REQ");
                break;
        case eflash_QSPI_ERASE_DONE:
                NRF_LOG_INFO("eflash_QSPI_ERASE_DONE");
                break;
        case eflash_QSPI_UNINIT:
                NRF_LOG_INFO("eflash_QSPI_UNINIT");
                break;

        default:
                break;

        }
}


int main(void)
{
        uint32_t i;
        uint32_t err_code;

        err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();

        NRF_LOG_INFO("QSPI write and read example using 24bit addressing mode");

        srand(0);

        for (i = 0; i < QSPI_TEST_DATA_SIZE; ++i)
        {
                m_buffer_tx[i] = (uint8_t)rand();
        }

        m_flash_qspi_init.evt_handler = module_flash_qspi_handler;
        m_flash_qspi_init.base_address = QSPI_BASE_ADDRESS;
        module_flash_qspi_init(&m_flash_qspi_init);

        NRF_LOG_INFO("0");

        err_code = flash_qspi_page_erase(0);
        APP_ERROR_CHECK(err_code);

        err_code = flash_qspi_page_erase(0xFF);
        APP_ERROR_CHECK(err_code);

        // NRF_LOG_HEXDUMP_INFO(m_buffer_tx, sizeof(m_buffer_tx));

//        err_code = flash_qspi_write(m_buffer_tx, 0, QSPI_TEST_DATA_SIZE/4);
//        APP_ERROR_CHECK(err_code);

//        err_code = flash_qspi_page_erase(1);
//        APP_ERROR_CHECK(err_code);

//        err_code = flash_qspi_write(m_buffer_tx, 0x1000, QSPI_TEST_DATA_SIZE/4);
//        APP_ERROR_CHECK(err_code);

        err_code = flash_qspi_write(m_buffer_tx, QSPI_BASE_ADDRESS, QSPI_TEST_DATA_SIZE);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Process of writing data start");

        err_code = nrf_drv_qspi_read(m_buffer_rx, QSPI_TEST_DATA_SIZE, QSPI_BASE_ADDRESS);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Data read");

        err_code = flash_qspi_write(m_buffer_tx, QSPI_BASE_ADDRESS+QSPI_TEST_DATA_SIZE, QSPI_TEST_DATA_SIZE);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Process of writing data start");

        err_code = nrf_drv_qspi_read(m_buffer_rx, QSPI_TEST_DATA_SIZE+QSPI_TEST_DATA_SIZE, QSPI_BASE_ADDRESS);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Data read");

        // NRF_LOG_HEXDUMP_INFO(m_buffer_rx, sizeof(m_buffer_rx));

        NRF_LOG_INFO("Compare...");
        NRF_LOG_INFO("======================");
        NRF_LOG_INFO("======================");
        NRF_LOG_INFO("======================");

        if (memcmp(m_buffer_tx, m_buffer_rx, QSPI_TEST_DATA_SIZE) == 0)
        {
                NRF_LOG_INFO("Data SAME SAME");
        }
        else
        {
                NRF_LOG_INFO("Data Difference");
        }
//        NRF_LOG_FLUSH();
        module_flash_qspi_uninit();

        NRF_LOG_INFO("Finish");
        for (;;)
        {

                if (NRF_LOG_PROCESS())
                {
                        NRF_LOG_FLUSH();
                }
        }
}

/** @} */
