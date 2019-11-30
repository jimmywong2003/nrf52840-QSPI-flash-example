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

 #include "sdk_common.h"


 #include "module_flash_qspi.h"
 #include <stdint.h>
 #include <string.h>
 #include <stdbool.h>
 #include "nordic_common.h"
 #include "nrf_soc.h"
 #include "app_util_platform.h"

 #include "nrf_log.h"
 #include "nrf_log_ctrl.h"
 #include "nrf_log_default_backends.h"

 #include "nrf_delay.h"

 #include "sdk_config.h"

#define GPIO_DEBUG_PIN_2        15
#define GPIO_DEBUG_PIN_3        16

#define GPIO_DEBUG_WRITE_REQ_PIN        NRF_GPIO_PIN_MAP(1,10)
#define GPIO_DEBUG_WRITE_DONE_PIN       NRF_GPIO_PIN_MAP(1,11)

#define GPIO_DEBUG_READ_PIN
#define GPIO_DEBUG_ERASE_PIN

 #if 1///NRF_MODULE_ENABLED(NRF_FSTORAGE_QSPI)

 #define QSPI_STD_CMD_WRSR   0x01
 #define QSPI_STD_CMD_RSTEN  0x66
 #define QSPI_STD_CMD_RST    0x99

#define QSPI_TEST_DATA_SIZE 0x20//256

// Size per page (0x1000) -- 4096KB
#define PAGE_BLOCK_SIZE     0x1000

// #define WAIT_FOR_PERIPH() do { \
//                while (!m_finished) {} \
//                m_finished = false;    \
// //} while (0)

//#define QSPI_BLOCK_MODE

static uint32_t volatile m_qspi_base_address = 0;

//static volatile bool m_finished = false;
static volatile flash_qspi_evt_t m_flash_qspi_write_evt;// = eflash_QSPI_IDLE;
static volatile flash_qspi_evt_t m_flash_qspi_erase_evt;// = eflash_QSPI_IDLE;
static volatile flash_qspi_evt_t m_flash_qspi_read_evt;// = eflash_QSPI_IDLE;

static uint8_t m_buffer_tx[QSPI_TEST_DATA_SIZE];
static uint8_t m_buffer_rx[QSPI_TEST_DATA_SIZE];

static bool flash_qspi_is_initialized = false;

static void waiting_for_qspi_write_peripheral(flash_qspi_evt_t const source_evt, flash_qspi_evt_t const target_evt)
{
#ifndef QSPI_BLOCK_MODE
        m_flash_qspi_write_evt = source_evt;
        NRF_LOG_DEBUG("waiting_for_qspi_peripheral, evt = %d, %d", m_flash_qspi_write_evt, target_evt);
        NRF_LOG_FLUSH();
        do {
        } while (m_flash_qspi_write_evt != target_evt);
#endif
}

static void waiting_for_qspi_erase_peripheral(flash_qspi_evt_t const source_evt, flash_qspi_evt_t const target_evt)
{
#ifndef QSPI_BLOCK_MODE
        m_flash_qspi_erase_evt = source_evt;
        NRF_LOG_DEBUG("waiting_for_qspi_peripheral, evt = %d, %d", m_flash_qspi_erase_evt, target_evt);
        NRF_LOG_FLUSH();
        do {
        } while (m_flash_qspi_erase_evt != target_evt);
#endif
}

static void waiting_for_qspi_read_peripheral(flash_qspi_evt_t const source_evt, flash_qspi_evt_t const target_evt)
{
#ifndef QSPI_BLOCK_MODE
        m_flash_qspi_read_evt = source_evt;
        NRF_LOG_DEBUG("waiting_for_qspi_peripheral, evt = %d, %d", m_flash_qspi_read_evt, target_evt);
        NRF_LOG_FLUSH();
        do {
        } while (m_flash_qspi_read_evt != target_evt);
#endif
}

static module_flash_qspi_evt_handler_t m_cb_flash_qspi_handler;

static void qspi_callback_handler(nrf_drv_qspi_evt_t event, void * p_context)
{
        UNUSED_PARAMETER(event);
        UNUSED_PARAMETER(p_context);

//        m_finished = true;
        module_flash_qspi_report_t evt;

        //NRF_LOG_INFO("qspi_handler %d", m_flash_qspi_evt);
        if (m_flash_qspi_read_evt == eflash_QSPI_READ_REQ)
        {
                evt.evt_type = eflash_QSPI_READ_DONE;
                m_flash_qspi_read_evt = eflash_QSPI_READ_DONE;
        }
        if (m_flash_qspi_erase_evt == eflash_QSPI_ERASE_REQ)
        {
                evt.evt_type = eflash_QSPI_ERASE_DONE;
                m_flash_qspi_erase_evt = eflash_QSPI_ERASE_DONE;
        }
        if (m_flash_qspi_write_evt == eflash_QSPI_WRITE_REQ)
        {
                evt.evt_type = eflash_QSPI_WRITE_DONE;
                m_flash_qspi_write_evt = eflash_QSPI_WRITE_DONE;
                nrf_gpio_pin_toggle(GPIO_DEBUG_WRITE_DONE_PIN);
        }

        m_cb_flash_qspi_handler(&evt);

}

static void configure_memory()
{
        uint8_t temporary = 0x40;
        uint32_t err_code;
        nrf_qspi_cinstr_conf_t cinstr_cfg = {
                .opcode    = QSPI_STD_CMD_RSTEN,
                .length    = NRF_QSPI_CINSTR_LEN_1B,
                .io2_level = true,
                .io3_level = true,
                .wipwait   = true,
                .wren      = true
        };

        // Send reset enable
        err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
        APP_ERROR_CHECK(err_code);

        // Send reset command
        cinstr_cfg.opcode = QSPI_STD_CMD_RST;
        err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
        APP_ERROR_CHECK(err_code);

        // Switch to qspi mode
        cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
        cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
        err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, &temporary, NULL);
        APP_ERROR_CHECK(err_code);
}

ret_code_t flash_qspi_read(uint32_t * p_rx_buffer, uint32_t const src_address, uint32_t rx_buffer_length)
{
        ASSERT(p_rx_buffer);
        ASSERT(rx_buffer_length > 0);

        if (rx_buffer_length > PAGE_BLOCK_SIZE)
                return NRF_ERROR_INVALID_ADDR;

        if (flash_qspi_is_initialized == false)
                return NRF_ERROR_INVALID_STATE;

        uint32_t read_address = m_qspi_base_address + src_address;

        module_flash_qspi_report_t evt;
        evt.evt_type = eflash_QSPI_READ_REQ;
        m_cb_flash_qspi_handler(&evt);

        NRF_LOG_DEBUG("flash_qspi_read: address = %x, len = %x", read_address, rx_buffer_length);

        ret_code_t err_code;
        err_code = nrf_drv_qspi_read(p_rx_buffer, rx_buffer_length, read_address);
        APP_ERROR_CHECK(err_code);

        waiting_for_qspi_read_peripheral(eflash_QSPI_READ_REQ, eflash_QSPI_READ_DONE);

        NRF_LOG_INFO("Data read");

        return NRF_SUCCESS;
}

ret_code_t flash_qspi_write(uint32_t * p_tx_buffer, uint32_t const dst_address, uint32_t tx_buffer_length)
{
        ASSERT(p_tx_buffer);
        ASSERT(tx_buffer_length > 0);

        if (tx_buffer_length > PAGE_BLOCK_SIZE)
        {
                return NRF_ERROR_INVALID_ADDR;
        }

        if (flash_qspi_is_initialized == false)
                return NRF_ERROR_INVALID_STATE;

        uint32_t write_address = m_qspi_base_address + dst_address;

//        NRF_LOG_INFO("flash_qspi_write len = %d", tx_buffer_length*4);
//        NRF_LOG_HEXDUMP_INFO(p_tx_buffer, tx_buffer_length);
        NRF_LOG_INFO("flash_qspi_write address = %x, len = %x", write_address, tx_buffer_length);

        module_flash_qspi_report_t evt;
        evt.evt_type = eflash_QSPI_WRITE_REQ;
        m_cb_flash_qspi_handler(&evt);

        nrf_gpio_pin_toggle(GPIO_DEBUG_WRITE_REQ_PIN);

        ret_code_t err_code;
        err_code = nrf_drv_qspi_write(p_tx_buffer, tx_buffer_length, dst_address);// write_address);
        APP_ERROR_CHECK(err_code);

        // Waiting for the callback
        waiting_for_qspi_write_peripheral(eflash_QSPI_WRITE_REQ, eflash_QSPI_WRITE_DONE);

        // WAIT_FOR_PERIPH();
        NRF_LOG_INFO("Process of writing data start");

        return NRF_SUCCESS;
}


ret_code_t flash_qspi_page_erase(uint32_t page_number)
{
//        m_finished = false;
        ret_code_t err_code;

        if (flash_qspi_is_initialized == false)
                return NRF_ERROR_INVALID_STATE;

        uint32_t erase_address = m_qspi_base_address + page_number * PAGE_BLOCK_SIZE;

        NRF_LOG_INFO("Address (0x%08x) Erase Page number %d", erase_address, page_number);

        module_flash_qspi_report_t evt;
        evt.evt_type = eflash_QSPI_ERASE_REQ;
        m_cb_flash_qspi_handler(&evt);

        err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, erase_address);
        APP_ERROR_CHECK(err_code);

        waiting_for_qspi_erase_peripheral(eflash_QSPI_ERASE_REQ, eflash_QSPI_ERASE_DONE);

        // WAIT_FOR_PERIPH();
        NRF_LOG_INFO("Process of erasing first block start");

        return NRF_SUCCESS;
}


void module_flash_qspi_uninit(void)
{
        nrf_drv_qspi_uninit();

        module_flash_qspi_report_t evt;
        evt.evt_type = eflash_QSPI_UNINIT;
        m_cb_flash_qspi_handler(&evt);

        flash_qspi_is_initialized = false;

}

ret_code_t module_flash_qspi_init(module_flash_qspi_init_t * p_qspi_init)
{
        ret_code_t err_code;
        nrf_drv_qspi_config_t config = NRF_DRV_QSPI_DEFAULT_CONFIG;

        m_qspi_base_address = p_qspi_init->base_address;
        m_cb_flash_qspi_handler = p_qspi_init->evt_handler;

        if (flash_qspi_is_initialized)
                return NRF_ERROR_INVALID_STATE;

#ifndef QSPI_BLOCK_MODE
        err_code = nrf_drv_qspi_init(&config, qspi_callback_handler, NULL);
#else
        err_code = nrf_drv_qspi_init(&config, NULL, NULL);
#endif
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("QSPI Initialized");

        nrf_gpio_cfg_output(GPIO_DEBUG_WRITE_REQ_PIN);
        nrf_gpio_pin_clear(GPIO_DEBUG_WRITE_REQ_PIN);

        nrf_gpio_cfg_output(GPIO_DEBUG_WRITE_DONE_PIN);
        nrf_gpio_pin_clear(GPIO_DEBUG_WRITE_DONE_PIN);

        configure_memory();

        flash_qspi_is_initialized = true;

//        err_code = flash_qspi_page_erase(0xFD);
//        APP_ERROR_CHECK(err_code);
//
//        err_code = flash_qspi_page_erase(0xFE);
//        APP_ERROR_CHECK(err_code);
//
//        err_code = flash_qspi_page_erase(0xFF);
//        APP_ERROR_CHECK(err_code);

        return NRF_SUCCESS;
}

#endif //#if NRF_MODULE_ENABLED(NRF_FSTORAGE_QSPI)
