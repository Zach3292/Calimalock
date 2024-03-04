/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


/****************************************************************************
*
* This file is used for eddystone receiver.
*
****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "esp_eddystone_protocol.h"
#include "esp_eddystone_api.h"

#include "led_strip.h"

// GPIO assignment
#define LED_STRIP__GPIO 3
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 12
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define GPIO_OUTPUT_IO_0     2 
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))

static const char* DEMO_TAG = "EDDYSTONE_DEMO";
static const char* TAG = "LED_STRIP";

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-C3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void esp_eddystone_show_inform(const esp_eddystone_result_t* res);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static void esp_eddystone_show_inform(const esp_eddystone_result_t* res)
{
    switch(res->common.frame_type)
    {
        case EDDYSTONE_FRAME_TYPE_UID: {
            ESP_LOGI(DEMO_TAG, "Eddystone UID inform:");
            ESP_LOGI(DEMO_TAG, "Measured power(RSSI at 0m distance):%d dbm", res->inform.uid.ranging_data);
            ESP_LOGI(DEMO_TAG, "EDDYSTONE_DEMO: Namespace ID:0x");
            esp_log_buffer_hex(DEMO_TAG, res->inform.uid.namespace_id, 10);
            ESP_LOGI(DEMO_TAG, "EDDYSTONE_DEMO: Instance ID:0x");
            esp_log_buffer_hex(DEMO_TAG, res->inform.uid.instance_id, 6);
            break;
        }
        case EDDYSTONE_FRAME_TYPE_URL: {
            ESP_LOGI(DEMO_TAG, "Eddystone URL inform:");
            ESP_LOGI(DEMO_TAG, "Measured power(RSSI at 0m distance):%d dbm", res->inform.url.tx_power);
            ESP_LOGI(DEMO_TAG, "URL: %s", res->inform.url.url);
            break;
        }
        case EDDYSTONE_FRAME_TYPE_TLM: {
            ESP_LOGI(DEMO_TAG, "Eddystone TLM inform:");
            ESP_LOGI(DEMO_TAG, "version: %d", res->inform.tlm.version);
            ESP_LOGI(DEMO_TAG, "battery voltage: %d mV", res->inform.tlm.battery_voltage);
            ESP_LOGI(DEMO_TAG, "beacon temperature in degrees Celsius: %6.1f", res->inform.tlm.temperature);
            ESP_LOGI(DEMO_TAG, "adv pdu count since power-up: %" PRIu32, res->inform.tlm.adv_count);
            ESP_LOGI(DEMO_TAG, "time since power-up: %" PRIu32 " s", (res->inform.tlm.time)/10);
            break;
        }
        default:
            break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 10000;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan start failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Start scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    esp_eddystone_result_t eddystone_res;
                    memset(&eddystone_res, 0, sizeof(eddystone_res));
                    esp_err_t ret = esp_eddystone_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, &eddystone_res);
                    if (ret) {
                        // error:The received data is not an eddystone frame packet or a correct eddystone frame packet.
                        // just return
                        return;
                    } else {
                        // The received adv data is a correct eddystone frame packet.
                        // Here, we get the eddystone infomation in eddystone_res, we can use the data in res to do other things.
                        // For example, just print them:
                        ESP_LOGI(DEMO_TAG, "--------Eddystone Found----------");
                        esp_log_buffer_hex("EDDYSTONE_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                        esp_eddystone_show_inform(&eddystone_res);

                        if (eddystone_res.common.frame_type == EDDYSTONE_FRAME_TYPE_UID)
                        {   
                            if (eddystone_res.inform.uid.namespace_id[0] == 0x00)
                            {
                                // ajouter une fonction qui allume les LEDs
                                // ajouter un interrupt update pour le bouton pour unlock
                                // ajouter un interrupt update pour le reed pour unlock

                            }
                            else if(eddystone_res.inform.uid.namespace_id[0] == 0x01){
                                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                            }
                            else if(eddystone_res.inform.uid.namespace_id[0] == 0x02){
                                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Stop scan successfully");
            }
            break;
        }
        default:
            break;
    }
}

void esp_eddystone_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG,"Register callback");

    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void esp_eddystone_init(void)
{
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    esp_bluedroid_enable();
    esp_eddystone_appRegister();
}

void set_led_fill(led_strip_handle_t led_strip, uint8_t red, uint8_t green, uint8_t blue){
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
    }
    /* Refresh the strip to send data */
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void clear_led(led_strip_handle_t led_strip){
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
}

void app_main(void)
{
    // //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // //disable pull-down mode
    io_conf.pull_down_en = 0;
    // //disable pull-up mode
    io_conf.pull_up_en = 0;
    // //configure GPIO with the given settings
    gpio_config(&io_conf);

    led_strip_handle_t led_strip = configure_led();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_eddystone_init();

    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);

    
}
