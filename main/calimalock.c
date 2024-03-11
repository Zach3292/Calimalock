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
#include <stdlib.h>
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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "soc/soc_caps.h"

#include "led_strip.h"

// GPIO assignment
#define LED_STRIP_GPIO 3
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 12
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define GPIO_OUTPUT_IO_0     10
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))

#define GPIO_INPUT_A  4
#define GPIO_INPUT_B  5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_A) | (1ULL<<GPIO_INPUT_B))

#define ESP_INTR_FLAG_DEFAULT 0


#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2 // Should be change to the right pin

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static const double voltage_treshold = 3.6; // Should be change to the right value;

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


led_strip_handle_t led_strip;

static const char* DEMO_TAG = "EDDYSTONE";
static const char* TAG = "LED_STRIP";
static const char* BATTERY_TAG = "BATTERY";

bool unlocked = false;
bool button_pressed = false;
bool battery_check = false;

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,   // The GPIO that connected to the LED strip's data line
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

static void set_led_fill(led_strip_handle_t led_strip, uint8_t red, uint8_t green, uint8_t blue){
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
    }
    /* Refresh the strip to send data */
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

static void clear_led(led_strip_handle_t led_strip){
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

static void open_lock_intr(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == GPIO_INPUT_A)
            {   
                button_pressed = true;
            }
            else if (io_num == GPIO_INPUT_B)
            {
                unlocked = true;
            }  
        }
    }
}

static esp_ble_adv_data_t ble_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
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
            uint32_t duration = 0;
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
                            if (eddystone_res.inform.uid.namespace_id[0] == 0x03)
                            {
                                if (eddystone_res.inform.uid.namespace_id[1] == 0x00)
                                {
                                    unlocked = false; 
                                } else if (eddystone_res.inform.uid.namespace_id[1] == 0x01)
                                {
                                    unlocked = true;
                                }  
                            }else if (eddystone_res.inform.uid.namespace_id[0] == 0x06)
                            {
                                esp_restart();
                            }
                            else if (eddystone_res.inform.uid.namespace_id[0] == 0xFB)
                            {
                                battery_check = true;
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
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
            if((err = param->adv_data_raw_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Set raw adv data failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Set raw adv data successfully");
                esp_ble_gap_start_advertising(&ble_adv_params); // always broadcast?
            }
            break;
        }
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:{
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG, "Advertising start failed");
            }
            break;
        }
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:{
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(DEMO_TAG, "Stop adv successfully");
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

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(BATTERY_TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(BATTERY_TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(BATTERY_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(BATTERY_TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(BATTERY_TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(BATTERY_TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(BATTERY_TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}


double readBatteryVoltage(){
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
    if (do_calibration1_chan0) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
    }
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    return voltage[0][0];
}

void loop()
{
    if (button_pressed == true)
    {
        if (unlocked == true)
        {
            gpio_set_level(GPIO_OUTPUT_IO_0, 1);
            for (size_t i = 0; i < 5; i++)
            {
                set_led_fill(led_strip, 0, 10, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                clear_led(led_strip);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
        } else
        {
            for (size_t i = 0; i < 5; i++)
            {
                set_led_fill(led_strip, 10, 0, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                clear_led(led_strip);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        button_pressed = false;
    } else if (battery_check == true)
    {
        if (readBatteryVoltage() > voltage_treshold)
        {
                set_led_fill(led_strip, 0, 10, 0);
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                clear_led(led_strip);
        } else
        {
                set_led_fill(led_strip, 10, 0, 0);
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                clear_led(led_strip);
        battery_check = false;
    }   
    }
}

void loopTask(void *pvParameters)
{
    for(;;) {
        loop();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}




void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO2
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(open_lock_intr, "open_lock_intr", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_A, gpio_isr_handler, (void*) GPIO_INPUT_A);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_B, gpio_isr_handler, (void*) GPIO_INPUT_B);

    gpio_set_level(GPIO_OUTPUT_IO_0, 0);

    led_strip = configure_led();
    clear_led(led_strip);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_eddystone_init();

    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);

    xTaskCreate(loopTask, "loopTask", 8192, NULL, 1, NULL);
}

