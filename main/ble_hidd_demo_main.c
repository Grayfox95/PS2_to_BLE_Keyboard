/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "ps2_dev.h"
#include "ps2_to_hid.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"
#define HID_MAX_KEY_PRESSED 8

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool hid_conn_status = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID BT Keyboard"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            // The BLE is connected, stop the advertising to reduce power consumptions. 
            esp_ble_gap_stop_advertising();
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            hid_conn_status = false;
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N0);
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            // The BLE is disconnected, start the advertising. 
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            ps2_update_led(*(param->led_write.data));

            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        
        if(param->ble_security.auth_cmpl.success)
        {
            hid_conn_status = true;
            // Set low transmission power
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N9);
        }
        
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

int8_t search_elem(uint8_t elem, uint8_t * buffer, uint8_t tail)
{
    uint8_t i = 0;
    for(i=0; i < tail; i++)
    {
        if(buffer[i] == elem)
        {
            // The element is present.
            return i;
        }
    }
    return -1;
}

bool hid_dev_is_connected(void)
{
    return hid_conn_status;
}

int8_t hid_scan_code_conversion(ps2_code_t ps2_code, uint16_t* hid_code, key_mask_t* mask, bool* consumer)
{
    int8_t is_consumer = 0;

    // Initialize hid_code, mask and consumer to zero.
    *hid_code = 0;
    *mask = 0;
    *consumer = false;

    // Check if the ps2 code is extended
    if(ps2_code.extended_f == true)
    {
        if(ps2_code.code <= MAX_EXT_SCANCODE)
        {
            switch(ps2_code.code)
            {
                case 0x1F:  // LEFT GUI
                    *mask = LEFT_GUI_KEY_MASK;
                break;
                case 0x14:  // RIGHT CTRL
                    *mask = RIGHT_CONTROL_KEY_MASK;
                break;
                case 0x11:  // RIGHT ALT
                    *mask = RIGHT_ALT_KEY_MASK;
                break;
                case 0x27:  // RIGHT GUI
                    *mask = RIGHT_GUI_KEY_MASK;
                break;
                default:
                    // It is an extended code, convert from the related lookup table.
                    *hid_code = ext_scancode_to_hid[ps2_code.code];
                    // Check if it is a consumer value.
                    is_consumer = search_elem(*hid_code, hid_consumer_values, HID_CONSUMER_MAX_VALUE);
                    if(is_consumer >= 0)
                    {
                        *consumer = true;
                    }
                    else 
                    {
                        *consumer = false;
                    }
                break;
            }
        }
        else
        {
            // It is not inside the extended values.
            ESP_LOGE(HID_DEMO_TAG, "PS2 ext code out of max value.");
            return -1;
        }
    }
    else
    {
        // It cannot be a consumer value, set consumer to false.
        *consumer = false;
        
        if(ps2_code.code <= MAX_SCANCODE)
        {
            // It is not an extended code, verify if it is a mask key or not.
            switch(ps2_code.code)
            {
                case 0x11:  // LEFT ALT
                    *mask = LEFT_ALT_KEY_MASK;
                break;
                case 0x12:  // LEFT SHIFT
                    *mask = LEFT_SHIFT_KEY_MASK;
                break;
                case 0x14:  // LEFT CTRL
                    *mask = LEFT_CONTROL_KEY_MASK;
                break;
                case 0x59:  // RIGHT SHIFT
                    *mask = RIGHT_SHIFT_KEY_MASK;
                break;
                default:
                    // Convert it from the related lookup table.
                    *hid_code = scancode_to_hid[ps2_code.code];
                break;
            }
            
        }
        else
        {
            // It is out from the maximum PS2 code.
            ESP_LOGE(HID_DEMO_TAG, "PS2 code out of max value.");
            return -1;
        }
    }
    return 0;
}

void hid_demo_task(void *pvParameters)
{
    ps2_code_t ps2_code;
    uint16_t hid_code = HID_KEY_RESERVED;
    uint8_t tail = 0;
    uint8_t keys_pressed[HID_MAX_KEY_PRESSED] = {0};
    key_mask_t keys_mask = 0;
    key_mask_t mask = 0;
    uint8_t update_f = 0;
    bool consumer_f = false;
    bool consumer_pressed = false;
    int64_t time = 0;
    int8_t res = -1;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1) {
        if (sec_conn) 
        {
            // Wait for the reception of a new PS2 code.
            if (xQueueReceive(ps2_tx_code_queue, &ps2_code, portMAX_DELAY)) 
            {
                int8_t i = 0; 
                //printf("%dT%d0x%X\n",tail, ps2_code.type, ps2_code.code);
                //printf("T: %d, C: %d\n", ps2_code.type, ps2_code.code);
                // A new code is received, convert it into HID format.
                res = hid_scan_code_conversion(ps2_code, &hid_code, &mask, &consumer_f);
            
                // Check if the conversion returned correctly and if the value is different than 0
                if(res == 0)
                {
                    // Check if it a consumer value or not.
                    if (consumer_f == false)
                    {
                        // Check if it is a simple code
                        if(hid_code != 0)
                        {
                        // Search if it is already present in the buffer and return index.
                            i = search_elem(hid_code, keys_pressed, tail);
                            // Check for the ps2 code type.
                            switch(ps2_code.type)
                            {
                                case MAKE_CODE:
                                    // Check if the character is not present.
                                    if(i < 0)
                                    {
                                        // Check for the maximum number of pressed button.
                                        if (tail < HID_MAX_KEY_PRESSED)
                                        {
                                            // Add character in the buffer.
                                            keys_pressed[tail] = hid_code;
                                            tail++; 
                                            update_f = 1;
                                        }
                                    } 
       
                                break;
                                case BREAK_CODE:
                                    uint8_t k = 0;
                                    if(i >= 0)
                                    {
                                        // Character is present, then shift all the remaining elements on the left.
                                        for(k=i; k < tail; k++)
                                        {
                                            keys_pressed[k] = keys_pressed[k+1];
                                        }
                                        keys_pressed[tail] = 0x00;
                                        tail--;
                                        update_f = 1;
                                        break;
                                    }
                                break;
                                default:
                                break;
                            }
                             
                        }
                        else if(mask != 0)
                        {
                            switch(ps2_code.type)
                            {
                                case MAKE_CODE:
                                    keys_mask |= mask;
                                    update_f = 1;
                                break;
                                case BREAK_CODE:
                                    keys_mask &= ~mask;
                                    update_f = 1;
                                break;
                                default:
                                    update_f = 0;
                                break;
                            }
                        }

                        if(update_f == 1)
                        {
                            //  printf("h=%d\n", hid_code);

                            // for (uint8_t i = 0; i<HID_MAX_KEY_PRESSED; i++)
                            // {
                            //     printf("%d\t", keys_pressed[i]);
                            // }
                            // printf("\n\n");
                            esp_hidd_send_keyboard_value(hid_conn_id, keys_mask, keys_pressed, tail+1);
                            update_f = 0;
                        }
                    }
                    else
                    {
                        if(hid_code != 0)
                        {
                            // It is a consumer value.
                            switch(ps2_code.type)
                            {
                                case MAKE_CODE:
                                    consumer_pressed = true;
                                break;
                                case BREAK_CODE:
                                    consumer_pressed = false;
                                break;
                                default:
                                break;
                            }
                        }
                        
                        //printf("hid_conscode=%d\n", hid_code);
                        esp_hidd_send_consumer_value(hid_conn_id, hid_code, consumer_pressed);

                    }
                }
                
                //printf("HID%d\n", hid_code);
                /*esp_hidd_send_consumer_value(hid_conn_id, HID_KEY_LEFT_ARROW, true);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                esp_hidd_send_consumer_value(hid_conn_id, HID_KEY_LEFT_ARROW, false);*/
            }
        }
        else
        {
            // Waiting for a connection
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            printf("Waiting for a connection...\n");
        }
    }
}


void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed", __func__);
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
    }

    // Register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // Create hid_task and pin it to core 0.
    xTaskCreatePinnedToCore(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL, 0);

    // Create ps2_task and pin it to core 1.
    xTaskCreatePinnedToCore(ps2_data_task, "ps2_data_task", 4096, NULL, 20, NULL, 1); 
    
}
