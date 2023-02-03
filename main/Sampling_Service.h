#ifndef SAMPLING_SERVICE_H
#define SAMPLING_SERVICE_H

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "gatts_App.h"

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)
#define PROFILE_A_APP_ID 0
#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333

#define GATTS_SAMPLING_CHAR_NUM     16
#define GATTS_HANDLE_VAL_PER_CHAR   2
#define GATTS_HANDLE_VAL_PER_DES    1

#define GATTS_NUM_HANDLE_TEST_A     4 + (GATTS_SAMPLING_CHAR_NUM * GATTS_HANDLE_VAL_PER_CHAR) + (GATTS_SAMPLING_CHAR_NUM * GATTS_HANDLE_VAL_PER_DES) 


extern esp_attr_value_t gatts_demo_char1_val;


extern void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
#endif