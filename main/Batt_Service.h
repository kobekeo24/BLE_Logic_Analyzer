#ifndef BATT_SERVICE_H
#define BATT_SERVICE_H

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define GATTS_SERVICE_UUID_TEST_B   0x180F//Battery Service
#define GATTS_CHAR_UUID_TEST_B      0x2A19//Battery Level
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define PROFILE_B_APP_ID 1


extern void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
#endif