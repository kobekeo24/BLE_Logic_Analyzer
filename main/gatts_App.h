#ifndef GATTS_APP_H
#define GATTS_APP_H
#include "driver/gpio.h"
#include "driver/adc.h"

#define CHANNEL_A_ADC			ADC1_CHANNEL_6
#define CHANNEL_B_ADC			ADC1_CHANNEL_7

#define PROFILE_NUM 2

#define TEST_DEVICE_NAME            "ESP_GATTS_DEMO"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define GATTS_MAX_CHARS             20

#define PREPARE_BUF_MAX_SIZE 1024

#define GATTS_TAG "GATTS_DEMO"

typedef struct  {
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
}gatts_char_inst;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    gatts_char_inst charList[GATTS_MAX_CHARS];
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

extern struct gatts_profile_inst gl_profile_tab[PROFILE_NUM];

extern uint16_t getADCVal(adc1_channel_t ADC_Channel);

extern esp_ble_adv_params_t adv_params;
extern esp_gatt_char_prop_t a_property;
extern uint8_t adv_service_uuid128[32];
extern uint8_t adv_config_done;

extern void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
extern void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

#endif