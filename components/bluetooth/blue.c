#include "blue.h"

bool is_init_BLE = false;
// Cấu hình dữ liệu quảng bá BLE
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            //esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}


void init_BLE(void){
    esp_err_t ret;

    // Khởi tạo NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Giải phóng bộ nhớ Bluetooth Classic
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Cấu hình và bật bộ điều khiển BLE
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Khởi tạo Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Đăng ký callback cho GAP
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    // Đặt tên thiết bị BLE
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(BLE_ADV_NAME));

    // Cấu hình dữ liệu quảng bá
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    is_init_BLE = true;
}


void start_advertising_BLE(void){
    esp_ble_gap_start_advertising(&adv_params);
    //printf("Đang quảng bá BLE\n");
}
void stop_adertising_BLE(void){
    esp_ble_gap_stop_advertising();
    //printf("Dừng quảng bá BLE\n");
}



