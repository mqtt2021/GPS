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
    printf("Đang quảng bá BLE\n");
}
void stop_adertising_BLE(void){
    esp_ble_gap_stop_advertising();
    printf("Dừng quảng bá BLE\n");
}


// static bool is_ble_advertising = false; // Biến toàn cục để lưu trạng thái quảng bá
// // Cấu hình dữ liệu quảng bá BLE
// static esp_ble_adv_data_t adv_data = {
//     .set_scan_rsp = false,
//     .include_name = true,
//     .include_txpower = true,
//     .min_interval = 0x20,
//     .max_interval = 0x40,
//     .appearance = 0x00,
//     .manufacturer_len = 0,
//     .p_manufacturer_data = NULL,
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     .service_uuid_len = 0,
//     .p_service_uuid = NULL,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };

// static esp_ble_adv_params_t adv_params = {
//     .adv_int_min = 0x20,
//     .adv_int_max = 0x40,
//     .adv_type = ADV_TYPE_IND,
//     .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
//     .channel_map = ADV_CHNL_ALL,
//     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {

//     if (param == NULL) {
//         ESP_LOGE("GAP", "param NULL!");
//         return;
//     }

//     switch (event) {
//         case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
//             esp_ble_gap_start_advertising(&adv_params);
//             break;
//         case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//             if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGI("GAP", "Quảng bá BLE thành công!");
//                 is_ble_advertising = true; // Đánh dấu đang quảng bá
//             } else {
//                 ESP_LOGE("GAP", "Quảng bá BLE thất bại!");
//                 is_ble_advertising = false; // Đánh dấu không quảng bá
//             }
//             break;
//         case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
//             if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGI("GAP", "Dừng quảng bá BLE thành công!");
//                 is_ble_advertising = false; // Đánh dấu đã dừng quảng bá
//             } else {
//                 ESP_LOGE("GAP", "Dừng quảng bá BLE thất bại!");
//             }
//             break;

//         default:
//             break;
//     }
// }

// void stop_ble_advertising(void) {
//     if (!is_ble_advertising) {
//         ESP_LOGW("BLE", "BLE không đang quảng bá, không cần dừng!");
//         return;
//     }

//     // Dừng quảng bá BLE
//     esp_err_t ret = esp_ble_gap_stop_advertising();
//     if (ret == ESP_OK) {
//         ESP_LOGI("BLE", "Đã dừng quảng bá BLE thành công!");
//     } else {
//         ESP_LOGE("BLE", "Không thể dừng quảng bá BLE: %s", esp_err_to_name(ret));
//     }
// }

// void shutdown_ble(void) {
//     // Tắt Bluedroid
//     if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
//         esp_err_t ret = esp_bluedroid_disable();
//         if (ret == ESP_OK) {
//             ESP_LOGI("BLE", "Đã tắt Bluedroid thành công!");
//         } else {
//             ESP_LOGE("BLE", "Không thể tắt Bluedroid: %s", esp_err_to_name(ret));
//         }
//     }

//     // Hủy khởi tạo Bluedroid
//     if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
//         esp_err_t ret = esp_bluedroid_deinit();
//         if (ret == ESP_OK) {
//             ESP_LOGI("BLE", "Đã hủy khởi tạo Bluedroid thành công!");
//         } else {
//             ESP_LOGE("BLE", "Không thể hủy khởi tạo Bluedroid: %s", esp_err_to_name(ret));
//         }
//     }

//     // Tắt Bluetooth Controller
//     if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
//         esp_err_t ret = esp_bt_controller_disable();
//         if (ret == ESP_OK) {
//             ESP_LOGI("BLE", "Đã tắt Bluetooth Controller thành công!");
//         } else {
//             ESP_LOGE("BLE", "Không thể tắt Bluetooth Controller: %s", esp_err_to_name(ret));
//         }
//     }

//     // Giải phóng bộ nhớ BLE
//     if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
//         esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
//         if (ret == ESP_OK) {
//             ESP_LOGI("BLE", "Đã giải phóng bộ nhớ BLE thành công!");
//         } else {
//             ESP_LOGE("BLE", "Không thể giải phóng bộ nhớ BLE: %s", esp_err_to_name(ret));
//         }
//     }

//     ESP_LOGI("BLE", "BLE đã được tắt hoàn toàn!");
// }

// void reset_bluetooth(void) {
//     // Tắt Bluedroid nếu đang bật
//     if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
//         ESP_LOGI("BT", "Tắt Bluedroid...");
//         esp_err_t ret = esp_bluedroid_disable();
//         if (ret != ESP_OK) {
//             ESP_LOGE("BT", "Không thể tắt Bluedroid: %s", esp_err_to_name(ret));
//             return;
//         }
//     }

//     // Hủy khởi tạo Bluedroid nếu đã được khởi tạo
//     if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
//         ESP_LOGI("BT", "Hủy khởi tạo Bluedroid...");
//         esp_err_t ret = esp_bluedroid_deinit();
//         if (ret != ESP_OK) {
//             ESP_LOGE("BT", "Không thể hủy khởi tạo Bluedroid: %s", esp_err_to_name(ret));
//             return;
//         }
//     }

//     // Tắt Bluetooth Controller nếu đang bật
//     if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
//         ESP_LOGI("BT", "Tắt Bluetooth Controller...");
//         esp_err_t ret = esp_bt_controller_disable();
//         if (ret != ESP_OK) {
//             ESP_LOGE("BT", "Không thể tắt Bluetooth Controller: %s", esp_err_to_name(ret));
//             return;
//         }
//     }

//     // Chờ Bluetooth Controller về trạng thái IDLE
//     int timeout = 0;
//     while (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE && timeout < 10) {
//         ESP_LOGI("BT", "Đang chờ Bluetooth tắt hoàn toàn...");
//         vTaskDelay(pdMS_TO_TICKS(100));
//         timeout++;
//     }
//     if (timeout >= 10) {
//         ESP_LOGE("BT", "Timeout khi chờ Bluetooth tắt!");
//         return;
//     }

//     // Giải phóng bộ nhớ BLE
//     if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
//         ESP_LOGI("BT", "Giải phóng bộ nhớ BLE...");
//         esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
//         if (ret != ESP_OK) {
//             ESP_LOGE("BT", "Không thể giải phóng bộ nhớ BLE: %s", esp_err_to_name(ret));
//             return;
//         }
//     }

//     ESP_LOGI("BT", "Bluetooth đã được reset hoàn toàn!");
// }

// void handle_advertising_BLE(void) {
//     // Kiểm tra xem Bluetooth Controller đã được init chưa
//     esp_bt_controller_status_t status = esp_bt_controller_get_status();
//     if (status != ESP_BT_CONTROLLER_STATUS_IDLE) {
//         ESP_LOGW("BT", "Bluetooth Controller đã ở trạng thái: %d", status);
//         return;
//     }

//     esp_err_t ret;

//     // Khởi tạo NVS
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // Cấu hình và bật bộ điều khiển BLE
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

//     // Khởi tạo Bluedroid
//     ESP_ERROR_CHECK(esp_bluedroid_init());
//     ESP_ERROR_CHECK(esp_bluedroid_enable());

//     // Đăng ký callback cho GAP
//     ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

//     // Đặt tên thiết bị BLE
//     ESP_ERROR_CHECK(esp_ble_gap_set_device_name(BLE_ADV_NAME));

//     // Cấu hình dữ liệu quảng bá
//     ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
// }



// #include "blue.h"

// // Cấu hình dữ liệu quảng bá BLE
// static esp_ble_adv_data_t adv_data = {
//     .set_scan_rsp = false,
//     .include_name = true,
//     .include_txpower = true,
//     .min_interval = 0x20,
//     .max_interval = 0x40,
//     .appearance = 0x00,
//     .manufacturer_len = 0,
//     .p_manufacturer_data = NULL,
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     .service_uuid_len = 0,
//     .p_service_uuid = NULL,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };

// static esp_ble_adv_params_t adv_params = {
//     .adv_int_min = 0x20,
//     .adv_int_max = 0x40,
//     .adv_type = ADV_TYPE_IND,
//     .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
//     .channel_map = ADV_CHNL_ALL,
//     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };


// void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
//     switch (event) {
//         case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
//             esp_ble_gap_start_advertising(&adv_params);
//             break;
//         default:
//             break;
//     }
// }


// void reset_bluetooth(void) {
//     if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
//         ESP_LOGI("BT", "Tắt Bluedroid...");      
//         ESP_ERROR_CHECK(esp_bluedroid_disable());
//     }
//     if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
//         ESP_ERROR_CHECK(esp_bluedroid_deinit());
//     }

//     if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
//         ESP_LOGI("BT", "Tắt Bluetooth Controller...");
//         ESP_ERROR_CHECK(esp_bt_controller_disable());
//     }

//     // Chờ cho đến khi Bluetooth thực sự về trạng thái IDLE
//     while (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {
//         ESP_LOGI("BT", "Đang chờ Bluetooth tắt hoàn toàn...");
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }

//     if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
//         ESP_LOGI("BT", "Giải phóng bộ nhớ BLE...");
//         ESP_ERROR_CHECK(esp_bt_controller_deinit());
//         ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
//     }

//     ESP_LOGI("BT", "Bluetooth đã được reset hoàn toàn!");
// }


// void handle_advertising_BLE(void){

//      // Kiểm tra xem Bluetooth Controller đã được init chưa
//     if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {
//         ESP_LOGW("BT", "Bluetooth Controller đã khởi tạo, bỏ qua init!");
//         return;
//     }

//     esp_err_t ret;

//     // Khởi tạo NVS
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
//         ESP_LOGW("BT", "Bluetooth stack đã chạy, không cần giải phóng bộ nhớ BLE!");
//     } else {
//         ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
//     }
    
//     // Cấu hình và bật bộ điều khiển BLE
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

//     // Khởi tạo Bluedroid
//     ESP_ERROR_CHECK(esp_bluedroid_init());
//     ESP_ERROR_CHECK(esp_bluedroid_enable());

//     // Đăng ký callback cho GAP
//     ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

//     // Đặt tên thiết bị BLE
//     ESP_ERROR_CHECK(esp_ble_gap_set_device_name(BLE_ADV_NAME));

//     // Cấu hình dữ liệu quảng bá
//     ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
// }





// static app_gap_cb_t m_dev_info;
// bool scan_enabled = true; // Biến để kiểm soát việc dừng quét
// app_gap_cb_t devices[MAX_DEVICES];  // Mảng lưu danh sách thiết bị
// int num_devices = 0;  // Số lượng thiết bị đã lưu

// char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
// {
//     if (bda == NULL || str == NULL || size < 18) {
//         return NULL;
//     }

//     uint8_t *p = bda;
//     sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
//             p[0], p[1], p[2], p[3], p[4], p[5]);
//     return str;
// }

// static char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size)
// {
//     if (uuid == NULL || str == NULL) {
//         return NULL;
//     }

//     if (uuid->len == 2 && size >= 5) {
//         sprintf(str, "%04x", uuid->uuid.uuid16);
//     } else if (uuid->len == 4 && size >= 9) {
//         sprintf(str, "%08"PRIx32, uuid->uuid.uuid32);
//     } else if (uuid->len == 16 && size >= 37) {
//         uint8_t *p = uuid->uuid.uuid128;
//         sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
//                 p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
//                 p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
//     } else {
//         return NULL;
//     }

//     return str;
// }

// static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
// {
//     uint8_t *rmt_bdname = NULL;
//     uint8_t rmt_bdname_len = 0;

//     if (!eir) {
//         return false;
//     }

//     rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
//     if (!rmt_bdname) {
//         rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
//     }

//     if (rmt_bdname) {
//         if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
//             rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
//         }

//         if (bdname) {
//             memcpy(bdname, rmt_bdname, rmt_bdname_len);
//             bdname[rmt_bdname_len] = '\0';
//         }
//         if (bdname_len) {
//             *bdname_len = rmt_bdname_len;
//         }
//         return true;
//     }

//     return false;
// }

// static void update_device_info(esp_bt_gap_cb_param_t *param) {
//     char bda_str[18];
//     uint32_t cod = 0;
//     int32_t rssi = -129; /* invalid value */
//     uint8_t *bdname = NULL;
//     uint8_t bdname_len = 0;
//     uint8_t *eir = NULL;
//     uint8_t eir_len = 0;
//     esp_bt_gap_dev_prop_t *p;

//     //ESP_LOGI(GAP_TAG, "Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));

//     for (int i = 0; i < param->disc_res.num_prop; i++) {
//         p = param->disc_res.prop + i;
//         switch (p->type) {
//             case ESP_BT_GAP_DEV_PROP_COD:
//                 cod = *(uint32_t *)(p->val);
//                 break;
//             case ESP_BT_GAP_DEV_PROP_RSSI:
//                 rssi = *(int8_t *)(p->val);
//                 break;
//             case ESP_BT_GAP_DEV_PROP_BDNAME:
//                 bdname_len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN : (uint8_t)p->len;
//                 bdname = (uint8_t *)(p->val);
//                 break;
//             case ESP_BT_GAP_DEV_PROP_EIR:
//                 eir_len = p->len;
//                 eir = (uint8_t *)(p->val);
//                 break;
//             default:
//                 break;
//         }
//     }

//     /* Kiểm tra xem thiết bị đã có trong danh sách chưa */
//     for (int i = 0; i < num_devices; i++) {
//         if (memcmp(devices[i].bda, param->disc_res.bda, ESP_BD_ADDR_LEN) == 0) {
//             /* Nếu đã tồn tại, cập nhật RSSI và đặt lại missing_count */
//             // ESP_LOGI(GAP_TAG, "Updating RSSI for %s: %" PRId32 " -> %" PRId32, 
//             //     bda_str, (int32_t)devices[i].rssi, (int32_t)rssi);
            
//             devices[i].rssi = rssi;
//             devices[i].missing_count = 0;  // Thiết bị xuất hiện, reset bộ đếm
//             return;
//         }
//     }

//     /* Nếu danh sách đầy, không thêm nữa */
//     if (num_devices >= MAX_DEVICES) {
//         //ESP_LOGW(GAP_TAG, "Device list full, skipping device: %s", bda_str);
//         return;
//     }

//     /* Thêm thiết bị mới vào danh sách */
//     app_gap_cb_t *p_dev = &devices[num_devices];
//     memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
//     p_dev->dev_found = true;
//     p_dev->cod = cod;
//     p_dev->rssi = rssi;
//     p_dev->missing_count = 0;  // Thiết lập giá trị ban đầu

//     if (bdname_len > 0) {
//         memcpy(p_dev->bdname, bdname, bdname_len);
//         p_dev->bdname[bdname_len] = '\0';
//         p_dev->bdname_len = bdname_len;
//     }

//     if (eir_len > 0) {
//         memcpy(p_dev->eir, eir, eir_len);
//         p_dev->eir_len = eir_len;
//     }

//     if (p_dev->bdname_len == 0) {
//         get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
//     }

//     //ESP_LOGI(GAP_TAG, "Added device %s, name %s to list", bda_str, p_dev->bdname);
//     num_devices++;  // Tăng số lượng thiết bị đã lưu
// }


// /* Hàm chuyển đổi từ RSSI sang khoảng cách (mét) */
// float rssi_to_distance(int rssi) {
//     return pow(10.0, ((TX_POWER - rssi) / (10.0 * ENV_FACTOR)));
// }

// /* Hàm in danh sách thiết bị */
// void print_device_list() {
//     //ESP_LOGI(GAP_TAG, "==== Device List (%d devices) ====", num_devices);
//     for (int i = 0; i < num_devices; i++) {
//         char bda_str[18];
//         bda2str(devices[i].bda, bda_str, 18);
//         float distance = rssi_to_distance(devices[i].rssi); // Chuyển đổi RSSI -> Khoảng cách
//         // ESP_LOGI(GAP_TAG, "Device %d: %s, Name: %s, RSSI: %" PRId32 ", Distance: %.2f m",
//         //     i + 1, bda_str, devices[i].bdname, (int32_t)devices[i].rssi, distance);
//     }
// }

// static void bt_app_gap_init(void)
// {
//     app_gap_cb_t *p_dev = &m_dev_info;
//     memset(p_dev, 0, sizeof(app_gap_cb_t));

//     p_dev->state = APP_GAP_STATE_IDLE;
// }


// static void remove_lost_devices() {
//     int i = 0;
//     while (i < num_devices) {
//         if (devices[i].missing_count >= 3) {  // Nếu mất tín hiệu 3 lần quét liên tiếp
//             //ESP_LOGW(GAP_TAG, "Removing lost device: %s", bda2str(devices[i].bda, NULL, 0));

//             // Dịch các phần tử còn lại lên để lấp chỗ trống
//             for (int j = i; j < num_devices - 1; j++) {
//                 devices[j] = devices[j + 1];
//             }
//             num_devices--;  // Giảm số lượng thiết bị
//         } else {
//             devices[i].missing_count++;  // Tăng bộ đếm nếu không tìm thấy
//             i++;
//         }
//     }
// }


// static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
// {
//     app_gap_cb_t *p_dev = &m_dev_info;
//     char bda_str[18];
//     char uuid_str[37];

//     switch (event) {
//     case ESP_BT_GAP_DISC_RES_EVT: {
//         update_device_info(param);
//         break;
//     }
//     case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
//         if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
//             ESP_LOGI(GAP_TAG, "Device discovery stopped.");

        
//             if(scan_enabled){
//                 char *json_payload = create_json_from_device_list(); // Chyển list thanh chuỗi json 
//                 //ESP_LOGI("MQTT", "Sending: %s", json_payload);
//                 mqtt_publish("GPS/BlueTooth/G001", json_payload );
//                 free(json_payload); // Giải phóng bộ nhớ sau khi sử dụng
//                 num_devices = 0;   
//                 memset(devices, 0, sizeof(devices)); // Xóa danh sách Device
//                 ESP_LOGI(GAP_TAG, "Restarting Bluetooth scan...");
//                 vTaskDelay(pdMS_TO_TICKS(1000));  
//                 esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
//             }

//             if(!scan_enabled){

//                 num_devices = 0;   
//                 memset(devices, 0, sizeof(devices)); // Xóa danh sách Device

//                 char mqtt_payload[BUFFER_SIZE];

//                 snprintf(mqtt_payload, sizeof(mqtt_payload),
//                         "["
//                         "{\"name\":\"None\",\"value\":\"0\",\"timestamp\":\"00:00:00\"},"
//                         "]"      
//                 );
//                 mqtt_publish("GPS/BlueTooth/G001", mqtt_payload);

//             }
            

//             // if(!scan_enabled){
//             //     stop_task_scan_bluetooth = true;
//             //     ESP_LOGI(GAP_TAG, "stop_task_scan_bluetooth = true\n");
//             // }

//             if (scan_enabled) {
                
//             }






//             if ( (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE ||
//                     p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)
//                     && p_dev->dev_found) {
//                 p_dev->state = APP_GAP_STATE_SERVICE_DISCOVERING;
//                 ESP_LOGI(GAP_TAG, "Discover services ...");
//                 esp_bt_gap_get_remote_services(p_dev->bda);
//             }
//         } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
//             //ESP_LOGI(GAP_TAG, "Discovery started.");
//         }
//         break;
//     }
//     case ESP_BT_GAP_RMT_SRVCS_EVT: {
//         if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
//                 p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
//             p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
//             if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
//                 //ESP_LOGI(GAP_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
//                 for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
//                     esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
//                     //ESP_LOGI(GAP_TAG, "--%s", uuid2str(u, uuid_str, 37));
//                 }
//             } else {
//                 //ESP_LOGI(GAP_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
//             }
//         }
//         break;
//     }
//     case ESP_BT_GAP_RMT_SRVC_REC_EVT:
//     default: {
//         //ESP_LOGI(GAP_TAG, "event: %d", event);
//         break;
//     }
//     }
//     return;
// }


// // Hàm dừng quét
// void stop_bluetooth_scan() {
//     scan_enabled = false; // Đánh dấu để không quét lại nữa
//     //ESP_LOGI(GAP_TAG, "Bluetooth scan manually stopped.");
// }


// void start_scan(void) {
    
//         scan_enabled = true;
//         esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
//         ESP_LOGI(GAP_TAG, "Bluetooth scanning started.");
// }

// void stop_scan(void) {    
//         scan_enabled = false;
//         // esp_bt_gap_cancel_discovery();
//         ESP_LOGI(GAP_TAG, "Bluetooth scanning stopped.");
// }

// void bt_app_gap_start_up(void)
// {

//     char bda_str[18] = {0};
//     /* Initialize NVS — it is used to store PHY calibration data and save key-value pairs in flash memory*/
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK( ret );

//     if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
//         //ESP_LOGW("BT", "Bluetooth stack đã chạy, không cần giải phóng bộ nhớ BLE!");
//     } else {
//         ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
//     }
    

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
//         //ESP_LOGE(GAP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }

//     if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
//         //ESP_LOGE(GAP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }

//     if ((ret = esp_bluedroid_init()) != ESP_OK) {
//         //ESP_LOGE(GAP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }

//     if ((ret = esp_bluedroid_enable()) != ESP_OK) {
//         //ESP_LOGE(GAP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
//         return;
//     }

//     //ESP_LOGI(GAP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));



    
//     /* register GAP callback function */
//     esp_bt_gap_register_callback(bt_app_gap_cb);

//     char *dev_name = "ESP_GAP_INQRUIY";
//     esp_bt_dev_set_device_name(dev_name);

//     /* set discoverable and connectable mode, wait to be connected */
//     esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

//     /* inititialize device information and status */
//     bt_app_gap_init();

//     /* start to discover nearby Bluetooth devices */
//     app_gap_cb_t *p_dev = &m_dev_info;
//     p_dev->state = APP_GAP_STATE_DEVICE_DISCOVERING;
//     //esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
// }
