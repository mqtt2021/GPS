#ifndef BLUE_H
#define BLUE_H


//Library

#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include <math.h>  // Thư viện toán học
#include <inttypes.h>
#include "sim_a7680c.h"
#include "main.h"
#include "esp_gap_ble_api.h"

#define BLE_ADV_NAME "ESP32_BLE"


//DEFINE
// #define GAP_TAG  "GAP"
// #define MAX_DEVICES 20
// #define TX_POWER -59   // Công suất phát tín hiệu mặc định ở khoảng cách 1m
// #define ENV_FACTOR 2.5 // Hệ số suy hao môi trường (thay đổi theo môi trường)

// typedef enum {
//     APP_GAP_STATE_IDLE = 0,
//     APP_GAP_STATE_DEVICE_DISCOVERING,
//     APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
//     APP_GAP_STATE_SERVICE_DISCOVERING,
//     APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
// } app_gap_state_t;


// typedef struct {
//     bool dev_found;
//     uint8_t bdname_len;
//     uint8_t eir_len;
//     int8_t rssi;
//     uint32_t cod;
//     uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
//     uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
//     esp_bd_addr_t bda;
//     app_gap_state_t state;
//     uint8_t missing_count;  // Số lần thiết bị không xuất hiện liên tiếp
// } app_gap_cb_t;

// extern app_gap_cb_t devices[MAX_DEVICES];  // Mảng lưu danh sách thiết bị
extern bool is_init_BLE;
// extern bool scan_enabled; // Biến để kiểm soát việc dừng quét

// void bt_app_gap_start_up(void);
// void print_device_list(void);
// void stop_bluetooth_scan(void);
// float rssi_to_distance(int rssi);
// void stop_scan(void);
// void start_scan(void);   
// char *bda2str(esp_bd_addr_t bda, char *str, size_t size);
// #ifdef __cplusplus
// }
// #endif
void init_BLE(void);
void start_advertising_BLE(void);
void stop_adertising_BLE(void);


#endif // SIM_A7680C_H   
