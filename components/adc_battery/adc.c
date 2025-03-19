#include "adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

typedef struct {
    float voltage;
    int percentage;
} battery_lookup_t;

battery_lookup_t battery_table[] = {
    {3.300, 100}, {3.259, 99}, {3.219, 98}, {3.181, 97}, {3.141, 96}, {3.104, 95},
    {3.064, 94}, {3.024, 92}, {2.986, 89}, {2.945, 86}, {2.907, 83}, {2.867, 79},
    {2.827, 74}, {2.791, 69}, {2.751, 62}, {2.713, 54}, {2.673, 46}, {2.635, 39},
    {2.594, 31}, {2.555, 25}, {2.516, 19}, {2.476, 14}, {2.438, 11}, {2.398, 8},
    {2.358, 6}, {2.320, 5}, {2.280, 3}, {2.220, 2}, {2.202, 1}, {2.164, 1},
    {2.084, 0}
};   


// battery_lookup_t battery_table[] = {
//     {3.300, 100}, {3.259, 98}, {3.219, 96}, {3.181, 94}, {3.141, 92}, {3.104, 90},
//     {3.064, 88}, {3.024, 86}, {2.986, 84}, {2.945, 82}, {2.907, 80}, {2.867, 78},
//     {2.827, 76}, {2.791, 74}, {2.751, 72}, {2.713, 70}, {2.673, 68}, {2.635, 66},
//     {2.594, 64}, {2.555, 62}, {2.516, 60}, {2.476, 58}, {2.438, 56}, {2.398, 54},
//     {2.358, 52}, {2.320, 50}, {2.280, 48}, {2.220, 46}, {2.202, 44}, {2.164, 42},
//     {2.084, 30}, {2.045, 10}, {2.005, 0}
// };

static const char *TAG = "BATTERY";

adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;
bool do_calibration = false;

// Hàm khởi tạo ADC
bool adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    if (adc_oneshot_new_unit(&init_config, &adc_handle) != ESP_OK) {
        //ESP_LOGE(TAG, "ADC init failed");
        return false;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11, // Sửa thành giá trị hợp lệ
    };
    if (adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &config) != ESP_OK) {
        //ESP_LOGE(TAG, "ADC channel config failed");
        return false;
    }

    // Kiểm tra và khởi tạo calibration (sửa đổi)
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_calibration = true;
    } else {
        //ESP_LOGW(TAG, "ADC Calibration not supported, using raw values");
    }
    return true;
}

// Hàm đọc điện áp pin
float read_battery_voltage(void) {
    int raw_value = 0;
    int voltage_mv = 0;
    int total_raw = 0;

    // if (adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &raw_value) != ESP_OK) {
    //     //ESP_LOGE(TAG, "Failed to read ADC");
    //     return 0.0;
    // }

    for (int i = 0; i < 50; i++) {
        int temp_raw = 0;
        if (adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &temp_raw) == ESP_OK) {
            total_raw += temp_raw;
        } else {
            // Nếu đọc lỗi, bỏ qua mẫu này và tiếp tục
            i--;
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // Đợi 5ms giữa các lần đọc để tránh nhiễu
    }

    raw_value = total_raw / 50;  // Tính trung bình

    if (do_calibration) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw_value, &voltage_mv);
    } else {
        // Nếu không có calibration, tính toán thủ công
        float max_adc_value = 4095.0;
        float reference_voltage = 3.3; // Điện áp tham chiếu khi phân áp
        voltage_mv = (raw_value * reference_voltage) / max_adc_value * 1000;
    }

    float battery_voltage = voltage_mv / 1000.0; // Chuyển đổi mV -> V
    //ESP_LOGI(TAG, "Battery Voltage: %.3fV", battery_voltage);
    return battery_voltage;
}

// Hàm chuyển đổi điện áp thành phần trăm pin
int voltage_to_percentage(float voltage) {
    size_t table_size = sizeof(battery_table) / sizeof(battery_table[0]);
    for (size_t i = 0; i < table_size; i++) {
        if (voltage >= battery_table[i].voltage) {
            return battery_table[i].percentage;
        }
    }
    return 0;
}
