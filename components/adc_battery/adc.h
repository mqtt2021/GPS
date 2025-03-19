#ifndef ADC_BATTERY_H
#define ADC_BATTERY_H

#include <stdint.h>
#include "driver/adc.h"
//#include "esp_adc_cal.h"
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "soc/soc_caps.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#define BATTERY_ADC_CHANNEL ADC_CHANNEL_7 // GPIO 35 trên ESP32
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_MAX_VOLTAGE 3.3
#define ADC_MIN_VOLTAGE 2.0

// Khai báo hàm khởi tạo ADC
bool adc_init(void);

// Khai báo hàm đọc điện áp pin
float read_battery_voltage(void);

// Khai báo hàm tính dung lượng pin
int voltage_to_percentage(float voltage);
#endif // ADC_BATTERY_H 
