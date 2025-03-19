#ifndef MAIN_H
#define MAIN_H

#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>  
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <stdio.h>
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <unistd.h>
#include "esp_sleep.h"
#include "sim_a7680c.h"
#include "gps.h"
#include "my_timer.h"
#include "adc.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include <stdbool.h>
#include "esp_mac.h"
#include "mpu6050.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "cmd_i2ctools.h"
#include "driver/timer.h"
#include <time.h>
#include <sys/time.h>
#include "soc/soc_caps.h"
#include "driver/rtc_io.h"
#include "nvs.h"  
#include "esp_timer.h"
#include "blue.h"

// Define GPIO pins
#define INT_MPU_PIN            GPIO_NUM_2
#define GPIO_GPS_TRIGGER        GPIO_NUM_13
#define GPIO_SIM_TRIGGER   GPIO_NUM_12
#define GPIO_PEN           GPIO_NUM_32
#define GPIO_GPS_PPS     GPIO_NUM_4
#define BUZZER     GPIO_NUM_18

extern int count;
extern bool wakeup_by_timer;
extern bool mpu_when_wakeup;
extern bool call_case_normal;
extern bool stop_task_scan_bluetooth;
extern char time_buffer_module_sim[10];  // ✅ Khai báo extern đúng kiểu


extern RTC_DATA_ATTR float stored_roll;
extern RTC_DATA_ATTR float stored_pitch;
extern RTC_DATA_ATTR float stored_yaw;
extern RTC_DATA_ATTR double fix_lattitude;
extern RTC_DATA_ATTR double fix_longtitude;
extern RTC_DATA_ATTR int radius;
extern RTC_DATA_ATTR int emergency;
      
bool is_valid_time_format(const char *time_str);   
void set_esp_sleep_time(const char* current_time, const char* wakeup_time); 
void start_uart_task(void);
void stop_uart_task(void);  
// void configure_led(void);
// void blink_led(void);
  
// // Hàm dọn dẹp GPIO trước khi vào chế độ ngủ
// void set_sleep_gpio();    
#endif // MAIN_H
