#ifndef MPU6050_H
#define MPU6050_H


// #ifdef __cplusplus
// extern "C" {
// #endif

//Library

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"
// #include "cmd_system.h"
#include "cmd_i2ctools.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "freertos/queue.h"
#include <stdbool.h>
#include "driver/timer.h"
#include "esp_mac.h"
#include <time.h>
#include <sys/time.h>
#include "soc/soc_caps.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "sim_a7680c.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "main.h"


//DEFINE
#define I2C_NUM I2C_NUM_0
#define I2C_SCL GPIO_NUM_22
#define I2C_SDA GPIO_NUM_21
#define CONFIG_EXAMPLE_EXT1_WAKEUP_PIN_1 2
#define CONFIG_EXAMPLE_EXT1_WAKEUP_PIN_2 4
#define CONFIG_EXAMPLE_EXT1_WAKEUP_MODE 1


extern volatile bool g_timer_read_mpu_done;
extern volatile bool g_timer_noiseMPU_done;
extern volatile bool alarm_mpu;


// Hàm I2C
esp_err_t i2c_master_init(void);
esp_err_t mpuReadfromReg (uint8_t Reg, uint8_t *ReadBuffer, size_t len);
esp_err_t mpuWriteReg(uint8_t Reg, uint8_t data);
// Khai báo các hàm timer
void my_timer_init_read_mpu(void);
void my_timer_start_read_mpu(void);
void my_timer_delete_read_mpu(void);

void my_timer_noiseMPU_init(void);
void my_timer_noiseMPU_start(void);
void my_timer_noiseMPU_delete(void);


void mpu6050_enable_motion_interrupt(void);
void mpu6050_enable_interrupt_pin(void);
void read_mpu6050_angles_first(float *roll_pointer, float *pitch_pointer, float *yaw_pointer);
void read_mpu6050_angles_alarm(float *stored_roll, float *stored_pitch, float *stored_yaw);

// #ifdef __cplusplus
// }
// #endif

#endif // SIM_A7680C_H   
