#ifndef MY_TIMER_H
#define MY_TIMER_H

#include <stdbool.h>
#include <stdio.h>
#include "driver/timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

// Nội dung của my_timer.h
extern volatile bool g_timer_done;
extern volatile bool g_timer_connect_mqtt_done;
void my_timer_init(void);
void my_timer_start(void);
void my_timer_delete(void);

void my_timer_connect_mqtt_init(void);
void my_timer_connect_mqtt_start(void);
void my_timer_connect_mqtt_delete(void);



// #ifdef __cplusplus
// }
// #endif

#endif // MY_TIMER_H
