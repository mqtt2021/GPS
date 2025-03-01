#ifndef SIM_A7680C_H
#define SIM_A7680C_H

// #ifdef __cplusplus
// extern "C" {
// #endif

//Library
#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"  
#include "main.h"  
#include "adc.h"  
#include "gps.h"
#include "freertos/event_groups.h"
//DEFINE
#define UART_SIM        UART_NUM_2
#define TXD_PIN         GPIO_NUM_17 // Default TXD pin for UART0
#define RXD_PIN         GPIO_NUM_16 // Default RXD pin for UART0
#define RTS_PIN         UART_PIN_NO_CHANGE
#define CTS_PIN         UART_PIN_NO_CHANGE
#define UART_BUFFER     1024
#define BAUD_RATE       115200
#define BUFFER_SIZE     400   
#define READ_TIMEOUT_MS 10000
#define COMMAND_DELAY_MS 1000
#define READ_TIMEOUT_CALL 30000
#define MQTT_RECONNECT_BIT BIT0
#define CALL_RECONNECT_BIT BIT1
extern char payload_json[256];  // Khai báo biến toàn cục
extern bool sub_topic;  // Khai báo biến toàn cục
// Function declarations
void uartsim_init(void);
void send_at_command(const char *command);
void read_uart_response(void);
void mqtt_connect(void);
void mqtt_publish(const char *topic, const char *message);
void send_gps_data_to_mqtt(void);
void extract_payload(const char *response);
void mqtt_subcribe(const char *topic);
void subcribe_topic_mqtt(void);
void module_sim_call_sms(void);
void uartsim_delete(void);
void handle_create_event(void);
// #ifdef __cplusplus
// }
// #endif

#endif // SIM_A7680C_H   
