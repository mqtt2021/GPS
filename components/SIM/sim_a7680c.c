#include "sim_a7680c.h"

bool start_call = false;
bool connect_mqtt_sever = false;
bool sub_topic = false;
char payload_json[256];  // Buffer cố định để lưu JSON
bool ready_sim = false;
bool sim_reset = false;
bool out_time_call = false;
bool send_connect_mqtt_sever = false;
bool send_command_pub = false;
bool send_command_call = false;
bool end_call = false;



int64_t elapsed_time = 0; // Biến toàn cục để lưu thời gian gọi
int64_t start_call_time = 0;
bool flag_call = false;
bool flag_call_not_reply = false;
bool flag_call_reply = false;


static EventGroupHandle_t mqtt_event_group;


static const char *TAG = "SIM_A7680C";

void extract_json_payload(const char *response) {
   
    char *start = strstr(response, "+CMQTTRXPAYLOAD: 0,");
    if (start) {
        start = strchr(start, '\n');  // Tìm ký tự xuống dòng sau `+CMQTTRXPAYLOAD:`
        if (start) {
            start++;  // Bỏ qua ký tự xuống dòng
            while (*start == '\n' || *start == '\r') start++;  // Bỏ qua các ký tự xuống dòng nếu có

            char *end = strstr(start, "+CMQTTRXEND: 0");  // Tìm điểm kết thúc JSON
            if (end) {
                size_t length = end - start;
                if (length > 255) length = 255;  // Giới hạn độ dài tối đa 255 ký tự tránh lỗi bộ nhớ
                strncpy(payload_json, start, length);
                payload_json[length] = '\0';  // Đảm bảo chuỗi kết thúc

                printf("payload_json: %s\n", payload_json);
                printf("///////////////////////////////////////////////\n");
                
                //xEventGroupSetBits(event_group_sub_topic, EVENT_SUB_TOPIC); 
            }
        }
    }
    else{
        printf("start NULL\n");
    }
}

// void uartsim_init(void) {
//     uart_config_t uart_config = {  
//         .baud_rate = BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT
//     };

//     // Use UART0 and configure its pins
//     ESP_ERROR_CHECK(uart_driver_install(UART_SIM, UART_BUFFER * 2, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_SIM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(UART_SIM, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN));
// }

QueueHandle_t uart_queue;  // Queue để nhận sự kiện UART

// void uart_event_task(void *pvParameters) {
//     uart_event_t event;
//     uint8_t data[BUFFER_SIZE];

//     while (1) {
//         if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
//             switch (event.type) {
//                 case UART_DATA:
//                     // Đọc dữ liệu từ UART
//                     int len = uart_read_bytes(UART_SIM, data, event.size, pdMS_TO_TICKS(100));
//                     if (len > 0) {
//                         data[len] = '\0';  // Thêm ký tự kết thúc chuỗi
//                         ESP_LOGI(TAG, "📩 Received Data: %s", (char *)data);

//                         // Kiểm tra phản hồi từ module SIM
//                         if (strstr((char *)data, "OK")) {
//                             ESP_LOGI(TAG, "✅ Response: OK");
//                             connect_mqtt_sever = true;
//                             ready_sim = true;
//                             // if(sub_topic){
//                             //     extract_json_payload(response);
//                             // }
//                         } else if (strstr((char *)data, "ERROR")) {
//                             ESP_LOGW(TAG, "❌ Response: ERROR");
//                         }
//                     }
//                     break;

//                 case UART_FIFO_OVF:
//                     ESP_LOGW(TAG, "⚠️ UART Buffer Overflow");
//                     uart_flush_input(UART_SIM);
//                     xQueueReset(uart_queue);
//                     break;

//                 case UART_BUFFER_FULL:
//                     ESP_LOGW(TAG, "⚠️ UART Buffer Full");
//                     uart_flush_input(UART_SIM);
//                     xQueueReset(uart_queue);
//                     break;

//                 default:
//                     ESP_LOGI(TAG, "Other UART event: %d", event.type);
//                     break;
//             }
//         }
//     }
// }
static char mqtt_rx_buffer[BUFFER_SIZE] = {0};  // Buffer chứa dữ liệu UART
static int buffer_index = 0;

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[128];  // Buffer tạm thời để đọc dữ liệu mới

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    TickType_t start_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu

                    while (1) {  
                        int len = uart_read_bytes(UART_SIM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
                        if (len > 0) {
                            data[len] = '\0';  // Đảm bảo dữ liệu là chuỗi kết thúc bằng '\0'

                            // Ghép dữ liệu vào buffer chính
                            if (buffer_index + len < BUFFER_SIZE - 1) {  
                                strncat(mqtt_rx_buffer, (char *)data, len);  
                                buffer_index += len;  
                            } else {  
                               ESP_LOGW(TAG, "⚠️ Buffer overflow, clearing buffer!");  
                                memset(mqtt_rx_buffer, 0, BUFFER_SIZE);  
                                buffer_index = 0;  
                            }
                            

                            // Kiểm tra nếu dữ liệu có chứa "OK" và không chứa "ERROR"
                            if (strstr(mqtt_rx_buffer, "OK") && !strstr(mqtt_rx_buffer, "ERROR")) {
                                if(send_connect_mqtt_sever){ // gửi lệnh connet cuối
                                    connect_mqtt_sever = true;
                                }
                                ESP_LOGI(TAG, "✅ Response: OK");
                                
                                if (start_call) {
                                    ready_sim = true;
                                    //flag_call = true;
                                    // Tiếp tục vòng lặp mà không break
                                } else {
                                    break;  // Thoát khỏi vòng lặp nếu không gọi điện
                                }

                                if(send_command_call){
                                    end_call = true;
                                }

                                
                            } 
                            if (strstr(mqtt_rx_buffer, "ERROR") ) {

                                if(send_command_pub){
                                    send_command_pub = false;
                                    xEventGroupSetBits(mqtt_event_group, MQTT_RECONNECT_BIT); // reset connect
                                    
                                }
                                // if(start_call){
                                //     send_command_pub = false;
                                //     xEventGroupSetBits(mqtt_event_group, CALL_RECONNECT_BIT); // reset connect
                                    
                                // }
                                ESP_LOGW(TAG, "❌ Response: ERROR");
                                //mqtt_connect();
                                if(start_call){
                                    flag_call = false;
                                    break;  // Thoát khỏi vòng lặp while nhận dữ liệu
                                }

                                break;  // Thoát khỏi vòng lặp while nhận dữ liệu
                            }

                            if (strstr(mqtt_rx_buffer, "VOICE CALL: END")) {
                                ESP_LOGW(TAG, "❌ Response: VOICE CALL: END");
                                
                                if(start_call){
                                    flag_call = true;                    
                                    out_time_call = false;
                                    break;  // Thoát khỏi vòng lặp while nhận dữ liệu
                                }
                                break;  // Thoát khỏi vòng lặp while nhận dữ liệu
                            }
                        }

                        if(send_command_call){
                            if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(30000)) {

                                ESP_LOGW(TAG, "⏳ Timeout Call");
                                break;
                            }
                        }
                        else{
                             // Kiểm tra timeout 1 giây
                            if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(4000)) {
                                ESP_LOGW(TAG, "⏳ Timeout");
                                break;
                            }
                        }

                       
                    }

                   // Đảm bảo chuỗi kết thúc đúng
                   mqtt_rx_buffer[buffer_index] = '\0';

                   // In toàn bộ dữ liệu nhận được
                   ESP_LOGI(TAG, "📩 Received Data:\n%s", mqtt_rx_buffer);
                   
                   if(sub_topic){
                       extract_json_payload(mqtt_rx_buffer);
                   }

                    // Reset buffer sau khi xử lý xong
                    memset(mqtt_rx_buffer, 0, BUFFER_SIZE);
                    buffer_index = 0;

                    break;
                }

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "⚠️ UART Buffer Overflow");
                    uart_flush_input(UART_SIM);
                    xQueueReset(uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "⚠️ UART Buffer Full");
                    uart_flush_input(UART_SIM);
                    xQueueReset(uart_queue);
                    break;

                default:
                    ESP_LOGI(TAG, "Other UART event: %d", event.type);
                    break;
                }
        }
    }
}


void uartsim_init(void) {
    uart_config_t uart_config = {  
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // Cài đặt driver UART với queue
    ESP_ERROR_CHECK(uart_driver_install(UART_SIM, UART_BUFFER * 2, UART_BUFFER * 2, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_SIM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_SIM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Tạo task xử lý ngắt UART
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    
}



void uartsim_delete(void) {
    // ESP_ERROR_CHECK(uart_driver_delete(UART_SIM));
    // gpio_reset_pin(TXD_PIN);
    // gpio_reset_pin(RXD_PIN);
    // gpio_reset_pin(RTS_PIN);
    // gpio_reset_pin(CTS_PIN);
}



void send_at_command(const char *command) {  
    char cmd_buffer[BUFFER_SIZE];
    snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\r\n", command);      
    //ESP_LOGI(TAG, "Send AT: %s", cmd_buffer);   
    uart_write_bytes(UART_SIM, cmd_buffer, strlen(cmd_buffer));
}

// void extract_length_json(const char *response) {
//     // Tìm vị trí của chuỗi "+CMQTTRXPAYLOAD: 0,"
//     const char *start_pos_1 = strstr(response, "+CMQTTRXPAYLOAD: 0,");      
//     if (start_pos_1 != NULL) {
//         start_pos_1 += strlen("+CMQTTRXPAYLOAD: 0,");  // Chuyển con trỏ đến sau "+CMQTTRXlength_json: 0,"

//         // Tìm ký tự xuống dòng '\r\n' sau chuỗi "+CMQTTRXPAYLOAD: 0,"
//         const char *end_pos_1 = strchr(start_pos_1, '\r\n');
//         if (end_pos_1 != NULL) {
//             // Tính độ dài chuỗi cần cắt
//             size_t length = end_pos_1 - start_pos_1;

//             // Cắt chuỗi con từ start_pos đến end_pos
//             char length_json[length + 1];
//             strncpy(length_json, start_pos_1, length);   
//             length_json[length] = '\0';  // Kết thúc chuỗi bằng '\0'

//             printf("length_json: %s\n", length_json);
            
//             printf("///////////////////////////////////////////////\n");

//             char str1[100] = "+CMQTTRXPAYLOAD: 0,";
//             // Ghép str1 với str2
//             strcat(str1, length_json);
//             printf("ghep chuoi %s\n", str1);

//             const char *start_pos_2 = strstr(response, str1);
//             if (start_pos_2 != NULL) {   
//                 start_pos_2 += strlen("+CMQTTRXPAYLOAD: 0,");  // Chuyển con trỏ đến sau "+CMQTTRXlength_json: 0,"

//                 // Tìm ký tự xuống dòng '\r\n' sau chuỗi "+CMQTTRXPAYLOAD: 0,"
//                 const char *end_pos_2 = strchr(start_pos_2, '\r\n');
//                 if (end_pos_2 != NULL) {

//                 }
//             }

//             xEventGroupSetBits(event_group_sub_topic, EVENT_SUB_TOPIC);   

//         }
//     }
// }




void read_uart_response(void)
{
    uint8_t buf[BUFFER_SIZE];
    int total_len = 0;
    char response[UART_BUFFER];
    memset(response, 0, sizeof(response));
    int64_t start_time = esp_timer_get_time();

    while (true) {
        int64_t now = esp_timer_get_time();
        if ((now - start_time) / 1000 > READ_TIMEOUT_MS) {
            break;
        }
        int len = uart_read_bytes(UART_SIM, buf, BUFFER_SIZE , 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (total_len + len < UART_BUFFER) {
                memcpy(response + total_len, buf, len);
                total_len += len;
            }
            if (strstr(response, "\r\nOK\r\n") || strstr(response, "\r\nERROR\r\n")) {
                if(strstr(response, "\r\nOK\r\n"))
                {
                    ESP_LOGI(TAG, "Response: OK");
                    connect_mqtt_sever = true;
                    ready_sim = true;
                }
                if(strstr(response, "\r\nERROR\r\n"))
                {
                    ESP_LOGI(TAG, "Response: ERROR");
                }
                break;    
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    if (total_len > 0) {
        response[total_len] = '\0';
        ESP_LOGI(TAG, "Total Response: %s\n", response);
        if(sub_topic){
            extract_json_payload(response);
        }
       
        ESP_LOGI(TAG, "-------------------------------------\n");
    } else {
        ESP_LOGI(TAG, "No response or timeout");
    }
}

// void read_uart_response_SMS(void)
// {
//     uint8_t buf[BUFFER_SIZE];
//     int total_len = 0;
//     char response[UART_BUFFER];
//     memset(response, 0, sizeof(response));
//     int64_t start_time = esp_timer_get_time();

//     while (true) {
//         int64_t now = esp_timer_get_time();
//         if ((now - start_time) / 2000 > READ_TIMEOUT_MS) {
//             break;
//         }
//         int len = uart_read_bytes(UART_SIM, buf, BUFFER_SIZE , 20 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             if (total_len + len < UART_BUFFER) {
//                 memcpy(response + total_len, buf, len);
//                 total_len += len;
//             }
//             if (strstr(response, "\r\nOK\r\n") || strstr(response, "\r\nERROR\r\n")) {
//                 if(strstr(response, "\r\nOK\r\n"))
//                 {
                    
//                 }
//                 if(strstr(response, "\r\nERROR\r\n"))
//                 {
//                     ESP_LOGI(TAG, "Response: ERROR");
//                 }
//                 break;    
//             }
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }

//     if (total_len > 0) {
//         response[total_len] = '\0';
//         ESP_LOGI(TAG, "Total Response: %s\n", response);
//         //ESP_LOGI(TAG, "-------------------------------------\n");
//     } else {
//         //ESP_LOGI(TAG, "No response or timeout");
//     }
   
// }



// void read_uart_response_call_sms(void)
// {

//     uint8_t buf[BUFFER_SIZE];
//     int total_len = 0;
//     char response[UART_BUFFER];
//     memset(response, 0, sizeof(response));

//     start_call_time = esp_timer_get_time(); // Lưu thời gian bắt đầu cuộc gọi
//     printf("Cuộc gọi bắt đầu %lld giây!\n", start_call_time);
//     int64_t start_time = esp_timer_get_time();
//     while (true) {
//         int64_t now = esp_timer_get_time();
//         if ((now - start_time) / 1000 > READ_TIMEOUT_CALL) {
//             out_time_call = true;
//             break;
//         }
//         int len = uart_read_bytes(UART_SIM, buf, BUFFER_SIZE , 20 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             if (total_len + len < UART_BUFFER) {
//                 memcpy(response + total_len, buf, len);
//                 total_len += len;
//             }
//             if (strstr(response, "\r\nOK\r\n") || strstr(response, "\r\nERROR\r\n") || strstr(response, "\r\nVOICE CALL: END\r\n") ) {
//                 if(strstr(response, "\r\nOK\r\n"))
//                 {
//                     flag_call = true;
//                 }
//                 if(strstr(response, "\r\nERROR\r\n"))
//                 {
//                     //ESP_LOGI(TAG, "Response: ERROR");
//                     flag_call = false;
//                     // gpio_set_level(GPIO_NUM_18, 1);
//                     // vTaskDelay(pdMS_TO_TICKS(5000));
//                     // gpio_set_level(GPIO_NUM_18, 0); 
//                     break;    
//                 }
//                 if(strstr(response, "\r\nVOICE CALL: END\r\n"))
//                 {
//                     flag_call = true;
//                     //ESP_LOGI(TAG, "Response: ERROR");
//                     out_time_call = false;

//                     // elapsed_time = (esp_timer_get_time() - start_call_time) / 1000000; // Tính thời gian đã gọi (giây)
//                     // printf("Cuộc gọi kết thúc sau %lld giây!\n", elapsed_time);
//                     break;    
//                 }     
//             }
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }
// }


void mqtt_publish(const char *topic, const char *message) {
    char cmd[500];
    
    send_command_pub = false; 

    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", (int)strlen(topic));
    //ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTTOPIC=0,%d",(int)strlen(topic));
    send_at_command(cmd); 
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    //ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();
    //ESP_LOGI(TAG, "Begin send AT command: GPS/Status/G001");
    send_at_command(topic);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    //ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", (int)strlen(message));
    //ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTPAYLOAD=0,%d",(int)strlen(message));
    send_at_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    //ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();

    //ESP_LOGI(TAG, "Begin send AT command: message");
    send_at_command(message);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    //ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();

    send_command_pub = true;
    //ESP_LOGI(TAG, "Begin send AT command:AT+CMQTTPUB=0,1,60,1");
    send_at_command("AT+CMQTTPUB=0,1,60,1");
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    
    //ESP_LOGI(TAG, "Begin read_uart_response");

    //read_uart_response();

           

            // gpio_set_level(GPIO_NUM_18, 1);
            // vTaskDelay(pdMS_TO_TICKS(2000));
            // gpio_set_level(GPIO_NUM_18, 0); 
}

void mqtt_connect(void)
{   
    int retry_count = 0;
   
    ESP_LOGI(TAG, "=== MQTT CONNECT START ===");
    int reset_sim = 0;

    while(reset_sim < 9){

        while (retry_count < 5) {

            gpio_set_level(GPIO_NUM_18, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(GPIO_NUM_18, 0); 

            ESP_LOGI(TAG, "Attempt %d to connect MQTT", retry_count + 1);
            //ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTDISC=0,60");
            send_connect_mqtt_sever = false;
            ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTREL=0\n");
            send_at_command("AT+CMQTTDISC=0,60");
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            ESP_LOGI(TAG, "Begin read_uart_response");
            //read_uart_response();

            ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTREL=0\n");
            send_at_command("AT+CMQTTREL=0");
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            ESP_LOGI(TAG, "Begin read_uart_response");
            //read_uart_response();

            ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTSTOP\n");
            send_at_command("AT+CMQTTSTOP");
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            ESP_LOGI(TAG, "Begin read_uart_response");
            //read_uart_response();
            
            ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTSTART\n");
            send_at_command("AT+CMQTTSTART");
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            ESP_LOGI(TAG, "Begin read_uart_response");
            //read_uart_response();

            ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTACCQ=0\n");
            send_at_command("AT+CMQTTACCQ=0,\"ESP32\",0");
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            ESP_LOGI(TAG, "Begin read_uart_response");
            //read_uart_response();

            const char *mqtt_connect_cmd = "AT+CMQTTCONNECT=0,\"tcp://20.41.104.186:1883\",60,1";
            //ESP_LOGI(TAG, "Begin send AT command: tcp://52.140.102.169:1883\n");   
            
            send_connect_mqtt_sever = true;
            send_at_command(mqtt_connect_cmd);
            //send_connect_mqtt_sever = true;
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            ESP_LOGI(TAG, "Begin read_uart_response\n");
            //read_uart_response();
            
            
            if (connect_mqtt_sever == true) {
                ESP_LOGI(TAG, "CONNECTED SUCESSFULLY");   
                connect_mqtt_sever = false;
                send_connect_mqtt_sever = false;

                gpio_set_level(GPIO_NUM_18, 1);
                vTaskDelay(pdMS_TO_TICKS(5000));
                gpio_set_level(GPIO_NUM_18, 0); 
            
                
                break;
            } else {
                ESP_LOGI(TAG, "CONNECTED UNSUCESSFULLY");
                retry_count++;
                vTaskDelay(pdMS_TO_TICKS(1000));  // Đợi trước khi thử lại
                send_connect_mqtt_sever = false;
            }
        }  

        if(retry_count == 5 || retry_count > 5 ){
            reset_sim++;
            gpio_set_level(GPIO_SIM_TRIGGER, 0);
            gpio_pulldown_en(GPIO_SIM_TRIGGER); // Bật pull-down để giữ mức thấp
            gpio_hold_en(GPIO_SIM_TRIGGER); // Giữ trạng thái khi ngủ   
        
            gpio_set_level(GPIO_PEN, 0);
            gpio_pulldown_en(GPIO_PEN); // Bật pull-down để giữ mức thấp
            gpio_hold_en(GPIO_PEN); // Giữ trạng thái khi ngủ 

            //gpio_set_level(GPIO_NUM_18, 1);  // on
            vTaskDelay(pdMS_TO_TICKS(6000));

            gpio_hold_dis(GPIO_SIM_TRIGGER);
            gpio_hold_dis(GPIO_PEN); 
            gpio_set_level(GPIO_SIM_TRIGGER, 1);
            gpio_set_level(GPIO_PEN, 1); // Bật PEN, bật sim
            
            //gpio_set_level(GPIO_NUM_18, 0);  // on
            

            retry_count = 0;
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else{
            break;
        }
    }

    ESP_LOGI(TAG, "=== MQTT CONNECT END ===");
}


void mqtt_reconnect_task(void *pvParameters) {
    while (1) {
        // Chờ cho đến khi có sự kiện
        xEventGroupWaitBits(mqtt_event_group, MQTT_RECONNECT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

                                    // gpio_set_level(GPIO_NUM_18, 1);
                                    // vTaskDelay(pdMS_TO_TICKS(3000));
                                    // gpio_set_level(GPIO_NUM_18, 0); 

        ESP_LOGI(TAG, "🔄 Reconnecting to MQTT...");

        // Gọi lại hàm kết nối
        mqtt_connect();
    }
}



// void sim_reset_gpio_task(void *pvParameters) {

//     while (1) {
//         // Chờ cho đến khi có sự kiện
//         xEventGroupWaitBits(mqtt_event_group, CALL_RECONNECT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

//                                     gpio_set_level(GPIO_NUM_18, 1);
//                                     vTaskDelay(pdMS_TO_TICKS(3000));
//                                     gpio_set_level(GPIO_NUM_18, 0); 

//         ESP_LOGI(TAG, "🔄 Reconnecting to MQTT...");

//         // Reset GPIO SIM
//         gpio_set_level(GPIO_SIM_TRIGGER, 1);
//         vTaskDelay(pdMS_TO_TICKS(5000));
//         gpio_set_level(GPIO_SIM_TRIGGER, 0); 
//     }

// }



void module_sim_call_sms(void)
{  

    mqtt_event_group = xEventGroupCreate();
    xTaskCreate(mqtt_reconnect_task, "mqtt_reconnect_task", 4096, NULL, 5, NULL);
    // xTaskCreate(sim_reset_gpio_task, "sim_reset_gpio_task", 4096, NULL, 5, NULL);

    //vTaskDelay(pdMS_TO_TICKS(10000)); //
   
    
    ESP_LOGI(TAG, "=== CALL START ===");
        
    int count_reset_sim  = 0; // số lần reset chân gpio_sim
    int count_AT_sim = 0;  // Số lần send command xác nhận sim hoạt động
    int retry_count = 0; // số lần gọi điện


    while(count_reset_sim < 10){
        ESP_LOGI(TAG, "Bật chân sim lần %d\n", count_reset_sim );
        

        while(count_AT_sim < 10 ){
            
            ESP_LOGI(TAG, "Gửi lệnh sẵn sàng lần %d\n", count_AT_sim);
            start_call = true;

            send_at_command("AT+CPIN?");
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
            //read_uart_response();
            if(ready_sim){
                ESP_LOGI(TAG, "Sim sẵn sàng \n");
                break;
            }
            count_AT_sim++;
        }

        if( count_AT_sim == 10 || count_AT_sim > 10  ){    // sim bị treo, cần reset
            ESP_LOGI(TAG, "sim bị treo, cần reset \n");
            gpio_set_level(GPIO_SIM_TRIGGER, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));   
            gpio_set_level(GPIO_SIM_TRIGGER, 1); 
            count_AT_sim = 0;
            count_reset_sim++;

        }
        else{
                ready_sim = false;
               
                while (!end_call || retry_count < 5) {
                        send_command_call = true;
                        ESP_LOGI(TAG, "Attempt %d to Call", retry_count + 1);
                        send_at_command("ATD0559687397;");
                        vTaskDelay(pdMS_TO_TICKS(5000));
                        //read_uart_response_call_sms();
                        retry_count++;
                      
            
                    // if(!flag_call){
                    //     start_call = false;
                    //     retry_count++;
                    //     ESP_LOGI(TAG, "Call faild\n");
                    // }
            
                    // if (flag_call && out_time_call) {
                    //     start_call = false;
                    //     // gpio_set_level(GPIO_NUM_18, 1);
                    //     // vTaskDelay(pdMS_TO_TICKS(5000));
                    //     // gpio_set_level(GPIO_NUM_18, 0);
                    //     flag_call_not_reply = true;
                    //     break;
                        
                    // }
                    
                    // if(flag_call && !out_time_call){
                    //     start_call = false;
                    //     // gpio_set_level(GPIO_NUM_18, 1);
                    //     // vTaskDelay(pdMS_TO_TICKS(1000));
                    //     // gpio_set_level(GPIO_NUM_18, 0);
                    //     flag_call_reply = true;
                    //     break;
                    // } 

                    
                }
            
           
        }

        if(end_call){
            end_call = false;
            break;
        }
        if(retry_count == 5 || retry_count > 5){
            retry_count = 0;
            break;
        }
    }

    if( count_reset_sim > 5 || count_reset_sim == 5){
        flag_call = false;
    }


    
    
    // if(!flag_call){
    //     ESP_LOGI(TAG, "Call faild\n");
    // }
    // if(flag_call_not_reply){
    //     ESP_LOGI(TAG, "Call successfull and User not reply\n");
    // }
    // if(flag_call_reply){
    //     ESP_LOGI(TAG, "Call successfull and User reply\n");
    // }

    int count_sms = 7;
    //// Gửi SMS
    while (count_sms < 1)
    {             
        ESP_LOGI(TAG, "Begin send SMS");
        send_at_command("AT");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("ATE0");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CGSN");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
        
        send_at_command("ATI");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CPIN?");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CIMI");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CICCID");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CSQ");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CPSI?");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CMGF?");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CSCS=\"GSM\"");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        //read_uart_response_SMS();
    
        send_at_command("AT+CMGS=\"0559687397\"");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
       // read_uart_response_SMS();
    
        // char cmd_buffer[BUFFER_SIZE] = ;
        // snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\r\n", command);      
        // //ESP_LOGI(TAG, "Send AT: %s", cmd_buffer);   
        // uart_write_bytes(UART_SIM, cmd_buffer, strlen(cmd_buffer));
    
        send_at_command("ALARM GPS 001"); // Gửi nội dung tin nhắn
        vTaskDelay(pdMS_TO_TICKS(300));  // Chờ một chút trước khi gửi Ctrl+Z
        send_at_command("\x1A"); // Gửi Ctrl+Z để xác nhận gửi SMS
        vTaskDelay(pdMS_TO_TICKS(1000)); // Chờ module SIM xử lý
        //read_uart_response_SMS();

        ESP_LOGI(TAG, "End send SMS");
        count_sms++;            
    }
}

void mqtt_subcribe(const char *topic){
    char cmd[128];   
    snprintf(cmd, sizeof(cmd), "AT+CMQTTSUBTOPIC=0,%d,1",(int)strlen(topic));
    ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTSUBTOPIC=0,%d,1",(int)strlen(topic));
    send_at_command(cmd);    
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();

    ESP_LOGI(TAG, "Begin send AT command: GPS/Setting/G001");
    send_at_command(topic);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();

    sub_topic = true;  
    ESP_LOGI(TAG, "Begin send AT command: AT+CMQTTSUB=0");
    send_at_command("AT+CMQTTSUB=0");
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    ESP_LOGI(TAG, "Begin read_uart_response");
    //read_uart_response();

}


void handle_create_event(void){
    mqtt_event_group = xEventGroupCreate();
    xTaskCreate(mqtt_reconnect_task, "mqtt_reconnect_task", 4096, NULL, 5, NULL);
}

void send_gps_data_to_mqtt(void) {

    char mqtt_payload[BUFFER_SIZE];
    snprintf(mqtt_payload, sizeof(mqtt_payload),
                "["
                "{\"name\":\"Latitude\",\"value\":%d,\"timestamp\":\"%s\"},"
                "{\"name\":\"Longitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                "{\"name\":\"Battery\",\"value\":\"%d%%\",\"timestamp\":\"%s\"},"
                "{\"name\":\"Stolen\",\"value\":%s,\"timestamp\":\"%s\"},"
                "{\"name\":\"Bluetooth\",\"value\":%s,\"timestamp\":\"%s\"},"
                "{\"name\":\"Move\",\"value\":%s,\"timestamp\":\"%s\"}"
                "]",
                count,
                global_gps_data.time,
                global_gps_data.longitude,
                global_gps_data.time,
                global_gps_data.battery_capacity,
                global_gps_data.time,
                global_gps_data.Stolen ? "true" : "false",
                global_gps_data.time, 
                global_gps_data.bluetooth ? "true" : "false",
                global_gps_data.time,  
                global_gps_data.move ? "true" : "false",
                global_gps_data.time     
            );
    mqtt_publish("GPS/Status/G002", mqtt_payload);
}

void subcribe_topic_mqtt(void) {
        mqtt_subcribe("GPS/Setting/G002");
        //ESP_LOGI(TAG, "Sent MQTT message: %s", mqtt_length_json);
}  

