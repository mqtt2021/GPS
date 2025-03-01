

#include "main.h"
#include "cJSON.h"
#include <ctype.h>  // Thư viện chứa isspace()

static const char *TAG = "example";
// ------------------------- GPS Variables -------------------------
int gps_flag = 0;
int try_connect_gps = 0;
volatile bool gps_task_done = false; // Cờ báo hiệu task đã hoàn thành

typedef struct {
    double Longitude;
    double Latitude;
    int SafeRadius;
    char CurrentTime[50];
    char AlarmTime[50];
    char BlueTooth[10];
} GpsData;

int count = 9090;
bool send_data  = false ;
bool isProccessing = false;

int count_sub = 0;

// EventGroupHandle_t event_group;
// EventGroupHandle_t event_group_sub_topic;

void parse_json(const char *json_str, GpsData *data) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        printf("Error parsing JSON\n");
        return;
    }

    cJSON *longitude = cJSON_GetObjectItem(root, "Longitude");
    cJSON *latitude = cJSON_GetObjectItem(root, "Latitude");
    cJSON *safeRadius = cJSON_GetObjectItem(root, "SafeRadius");
    cJSON *currentTime = cJSON_GetObjectItem(root, "CurrentTime");
    cJSON *alarmTime = cJSON_GetObjectItem(root, "AlarmTime");
    cJSON *blueTooth = cJSON_GetObjectItem(root, "BlueTooth");

    if (longitude) data->Longitude = longitude->valuedouble;
    if (latitude) data->Latitude = latitude->valuedouble;
    if (safeRadius) data->SafeRadius = safeRadius->valueint;
    if (currentTime) strncpy(data->CurrentTime, currentTime->valuestring, sizeof(data->CurrentTime) - 1);
    if (alarmTime) strncpy(data->AlarmTime, alarmTime->valuestring, sizeof(data->AlarmTime) - 1);
    if (blueTooth) strncpy(data->BlueTooth, blueTooth->valuestring, sizeof(data->BlueTooth) - 1);

    cJSON_Delete(root); // Giải phóng bộ nhớ sau khi dùng xong
}

bool read_uart2_data(char *buffer, int buffer_size) {
    static int buffer_pos = 0;

    // Đọc dữ liệu từ UART2
    int len = uart_read_bytes(GPS_UART2_PORT_NUM,
                              (uint8_t *)(buffer + buffer_pos),
                              buffer_size - buffer_pos - 1,
                              20 / portTICK_PERIOD_MS);
    if (len > 0) {
        buffer_pos += len;
        buffer[buffer_pos] = '\0'; // Null-terminate chuỗi

        printf("buffer chua xu li: %s\n", buffer);

        // Tìm ký tự xuống dòng
        char *newline_ptr;

        while ((newline_ptr = strchr(buffer, '\n')) != NULL) { //Hàm strchr trả về con trỏ đến vị trí đầu tiên của ký tự này trong chuỗi, hoặc trả về NULL nếu không tìm thấy
            *newline_ptr = '\0'; // Kết thúc chuỗi tại ký tự xuống dòng

            // Loại bỏ ký tự carriage return (CRLF) nếu có
            if (newline_ptr > buffer && *(newline_ptr - 1) == '\r') {
                *(newline_ptr - 1) = '\0';
            }

            printf("buffer sau xu li: %s\n", buffer);

            // Xử lý dữ liệu bắt đầu bằng "$GNRMC"
            if (strncmp(buffer, "$GNRMC", 6) == 0) { //hàm so sánh hai chuỗi với nhau, chỉ so sánh tối đa 6 ký tự đầu tiên, trả về giá trị 0 nếu hai chuỗi giống nhau 
                ESP_LOGI(TAG, "Received GNRMC Data: %s\n", buffer);
                processGNRMC(buffer);
                buffer_pos = 0; // Reset buffer sau khi xử lý
                return true;
            }

            // Di chuyển phần còn lại của buffer về đầu
            int remaining = buffer_pos - (newline_ptr - buffer) - 1;
            memmove(buffer, newline_ptr + 1, remaining); // bắt đầu từ vị trí newline_ptr + 1, sao chép remaining byte lên đầu buffer, phần ở sau vẫn giữ lại
            buffer_pos = remaining;
            buffer[buffer_pos] = '\0';
        }

        // Xử lý trường hợp buffer đầy mà không có ký tự newline
        if (buffer_pos >= buffer_size - 1) {
            ESP_LOGW(TAG, "Buffer full without receiving newline. Clearing buffer.");
            buffer_pos = 0;
            buffer[buffer_pos] = '\0';
        }
    }

    return false; // Không có dữ liệu hợp lệ được xử lý
}


// ------------------ Quản Lý Task UART ------------------ 
TaskHandle_t uart2TaskHandle = NULL;
TaskHandle_t uart0TaskHandle = NULL;
volatile bool flag_stop_uart_task = false; // Cờ để dừng task
// ------------------ Task UART2 ------------------ read data GPS
void uart2_task(void *pvParameters) {
    char *buffer = (char *)malloc(GPS_UART2_BUFFER_SIZE);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for UART2 buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Start uart2_task, monitoring UART2 data...");

    while (!flag_stop_uart_task) {
        

        // bool success = read_uart2_data(buffer, GPS_UART2_BUFFER_SIZE);
        // if (success) {
        //     ESP_LOGI(TAG, "Data successfully processed.");
        // }

        vTaskDelay(pdMS_TO_TICKS(500)); // Tránh chiếm CPU
    }

    free(buffer);
    ESP_LOGI(TAG, "uart2_task exiting...");
    uart2TaskHandle = NULL;
    vTaskDelete(NULL);
}


void trim(char *str) {
    if (str == NULL) return;

    // Xóa khoảng trắng đầu chuỗi
    while (isspace((unsigned char)*str)) str++;

    // Xóa khoảng trắng cuối chuỗi
    char *end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    
    *(end + 1) = '\0';  // Thêm ký tự kết thúc
}



void uart0_task(void *pvParameters) {

    //gpio_set_level(GPIO_NUM_18, 1);

    int count_uart0 = 0;

    while (count_uart0 < 1) {
        subcribe_topic_mqtt();
        count_uart0++;
    }
    while (!flag_stop_uart_task) {
        //read_uart_response();
       
        GpsData data;
        parse_json(payload_json, &data);

        printf("Longitude: %f\n", data.Longitude);
        printf("Latitude: %f\n", data.Latitude);
        printf("SafeRadius: %d\n", data.SafeRadius);
        printf("CurrentTime: %s\n", data.CurrentTime);
        printf("AlarmTime: %s\n", data.AlarmTime);
        printf("BlueTooth: %s\n", data.BlueTooth);
        
        trim(data.BlueTooth);  // Xóa khoảng trắng, xuống dòng

        if (data.BlueTooth[0] != '\0') {  // Nếu có ký tự
            gpio_set_level(GPIO_NUM_18, 0);
            printf("🔊 Bật BUZZER - BlueTooth có dữ liệu: '%s'\n", data.BlueTooth);
        } else {  // Nếu rỗng
            gpio_set_level(GPIO_NUM_18, 1);
            printf("🔇 Tắt BUZZER - BlueTooth rỗng\n");
        }

        
        //gpio_set_level(GPIO_NUM_18, 1);
        if (strcmp(data.BlueTooth, "on") == 0) {
            gpio_set_level(GPIO_NUM_18, 1);
            printf("Bật BUZZER\n");
        } else if (strcmp(data.BlueTooth, "off") == 0) {
            gpio_set_level(GPIO_NUM_18, 0);
            printf("Tắt BUZZER\n");
        } else {
            printf("Giá trị BlueTooth không hợp lệ: '%s'\n", data.BlueTooth);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Chờ 500ms để giảm tải CPU
    }
    sub_topic = false;
    uart0TaskHandle = NULL;
    ESP_LOGI(TAG, "uart0_task exiting...");
    vTaskDelete(NULL);
}

void start_uart_task() {
    if (uart2TaskHandle == NULL) {
        flag_stop_uart_task = false;  
        ESP_LOGI(TAG, "Creating uart2_task...");
        if (xTaskCreate(uart2_task, "uart2_task", 8192, NULL, 5, &uart2TaskHandle) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create uart2_task");
            return;
        }
    } else {
        ESP_LOGW(TAG, "uart2_task is already running.");
    }

    // if (uart0TaskHandle == NULL) {
    //     ESP_LOGI(TAG, "Creating uart0_task...");
    //     if (xTaskCreate(uart0_task, "uart0_task", 8192, NULL, 5, &uart0TaskHandle) != pdPASS) {
    //         ESP_LOGE(TAG, "Failed to create uart0_task");
    //         return;
    //     }
    // } else {
    //     ESP_LOGW(TAG, "uart0_task is already running.");
    // }
}

void stop_uart_task() {
    if (uart2TaskHandle != NULL || uart0TaskHandle != NULL) {
        ESP_LOGI(TAG, "Stopping UART tasks...");

        flag_stop_uart_task = true;  

       // Chờ task tự thoát
       int timeout = 10; // Giới hạn 10 lần kiểm tra
       while ((uart2TaskHandle != NULL || uart0TaskHandle != NULL) && timeout-- > 0) {
           ESP_LOGI(TAG, "Waiting for UART tasks to stop...");
           vTaskDelay(pdMS_TO_TICKS(500)); 
       }

        if (uart2TaskHandle != NULL) {
            ESP_LOGW(TAG, "Force deleting uart2_task!");
            vTaskDelete(uart2TaskHandle);
            uart2TaskHandle = NULL;
        }

        // if (uart0TaskHandle != NULL) {
        //     ESP_LOGW(TAG, "Force deleting uart0_task!");
        //     vTaskDelete(uart0TaskHandle);
        //     uart0TaskHandle = NULL;
        // }

        ESP_LOGI(TAG, "UART tasks stopped.");
    } else {
        ESP_LOGW(TAG, "No UART task is running.");
    }
}




void gps_pps_monitor() {
    TickType_t startTick = xTaskGetTickCount(); // Lấy thời gian bắt đầu
    const TickType_t runDuration = pdMS_TO_TICKS(3 * 60 * 1000); // 5 phút

    //ESP_LOGI(TAG, "Starting GPS PPS monitoring for 2 minutes...");

    // Cấu hình GPIO
    gpio_reset_pin(GPIO_GPS_PPS);
    gpio_set_direction(GPIO_GPS_PPS, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_GPS_PPS, GPIO_PULLDOWN_ONLY); // Kéo xuống mặc định
    
    // gpio_set_level(GPIO_NUM_18, 1);
    //vTaskDelay(pdMS_TO_TICKS(2000)); // Chống rung (debounce)
    // gpio_set_level(GPIO_NUM_18, 0);


    // Vòng lặp chạy trong 2 phút
    while ((xTaskGetTickCount() - startTick) < runDuration) {
        if (gpio_get_level(GPIO_GPS_PPS) == 1) {
            gps_flag++;
            ESP_LOGI(TAG, "GPS PPS detected! gps_flag = %d", gps_flag);
            vTaskDelay(pdMS_TO_TICKS(100)); // Chống rung (debounce)
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Kiểm tra mỗi 10ms
        
        if (gps_flag >= 10)
        {
            break;
        }
    }
    ESP_LOGI(TAG, "2 minutes elapsed. Final gps_flag count: %d", gps_flag);
    // Đặt cờ báo hiệu task đã hoàn thành
    gps_task_done = true;
}

void configure_led(void)
{

    gpio_reset_pin(GPIO_GPS_TRIGGER); //GPS
    gpio_set_direction(GPIO_GPS_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_GPS_TRIGGER, 1);

    gpio_reset_pin(GPIO_SIM_TRIGGER);
    gpio_set_direction(GPIO_SIM_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_SIM_TRIGGER, 0);
    gpio_pulldown_en(GPIO_SIM_TRIGGER); // Bật pull-down để giữ mức thấp
    gpio_hold_en(GPIO_SIM_TRIGGER); // Giữ trạng thái khi ngủ   

    gpio_reset_pin(GPIO_PEN); // PEN_SIM
    gpio_set_direction(GPIO_PEN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_PEN, 0);
    gpio_pulldown_en(GPIO_PEN); // Bật pull-down để giữ mức thấp
    gpio_hold_en(GPIO_PEN); // Giữ trạng thái khi ngủ 

    gpio_reset_pin(BUZZER); // BUZZER
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER, 0);

}


static void deep_sleep_task(void *args)
{
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep\n");
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from ext0\n");
            break;
        }

        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
                gpio_set_level(GPIO_SIM_TRIGGER, 0);
                gpio_pulldown_en(GPIO_SIM_TRIGGER); // Bật pull-down để giữ mức thấp
                gpio_hold_en(GPIO_SIM_TRIGGER); // Giữ trạng thái khi ngủ   
            
                gpio_set_level(GPIO_PEN, 0);
                gpio_pulldown_en(GPIO_PEN); // Bật pull-down để giữ mức thấp
                gpio_hold_en(GPIO_PEN); // Giữ trạng thái khi ngủ 
                // xTaskCreate(task_handle_motion, "Task_Handle_Motion", 2048, NULL, 11, NULL);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }

        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
            break;
        }

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    printf("Entering deep sleep\n");
    // enter deep sleep
    gpio_set_level(GPIO_SIM_TRIGGER, 0);
    gpio_pulldown_en(GPIO_SIM_TRIGGER); // Bật pull-down để giữ mức thấp
    gpio_hold_en(GPIO_SIM_TRIGGER); // Giữ trạng thái khi ngủ   

    gpio_set_level(GPIO_PEN, 0);
    gpio_pulldown_en(GPIO_PEN); // Bật pull-down để giữ mức thấp
    gpio_hold_en(GPIO_PEN); // Giữ trạng thái khi ngủ 

    gpio_set_level(GPIO_GPS_TRIGGER, 1);

    esp_deep_sleep_start();
}


void example_deep_sleep_register_ext1_wakeup(void)
{
    const int ext_wakeup_pin_1 = CONFIG_EXAMPLE_EXT1_WAKEUP_PIN_1;
    const int ext_wakeup_pin_2 = CONFIG_EXAMPLE_EXT1_WAKEUP_PIN_2;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;
    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, CONFIG_EXAMPLE_EXT1_WAKEUP_MODE));
}

// Khai báo biến RTC với giá trị mặc định khó xảy ra
RTC_DATA_ATTR float stored_roll = -9999.0;
RTC_DATA_ATTR float stored_pitch = -9999.0;
RTC_DATA_ATTR float stored_yaw = -9999.0;

void app_main(void)
{
    configure_led();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    mpu6050_enable_interrupt_pin();
    mpu6050_enable_motion_interrupt(); // CẤU HÌNH MPU
    //uartsim_init(); // UART SIM
    //init_uart2(); // UART GPS
    example_deep_sleep_register_ext1_wakeup(); // cấu hình đánh thức

    gpio_hold_dis(GPIO_SIM_TRIGGER);
    gpio_hold_dis(GPIO_PEN); 
    gpio_set_level(GPIO_SIM_TRIGGER, 1);
    gpio_set_level(GPIO_PEN, 1); // Bật PEN, bật sim
    gpio_set_level(GPIO_GPS_TRIGGER, 0); // Bật GPS

    //handle_create_event();

    float current_roll = 0, current_pitch = 0, current_yaw = 0;
    read_mpu6050_angles_first(&current_roll, &current_pitch, &current_yaw);
    
                    //mqtt_connect();  
                   
                    // while(try_connect_gps < 5){
                        // gps_pps_monitor();
        
                        // if (gps_flag >= 10) {
                    
                            //uartsim_init(); // bật lại uart sim
                            //gpio_set_level(GPIO_SIM_TRIGGER, 1);  // bật sim -> kết nối mqtt
                            //gpio_set_level(GPIO_PEN, 1);   
                            //vTaskDelay(pdMS_TO_TICKS(100)); // 30 giây
                            
                            //my_timer_connect_mqtt_start();
            
                            // 3) Ta dùng vòng while(1) để dễ dàng “lặp” thêm 5 phút
                            while (true)
                            {
                                // a) Bắt đầu đếm 5 phút

                                // if(g_timer_connect_mqtt_done){
                                //     mqtt_connect(); 
                                //     g_timer_connect_mqtt_done = false;
                                //     my_timer_connect_mqtt_delete();
                                //     my_timer_connect_mqtt_start();
        
                                // }
            
                                // b) Chờ cho đến khi 5 phút kết thúc                      
                                
                                    // gpio_set_level(GPIO_NUM_18, 1);
                                    // vTaskDelay(pdMS_TO_TICKS(2000));
                                    // gpio_set_level(GPIO_NUM_18, 0);
            
                                    // Khởi tạo Task UART2
                                    start_uart_task();
                                    // Chờ Task UART2 chạy 30 giây
                                    vTaskDelay(pdMS_TO_TICKS(50 * 1000)); // 60 giây
                                    // Dừng Task UART2
                                    stop_uart_task();
                                
                                    ESP_LOGI(TAG, "READ_GPS_TASK finish");
                                
                                    //In ra dữ liệu sẽ gửi lên MQTT
                                    ESP_LOGI(TAG, "Updated global GPS data: Time=%s, Stolen=%s, Lat=%.6f, Lon=%.6f, Batt=%d%%, Date=%s",
                                    global_gps_data.time, 
                                    global_gps_data.Stolen ? "true" : "false",  // Chuyển đổi bool thành chuỗi
                                    global_gps_data.latitude, 
                                    global_gps_data.longitude, 
                                    global_gps_data.battery_capacity, 
                                    global_gps_data.date);

                                    //mqtt_connect(); 
                                    send_gps_data_to_mqtt();
                                    count++;
                                    
                                
                            
                                // my_timer_delete();
                                // // c) Timer báo hết 5 phút => reset cờ
                                // g_timer_done = false;  
            
                                // d) Kiểm tra điều kiện
                                // ESP_LOGI(TAG, "Checking dif_location condition...");
                                // check_dif_location(); // Cập nhật giá trị `dif_location`
                                // if (diff_location_flag == false) {
                                //     // Nếu *KHÔNG* thỏa ⇒ break => dừng lặp => đi ngủ
                                //     ESP_LOGI(TAG, "Safe Location => Go to deep sleep.");
                                //     break;
                                // } else {
                                //     ESP_LOGI(TAG, "Different Location => RUN another 5 minutes loop.");
                                //     // Nếu thỏa => tiếp tục vòng while => timer_start() lần nữa
                                //     // (Vòng lặp lại từ đầu)
                                // }
                            }
                        
                            // 4) Sau khi thoát vòng lặp => Deep Sleep
                            ESP_LOGI(TAG, "Now going to Deep Sleep...");
                        
                        // }
            
                        // else
                        // {
                        //     printf("NO GPS SIGNAL, try again....\n");
                        //     // gpio_set_level(GPIO_GPS_TRIGGER, 1);
                        //     // gpio_set_level(GPIO_SIM_TRIGGER, 0);
                        //     // gpio_set_level(GPIO_PEN, 0);
                        //     gpio_set_level(GPIO_GPS_TRIGGER, 1); // Tắt  GPS
                        //     vTaskDelay(pdMS_TO_TICKS(3 * 1000)); // 2 giây
                        //     gpio_set_level(GPIO_GPS_TRIGGER, 0); // Bật  GPS
                        //     try_connect_gps++;
            
                        // }
                    // }

    // góc hiện tại từ MPU6050
    //float current_roll = 0, current_pitch = 0, current_yaw = 0;
   
    if (stored_roll == -9999.0 && stored_pitch == -9999.0 && stored_yaw == -9999.0) {
        // Nếu lần đầu tiên (chưa có dữ liệu trong NVS) → Lưu góc vào NVS
        read_mpu6050_angles_first(&current_roll, &current_pitch, &current_yaw);
        
        //save_angle_to_nvs(current_roll, current_pitch, current_yaw);

        stored_roll = current_roll;
        stored_pitch = current_pitch;
        stored_yaw = current_yaw;
        
        ESP_LOGI(TAG, "Saved initial angles to NVS: Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                 current_roll, current_pitch, current_yaw);
    } else {
        read_mpu6050_angles_alarm(&stored_roll, &stored_pitch, &stored_yaw);

        if(alarm_mpu){
            module_sim_call_sms();
            mqtt_connect();
            while(try_connect_gps < 9){
                gps_pps_monitor();

                if (gps_flag >= 10) {
            
                    //uartsim_init(); // bật lại uart sim
                    //gpio_set_level(GPIO_SIM_TRIGGER, 1);  // bật sim -> kết nối mqtt
                    //gpio_set_level(GPIO_PEN, 1);   
                    //vTaskDelay(pdMS_TO_TICKS(100)); // 30 giây
                    
                    ESP_LOGI(TAG, "Starting Reading GPS Data");
    
                    // 3) Ta dùng vòng while(1) để dễ dàng “lặp” thêm 5 phút
                    while (true)
                    {
                        // a) Bắt đầu đếm 5 phút
                        my_timer_start();  
    
                        // b) Chờ cho đến khi 5 phút kết thúc                      
                        while (!g_timer_done)
                        {
                            // gpio_set_level(GPIO_NUM_18, 1);
                            // vTaskDelay(pdMS_TO_TICKS(2000));
                            // gpio_set_level(GPIO_NUM_18, 0);
    
                            // Khởi tạo Task UART2
                            start_uart_task();
                            // Chờ Task UART2 chạy 30 giây
                            vTaskDelay(pdMS_TO_TICKS(50 * 1000)); // 60 giây
                            // Dừng Task UART2
                            stop_uart_task();
                        
                            ESP_LOGI(TAG, "READ_GPS_TASK finish");
                        
                            //In ra dữ liệu sẽ gửi lên MQTT
                            ESP_LOGI(TAG, "Updated global GPS data: Time=%s, Stolen=%s, Lat=%.6f, Lon=%.6f, Batt=%d%%, Date=%s",
                            global_gps_data.time, 
                            global_gps_data.Stolen ? "true" : "false",  // Chuyển đổi bool thành chuỗi
                            global_gps_data.latitude, 
                            global_gps_data.longitude, 
                            global_gps_data.battery_capacity, 
                            global_gps_data.date);
                            
                            send_gps_data_to_mqtt();
                        }
                    
                        my_timer_delete();
                        // c) Timer báo hết 5 phút => reset cờ
                        g_timer_done = false;  
    
                        // d) Kiểm tra điều kiện
                        // ESP_LOGI(TAG, "Checking dif_location condition...");
                        // check_dif_location(); // Cập nhật giá trị `dif_location`
                        // if (diff_location_flag == false) {
                        //     // Nếu *KHÔNG* thỏa ⇒ break => dừng lặp => đi ngủ
                        //     ESP_LOGI(TAG, "Safe Location => Go to deep sleep.");
                        //     break;
                        // } else {
                        //     ESP_LOGI(TAG, "Different Location => RUN another 5 minutes loop.");
                        //     // Nếu thỏa => tiếp tục vòng while => timer_start() lần nữa
                        //     // (Vòng lặp lại từ đầu)
                        // }
                    }
                
                    // 4) Sau khi thoát vòng lặp => Deep Sleep
                    ESP_LOGI(TAG, "Now going to Deep Sleep...");
                
                }
    
                else
                {
                    printf("NO GPS SIGNAL, try again....\n");
                    // gpio_set_level(GPIO_GPS_TRIGGER, 1);
                    // gpio_set_level(GPIO_SIM_TRIGGER, 0);
                    // gpio_set_level(GPIO_PEN, 0);
                    gpio_set_level(GPIO_GPS_TRIGGER, 1); // Tắt  GPS
                    vTaskDelay(pdMS_TO_TICKS(2 * 1000)); // 2 giây
                    gpio_set_level(GPIO_GPS_TRIGGER, 0); // Bật  GPS
                    try_connect_gps++;
    
                }
            }

            
        }
    }

    xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 10, NULL);

  

}
