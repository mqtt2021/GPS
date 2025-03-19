
#include "main.h"   
#include "cJSON.h"
#include <ctype.h>  // Thư viện chứa isspace()
#include "esp_task_wdt.h"

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
    int Emergency;
    char PhoneNumber[50];
} GpsData;

GpsData data;   

int count = 9090;
int count_send_alarm_clock = 0;
bool send_data  = false ;
bool isProccessing = false;
bool call_case_normal = false;
bool scan_bluetooth = false;
bool wakeup_by_timer = false;
bool mpu_when_wakeup  = false;

// Buffer để lưu trữ thời gian lấy từ module sim
char time_buffer_module_sim[10] = {0};  // Khởi tạo mảng rỗng
// Buffer để lưu trữ Alarm Time
char time_buffer_alarm[10];
bool ini_para_bluetooth = false;
bool stop_task_scan_bluetooth = false;
bool stop_send_data_gps = false;
int count_sub = 0;

void parse_json(const char *json_str, GpsData *data) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        //////printf("Error parsing JSON\n");
        return;
    }

    cJSON *longitude = cJSON_GetObjectItem(root, "Longitude");
    cJSON *latitude = cJSON_GetObjectItem(root, "Latitude");
    cJSON *safeRadius = cJSON_GetObjectItem(root, "SafeRadius");
    cJSON *currentTime = cJSON_GetObjectItem(root, "CurrentTime");
    cJSON *alarmTime = cJSON_GetObjectItem(root, "AlarmTime");
    cJSON *blueTooth = cJSON_GetObjectItem(root, "BlueTooth");
    cJSON *emergency = cJSON_GetObjectItem(root, "Emergency");
    cJSON *phoneNumber = cJSON_GetObjectItem(root, "PhoneNumber");

    if (longitude) data->Longitude = longitude->valuedouble;
    if (latitude) data->Latitude = latitude->valuedouble;
    if (safeRadius) data->SafeRadius = safeRadius->valueint;
    if (currentTime) strncpy(data->CurrentTime, currentTime->valuestring, sizeof(data->CurrentTime) - 1);
    if (alarmTime) strncpy(data->AlarmTime, alarmTime->valuestring, sizeof(data->AlarmTime) - 1);
    if (blueTooth) strncpy(data->BlueTooth, blueTooth->valuestring, sizeof(data->BlueTooth) - 1);
    if (emergency) data->Emergency = emergency->valueint;   
    if (phoneNumber) strncpy(data->PhoneNumber, phoneNumber->valuestring, sizeof(data->PhoneNumber) - 1);    
    
    cJSON_Delete(root); // Giải phóng bộ nhớ sau khi dùng xong  
} 

// Hàm để trích xuất thời gian từ chuỗi AlarmTime
void extract_time_alarm(const char* iso_time, char* time_buffer_alarm) {   
    // // Tìm vị trí của ký tự 'T'
    // const char* t_pos = strchr(iso_time, 'T');
    // if (t_pos == NULL) {
    //     // Nếu không tìm thấy 'T', trả về chuỗi rỗng
    //     time_buffer_alarm[0] = '\0';
    //     return;
    // }

    // // Di chuyển con trỏ đến vị trí sau 'T'
    // t_pos++;

    // // Tìm vị trí của ký tự '.' hoặc 'Z' để xác định phần kết thúc của thời gian
    // const char* end_pos = strchr(t_pos, '.');
    // if (end_pos == NULL) {
    //     end_pos = strchr(t_pos, 'Z');
    // }

    // if (end_pos == NULL) {
    //     // Nếu không tìm thấy '.' hoặc 'Z', trả về chuỗi rỗng
    //     time_buffer_alarm[0] = '\0';
    //     return;
    // }

    // // Tính độ dài của chuỗi thời gian
    // int time_length = end_pos - t_pos;

    // // Sao chép chuỗi thời gian vào buffer
    // strncpy(time_buffer_alarm, t_pos, time_length);
    // time_buffer_alarm[time_length] = '\0'; // Kết thúc chuỗi


    const char* t_pos = strchr(iso_time, 'T'); // Tìm vị trí của 'T'
    if (!t_pos) {
        time_buffer_alarm[0] = '\0'; // Nếu không tìm thấy, trả về chuỗi rỗng
        return;
    }
    t_pos++; // Bỏ qua 'T'

    // Sao chép trực tiếp phần HH:MM:SS (8 ký tự)
    strncpy(time_buffer_alarm, t_pos, 8);
    time_buffer_alarm[8] = '\0'; // Kết thúc chuỗi

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

        //ESP_LOGI(TAG, "response GPS: %s\n",buffer);   
                         
        // Tìm ký tự xuống dòng
        char *newline_ptr;

        while ((newline_ptr = strchr(buffer, '\n')) != NULL) { //Hàm strchr trả về con trỏ đến vị trí đầu tiên của ký tự này trong chuỗi, hoặc trả về NULL nếu không tìm thấy
            *newline_ptr = '\0'; // Kết thúc chuỗi tại ký tự xuống dòng

            // Loại bỏ ký tự carriage return (CRLF) nếu có
            if (newline_ptr > buffer && *(newline_ptr - 1) == '\r') {
                *(newline_ptr - 1) = '\0';
            }

            // Xử lý dữ liệu bắt đầu bằng "$GNRMC"
            if (strncmp(buffer, "$GNRMC", 6) == 0) { //hàm so sánh hai chuỗi với nhau, chỉ so sánh tối đa 6 ký tự đầu tiên, trả về giá trị 0 nếu hai chuỗi giống nhau 
                ////ESP_LOGI(TAG, "Received GNRMC Data: %s\n", buffer);
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
            ////ESP_LOGW(TAG, "Buffer full without receiving newline. Clearing buffer.");
            buffer_pos = 0;
            buffer[buffer_pos] = '\0';
        }
    }

    return false; // Không có dữ liệu hợp lệ được xử lý
}

// ------------------ Quản Lý Task UART ------------------ 
TaskHandle_t uart2TaskHandle = NULL;
TaskHandle_t uart0TaskHandle = NULL;
TaskHandle_t TaskScanBlueToothHandle = NULL;

volatile bool flag_stop_uart_task = false; // Cờ để dừng task
// ------------------ Task UART2 ------------------ read data GPS
void uart2_task(void *pvParameters) {
    char *buffer = (char *)malloc(GPS_UART2_BUFFER_SIZE);
    if (!buffer) {
        //ESP_LOGE(TAG, "Failed to allocate memory for UART2 buffer");
        vTaskDelete(NULL);
        return;
    }
    //ESP_LOGI(TAG, "Start uart2_task, monitoring UART2 data...");

    while (!flag_stop_uart_task) { 
        
        // if(mpu_when_wakeup && !alarm_mpu){

        //     mpu_when_wakeup = false;

        //     read_mpu6050_angles_alarm(&stored_roll, &stored_pitch, &stored_yaw);
           
        //     if(alarm_mpu){

        //         ESP_LOGI(TAG, "ALARMMMMMMM");
        //         //module_sim_call_sms();   
        //     }
        // }

        bool success = read_uart2_data(buffer, GPS_UART2_BUFFER_SIZE);
        if (success) {  
            //ESP_LOGI(TAG, "Data successfully processed.");
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Tránh chiếm CPU
    }

    free(buffer);
    //ESP_LOGI(TAG, "uart2_task exiting...");
    uart2TaskHandle = NULL;
    vTaskDelete(NULL);
}

void uart0_task(void *pvParameters) {
    sub_topic = true;
    while (!flag_stop_uart_task) {
        parse_json(payload_json, &data);
        ////printf("Longitude: %f\n", data.Longitude);
        ////printf("Latitude: %f\n", data.Latitude);
        ////printf("SafeRadius: %d\n", data.SafeRadius);
        ////printf("CurrentTime: %s\n", data.CurrentTime);
        ////printf("AlarmTime: %s\n", data.AlarmTime);
        ////printf("BlueTooth: %s\n", data.BlueTooth);

        
        fix_lattitude = data.Latitude;
        fix_longtitude = data.Longitude;
        radius = data.SafeRadius;
        

        

            if (strcmp(data.BlueTooth, "ON") == 0) {
                if(!ini_para_bluetooth){
                    if(!is_init_BLE){
                        init_BLE();
                    }
                    
                    start_advertising_BLE();
                    ini_para_bluetooth = true;
                }
                gpio_set_level(GPIO_NUM_18, 1);
                ////printf("Bật BUZZER\n");
            } else if (strcmp(data.BlueTooth, "OFF") == 0) {
                if(ini_para_bluetooth){
                    stop_adertising_BLE();
                    ini_para_bluetooth = false;
                }
                gpio_set_level(GPIO_NUM_18, 0);
                ////printf("Tắt BUZZER\n");
            } else {
                ////printf("Giá trị BlueTooth không hợp lệ: '%s'\n", data.BlueTooth);
            }   
            vTaskDelay(pdMS_TO_TICKS(1000));  // Chờ 500ms để giảm tải CPU
    }
    uart0TaskHandle = NULL;
    //ESP_LOGI(TAG, "uart0_task exiting...");
    vTaskDelete(NULL);
}
  
void start_uart_task(void) {
    if (uart2TaskHandle == NULL) {
        flag_stop_uart_task = false;  
        //ESP_LOGI(TAG, "Creating uart2_task...");
        if (xTaskCreate(uart2_task, "uart2_task", 8192, NULL, 5, &uart2TaskHandle) != pdPASS) {
            ////ESP_LOGE(TAG, "Failed to create uart2_task");
            return;
        }
    } else {
        //ESP_LOGW(TAG, "uart2_task is already running.");
    }

    if (uart0TaskHandle == NULL) {
        //ESP_LOGI(TAG, "Creating uart0_task...");
        if (xTaskCreate(uart0_task, "uart0_task", 8192, NULL, 5, &uart0TaskHandle) != pdPASS) {
            ////ESP_LOGE(TAG, "Failed to create uart0_task");
            return;
        }
    } else {
        //ESP_LOGW(TAG, "uart0_task is already running.");
    }
}  

void stop_uart_task(void) {
    if (uart2TaskHandle != NULL || uart0TaskHandle != NULL) {
        ESP_LOGI(TAG, "Stopping UART tasks...");
        sub_topic = false;
        flag_stop_uart_task = true;  

       // Chờ task tự thoát
       int timeout = 10; // Giới hạn 10 lần kiểm tra
       while ((uart2TaskHandle != NULL || uart0TaskHandle != NULL) && timeout-- > 0) {
           ////ESP_LOGI(TAG, "Waiting for UART tasks to stop...");
           vTaskDelay(pdMS_TO_TICKS(500)); 
       }

        if (uart2TaskHandle != NULL) {
            //ESP_LOGW(TAG, "Force deleting uart2_task!");
            vTaskDelete(uart2TaskHandle);
            uart2TaskHandle = NULL;
        }

        if (uart0TaskHandle != NULL) {
            //ESP_LOGW(TAG, "Force deleting uart0_task!");
            vTaskDelete(uart0TaskHandle);
            uart0TaskHandle = NULL;
        }

        ////ESP_LOGI(TAG, "UART tasks stopped.");
    } else {
        //ESP_LOGW(TAG, "No UART task is running.");
    }
}

void gps_pps_monitor() {
    TickType_t startTick = xTaskGetTickCount(); // Lấy thời gian bắt đầu
    const TickType_t runDuration = pdMS_TO_TICKS(15 * 60 * 1000); // 15 phút

    ////ESP_LOGI(TAG, "Starting GPS PPS monitoring for 3 minutes...");

    // Cấu hình GPIO
    gpio_reset_pin(GPIO_GPS_PPS);
    gpio_set_direction(GPIO_GPS_PPS, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_GPS_PPS, GPIO_PULLDOWN_ONLY); // Kéo xuống mặc định

    // Vòng lặp chạy trong 2 phút
    while ((xTaskGetTickCount() - startTick) < runDuration) {
        if (gpio_get_level(GPIO_GPS_PPS) == 1) {
            gps_flag++;
            ////ESP_LOGI(TAG, "GPS PPS detected! gps_flag = %d", gps_flag);
            vTaskDelay(pdMS_TO_TICKS(100)); // Chống rung (debounce)
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Kiểm tra mỗi 10ms
        
        if (gps_flag >= 10)
        {
            gpio_set_level(GPIO_NUM_18, 1);
           

            break;
        }
    }
    ////ESP_LOGI(TAG, "2 minutes elapsed. Final gps_flag count: %d", gps_flag);
    // Đặt cờ báo hiệu task đã hoàn thành
    gps_task_done = true;
}

void configure_led(void)
{
    gpio_reset_pin(GPIO_GPS_TRIGGER); //GPS  
    gpio_set_direction(GPIO_GPS_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_GPS_TRIGGER, 0);      

    gpio_reset_pin(GPIO_SIM_TRIGGER);
    gpio_set_direction(GPIO_SIM_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_SIM_TRIGGER, 1);  

    gpio_reset_pin(GPIO_PEN); // PEN_SIM
    gpio_set_direction(GPIO_PEN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_PEN, 1);

    gpio_reset_pin(BUZZER); // BUZZER
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER, 0);

}

void print_wakeup_reason(void) {
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ////ESP_LOGI("WAKEUP", "Thức dậy do GPIO (EXT0)");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ////ESP_LOGI("WAKEUP", "Thức dậy do nhiều GPIO (EXT1)");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            ////ESP_LOGI("WAKEUP", "Thức dậy do Timer");    
            wakeup_by_timer = true;      
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            ////ESP_LOGI("WAKEUP", "Thức dậy do Cảm ứng");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            ////ESP_LOGI("WAKEUP", "Thức dậy do ULP (Ultra Low Power)");
            break;
        case ESP_SLEEP_WAKEUP_UART:
            ////ESP_LOGI("WAKEUP", "Thức dậy do UART (ESP32-S3)");
            break;
        case ESP_SLEEP_WAKEUP_WIFI:
            ////ESP_LOGI("WAKEUP", "Thức dậy do WiFi (ESP32-S3)");
            break;
        default:
            ////ESP_LOGI("WAKEUP", "Thức dậy không xác định (Power-on hoặc Reset)");
            break;
    }
}

static void deep_sleep_task(void *args)
{
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            ////printf("Wake up from timer. Time spent in deep sleep\n");
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT0: {
            ////printf("Wake up from ext0\n");
            break;
        }

        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                ////printf("Wake up from GPIO %d\n", pin);
            } else {
                ////printf("Wake up from GPIO\n");
            }
            break;
        }

        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            ////printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
            break;
        }

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ////printf("Not a deep sleep reset\n");
    }
    // enter deep sleep

}

void example_deep_sleep_register_ext1_wakeup(void)
{
    const uint64_t ext_wakeup_pin_mask = 1ULL << GPIO_NUM_2; // Chỉ dùng GPIO 2
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH));
    ////printf("ESP sẽ thức dậy khi GPIO 2 chuyển từ HIGH xuống LOW\n");
}

int time_to_seconds(const char* time_str) {
    int h, m, s;
    if (sscanf(time_str, "%d:%d:%d", &h, &m, &s) != 3) {
        return -1;  // Lỗi định dạng
    }
    return h * 3600 + m * 60 + s;
}
// Tính thời gian ngủ
void set_esp_sleep_time(const char* current_time, const char* wakeup_time) {
    int current_seconds = time_to_seconds(current_time);
    int wakeup_seconds = time_to_seconds(wakeup_time);

    if (current_seconds == -1 || wakeup_seconds == -1) {
        ////printf("Lỗi định dạng thời gian!\n");
        return;
    }

    int sleep_time = wakeup_seconds - current_seconds;
    if (sleep_time <= 0) {
        sleep_time += 24 * 3600;  // Nếu wakeup_time nhỏ hơn current_time, tức là qua ngày hôm sau
    }

    // Chuyển sleep_time sang uint64_t để tránh tràn số khi nhân với 1000000
    uint64_t sleep_time_us = (uint64_t)sleep_time * 1000000ULL;

    ESP_LOGI("MAIN", "ESP sẽ ngủ trong %d giây", sleep_time);
 
    // Đặt ESP vào deep sleep trong sleep_time giây
    esp_sleep_enable_timer_wakeup(sleep_time_us);

                gpio_set_level(GPIO_NUM_18, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                gpio_set_level(GPIO_NUM_18, 0);

    //printf("ESP sẽ ngủ trong %d giây\n", sleep_time);
}

// Hàm kiểm tra chuỗi có đúng định dạng "hh:mm:ss" không
bool is_valid_time_format(const char *time_str) {
    if (strlen(time_str) != 8) {
        return false; // Độ dài không đúng định dạng "HH:MM:SS"
    }

    return isdigit((unsigned char)time_str[0]) && isdigit((unsigned char)time_str[1]) &&
           time_str[2] == ':' &&
           isdigit((unsigned char)time_str[3]) && isdigit((unsigned char)time_str[4]) &&
           time_str[5] == ':' &&
           isdigit((unsigned char)time_str[6]) && isdigit((unsigned char)time_str[7]);
}

// Khai báo biến RTC với giá trị mặc định khó xảy ra
RTC_DATA_ATTR float stored_roll = -9999.0;
RTC_DATA_ATTR float stored_pitch = -9999.0;
RTC_DATA_ATTR float stored_yaw = -9999.0;
RTC_DATA_ATTR int emergency = 0;
RTC_DATA_ATTR double fix_lattitude = 0;
RTC_DATA_ATTR double fix_longtitude = 0;
RTC_DATA_ATTR int radius = 0;

void app_main(void)
{
    print_wakeup_reason();   
   
    // Tắt giữ trạng thái GPIO sau khi thức dậy
    gpio_hold_dis(GPIO_SIM_TRIGGER);
    gpio_hold_dis(GPIO_PEN);
    gpio_hold_dis(GPIO_GPS_TRIGGER);                 
    
    // Cho phép ESP điều khiển lại GPIO
    gpio_deep_sleep_hold_dis();
    configure_led();  

    gpio_set_level(GPIO_NUM_18, 1);
    vTaskDelay(pdMS_TO_TICKS(5000));
    gpio_set_level(GPIO_NUM_18, 0); 

    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_enable_interrupt_pin();
    mpu6050_enable_motion_interrupt(); // CẤU HÌNH MPU
    uartsim_init(); // UART SIM    
    init_uart2(); // UART GPS
    adc_init(); // cấu hình ADC
    example_deep_sleep_register_ext1_wakeup(); // cấu hình đánh thức GPIO 

    // ESP mới khởi động, lấy dữ liệu Setting về
    if(emergency == 0){
       ////ESP_LOGI(TAG, "CHECK EMERGENCY");
        
        mqtt_connect();     // Kết nối MQTT   
        //get_time_from_module_sim();  // Lấy dữ liệu thời gian về
        subcribe_topic_mqtt();  // Lấy dữ liệu cài đặt về
        TickType_t start_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
        while( true ){
            parse_json(payload_json, &data);

            if (strcmp(data.BlueTooth, "ON") == 0 || strcmp(data.BlueTooth, "OFF") == 0)
            {
                ////printf("data.Emergency %d\n", data.Emergency);
                if(data.Emergency){ 
                    emergency = 1; // Chế độ khẩn cấp
                    gpio_set_level(GPIO_NUM_18, 1);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    gpio_set_level(GPIO_NUM_18, 0);
                    fix_lattitude = data.Latitude;
                    fix_longtitude = data.Longitude;
                    radius = data.SafeRadius;
                    ////ESP_LOGI(TAG, "EMERGENCY");
                }
                else{
                    emergency = 2; // Chế độ hàng rào
                    fix_lattitude = data.Latitude;
                    fix_longtitude = data.Longitude;
                    radius = data.SafeRadius;
                    gpio_set_level(GPIO_NUM_18, 1);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    gpio_set_level(GPIO_NUM_18, 0); 
                    ////ESP_LOGI(TAG, "NORMAL");  
                    ////ESP_LOGI(TAG, "Lat: %.5f , Lng : %.5f ", fix_lattitude , fix_longtitude);
                    ////ESP_LOGI(TAG, "Radius: %d", radius);
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));

            if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(30000)) {
                ////ESP_LOGW(TAG, "⏳ Timeout");
                mqtt_connect(); 
                subcribe_topic_mqtt(); 
                TickType_t start_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
            }    
        }
    }  

    if(wakeup_by_timer){      
        handle_create_event();
        mqtt_connect();     // Kết nối MQTT   
        my_timer_start();

        while (!g_timer_done)
        {
                subcribe_topic_mqtt(); 
                // Khởi tạo Task UART2
                start_uart_task();
                // Chờ Task UART2 chạy 30 giây
                vTaskDelay(pdMS_TO_TICKS(50 * 1000)); // 60 giây
                // Dừng Task UART2
                stop_uart_task();

                if(global_gps_data.latitude > 0){ // Đã bắt được tọa độ, đi ngủ
                    if(count_send_alarm_clock < 5){   
                        send_gps_data_to_mqtt();
                    }
                    else{
                        count_send_alarm_clock = 0;
                        check_dif_location(); // Cập nhật giá trị `dif_location`
                        if (diff_location_flag == false) {
                            break;
                        } else {
                
                        }
                    }
                    count_send_alarm_clock++;
                }
                vTaskDelay(pdMS_TO_TICKS(3000)); // 60 giây
        }
          
        my_timer_delete();  
        g_timer_done = false;

        if(global_gps_data.latitude > 0){ // Đã bắt được tọa độ
            
        }
        else{
    
            float voltage = read_battery_voltage();    
            int percentage = voltage_to_percentage(voltage);
            global_gps_data.battery_capacity = percentage;  

            get_time_from_module_sim();  // Lấy dữ liệu thời gian hiện tại về
            TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
            while( true ){   
                if (is_valid_time_format(time_buffer_module_sim)) {
                    // //printf("Thời gian báo thức: %s\n", time_buffer_alarm);
                    // //printf("Thời gian hiện tại: %s\n", time_buffer_module_sim);
                    break;  
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                if (xTaskGetTickCount() - start_time_get_current_time > pdMS_TO_TICKS(30000)) {
                    ////ESP_LOGW(TAG, "⏳ Timeout");
                    get_time_from_module_sim();
                    TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
                }    
            }

                        char mqtt_payload[BUFFER_SIZE];  
                        snprintf(mqtt_payload, sizeof(mqtt_payload),
                        "["    
                        "{\"name\":\"Latitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Longitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Battery\",\"value\":%d,\"timestamp\":\"%s\"},"   
                        "{\"name\":\"Stolen\",\"value\":false,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Bluetooth\",\"value\":%s,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Move\",\"value\":false,\"timestamp\":\"%s\"}"
                        "]",
                        global_gps_data.latitude,  
                        time_buffer_module_sim,
                        global_gps_data.longitude,  
                        time_buffer_module_sim,
                        global_gps_data.battery_capacity,
                        time_buffer_module_sim,
                        // global_gps_data.Stolen ? "true" : "false",
                        time_buffer_module_sim, 
                        global_gps_data.bluetooth ? "true" : "false",
                        time_buffer_module_sim,  
                        // global_gps_data.move ? "true" : "false",
                        time_buffer_module_sim       
                        );
    
                        mqtt_publish("GPS/Status/G002", mqtt_payload);
        }
    }
    else{

        // góc hiện tại từ MPU6050   
        float current_roll = 0, current_pitch = 0, current_yaw = 0;

        if(emergency == 1){ // Chế độ khẩn cấp
            if (stored_roll == -9999.0 && stored_pitch == -9999.0 && stored_yaw == -9999.0) {  // Mới bật nguồn lên
                // Nếu lần đầu tiên chưa có dữ liệu 
                read_mpu6050_angles_first(&current_roll, &current_pitch, &current_yaw);
                stored_roll = current_roll;
                stored_pitch = current_pitch;
                stored_yaw = current_yaw;
                ////ESP_LOGI(TAG, "Saved initial angles to RTC IO : Roll=%.2f, Pitch=%.2f, Yaw=%.2f", current_roll, current_pitch, current_yaw);
            } else {
                read_mpu6050_angles_alarm(&stored_roll, &stored_pitch, &stored_yaw);
                if(alarm_mpu){     
                        handle_create_event();  
                        module_sim_call_sms();    
                        mqtt_connect(); 

                        float voltage = read_battery_voltage();    
                        int percentage = voltage_to_percentage(voltage);
                        global_gps_data.battery_capacity = percentage;  
            
                        get_time_from_module_sim();  // Lấy dữ liệu thời gian hiện tại về
                        TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
                        while( true ){   
                            if (is_valid_time_format(time_buffer_module_sim)) {
                                // //printf("Thời gian báo thức: %s\n", time_buffer_alarm);
                                // //printf("Thời gian hiện tại: %s\n", time_buffer_module_sim);
                                break;  
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            if (xTaskGetTickCount() - start_time_get_current_time > pdMS_TO_TICKS(30000)) {
                                ////ESP_LOGW(TAG, "⏳ Timeout");
                                get_time_from_module_sim();
                                TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
                            }    
                        }

                        char mqtt_payload[BUFFER_SIZE];  
                        snprintf(mqtt_payload, sizeof(mqtt_payload),
                        "["    
                        "{\"name\":\"Latitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Longitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Battery\",\"value\":%d,\"timestamp\":\"%s\"},"   
                        "{\"name\":\"Stolen\",\"value\":true,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Bluetooth\",\"value\":%s,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Move\",\"value\":true,\"timestamp\":\"%s\"}"    
                        "]",
                        global_gps_data.latitude,  
                        time_buffer_module_sim,
                        global_gps_data.longitude,  
                        time_buffer_module_sim,
                        global_gps_data.battery_capacity,
                        time_buffer_module_sim,
                        // global_gps_data.Stolen ? "true" : "false",
                        time_buffer_module_sim, 
                        global_gps_data.bluetooth ? "true" : "false",
                        time_buffer_module_sim,  
                        // global_gps_data.move ? "true" : "false",
                        time_buffer_module_sim       
                        );
    
                        mqtt_publish("GPS/Status/G002", mqtt_payload);

                        while (true)
                        {
                                subcribe_topic_mqtt();  // Lấy dữ liệu cài đặt về
                                // Khởi tạo Task UART2
                                start_uart_task();
                                // Chờ Task UART2 chạy 30 giây
                                vTaskDelay(pdMS_TO_TICKS(50 * 1000)); // 60 giây
                                // Dừng Task UART2
                                stop_uart_task();
                                 
                                if(global_gps_data.latitude > 0){ // Đã bắt được tọa độ
                                        send_gps_data_to_mqtt();
                                } 
                                vTaskDelay(pdMS_TO_TICKS(3000));
                        }                     
                }
            }
        }

        if(emergency == 2){ // Chế độ bình thường, bật hàng rào
        
            if (stored_roll == -9999.0 && stored_pitch == -9999.0 && stored_yaw == -9999.0) {  // Mới bật nguồn lên
                
                // Nếu lần đầu tiên chưa có dữ liệu 
                read_mpu6050_angles_first(&current_roll, &current_pitch, &current_yaw);
        
                stored_roll = current_roll;
                stored_pitch = current_pitch;
                stored_yaw = current_yaw;
                
                ////ESP_LOGI(TAG, "Saved initial angles to RTC IO : Roll=%.2f, Pitch=%.2f, Yaw=%.2f",current_roll, current_pitch, current_yaw);
                        
            } else {
                read_mpu6050_angles_alarm(&stored_roll, &stored_pitch, &stored_yaw);
                if(alarm_mpu){
                        handle_create_event();  
                        mqtt_connect();

                        float voltage = read_battery_voltage();    
                        int percentage = voltage_to_percentage(voltage);
                        global_gps_data.battery_capacity = percentage;  
            
                        get_time_from_module_sim();  // Lấy dữ liệu thời gian hiện tại về
                        TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
                        while( true ){   
                            if (is_valid_time_format(time_buffer_module_sim)) {
                                // //printf("Thời gian báo thức: %s\n", time_buffer_alarm);
                                // //printf("Thời gian hiện tại: %s\n", time_buffer_module_sim);
                                break;  
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            if (xTaskGetTickCount() - start_time_get_current_time > pdMS_TO_TICKS(30000)) {
                                ////ESP_LOGW(TAG, "⏳ Timeout");
                                get_time_from_module_sim();
                                TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
                            }    
                        }


                        char mqtt_payload[BUFFER_SIZE];  
                        snprintf(mqtt_payload, sizeof(mqtt_payload),
                        "["    
                        "{\"name\":\"Latitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Longitude\",\"value\":%.5f,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Battery\",\"value\":%d,\"timestamp\":\"%s\"},"   
                        "{\"name\":\"Stolen\",\"value\":false,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Bluetooth\",\"value\":%s,\"timestamp\":\"%s\"},"
                        "{\"name\":\"Move\",\"value\":true,\"timestamp\":\"%s\"}"
                        "]",
                        global_gps_data.latitude,  
                        time_buffer_module_sim,
                        global_gps_data.longitude,  
                        time_buffer_module_sim,
                        global_gps_data.battery_capacity,
                        time_buffer_module_sim,
                        // global_gps_data.Stolen ? "true" : "false",
                        time_buffer_module_sim, 
                        global_gps_data.bluetooth ? "true" : "false",
                        time_buffer_module_sim,  
                        // global_gps_data.move ? "true" : "false",
                        time_buffer_module_sim       
                        );
    
                        mqtt_publish("GPS/Status/G002", mqtt_payload);

                        while (true)
                        {
                            //subcribe_topic_mqtt();  // Lấy dữ liệu cài đặt về
                            // a) Bắt đầu đếm 5 phút
                            my_timer_connect_mqtt_start();  
        
                            // b) Chờ cho đến khi 5 phút kết thúc                      
                            while (!g_timer_connect_mqtt_done || global_gps_data.Stolen )
                            {      
                                subcribe_topic_mqtt();      
                                // Khởi tạo Task UART2
                                start_uart_task();
                                // Chờ Task UART2 chạy 50 giây
                                vTaskDelay(pdMS_TO_TICKS(50 * 1000)); // 60 giây
                                // Dừng Task UART2
                                stop_uart_task();

                                if(global_gps_data.latitude > 0){ // Đã bắt được tọa độ
                                    send_gps_data_to_mqtt();
                                }

                                vTaskDelay(pdMS_TO_TICKS(3000)); // 60 giây
                                
                            }
                        
                            my_timer_connect_mqtt_delete();
                            // c) Timer báo hết 5 phút => reset cờ
                            g_timer_connect_mqtt_done = false;  
                            //d) Kiểm tra điều kiện
                            check_dif_location(); // Cập nhật giá trị `dif_location`
                            if (diff_location_flag == false) {
                                break;
                            } else {
                               
                            }
                        }

                        read_mpu6050_angles_first(&current_roll, &current_pitch, &current_yaw);
                        stored_roll = current_roll;
                        stored_pitch = current_pitch;
                        stored_yaw = current_yaw;
                }
            } 
        }
    }

    mqtt_connect();     // Kết nối MQTT   
    subcribe_topic_mqtt();  // Lấy dữ liệu cài đặt về
    
    TickType_t start_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
    while( true ){
        parse_json(payload_json, &data);
        if (strcmp(data.BlueTooth, "ON") == 0 || strcmp(data.BlueTooth, "OFF") == 0)
        {

            if(data.Emergency){ 
                emergency = 1; // Chế độ khẩn cấp
                gpio_set_level(GPIO_NUM_18, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(GPIO_NUM_18, 0);
                ////ESP_LOGI(TAG, "EMERGENCY");

                fix_lattitude = data.Latitude;
                fix_longtitude = data.Longitude;
                radius = data.SafeRadius;
            }
            else{
                emergency = 2; // Chế độ hàng rào
                fix_lattitude = data.Latitude;
                fix_longtitude = data.Longitude;
                radius = data.SafeRadius;
                gpio_set_level(GPIO_NUM_18, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                gpio_set_level(GPIO_NUM_18, 0); 
                ////ESP_LOGI(TAG, "NORMAL");  
                ////ESP_LOGI(TAG, "Lat: %.5f , Lng : %.5f ", fix_lattitude , fix_longtitude);
                ////ESP_LOGI(TAG, "Radius: %d", radius);
            }
            
            extract_time_alarm(data.AlarmTime, time_buffer_alarm);
           
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(30000)) {
            //ESP_LOGW(TAG, "⏳ Timeout");
            mqtt_connect(); 
            subcribe_topic_mqtt(); 
            TickType_t start_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
        }    
    }

    get_time_from_module_sim();  // Lấy dữ liệu thời gian hiện tại về
    TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
    while( true ){
        if (is_valid_time_format(time_buffer_module_sim)) {
            //printf("Thời gian báo thức: %s\n", time_buffer_alarm);
            //printf("Thời gian hiện tại: %s\n", time_buffer_module_sim);
            set_esp_sleep_time(time_buffer_module_sim, time_buffer_alarm);
            break;  
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        if (xTaskGetTickCount() - start_time_get_current_time > pdMS_TO_TICKS(30000)) {
            ////ESP_LOGW(TAG, "⏳ Timeout");
            get_time_from_module_sim();
            TickType_t start_time_get_current_time = xTaskGetTickCount(); // Lưu thời gian bắt đầu nhận dữ liệu
        }    
    }

    gpio_set_direction(GPIO_SIM_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_SIM_TRIGGER, 0);  // Đưa chân GPIO xuống mức thấp
    gpio_hold_en(GPIO_SIM_TRIGGER); // Giữ trạng thái khi ngủ

    gpio_set_direction(GPIO_PEN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_PEN, 0);  // Đưa chân GPIO xuống mức thấp
    gpio_hold_en(GPIO_PEN); // Giữ trạng thái khi ngủ

    gpio_set_direction(GPIO_GPS_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_GPS_TRIGGER, 1);  // Đưa chân GPIO xuống mức thấp
    gpio_hold_en(GPIO_GPS_TRIGGER); // Giữ trạng thái khi ngủ   

    gpio_deep_sleep_hold_en();
    //printf("Sleepppppppppppppppppppp\n");
    esp_deep_sleep_start();  
    //xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 10, NULL);
}
