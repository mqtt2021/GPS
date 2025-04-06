#include "my_timer.h"

/* 
 * Đặt tên TAG để log cho module này,
 * nếu dùng LOG trong ISR nhớ dùng ESP_EARLY_LOGx 
 */
static const char *TAG = "MY_TIMER_MODULE";
volatile bool g_timer_done = false;                                                                  
volatile bool g_timer_connect_mqtt_done = false;

/* 
 * Callback được gọi mỗi khi timer chạm ngưỡng alarm.
 * Lưu ý: ISR nên đặt trong IRAM nếu dùng các hàm log hay xử lý nhanh.
 */

 bool IRAM_ATTR my_timer_isr_callback(void *args)
{
    // Tắt timer sau khi hoàn thành
    timer_pause(TIMER_GROUP_0, TIMER_1);  
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);  // Reset bộ đếm về 0
    g_timer_done = true;

    return pdFALSE;
}
/**
 * @brief  Khởi tạo timer (prescaler, alarm, auto_reload).
 */
void my_timer_init(void)
{
    timer_config_t config = {
        .divider     = 80,                // 1 tick = 1µs (nếu APB clk = 80 MHz)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .auto_reload = false,             // <--- KHÔNG auto-reload
        .alarm_en    = TIMER_ALARM_EN
    };
    timer_init(TIMER_GROUP_0, TIMER_1, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);

    // 5 phút = 300 giây = 300 * 1.000.000 tick = 300.000.000
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 2400000000ULL);

    // Cho phép interrupt
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);

    // Đăng ký callback
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, my_timer_isr_callback, NULL, 0);
    
}


/**
 * @brief  Bắt đầu chạy timer.
 */
void my_timer_start(void)
{
    my_timer_init();
    //ESP_LOGI(TAG, "🚀 Bắt đầu timer...");

    timer_start(TIMER_GROUP_0, TIMER_1);
}

/**
 * @brief  Tạm dừng timer.
 */
void my_timer_delete(void)
{
    // 1️⃣ Dừng timer trước khi xóa
    timer_pause(TIMER_GROUP_0, TIMER_1);
    
    // 2️⃣ Xóa callback (nếu có)
    timer_isr_callback_remove(TIMER_GROUP_0, TIMER_1);
    
    // 3️⃣ Giải phóng interrupt (nếu đã đăng ký)
    timer_disable_intr(TIMER_GROUP_0, TIMER_1);
    
    //ESP_LOGI(TAG, "✅ Timer đã bị xóa hoàn toàn.----------------------------------------------------------------");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IRAM_ATTR my_timer_connect_mqtt_isr_callback(void *args)
{
    // Tắt timer sau khi hoàn thành
    timer_pause(TIMER_GROUP_1, TIMER_0);  
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);  // Reset bộ đếm về 0
    g_timer_connect_mqtt_done = true;

    return pdFALSE;
}
/**
 * @brief  Khởi tạo timer (prescaler, alarm, auto_reload).
 */
void my_timer_connect_mqtt_init(void)
{
    timer_config_t config = {
        .divider     = 80,                // 1 tick = 1µs (nếu APB clk = 80 MHz)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .auto_reload = false,             // <--- KHÔNG auto-reload
        .alarm_en    = TIMER_ALARM_EN
    };
    timer_init(TIMER_GROUP_1, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);

    // 5 phút = 300 giây = 300 * 1.000.000 tick = 300.000.000
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 300000000ULL);

    // Cho phép interrupt
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);

    // Đăng ký callback
    timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, my_timer_connect_mqtt_isr_callback, NULL, 0);
    
}


/**
 * @brief  Bắt đầu chạy timer.
 */
void my_timer_connect_mqtt_start(void)
{
    my_timer_connect_mqtt_init();
    //ESP_LOGI(TAG, "🚀 Bắt đầu timer...");

    timer_start(TIMER_GROUP_1, TIMER_0);
}

/**
 * @brief  Tạm dừng timer.
 */
void my_timer_connect_mqtt_delete(void)
{
    // 1️⃣ Dừng timer trước khi xóa
    timer_pause(TIMER_GROUP_1, TIMER_0);
    
    // 2️⃣ Xóa callback (nếu có)
    timer_isr_callback_remove(TIMER_GROUP_1, TIMER_0);
    
    // 3️⃣ Giải phóng interrupt (nếu đã đăng ký)
    timer_disable_intr(TIMER_GROUP_1, TIMER_0);
    
    //ESP_LOGI(TAG, "✅ Timer đã bị xóa hoàn toàn.----------------------------------------------------------------");
}