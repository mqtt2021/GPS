#include "mpu6050.h"



#define MPU6050_ADDR 0x68
#define ACC_FS_2G 16384.0  // Độ phân giải ±2g
#define GRAVITY 9.81       // Trọng lực trái đất (m/s²)
#define THRESHOLD 0.1      // Ngưỡng bỏ qua nhiễu nhỏ
#define DT 0.01            // Khoảng thời gian giữa 2 lần lấy mẫu (10ms)
static const char *TAG = "i2c-tools";

volatile bool g_timer_read_mpu_done = false;
volatile bool g_timer_noiseMPU_done = false;
volatile bool alarm_mpu = false;

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


esp_err_t mpuReadfromReg (uint8_t Reg, uint8_t *ReadBuffer, size_t len)
{
	return (i2c_master_write_read_device(I2C_NUM, 0x68, &Reg, 1, ReadBuffer, len, 2000));
}


esp_err_t mpuWriteReg (uint8_t Reg, uint8_t data)
{
	uint8_t writeBuf[2];  // writeBuf[len+1];
	writeBuf[0] = Reg;
	writeBuf[1] = data;
	return (i2c_master_write_to_device(I2C_NUM, 0x68, writeBuf, 2, 1000));
}

bool IRAM_ATTR my_timer_isr_callback_read_mpu(void *args)
{
    // Tắt timer sau khi hoàn thành
    timer_pause(TIMER_GROUP_0, TIMER_0);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);  // Reset bộ đếm về 0
    g_timer_read_mpu_done = true;

    return pdFALSE;
}

bool IRAM_ATTR my_timer_noiseMPU_isr_callback(void *args)
{
    // Tắt timer sau khi hoàn thành
    timer_pause(TIMER_GROUP_1, TIMER_1);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);  // Reset bộ đếm về 0
    g_timer_noiseMPU_done = true;

    return pdFALSE;
}

void my_timer_init_read_mpu(void)
{
    timer_config_t config = {
        .divider     = 80,                // 1 tick = 1µs (nếu APB clk = 80 MHz)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .auto_reload = false,             // <--- KHÔNG auto-reload
        .alarm_en    = TIMER_ALARM_EN
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 10000000ULL); // 10 giây
    // Cho phép interrupt
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    // Đăng ký callback
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, my_timer_isr_callback_read_mpu, NULL, 0);

}

void my_timer_start_read_mpu(void)
{
    
    my_timer_init_read_mpu();
    //ESP_LOGI(TAG, "🚀 Bắt đầu timer read MPU...");
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void my_timer_delete_read_mpu(void)
{
    //ESP_LOGI(TAG, "🗑 Đang xóa hoàn toàn timer...");

    // 1️⃣ Dừng timer trước khi xóa
    timer_pause(TIMER_GROUP_0, TIMER_0);

    // 2️⃣ Xóa callback (nếu có)
    timer_isr_callback_remove(TIMER_GROUP_0, TIMER_0);

    // 3️⃣ Giải phóng interrupt (nếu đã đăng ký)
    timer_disable_intr(TIMER_GROUP_0, TIMER_0);

    //ESP_LOGI(TAG, "✅ Timer đã bị xóa hoàn toàn.");
}

void my_timer_noiseMPU_init(void)
{
    timer_config_t config = {
        .divider     = 80,                // 1 tick = 1µs (nếu APB clk = 80 MHz)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .auto_reload = false,             // <--- KHÔNG auto-reload
        .alarm_en    = TIMER_ALARM_EN
    };
    timer_init(TIMER_GROUP_1, TIMER_1, &config);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);

    // 5 phút = 300 giây = 300 * 1.000.000 tick = 300.000.000
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, 600000ULL);

    // Cho phép interrupt
    timer_enable_intr(TIMER_GROUP_1, TIMER_1);

    // Đăng ký callback
    timer_isr_callback_add(TIMER_GROUP_1, TIMER_1, my_timer_noiseMPU_isr_callback, NULL, 0);

}

void my_timer_noiseMPU_start(void)
{
    my_timer_noiseMPU_init();
    //ESP_LOGI(TAG, "🚀 Bắt đầu lại timer...");
    timer_start(TIMER_GROUP_1, TIMER_1);
}

void my_timer_noiseMPU_delete(void)
{
    //ESP_LOGI(TAG, "🗑 Đang xóa hoàn toàn timer...");

    // 1️⃣ Dừng timer trước khi xóa
    timer_pause(TIMER_GROUP_1, TIMER_1);

    // 2️⃣ Xóa callback (nếu có)
    timer_isr_callback_remove(TIMER_GROUP_1, TIMER_1);

    // 3️⃣ Giải phóng interrupt (nếu đã đăng ký)
    timer_disable_intr(TIMER_GROUP_1, TIMER_1);

    //ESP_LOGI(TAG, "✅ Timer đã bị xóa hoàn toàn.");
}

   
void mpu6050_enable_motion_interrupt()        
{
    mpuWriteReg(0x6B, 0x00);  // Bật MPU6050
    mpuWriteReg(0x1C, 0x00);  // Chọn thang đo ±2g   
    mpuWriteReg(0x1F, 0x03);  // Ngưỡng rung động (tùy chỉnh)   
    mpuWriteReg(0x20, 0x01);  // Thời gian rung động tối thiểu
    mpuWriteReg(0x38, 0x40);  // Bật ngắt Motion
    //mpuWriteReg(0x69, 0x20);  // Cấu hình INT kéo xuống LOW khi có ngắt
    mpuWriteReg(0x69, 0x00);  // Cấu hình INT kéo lên HIGH khi có ngắt

}

// void mpu6050_isr_handler(void){
//         if(wakeup_by_timer){ // Đang báo thức mà có đứa đụng vào
//             mpu_when_wakeup = true;  
//         }
// }

void mpu6050_enable_interrupt_pin()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Bật pull-up nếu cần
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE // Ngắt khi INT chuyển từ LOW lên HIGH
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    //gpio_isr_handler_add(GPIO_NUM_2, mpu6050_isr_handler, NULL);
}

void read_mpu6050_angles_first(float *roll_pointer, float *pitch_pointer, float *yaw_pointer) {
// Biến lưu vận tốc
// float velocity_x = 0;
// float velocity_y = 0;
// float prev_time = 0;

//     uint8_t data[6];

//     // Cấu hình MPU6050
//     mpuWriteReg(0x6B, 0x00);  // Wake up MPU6050
//     mpuWriteReg(0x19, 0x00);  // Sample rate = 1KHz
//     mpuWriteReg(0x1C, 0x00);  // ACC ±2g
//     vTaskDelay(pdMS_TO_TICKS(100));

    // while (1) {
    //         uint8_t data[10];
    //         mpuReadfromReg(0x75, data, 1);
    //         mpuWriteReg(0x6B, 0);
    //         mpuWriteReg(0x19, 7); // sample rate 1KHz
    //         mpuWriteReg(0x1C, 0);  // ACC FS Range ±2g

    //         mpuReadfromReg(0x3B, data, 6);

    //         int16_t RAWX = (data[0]<<8)|data[1];
    //         int16_t RAWY = (data[2]<<8)|data[3];
    //         int16_t RAWZ = (data[4]<<8)|data[5];

    //         float xg = (float)RAWX/16384;
    //         float yg = (float)RAWY/16384;
    //         float zg = (float)RAWZ/16384;

    //         //ESP_LOGI(TAG, "\nx=%.2f\ty=%.2f\tz=%.2f", xg, yg, zg);
    //         vTaskDelay(pdMS_TO_TICKS(300)); // Delay 2 giây
    // }
        // Loại bỏ nhiễu nhỏ (deadband filter)
    //     if (fabs(ax) < THRESHOLD) ax = 0;
    //     if (fabs(ay) < THRESHOLD) ay = 0;

    //     // Tích phân để tính vận tốc
    //     velocity_x += ax * DT;
    //     velocity_y += ay * DT;

    //     // Giới hạn vận tốc để tránh trôi dạt (drift)
    //     if (fabs(velocity_x) < 0.05) velocity_x = 0;
    //     if (fabs(velocity_y) < 0.05) velocity_y = 0;

    //     //ESP_LOGI(TAG, "Ax=%.2f m/s² Ay=%.2f m/s² | Vx=%.2f m/s Vy=%.2f m/s", ax, ay, velocity_x, velocity_y);

    //     vTaskDelay(pdMS_TO_TICKS(10));  // Đọc dữ liệu mỗi 10ms
    // }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
        g_timer_read_mpu_done = false;
        my_timer_start_read_mpu();
        int msg;
        uint32_t last_time = esp_timer_get_time();  // Thời gian trước đó
        float roll = 0, pitch = 0, yaw = 0;  // Góc Euler

         // === Hiệu chỉnh Gyro ===
        float GYRO_OFFSET_X = 0, GYRO_OFFSET_Y = 0, GYRO_OFFSET_Z = 0;
        int samples = 100;  // Số lần lấy mẫu để tính trung bình

        for (int i = 0; i < samples; i++) {
            uint8_t data[6];
            mpuReadfromReg(0x43, data, 6);
            GYRO_OFFSET_X += (int16_t)((data[0] << 8) | data[1]);
            GYRO_OFFSET_Y += (int16_t)((data[2] << 8) | data[3]);
            GYRO_OFFSET_Z += (int16_t)((data[4] << 8) | data[5]);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay để đọc ổn định
        }

        GYRO_OFFSET_X /= samples;
        GYRO_OFFSET_Y /= samples;
        GYRO_OFFSET_Z /= samples;
        //ESP_LOGI(TAG, "Gyro Offset: X=%.2f, Y=%.2f, Z=%.2f", GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z);

        while(!g_timer_read_mpu_done){
                uint8_t data[14];
                mpuReadfromReg(0x3B, data, 14);

                // Đọc dữ liệu từ cảm biến
                int16_t RAW_AX = (data[0] << 8) | data[1];
                int16_t RAW_AY = (data[2] << 8) | data[3];
                int16_t RAW_AZ = (data[4] << 8) | data[5];
                int16_t RAW_GX = (data[8] << 8) | data[9];
                int16_t RAW_GY = (data[10] << 8) | data[11];
                int16_t RAW_GZ = (data[12] << 8) | data[13];

                // Chuyển đổi đơn vị
                float AX = RAW_AX / 16384.0;
                float AY = RAW_AY / 16384.0;
                float AZ = RAW_AZ / 16384.0;

                float GX = (RAW_GX - GYRO_OFFSET_X) / 131.0;
                float GY = (RAW_GY - GYRO_OFFSET_Y) / 131.0;
                float GZ = (RAW_GZ - GYRO_OFFSET_Z) / 131.0;

                // Tính thời gian delta (dt) - cập nhật nhanh hơn
                static uint64_t last_time = 0;
                uint64_t current_time = esp_timer_get_time(); // µs
                float dt = (current_time - last_time) / 1000000.0; // Chuyển µs -> s
                last_time = current_time;

                // Giới hạn dt để tránh sai số lớn
                if (dt > 0.1) dt = 0.01;

                // Roll, Pitch từ gia tốc kế
                float accel_roll = atan2(AY, AZ) * 180.0 / M_PI;
                float accel_pitch = atan2(-AX, sqrt(AY * AY + AZ * AZ)) * 180.0 / M_PI;

                // Roll, Pitch từ Gyro
                float gyro_roll = roll + GX * dt;
                float gyro_pitch = pitch + GY * dt;
                float gyro_yaw = yaw + GZ * dt;

                // Bộ lọc Complementary: tăng Accel lên 5% để giảm drift
                roll = 0.95 * gyro_roll + 0.05 * accel_roll;
                pitch = 0.95 * gyro_pitch + 0.05 * accel_pitch;
                yaw = 0.98 * yaw + 0.02 * gyro_yaw; // Giảm tích lũy drift

                //ESP_LOGI(TAG, "Roll: %.2f°  Pitch: %.2f°  Yaw: %.2f°", roll, pitch, yaw);

                // Gán kết quả ra con trỏ
                *roll_pointer = roll;  
                *pitch_pointer = pitch;      
                *yaw_pointer = yaw;    

            vTaskDelay(pdMS_TO_TICKS(10)); // Chạy nhanh hơn (10ms)

        }
        my_timer_delete_read_mpu();
        g_timer_read_mpu_done = false;
}

void read_mpu6050_angles_alarm(float *stored_roll, float *stored_pitch, float *stored_yaw) {

        g_timer_read_mpu_done = false;
        int msg;
        uint32_t last_time = esp_timer_get_time();  // Thời gian trước đó
        float roll = 0, pitch = 0, yaw = 0;  // Góc Euler

         // === Hiệu chỉnh Gyro ===
        float GYRO_OFFSET_X = 0, GYRO_OFFSET_Y = 0, GYRO_OFFSET_Z = 0;
        int samples = 100;  // Số lần lấy mẫu để tính trung bình

        for (int i = 0; i < samples; i++) {
            uint8_t data[6];
            mpuReadfromReg(0x43, data, 6);
            GYRO_OFFSET_X += (int16_t)((data[0] << 8) | data[1]);
            GYRO_OFFSET_Y += (int16_t)((data[2] << 8) | data[3]);
            GYRO_OFFSET_Z += (int16_t)((data[4] << 8) | data[5]);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay để đọc ổn định
        }

        GYRO_OFFSET_X /= samples;
        GYRO_OFFSET_Y /= samples;
        GYRO_OFFSET_Z /= samples;
        //ESP_LOGI(TAG, "Gyro Offset: X=%.2f, Y=%.2f, Z=%.2f", GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z);


        my_timer_start_read_mpu();
        my_timer_noiseMPU_start();

        while(!g_timer_read_mpu_done){
                uint8_t data[14];
                mpuReadfromReg(0x3B, data, 14);

                // Đọc dữ liệu từ cảm biến
                int16_t RAW_AX = (data[0] << 8) | data[1];
                int16_t RAW_AY = (data[2] << 8) | data[3];
                int16_t RAW_AZ = (data[4] << 8) | data[5];
                int16_t RAW_GX = (data[8] << 8) | data[9];
                int16_t RAW_GY = (data[10] << 8) | data[11];
                int16_t RAW_GZ = (data[12] << 8) | data[13];

                // Chuyển đổi đơn vị
                float AX = RAW_AX / 16384.0;
                float AY = RAW_AY / 16384.0;
                float AZ = RAW_AZ / 16384.0;

                float GX = (RAW_GX - GYRO_OFFSET_X) / 131.0;
                float GY = (RAW_GY - GYRO_OFFSET_Y) / 131.0;
                float GZ = (RAW_GZ - GYRO_OFFSET_Z) / 131.0;

                // Tính thời gian delta (dt) - cập nhật nhanh hơn
                static uint64_t last_time = 0;
                uint64_t current_time = esp_timer_get_time(); // µs
                float dt = (current_time - last_time) / 1000000.0; // Chuyển µs -> s
                last_time = current_time;

                // Giới hạn dt để tránh sai số lớn
                if (dt > 0.1) dt = 0.01;

                // Roll, Pitch từ gia tốc kế
                float accel_roll = atan2(AY, AZ) * 180.0 / M_PI;
                float accel_pitch = atan2(-AX, sqrt(AY * AY + AZ * AZ)) * 180.0 / M_PI;

                // Roll, Pitch từ Gyro
                float gyro_roll = roll + GX * dt;
                float gyro_pitch = pitch + GY * dt;
                float gyro_yaw = yaw + GZ * dt;

                // Bộ lọc Complementary: tăng Accel lên 5% để giảm drift
                roll = 0.95 * gyro_roll + 0.05 * accel_roll;
                pitch = 0.95 * gyro_pitch + 0.05 * accel_pitch;
                yaw = 0.98 * yaw + 0.02 * gyro_yaw; // Giảm tích lũy drift

                //ESP_LOGI(TAG, "Roll: %.2f°  Pitch: %.2f°  Yaw: %.2f°", roll, pitch, yaw);

                if(g_timer_noiseMPU_done){
                    //So sánh với góc lưu trong NVS
                    float delta_roll = fabs(roll - *stored_roll);
                    float delta_pitch = fabs(pitch - *stored_pitch);
                    float delta_yaw = fabs(yaw - *stored_yaw);

                    if (delta_roll > 10.0 || delta_pitch > 10.0 || delta_yaw > 10.0) {    
                        // ESP_LOGW(TAG, "Significant angle change detected!");
                        // ESP_LOGW(TAG, "Previous: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", *stored_roll, *stored_pitch, *stored_yaw);
                        // ESP_LOGW(TAG, "Current:  Roll=%.2f, Pitch=%.2f, Yaw=%.2f", roll, pitch, yaw);
                        my_timer_noiseMPU_delete();
                        g_timer_noiseMPU_done = false;
                        alarm_mpu = true;
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10)); // Chạy nhanh hơn (10ms)
        }
        my_timer_delete_read_mpu();
        g_timer_read_mpu_done = false;
}


