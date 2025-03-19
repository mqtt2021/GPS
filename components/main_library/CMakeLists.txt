idf_component_register(
    INCLUDE_DIRS .        # Thư mục hiện tại chứa file header
    REQUIRES nvs_flash esp_wifi espressif__led_strip gps_library my_timer adc_battery MPU bluetooth         # Liệt kê các component mà thư viện này phụ thuộc 
)

