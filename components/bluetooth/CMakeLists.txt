idf_component_register(
    SRCS "blue.c"             # File mã nguồn chính của ứng dụng
    INCLUDE_DIRS .           # Thư mục hiện tại chứa file header
    REQUIRES driver esp_timer bt SIM nvs_flash main_library      # Liệt kê các component mà thư viện này phụ thuộc
    #PRIV_REQUIRES adc_battery
)
