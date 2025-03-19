# File: components/sim_a7680c/CMakeLists.txt

# idf_component_register(
#         SRCS "sim_a7680c.c"
#         INCLUDE_DIRS "."  
#         PRIV_REQUIRES driver esp_timer main_library adc_battery) 

idf_component_register(
        SRCS "sim_a7680c.c"
        INCLUDE_DIRS "."  
        REQUIRES driver esp_timer main_library adc_battery) 
