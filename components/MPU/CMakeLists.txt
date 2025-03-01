idf_component_register(SRCS "i2ctools_example_main.c"
                            "cmd_i2ctools.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_timer console esp_system driver fatfs vfs nvs_flash SIM main_library

                       )
