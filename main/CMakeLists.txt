idf_component_register(SRCS "light.cpp" "light_controller.cpp" "main.cpp"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ""
                    REQUIRES "nvs_flash"
                    REQUIRES "esp_driver_gpio")
