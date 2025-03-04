idf_component_register(SRCS "main.cpp" "tasks/src/uart.cpp" "tasks/src/pid.cpp"
                    PRIV_REQUIRES spi_flash nvs_flash
                    REQUIRES driver)