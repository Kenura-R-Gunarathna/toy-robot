idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES console esp_system esp_psram spi_flash esp_driver_i2s esp_timer
)

# IDF defaults to gnu++2b; pin this component to C++20 (GNU flavour keeps IDF/FreeRTOS macros working)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)
