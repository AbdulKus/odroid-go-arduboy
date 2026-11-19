include(../src/sources.cmake)
idf_component_register(
    SRCS board.cpp main.cpp ${ARDUGO_SOURCES}
    INCLUDE_DIRS ${ARDUGO_INCLUDES}
    PRIV_REQUIRES app_update esp_adc esp_driver_dac esp_driver_gpio
        esp_driver_ledc esp_driver_sdspi esp_driver_spi esp_psram esp_timer fatfs sdmmc
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${ARDUGO_DEFINES})
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -Wno-missing-field-initializers)
