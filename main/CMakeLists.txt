include(../src/sources.cmake)
idf_component_register(
    SRCS board.cpp main.cpp ${ARDUGO_SOURCES}
    INCLUDE_DIRS ${ARDUGO_INCLUDES}
    PRIV_REQUIRES app_update driver esp_adc esp_driver_gpio
        esp_driver_ledc esp_driver_sdspi esp_driver_spi esp_psram esp_timer fatfs sdmmc
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${ARDUGO_DEFINES})
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -Wno-missing-field-initializers)

# Opcode handlers only write into the AVR state, which is allocated in internal RAM.
# Handlers that write to PSRAM live in other translation units.
# Hot JIT helpers and loop accelerators are tagged with ARDENS_IRAM; cold
# merged/fallback handlers stay in flash to free IRAM for the JIT code cache.
set_source_files_properties(${ARDENS}/src/absim_execute.cpp PROPERTIES
    COMPILE_OPTIONS "-mno-fix-esp32-psram-cache-issue;-fjump-tables")
if(ARDUGO_QUIET_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ARDUGO_QUIET_TEST)
    option(ARDUGO_RUN_BENCHMARKS "Run silent hardware benchmarks on startup" ON)
    if(ARDUGO_RUN_BENCHMARKS)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ARDUGO_RUN_BENCHMARKS)
    endif()
endif()
