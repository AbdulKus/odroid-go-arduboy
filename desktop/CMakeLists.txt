include(../src/sources.cmake)
add_library(ardugo-core STATIC ${ARDUGO_SOURCES})
target_include_directories(ardugo-core PUBLIC ${ARDUGO_INCLUDES})
target_compile_features(ardugo-core PUBLIC cxx_std_17)
target_compile_definitions(ardugo-core PRIVATE ${ARDUGO_DEFINES})
target_compile_options(ardugo-core PRIVATE -O3)

find_package(SDL2 REQUIRED)
add_executable(arduboy-go-desktop main.cpp)
target_link_libraries(arduboy-go-desktop PRIVATE ardugo-core SDL2::SDL2)

find_program(AVR_GCC avr-gcc REQUIRED)
find_program(AVR_OBJCOPY avr-objcopy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(DEMO_HEX ${CMAKE_CURRENT_BINARY_DIR}/demo.hex)
add_custom_command(
    OUTPUT ${DEMO_HEX}
    COMMAND ${AVR_GCC} -Os -mmcu=atmega32u4 ${ROOT}/tests/demo.c -o demo.elf
    COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom demo.elf ${DEMO_HEX}
    DEPENDS ${ROOT}/tests/demo.c
    VERBATIM
)
add_custom_target(ardugo-demo DEPENDS ${DEMO_HEX})
add_executable(ardugo-check ../tests/check.cpp)
target_link_libraries(ardugo-check PRIVATE ardugo-core)
add_dependencies(ardugo-check ardugo-demo)
add_test(NAME ardugo-check
    COMMAND ${Python3_EXECUTABLE} ${ROOT}/tests/run.py $<TARGET_FILE:ardugo-check> ${DEMO_HEX})
