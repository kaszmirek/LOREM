cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(lorem CXX C ASM)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(lib/VL53L0X/lib)
add_subdirectory(lib/picowota)

add_executable(lorem
    src/main.cpp
    src/tof.cpp
)

target_include_directories(lorem PRIVATE include)

target_link_libraries(lorem
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
    VL53L0X_Lib
    picowota_reboot
)

picowota_build_standalone(lorem)
picowota_build_combined(lorem)

pico_set_program_name(lorem "lorem")
pico_set_program_version(lorem "0.1.0")
pico_add_extra_outputs(lorem)
pico_enable_stdio_usb(lorem 1)
pico_enable_stdio_uart(lorem 0)

find_program(PICOTOOL picotool REQUIRED)
add_custom_target(flash
    COMMAND ${PICOTOOL} load -f -x $<TARGET_FILE_DIR:lorem>/lorem.uf2
    DEPENDS lorem
    COMMENT "Flashing lorem.uf2 via picotool"
)

add_custom_target(verify
    COMMAND ${PICOTOOL} verify $<TARGET_FILE_DIR:lorem>/lorem.uf2
    DEPENDS lorem
    COMMENT "Verifying flash contents"
)

add_custom_target(info
    COMMAND ${PICOTOOL} info
    COMMENT "Reading device info"
)
