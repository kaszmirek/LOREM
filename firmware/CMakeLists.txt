cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(lorem CXX C ASM)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(lib/VL53L0X/lib)

add_executable(lorem
    src/main.cpp
    src/encoder.cpp
    src/motor.cpp
    src/oled.cpp
    src/robot.cpp
    src/tof.cpp
)

target_include_directories(lorem PRIVATE include)

target_link_libraries(lorem
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
    VL53L0X_Lib
)

pico_set_program_name(lorem "lorem")
pico_set_program_version(lorem "0.1.0")
pico_add_extra_outputs(lorem)
pico_enable_stdio_usb(lorem 1)
pico_enable_stdio_uart(lorem 0)

add_executable(motor_test
    src/motor_test.cpp
    src/encoder.cpp
    src/motor.cpp
    src/oled.cpp
)

target_include_directories(motor_test PRIVATE include)

target_link_libraries(motor_test
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
)

pico_set_program_name(motor_test "motor_test")
pico_add_extra_outputs(motor_test)
pico_enable_stdio_usb(motor_test 1)
pico_enable_stdio_uart(motor_test 0)

add_executable(engine_ramp
    src/engine_ramp.cpp
    src/encoder.cpp
    src/motor.cpp
    src/oled.cpp
)

target_include_directories(engine_ramp PRIVATE include)

target_link_libraries(engine_ramp
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
)

pico_set_program_name(engine_ramp "engine_ramp")
pico_add_extra_outputs(engine_ramp)
pico_enable_stdio_usb(engine_ramp 1)
pico_enable_stdio_uart(engine_ramp 0)

add_custom_target(engine_ramp_flash
    COMMAND ${PICOTOOL} load -f -x $<TARGET_FILE_DIR:engine_ramp>/engine_ramp.uf2
    DEPENDS engine_ramp
    COMMENT "Flashing engine_ramp.uf2 via picotool"
)

find_program(PICOTOOL picotool REQUIRED)
add_custom_target(flash
    COMMAND ${PICOTOOL} load -f -x $<TARGET_FILE_DIR:lorem>/lorem.uf2
    DEPENDS lorem
    COMMENT "Flashing lorem.uf2 via picotool"
)

add_custom_target(motor_test_flash
    COMMAND ${PICOTOOL} load -f -x $<TARGET_FILE_DIR:motor_test>/motor_test.uf2
    DEPENDS motor_test
    COMMENT "Flashing motor_test.uf2 via picotool"
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
