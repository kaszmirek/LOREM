# Pulled from PICO_SDK_PATH env var or -DPICO_SDK_PATH=...
if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH not set. Pass -DPICO_SDK_PATH=<path> or set env var.")
endif()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to Pico SDK")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
