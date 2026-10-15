cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=gnu11>)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET bess-blinky)

include(${MBED_PATH}/tools/cmake/app.cmake)
project(${APP_TARGET})
add_subdirectory(${MBED_PATH})
add_executable(${APP_TARGET}
    main.cpp
    bme_bsec.cpp
    third_party/bme68x/bme68x.c
)
target_include_directories(${APP_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bme68x
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bsec2/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/ml          # anomaly_model.h, generated from ml/model.json
)
# Bosch BSEC2, soft-float Cortex-M4 build (see third_party/README.md).
target_link_libraries(${APP_TARGET} mbed-os ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bsec2/cortex-m4/libalgobsec.a)
mbed_set_post_build(${APP_TARGET})