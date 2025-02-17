# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pami2025 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include(${PROJECT_SOURCE_DIR}/libs/vl53l1x/import.cmake)

# Add executable. Default name is the project name, version 0.1

add_executable(pami2025 pami2025.cpp )

pico_set_program_name(pami2025 "pami2025")
pico_set_program_version(pami2025 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pami2025 0)
pico_enable_stdio_usb(pami2025 1)

# Add the standard library to the build
target_link_libraries(pami2025
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pami2025 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(pami2025 
        hardware_i2c
        hardware_timer
        pico_cyw43_arch_lwip_threadsafe_background
        VL53L1X_pico_api
        )


pico_add_extra_outputs(pami2025)

