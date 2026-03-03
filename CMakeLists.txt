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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# tinyusb
set(PICO_TINYUSB_PATH ${CMAKE_SOURCE_DIR}/lib/tinyusb)

# CMSIS
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/lib/CMSIS_5/CMSIS/Core)
add_compile_definitions(__ARM_FEATURE_DSP=1)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_usb_i2s_speaker C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_usb_i2s_speaker
        main.c
        usb_descriptors.c
        dsp.c
        )

pico_set_program_name(pico_usb_i2s_speaker "pico_usb_i2s_speaker")
pico_set_program_version(pico_usb_i2s_speaker "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_usb_i2s_speaker 1)
pico_enable_stdio_usb(pico_usb_i2s_speaker 0)

# Add the standard library to the build
target_link_libraries(pico_usb_i2s_speaker
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_usb_i2s_speaker PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

set(I2S_MAX_FREQ_KHZ 96)
set(I2S_QUEUE_LEN 10)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/pico-i2s-pio)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/CMSIS-DSP)

# Add any user requested libraries
target_link_libraries(pico_usb_i2s_speaker
        hardware_clocks
        hardware_dma
        pico_multicore
        tinyusb_device
        tinyusb_board
        pico-i2s-pio
        CMSISDSP
        )

pico_add_extra_outputs(pico_usb_i2s_speaker)

