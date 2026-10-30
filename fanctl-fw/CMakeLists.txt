# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD seeed_xiao_rp2040)
include(pico_sdk_import.cmake)

project(fanctl C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

# Driver stage: DRV8871 (default) or LOWSIDE (MOSFET trigger module)
set(FANCTL_DRIVER DRV8871 CACHE STRING "DRV8871 (default) or LOWSIDE")
set_property(CACHE FANCTL_DRIVER PROPERTY STRINGS LOWSIDE DRV8871)

add_executable(fanctl main.c)
target_compile_definitions(fanctl PRIVATE FANCTL_DRIVER=DRV_${FANCTL_DRIVER})
target_link_libraries(fanctl
    pico_stdlib
    hardware_pwm
    hardware_clocks
    hardware_watchdog)

pico_enable_stdio_usb(fanctl 1)
pico_enable_stdio_uart(fanctl 0)

pico_set_program_name(fanctl "fanctl")
pico_set_program_version(fanctl "2.0")
pico_add_extra_outputs(fanctl)   # fanctl.uf2
