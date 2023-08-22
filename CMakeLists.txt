cmake_Minimum_required(VERSION 3.19)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)
# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(blink C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Pico SDK
pico_sdk_init()

# FILE(GLOB FreeRTOS_src FREERTOS-Kernel/*.c)

add_executable(blink src/main.c)

add_library( FreeRTOS STATIC
        ${FreeRTOS_src}
        #FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/port.c
        FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
        FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

target_include_directories( FreeRTOS PUBLIC
        FreeRTOS-Kernel/include
        FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040
        FreeRTOS-Kernel/portable/GCC/ARM_CM0
)

target_link_libraries( blink
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        pico_stdio
        pico_stdlib
        hardware_gpio
        hardware_spi
        hardware_irq
        hardware_adc
)

# enable usb output, disable uart output
pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)

pico_add_extra_outputs(blink)
