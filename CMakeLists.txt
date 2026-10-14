cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/led.c
    Core/Src/oled.c
    Core/Src/track.c
    Core/Src/mpu6050.c
    Core/Src/dmp_port.c
    Core/Src/inv_mpu.c
    Core/Src/inv_mpu_dmp_motion_driver.c
    Core/Src/mpu6050_dmp.c
    Core/Src/motor.c
    Core/Src/line_follow.c
    Core/Src/h_course.c
    Core/badapple/badapple.c
    "Core/experimental code/task11_led_blink.c"
    "Core/experimental code/task13_track_sensor.c"
    "Core/experimental code/task21_uart_echo.c"
    "Core/experimental code/task22_encoder_printf.c"
    "Core/experimental code/task23_vofa_dma.c"
    "Core/experimental code/task31_tim2_pwm.c"
    "Core/experimental code/task31_pwm_encoder.c"
    "Core/experimental code/task32_motor_encoder.c"
    "Core/experimental code/task33_encoder_periodic.c"
    "Core/experimental code/task34_pid_vofa.c"
    "Core/experimental code/task41_iic_mpu6050.c"
    "Core/experimental code/task42_spi_oled.c"
    "Core/experimental code/task43_dmp_oled.c"
    "Core/experimental code/task51_pid_position.c"
    "Core/experimental code/task52_pid_incremental.c"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    Core/badapple
    "Core/experimental code"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)

# newlib-nano 默认不链接 printf 浮点支持，%f / %.1f 会打印为空。
# 引入 _printf_float 符号，让 printf 支持浮点格式化（如 VOFA+ FireWater 帧）。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-u,_printf_float)
