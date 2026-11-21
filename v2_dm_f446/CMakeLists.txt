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
set(CMAKE_PROJECT_NAME 111)

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
        User_file/1_Middleware/1_Driver/Src/drv_tim.cpp
        User_file/1_Middleware/1_Driver/Src/drv_can.cpp
        User_file/1_Middleware/1_Driver/Src/drv_math.cpp
        User_file/1_Middleware/1_Driver/Src/drv_uart.cpp

        User_file/2_Devier/Src/dvc_encoder.cpp
        User_file/2_Devier/Src/dvc_motor_dm.cpp
        User_file/2_Devier/Src/dvc_odrive.cpp
        User_file/2_Devier/Src/dvc_dr16.cpp

        User_file/1_Middleware/2_Algorithm/Src/alg_pid.cpp

        User_file/3_Chariot/1_Posture/Src/
        User_file/3_Chariot/2_Module/Src/crt_chassis.cpp

        User_file/4_Interaction/Src/robot.cpp

        User_file/5_Task/Src/task.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

        User_file/1_Middleware/1_Driver/Inc

        User_file/1_Middleware/2_Algorithm/Inc

        User_file/2_Devier/Inc

        User_file/3_Chariot/1_Posture/Inc
        User_file/3_Chariot/2_Module/Inc

        User_file/4_Interaction/Inc

        User_file/5_Task/Inc

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 为 C++ 文件设置编译选项
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
