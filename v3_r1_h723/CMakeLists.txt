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
set(CMAKE_PROJECT_NAME R1_robot_1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User_file/2_Device/Buzzer/Inc/dvc_buzzer.h
        User_file/3_Chariot/2_Module/Gantry/Inc/crt_gantry.h
        User_file/3_Chariot/2_Module/Gantry/Src/crt_gantry.cpp
        User_file/3_Chariot/2_Module/Navigation/Src/crt_navigation.cpp
        User_file/3_Chariot/2_Module/Navigation/Inc/crt_navigation.h
        User_file/2_Device/COMM/Inc/dvc_boardcomm.h
        User_file/2_Device/COMM/Src/dvc_boardcomm.cpp)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE USER_SOURCES
        "User_file/*.c"
        "User_file/*.cpp"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User_file/1_Middleware/1_Driver/Inc
        User_file/1_Middleware/2_Algorithm/Inc

        User_file/2_Device/BMI088/Inc
        User_file/2_Device/Buzzer/Inc
        User_file/2_Device/Motor/Inc
        User_file/2_Device/Remote/Inc
        User_file/2_Device/OPS/Inc
        User_file/2_Device/USB/Inc
        User_file/2_Device/COMM/Inc

        User_file/3_Chariot/1_Posture/Inc

        User_file/3_Chariot/2_Module/Chassis/Inc
        User_file/3_Chariot/2_Module/Gantry/Inc
        User_file/3_Chariot/2_Module/Navigation/Inc

        User_file/4_Interaction/Inc

        User_file/5_Task/Inc
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
        ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfdp_math.a
)
