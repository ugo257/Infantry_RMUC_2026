cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Default to the STM32CubeCLT toolchain when no toolchain is explicitly selected.
# This prevents accidental fallback to host compilers (e.g. Apple clang on macOS).
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_LIST_DIR}/cmake/stm32cubeclt-toolchain.cmake"
        CACHE FILEPATH "Toolchain file for STM32 cross-compilation"
    )
endif()

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Infantry_RMUC_2026)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
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
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/application/*.c"
    "${CMAKE_SOURCE_DIR}/bsp/*.c"
    "${CMAKE_SOURCE_DIR}/modules/*.c"
)

# Exclude test/temp files to reduce migration noise.
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/application/test/.*")
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/tempCodeRunnerFile\\.c$")
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/Chassis1\\.c$")

# Explicitly keep the tick timer module in build sources.
list(APPEND USER_SOURCES
    "${CMAKE_SOURCE_DIR}/modules/tick_timer/tick_timer.c"
)
list(REMOVE_DUPLICATES USER_SOURCES)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
)

# Add include paths
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/application/*.h"
    "${CMAKE_SOURCE_DIR}/bsp/*.h"
    "${CMAKE_SOURCE_DIR}/modules/*.h"
)
set(USER_INCLUDE_DIRS "")
foreach(hdr ${USER_HEADERS})
    get_filename_component(hdr_dir "${hdr}" DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS "${hdr_dir}")
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    ARM_MATH_LOOPUNROLL
)

set_source_files_properties(
    "${CMAKE_SOURCE_DIR}/modules/imu/ins_task.c"
    "${CMAKE_SOURCE_DIR}/modules/algorithm/QuaternionEKF.c"
    "${CMAKE_SOURCE_DIR}/modules/algorithm/kalman_filter.c"
    "${CMAKE_SOURCE_DIR}/application/chassis/omni_velocity_ctrl.c"
    "${CMAKE_SOURCE_DIR}/application/gimbal/gimbal.c"
    "${CMAKE_SOURCE_DIR}/modules/lqr_eso/lqr_eso.c"
    PROPERTIES
        COMPILE_OPTIONS "-O3;-fno-math-errno;-fsingle-precision-constant"
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    "${CMAKE_SOURCE_DIR}/modules/algorithm/AHRS.lib"
)

# AHRS.lib is built with 2-byte wchar_t; suppress the size-mismatch warning only.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--no-wchar-size-warning
)

if(CMAKE_OBJCOPY)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_NAME:${CMAKE_PROJECT_NAME}>.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_NAME:${CMAKE_PROJECT_NAME}>.bin
        COMMENT "Generating HEX and BIN firmware images"
    )
endif()

set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    ADDITIONAL_CLEAN_FILES
        "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_NAME:${CMAKE_PROJECT_NAME}>.hex"
        "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_NAME:${CMAKE_PROJECT_NAME}>.bin"
)
