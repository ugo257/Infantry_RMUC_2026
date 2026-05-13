# User-owned wrapper around the STM32CubeMX generated toolchain file.
# Keep editor-specific compiler discovery here so CubeMX can freely
# regenerate gcc-arm-none-eabi.cmake without removing our fixes.

include("${CMAKE_CURRENT_LIST_DIR}/gcc-arm-none-eabi.cmake")

set(STM32_GNU_TOOL_HINTS "")
set(STM32_NINJA_HINTS "")

if(APPLE)
    file(GLOB STM32_GNU_TOOL_HINTS "/opt/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin")
    file(GLOB STM32_NINJA_HINTS "/opt/ST/STM32CubeCLT_*/Ninja/bin")
elseif(WIN32)
    file(GLOB STM32_GNU_TOOL_HINTS
        "C:/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
        "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
    )
    file(GLOB STM32_NINJA_HINTS
        "C:/ST/STM32CubeCLT_*/Ninja/bin"
        "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeCLT_*/Ninja/bin"
    )
elseif(UNIX)
    file(GLOB STM32_GNU_TOOL_HINTS
        "/opt/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
        "/opt/st/stm32cubeclt_*/GNU-tools-for-STM32/bin"
    )
    file(GLOB STM32_NINJA_HINTS
        "/opt/ST/STM32CubeCLT_*/Ninja/bin"
        "/opt/st/stm32cubeclt_*/Ninja/bin"
    )
endif()

if(STM32_GNU_TOOL_HINTS)
    list(SORT STM32_GNU_TOOL_HINTS COMPARE NATURAL ORDER DESCENDING)
endif()

if(STM32_NINJA_HINTS)
    list(SORT STM32_NINJA_HINTS COMPARE NATURAL ORDER DESCENDING)
endif()

find_program(ARM_NONE_EABI_GCC NAMES arm-none-eabi-gcc HINTS ${STM32_GNU_TOOL_HINTS})
if(NOT ARM_NONE_EABI_GCC)
    message(FATAL_ERROR
        "arm-none-eabi-gcc was not found. Install STM32CubeCLT or add its GNU tools directory to PATH."
    )
endif()

get_filename_component(ARM_NONE_EABI_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)

find_program(ARM_NONE_EABI_GXX NAMES arm-none-eabi-g++ HINTS "${ARM_NONE_EABI_BIN_DIR}")
find_program(ARM_NONE_EABI_OBJCOPY NAMES arm-none-eabi-objcopy HINTS "${ARM_NONE_EABI_BIN_DIR}")
find_program(ARM_NONE_EABI_SIZE NAMES arm-none-eabi-size HINTS "${ARM_NONE_EABI_BIN_DIR}")

foreach(required_tool
    ARM_NONE_EABI_GXX
    ARM_NONE_EABI_OBJCOPY
    ARM_NONE_EABI_SIZE
)
    if(NOT ${required_tool})
        message(FATAL_ERROR
            "${required_tool} was not found next to ${ARM_NONE_EABI_GCC}."
        )
    endif()
endforeach()

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "Arm GNU C compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "Arm GNU ASM compiler" FORCE)
set(CMAKE_CXX_COMPILER "${ARM_NONE_EABI_GXX}" CACHE FILEPATH "Arm GNU C++ compiler" FORCE)
set(CMAKE_LINKER "${ARM_NONE_EABI_GXX}" CACHE FILEPATH "Arm GNU linker" FORCE)
set(CMAKE_OBJCOPY "${ARM_NONE_EABI_OBJCOPY}" CACHE FILEPATH "Arm GNU objcopy" FORCE)
set(CMAKE_SIZE "${ARM_NONE_EABI_SIZE}" CACHE FILEPATH "Arm GNU size" FORCE)

find_program(STM32_NINJA NAMES ninja HINTS ${STM32_NINJA_HINTS})
if(STM32_NINJA AND CMAKE_GENERATOR MATCHES "Ninja")
    set(CMAKE_MAKE_PROGRAM "${STM32_NINJA}" CACHE FILEPATH "Ninja executable" FORCE)
endif()
