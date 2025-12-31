cmake_minimum_required(VERSION 3.21)

project(blinky LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(MCU cortex-m0)
set(LDSCRIPT ${CMAKE_SOURCE_DIR}/linker/memory.ld)

add_executable(fw.elf
    src/main.c
    src/system.c
    startup/startup.s
)

# 👇 THIS IS THE FIX (your header lives in src/)
target_include_directories(fw.elf PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_definitions(fw.elf PRIVATE
    STM32F042x6
)

target_compile_options(fw.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:
        -mcpu=${MCU}
        -mthumb
        -ffreestanding
        -fno-builtin
        -fdata-sections
        -ffunction-sections
        -Og
        -g3
        -Wall
        -Wextra
    >
    $<$<COMPILE_LANGUAGE:ASM>:
        -mcpu=${MCU}
        -mthumb
        -x assembler-with-cpp
    >
)

target_link_options(fw.elf PRIVATE
    -mcpu=${MCU}
    -mthumb
    -T${LDSCRIPT}
    -nostartfiles
    -nostdlib
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/fw.map
    -Wl,--start-group
        -lc
        -lgcc
    -Wl,--end-group
)

# Force relink when linker script changes
set_property(TARGET fw.elf APPEND PROPERTY LINK_DEPENDS ${LDSCRIPT})

add_custom_command(TARGET fw.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:fw.elf> ${CMAKE_BINARY_DIR}/fw.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:fw.elf> ${CMAKE_BINARY_DIR}/fw.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:fw.elf>
)
