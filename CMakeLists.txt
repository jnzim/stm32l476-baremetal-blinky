cmake_minimum_required(VERSION 3.21)

project(blinky LANGUAGES C ASM)

# Firmware target
add_executable(fw.elf
    src/main.c
    startup/startup.s
)

# ===== Compile flags =====
target_compile_options(fw.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:
        -mcpu=cortex-m4
        -mthumb
        -ffreestanding
        -fno-builtin
        -fdata-sections
        -ffunction-sections
        -O0
        -g3
        -Wall
        -Wextra
    >
    $<$<COMPILE_LANGUAGE:ASM>:
        -mcpu=cortex-m4
        -mthumb
        -x assembler-with-cpp
    >
)

# ===== Linker flags =====
target_link_options(fw.elf PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -T ${CMAKE_SOURCE_DIR}/linker/memory.ld
    -nostartfiles
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/fw.map
)

# ===== Output formats =====
add_custom_command(TARGET fw.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   fw.elf fw.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary fw.elf fw.bin
    COMMAND ${CMAKE_SIZE} fw.elf
)