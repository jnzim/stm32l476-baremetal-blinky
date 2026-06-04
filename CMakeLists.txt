cmake_minimum_required(VERSION 3.21)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake" CACHE PATH "" FORCE)
project(blinky LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(MCU cortex-m4)
set(LDSCRIPT ${CMAKE_SOURCE_DIR}/linker/memory.ld)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(fw.elf
    src/main.c
    src/spi.c
    src/encoder.c
    src/ringBuffer.c
    src/plant.c
    src/control.c
    src/drive.c
    src/loops.c
    src/clock.c
    src/tim1.c
    src/pwm.c
    src/plant.c
    src/system_stm32f4xx.c
    startup/startup_stm32f446xx.s
)

target_include_directories(fw.elf PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(fw.elf PRIVATE
    STM32F446xx
)

# C compile options:
#   -mcpu / -mthumb          Cortex-M4, Thumb-2 instruction set
#   -mfpu / -mfloat-abi      Hardware FPU, single precision, 16 d-regs
#                            Required for cosf/sinf and FOC trig at 20kHz
#   -ffreestanding           Bare metal — no hosted stdlib assumptions
#   -fno-builtin             Don't silently replace calls with compiler builtins
#   -fdata/function-sections Each symbol in own section — enables dead code removal
#   -Og / -g3                Debug-friendly optimization, full debug symbols
#   -Wall / -Wextra          All standard and extra warnings
#
# ASM compile options:
#   -mcpu / -mthumb          Match C flags
#   -x assembler-with-cpp    Allow C preprocessor directives in .s files
target_compile_options(fw.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:
        -mcpu=${MCU}
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
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

# Link options:
#   -mcpu / -mthumb / -mfpu / -mfloat-abi   Must match compile flags
#   -T                                        Linker script — memory map
#   -nostartfiles                             Use startup_stm32f446xx.s
#   -specs=nano.specs                         Newlib-nano C library (hard-float aware)
#   -lnosys                                   Stub out syscalls (no OS)
#   -Wl,--gc-sections                         Remove unused sections
#   -Wl,-Map                                  Generate map file for size analysis
#   -L                                        Explicit path to hard-float libm
#   -lc / -lgcc / -lm                         C runtime, GCC support, math library
target_link_options(fw.elf PRIVATE
    -mcpu=${MCU}
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -T${LDSCRIPT}
    -nostartfiles
    -specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/fw.map
    -L/Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/arm-none-eabi/lib/thumb/v7e-m+fp/hard
    -Wl,--start-group
        -lc
        -lgcc
        -lm
        -lnosys
    -Wl,--end-group
)

target_link_libraries(fw.elf PRIVATE gcc)
target_link_libraries(fw.elf PRIVATE m)
# Force relink when linker script changes
set_property(TARGET fw.elf APPEND PROPERTY LINK_DEPENDS ${LDSCRIPT})

# Post-build: generate hex and binary, print size
add_custom_command(TARGET fw.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:fw.elf> ${CMAKE_BINARY_DIR}/fw.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:fw.elf> ${CMAKE_BINARY_DIR}/fw.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:fw.elf>
)