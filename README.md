cd ~/dev/stm/projects/blinky && cat > README.txt <<'EOF'
STM32L476RG Bare-Metal Blinky (No HAL, No CubeMX)
================================================

This project is a from-scratch bare-metal firmware for the
NUCLEO-L476RG (STM32L476RG, Cortex-M4F).

Host OS: macOS (Apple Silicon)
Toolchain: Arm GNU Toolchain
Build system: CMake + Ninja

No STM32 HAL.
No CubeMX.
No IDE-generated startup code.

Everything required to boot the MCU and blink the onboard LED
is implemented explicitly.

------------------------------------------------------------
WHAT THIS PROJECT DEMONSTRATES
------------------------------------------------------------

- Cross-compiling ARM Cortex-M firmware on macOS
- Proper separation of host vs target build concerns
- Explicit linker script defining FLASH and RAM layout
- Custom startup code:
  - Vector table
  - Stack initialization
  - .data copy (Flash -> RAM)
  - .bss zeroing
- Register-level GPIO control (no HAL)
- Professional CMake + toolchain setup
- Binary inspection using nm, objdump, and size

------------------------------------------------------------
TARGET HARDWARE
------------------------------------------------------------

Board: NUCLEO-L476RG
MCU:   STM32L476RG (ARM Cortex-M4F)
LED:   LD2 (PA5)

------------------------------------------------------------
PROJECT LAYOUT
------------------------------------------------------------

blinky/
  README.txt
  CMakeLists.txt
  toolchain.cmake
  linker/
    memory.ld
  startup/
    startup.s
  src/
    main.c

------------------------------------------------------------
TOOLCHAIN
------------------------------------------------------------

Compiler: arm-none-eabi-gcc
Build system: CMake + Ninja
Host OS: macOS (Apple Silicon)

A CMake toolchain file is used to ensure this is treated as a
bare-metal ARM target and to prevent macOS-specific flags from
leaking into the build.

------------------------------------------------------------
BUILD INSTRUCTIONS
------------------------------------------------------------

From the project root:

  rm -rf build
  cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake
  cmake --build build

Build outputs (in build/):

  fw.elf  - ELF with symbols
  fw.bin  - raw binary image
  fw.hex  - Intel HEX file
  fw.map  - linker map file

------------------------------------------------------------
STARTUP SEQUENCE (HIGH LEVEL)
------------------------------------------------------------

On reset, the firmware:

1. Loads the initial stack pointer from the vector table
2. Copies .data from Flash to RAM
3. Zeros .bss
4. Installs core exception vectors
5. Jumps to main()
6. Traps safely if main() ever returns

Startup logic lives in startup/startup.s.

------------------------------------------------------------
LED BLINK IMPLEMENTATION
------------------------------------------------------------

The application in src/main.c:

- Enables GPIOA clock via RCC
- Configures PA5 as push-pull output
- Toggles PA5 using the GPIO BSRR register
- Uses a simple busy-wait delay loop

This drives LD2 on the NUCLEO-L476RG.

------------------------------------------------------------
VERIFICATION COMMANDS
------------------------------------------------------------

  arm-none-eabi-nm build/fw.elf | grep main
  arm-none-eabi-objdump -h build/fw.elf
  arm-none-eabi-size build/fw.elf

------------------------------------------------------------
NOTES
------------------------------------------------------------

This project was created to refresh bare-metal MCU development
knowledge after ~15 years (previously Atmel Studio), using a
modern ARM Cortex-M platform and toolchain.

------------------------------------------------------------
POSSIBLE NEXT STEPS
------------------------------------------------------------

- Flash and debug via ST-LINK
- Replace busy-wait delay with SysTick
- Add UART output
- OpenOCD + GDB debugging
- Optional FreeRTOS bring-up

EOF
