STM32F042K6 Bare-Metal Blinky (No HAL, No CubeMX)
===============================================

This project is a from-scratch bare-metal firmware for the
**NUCLEO-F042K6** (STM32F042K6, ARM Cortex-M0).

Host OS: macOS  
Toolchain: Arm GNU Toolchain (arm-none-eabi)  
Build system: CMake (Unix Makefiles)  
Debugger / Flash: ST-LINK (st-flash, st-util)  
Editor: VS Code  

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
  - .data copy (Flash → RAM)
  - .bss zeroing
- Safe exception handling with breakpoint traps
- Register-level GPIO control (no HAL / CMSIS)
- Professional CMake + toolchain setup
- Flashing via ST-LINK using open-source tools
- Debugging with GDB + Cortex-Debug
- Binary inspection using nm, objdump, and size

------------------------------------------------------------
TARGET HARDWARE
------------------------------------------------------------

Board: NUCLEO-F042K6  
MCU:   STM32F042K6 (ARM Cortex-M0)  
Flash: 32 KB  
SRAM:  6 KB  

User LED:
- LD3
- GPIOB pin PB3 (Arduino D13)

------------------------------------------------------------
PROJECT LAYOUT
------------------------------------------------------------

blinky/
├── README.md
├── CMakeLists.txt
├── toolchain.cmake
├── linker/
│   └── memory.ld
├── startup/
│   └── startup.s
├── src/
│   ├── main.c
│   └── system.c
└── .vscode/
    ├── tasks.json
    ├── launch.json
    └── settings.json

------------------------------------------------------------
TOOLCHAIN
------------------------------------------------------------

Compiler: arm-none-eabi-gcc  
Assembler: arm-none-eabi-gcc  
Linker: arm-none-eabi-ld  

A CMake toolchain file is used to ensure this is treated as a
bare-metal ARM target and to prevent macOS-specific flags
(e.g. -arch arm64) from leaking into the build.

------------------------------------------------------------
BUILD INSTRUCTIONS
------------------------------------------------------------

From the project root:

```sh
rm -rf build
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake
cmake --build build
