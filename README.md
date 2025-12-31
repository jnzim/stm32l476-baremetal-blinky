STM32F042K6 Bare-Metal Blinky (No HAL, No CubeMX)
===============================================

This project is a **from-scratch bare-metal firmware** for the
**NUCLEO-F042K6** (STM32F042K6, ARM Cortex-M0).

The goal is to demonstrate **full MCU bring-up without vendor
frameworks**, showing exactly what is required to boot, initialize,
and run code on an STM32 microcontroller.

No STM32 HAL.  
No CubeMX.  
No IDE-generated startup code.

Everything required to boot the MCU and blink the onboard LED
is implemented explicitly.

------------------------------------------------------------
HOST & TOOLING
------------------------------------------------------------

Host OS: macOS  
Toolchain: Arm GNU Toolchain (`arm-none-eabi`)  
Build system: CMake (Unix Makefiles)  
Flash / Debug: ST-LINK (`st-flash`, `st-util`)  
Editor: VS Code  

------------------------------------------------------------
WHAT THIS PROJECT DEMONSTRATES
------------------------------------------------------------

- Cross-compiling ARM Cortex-M firmware on macOS
- Proper separation of **host vs target** build concerns
- Explicit linker script defining FLASH and SRAM layout
- Custom startup code:
  - Vector table definition
  - Stack pointer initialization
  - `.data` relocation (Flash → RAM)
  - `.bss` zero-initialization
- Safe exception handling with breakpoint traps
- Register-level GPIO control (no HAL / CMSIS)
- Professional CMake + cross-toolchain configuration
- Flashing via ST-LINK using open-source tools
- Debugging with GDB + Cortex-Debug
- Binary inspection using `nm`, `objdump`, and `size`

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
│ └── memory.ld
├── startup/
│ └── startup.s
├── src/
│ ├── main.c
│ └── system.c
└── .vscode/
├── tasks.json
├── launch.json
└── settings.


------------------------------------------------------------
TOOLCHAIN SETUP
------------------------------------------------------------

Compiler:   arm-none-eabi-gcc  
Assembler: arm-none-eabi-gcc  
Linker:    arm-none-eabi-ld  

A CMake toolchain file is used to:

- Force use of the ARM cross-compiler
- Mark the build as bare-metal (`CMAKE_SYSTEM_NAME=Generic`)
- Prevent macOS-specific flags from leaking into the build
- Disable host executable test runs during configuration

------------------------------------------------------------
BUILD INSTRUCTIONS
------------------------------------------------------------

From the project root:

```sh
rm -rf build
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake
cmake --build build
