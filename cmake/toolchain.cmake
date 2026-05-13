# STM32 bare-metal toolchain (Arm GNU Toolchain installed in /Applications)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Compilers
set(CMAKE_C_COMPILER
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc)

# Binutils
set(CMAKE_OBJCOPY
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-objcopy)
set(CMAKE_SIZE
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-size)

# --- CRITICAL: prevent macOS host flags from being injected ---
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "" FORCE)
set(CMAKE_OSX_DEPLOYMENT_TARGET "" CACHE STRING "" FORCE)
set(CMAKE_OSX_SYSROOT "" CACHE PATH "" FORCE)

# Also clear any cached initial flags that might contain -arch/-isysroot
set(CMAKE_C_FLAGS_INIT "" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_INIT "" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_INIT "" CACHE STRING "" FORCE)


