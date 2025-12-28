# STM32 bare-metal toolchain (Arm GNU Toolchain installed in /Applications)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Compilers (force the Arm toolchain, not Homebrew's arm-none-eabi-gcc)
set(CMAKE_C_COMPILER
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc)

# Binutils
set(CMAKE_OBJCOPY
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-objcopy)
set(CMAKE_SIZE
    /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-size)

# Prevent macOS from injecting host flags like -arch and -mmacosx-version-min
set(CMAKE_OSX_ARCHITECTURES "")
set(CMAKE_OSX_DEPLOYMENT_TARGET "")
