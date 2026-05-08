// system_stm32f4xx.c
#include "stm32f4xx.h"

uint32_t SystemCoreClock = 16000000U;

void SystemInit(void) {}

void __libc_init_array(void) {}  // stub — no C++ constructors in this project