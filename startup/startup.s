.syntax unified
.cpu cortex-m0
.thumb

/* -------------------------------------------------------------------------- */
/* Symbols from linker script                                                  */
/* -------------------------------------------------------------------------- */
.extern _estack
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss

/* Optional: if you don’t have SystemInit(), you can remove this call */
.weak SystemInit
.thumb_set SystemInit, Default_Handler

/* -------------------------------------------------------------------------- */
/* Vector table                                                                */
/* -------------------------------------------------------------------------- */
.section .isr_vector, "a", %progbits
.global g_pfnVectors
.type g_pfnVectors, %object

g_pfnVectors:
  .word _estack          /* Initial stack pointer */
  .word Reset_Handler    /* Reset Handler */
  .word NMI_Handler      /* NMI Handler */
  .word HardFault_Handler/* HardFault Handler */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word SVC_Handler      /* SVCall Handler */
  .word 0                /* Reserved */
  .word 0                /* Reserved */
  .word PendSV_Handler   /* PendSV Handler */
  .word SysTick_Handler  /* SysTick Handler */
  

  /* External Interrupts (IRQs) - STM32F0 has up to 32 here; we provide 32. */
  .word IRQ0_Handler
  .word IRQ1_Handler
  .word IRQ2_Handler
  .word IRQ3_Handler
  .word IRQ4_Handler
  .word IRQ5_Handler
  .word IRQ6_Handler
  .word IRQ7_Handler
  .word IRQ8_Handler
  .word IRQ9_Handler
  .word IRQ10_Handler
  .word IRQ11_Handler
  .word IRQ12_Handler
  .word IRQ13_Handler
  .word IRQ14_Handler
  .word IRQ15_Handler
  .word IRQ16_Handler
  .word IRQ17_Handler
  .word IRQ18_Handler
  .word IRQ19_Handler
  .word IRQ20_Handler
  .word IRQ21_Handler
  .word IRQ22_Handler
  .word IRQ23_Handler
  .word IRQ24_Handler
  .word IRQ25_Handler
  .word IRQ26_Handler
  .word IRQ27_Handler
  .word IRQ28_Handler
  .word IRQ29_Handler
  .word IRQ30_Handler
  .word IRQ31_Handler

.size g_pfnVectors, . - g_pfnVectors

/* -------------------------------------------------------------------------- */
/* Reset handler                                                               */
/* -------------------------------------------------------------------------- */
.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
  /* Copy .data from flash (_sidata) to SRAM (_sdata.._edata) */
  ldr  r0, =_sidata
  ldr  r1, =_sdata
  ldr  r2, =_edata
1:
  cmp  r1, r2
  bcc  2f
  b    3f
2:
  ldr  r3, [r0]
  str  r3, [r1]
  adds r0, r0, #4
  adds r1, r1, #4
  b    1b

3:
  /* Zero .bss (_sbss.._ebss) */
  ldr  r1, =_sbss
  ldr  r2, =_ebss
  movs r3, #0
4:
  cmp  r1, r2
  bcc  5f
  b    6f
5:
  str  r3, [r1]
  adds r1, r1, #4
  b    4b

6:
  /* Optional clock init (safe if you don't provide it) */
  bl   SystemInit

  /* Enter main */
  bl   main

  /* If main returns, trap */
7:
  b    7b

.size Reset_Handler, . - Reset_Handler

/* -------------------------------------------------------------------------- */
/* Exception / IRQ handlers                                                    */
/* -------------------------------------------------------------------------- */
.section .text.Handlers, "ax", %progbits

.global NMI_Handler
.type NMI_Handler, %function
NMI_Handler:
  bkpt #0
  b .

.global HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler:
  bkpt #0
  b .

.global SVC_Handler
.type SVC_Handler, %function
SVC_Handler:
  bkpt #0
  b .

.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
  bkpt #0
  b .



.word 0                /* Reserved */
.word PendSV_Handler    /* PendSV Handler */
.word SysTick_Handler   /* SysTick Handler */

.weak SysTick_Handler
.thumb_set SysTick_Handler, Default_Handler


/* Default catch-all for any IRQ you didn't implement */
.global Default_Handler
.type Default_Handler, %function
Default_Handler:
  bkpt #0
  b .

/* Weakly alias all IRQ handlers to Default_Handler */
.weak IRQ0_Handler
.thumb_set IRQ0_Handler, Default_Handler
.weak IRQ1_Handler
.thumb_set IRQ1_Handler, Default_Handler
.weak IRQ2_Handler
.thumb_set IRQ2_Handler, Default_Handler
.weak IRQ3_Handler
.thumb_set IRQ3_Handler, Default_Handler
.weak IRQ4_Handler
.thumb_set IRQ4_Handler, Default_Handler
.weak IRQ5_Handler
.thumb_set IRQ5_Handler, Default_Handler
.weak IRQ6_Handler
.thumb_set IRQ6_Handler, Default_Handler
.weak IRQ7_Handler
.thumb_set IRQ7_Handler, Default_Handler
.weak IRQ8_Handler
.thumb_set IRQ8_Handler, Default_Handler
.weak IRQ9_Handler
.thumb_set IRQ9_Handler, Default_Handler
.weak IRQ10_Handler
.thumb_set IRQ10_Handler, Default_Handler
.weak IRQ11_Handler
.thumb_set IRQ11_Handler, Default_Handler
.weak IRQ12_Handler
.thumb_set IRQ12_Handler, Default_Handler
.weak IRQ13_Handler
.thumb_set IRQ13_Handler, Default_Handler
.weak IRQ14_Handler
.thumb_set IRQ14_Handler, Default_Handler
.weak IRQ15_Handler
.thumb_set IRQ15_Handler, Default_Handler
.weak IRQ16_Handler
.thumb_set IRQ16_Handler, Default_Handler
.weak IRQ17_Handler
.thumb_set IRQ17_Handler, Default_Handler
.weak IRQ18_Handler
.thumb_set IRQ18_Handler, Default_Handler
.weak IRQ19_Handler
.thumb_set IRQ19_Handler, Default_Handler
.weak IRQ20_Handler
.thumb_set IRQ20_Handler, Default_Handler
.weak IRQ21_Handler
.thumb_set IRQ21_Handler, Default_Handler
.weak IRQ22_Handler
.thumb_set IRQ22_Handler, Default_Handler
.weak IRQ23_Handler
.thumb_set IRQ23_Handler, Default_Handler
.weak IRQ24_Handler
.thumb_set IRQ24_Handler, Default_Handler
.weak IRQ25_Handler
.thumb_set IRQ25_Handler, Default_Handler
.weak IRQ26_Handler
.thumb_set IRQ26_Handler, Default_Handler
.weak IRQ27_Handler
.thumb_set IRQ27_Handler, Default_Handler
.weak IRQ28_Handler
.thumb_set IRQ28_Handler, Default_Handler
.weak IRQ29_Handler
.thumb_set IRQ29_Handler, Default_Handler
.weak IRQ30_Handler
.thumb_set IRQ30_Handler, Default_Handler
.weak IRQ31_Handler
.thumb_set IRQ31_Handler, Default_Handler
