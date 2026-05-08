// =============================================================================
// spi2.c  —  SPI2 slave + DMA init, STM32F446RE
// =============================================================================
//
// WHAT THIS FILE DOES:
//   Configures the STM32 so that:
//   1. SPI2 operates as a slave (Pi is master, Pi drives the clock)
//   2. Every byte received on MOSI goes to memory via DMA (no CPU involvement)
//   3. Every byte clocked out on MISO comes from the telem double-buffer via DMA
//   4. When Pi asserts NSS (starts a transaction), an interrupt fires and
//      reloads the TX DMA pointer to the freshest telem slot
//   5. When a full 24-byte transaction completes on RX, an interrupt fires
//      and the opcode is decoded
//
// THE THREE HARDWARE BLOCKS INVOLVED:
//
//   SPI2 peripheral
//   │  Sits between the GPIO pins and the DMA.
//   │  Handles the actual shift-register protocol — clocking bits in/out.
//   │  We configure it as slave, 8-bit, Mode 0 (CPOL=0 CPHA=0).
//   │  We tell it to fire DMA requests instead of CPU interrupts for each byte.
//   │
//   ├─► DMA1 Stream 3  (SPI2 RX path)
//   │      Moves bytes from SPI2->DR (the received data register) to spi2_rx_buf[]
//   │      in memory. Fires an interrupt when 24 bytes have arrived (one full
//   │      transaction). CPU wakes up, decodes opcode, re-arms DMA for next txn.
//   │
//   └─► DMA1 Stream 4  (SPI2 TX path)
//          Moves bytes from telem_buf[] in memory to SPI2->DR (the transmit
//          data register). SPI2 clocks them out on MISO as Pi drives SCK.
//          Reloaded on every NSS falling edge to point at the freshest telem slot.
//
// DMA STREAM/CHANNEL MAPPING — F446 reference manual Table 42:
//   SPI2_RX  →  DMA1, Stream 3, Channel 0
//   SPI2_TX  →  DMA1, Stream 4, Channel 0
//   These are fixed in silicon. You cannot use other streams for SPI2.
//
// =============================================================================

#include "spi.h"
#include "stm32f4xx.h"

// -----------------------------------------------------------------------------
// Telem double-buffer
// Two slots. TIM1 ISR always writes to the inactive slot and flips the index.
// DMA TX always reads from the other slot.
// volatile = tell compiler "don't cache these in registers, don't reorder writes,
//            something outside your view (ISR + DMA) is touching this memory."
// -----------------------------------------------------------------------------
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// -----------------------------------------------------------------------------
// SPI2 RX buffer
// One transaction = 24 bytes max (sized to telem frame, our largest packet).
// DMA writes here; CPU reads it in the DMA complete ISR to decode the opcode.
// static = only visible in this file. volatile = DMA writes it, CPU reads it.
// -----------------------------------------------------------------------------
static volatile uint8_t spi2_rx_buf[24];


// =============================================================================
// spi2_init()
// Call once at startup, after SystemClock_Config(), before enabling interrupts.
// =============================================================================
void spi2_init(void) {

    // =========================================================================
    // STEP 1 — ENABLE CLOCKS
    // Every peripheral on STM32 has its clock gated off by default to save power.
    // You must explicitly turn on the clock before touching any register.
    // Touching a register with the clock off reads 0 and writes are silently lost.
    // =========================================================================

    // DMA1 lives on the AHB1 bus
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // SPI2 lives on the APB1 bus (slower bus, max 45 MHz on F446)
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // GPIOB — where SPI2 pins live (PB12–PB15)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // SYSCFG — needed to configure which GPIO port drives the EXTI line for NSS
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;


    // =========================================================================
    // STEP 2 — CONFIGURE GPIO PINS FOR SPI2 (Alternate Function mode)
    //
    // STM32 GPIO pins have 4 modes:
    //   00 = Input
    //   01 = Output (CPU controls the level)
    //   10 = Alternate Function (peripheral controls the pin)  ← we want this
    //   11 = Analog
    //
    // In Alternate Function mode, each pin can be routed to one of 16 functions
    // (AF0–AF15). SPI2 is AF5 on PB12–PB15. This is in the F446 datasheet
    // Table 11 (pin alternate function mapping). The reference manual has the
    // peripheral description; the datasheet has the pin mapping. Both required.
    //
    // Pins:
    //   PB12 = SPI2_NSS  (chip select, active low, Pi drives it)
    //   PB13 = SPI2_SCK  (clock, Pi drives it)
    //   PB14 = SPI2_MISO (master-in slave-out, STM drives it — telem to Pi)
    //   PB15 = SPI2_MOSI (master-out slave-in, Pi drives it — commands to STM)
    // =========================================================================

    // MODER register: 2 bits per pin, pins 12–15 are bits 24–31
    // Clear the 4 pins first, then set all to AF (10)
    GPIOB->MODER &= ~( (3u << 24) | (3u << 26) | (3u << 28) | (3u << 30) );
    GPIOB->MODER |=  ( (2u << 24) | (2u << 26) | (2u << 28) | (2u << 30) );

    // AFR (Alternate Function Register) — selects which AF (0–15) for each pin
    // AFR[0] = pins 0–7, AFR[1] = pins 8–15
    // Pins 12–15 are in AFR[1], each uses 4 bits
    // Bit positions in AFR[1]: pin12=bits[19:16], pin13=[23:20], pin14=[27:24], pin15=[31:28]
    GPIOB->AFR[1] &= ~( (0xFu << 16) | (0xFu << 20) | (0xFu << 24) | (0xFu << 28) );
    GPIOB->AFR[1] |=  ( (5u   << 16) | (5u   << 20) | (5u   << 24) | (5u   << 28) );

    // Output speed — set to high speed for SPI signals
    // OSPEEDR: 2 bits per pin, 11 = very high speed
    GPIOB->OSPEEDR |= ( (3u << 24) | (3u << 26) | (3u << 28) | (3u << 30) );


    // =========================================================================
    // STEP 3 — CONFIGURE SPI2 PERIPHERAL
    //
    // SPI2->CR1 controls the core SPI behavior.
    // SPI2->CR2 controls interrupts, DMA requests, and frame size.
    //
    // Key decisions:
    //   MSTR = 0  →  slave mode. Pi drives SCK and NSS.
    //   CPOL = 0  →  clock idle low
    //   CPHA = 0  →  data sampled on first (rising) edge  = Mode 0
    //   DFF  = 0  →  8-bit frames
    //   SSM  = 0  →  hardware NSS management (PB12 controls enable)
    //   SPE  = 1  →  SPI enable
    //
    // In slave mode there is no baud rate to configure — we just follow Pi's clock.
    // =========================================================================

    SPI2->CR1 = 0;  // start clean

    SPI2->CR1 = SPI_CR1_SPE;
    //           ^^^^^^^^^^^
    //           SPE = SPI Enable. Everything else is 0:
    //           MSTR=0 (slave), CPOL=0, CPHA=0, DFF=0 (8-bit), SSM=0 (hw NSS)

    // CR2: enable DMA requests for TX and RX
    // Instead of interrupting the CPU for every byte, SPI2 signals the DMA
    // controller directly. DMA moves the byte to/from memory with no CPU cost.
    SPI2->CR2 = SPI_CR2_TXDMAEN   // fire DMA request when TX buffer empty
              | SPI_CR2_RXDMAEN;  // fire DMA request when RX buffer has data


    // =========================================================================
    // STEP 4 — CONFIGURE DMA1 STREAM 3  (SPI2 RX → spi2_rx_buf)
    //
    // This stream moves bytes FROM the SPI2 receive register TO our rx buffer.
    // It counts down from 24. When it hits 0, it fires the TCIF3 interrupt.
    // CPU wakes up, decodes the opcode, re-arms the stream for the next txn.
    //
    // DMA stream registers:
    //   CR    = control (channel select, direction, increment, interrupts)
    //   NDTR  = number of data items to transfer (counts down to 0)
    //   PAR   = peripheral address (fixed — SPI2->DR, the data register)
    //   M0AR  = memory address (destination — spi2_rx_buf)
    // =========================================================================

    // Disable stream before configuring — required by hardware
    DMA1_Stream3->CR = 0;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);  // wait for hardware to confirm off

    DMA1_Stream3->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |  // Channel 0 — required for SPI2_RX on Stream 3
        DMA_SxCR_MINC               |  // Memory INCrement — advance rx_buf pointer each byte
                                       // Peripheral does NOT increment — always reads SPI2->DR
        (0u << DMA_SxCR_DIR_Pos)    |  // Direction: 00 = peripheral → memory
        DMA_SxCR_TCIE;                 // Transfer Complete Interrupt Enable
                                       // Fires when NDTR counts down to 0 (all 24 bytes received)

    DMA1_Stream3->NDTR = 24;                    // expect 24 bytes per transaction
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;   // source: SPI2 data register (fixed)
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf; // destination: our rx buffer

    // Enable the stream — it's now armed and waiting for SPI2 to assert a DMA request
    DMA1_Stream3->CR |= DMA_SxCR_EN;


    // =========================================================================
    // STEP 5 — CONFIGURE DMA1 STREAM 4  (telem_buf → SPI2 TX)
    //
    // This stream moves bytes FROM the telem double-buffer TO the SPI2 transmit
    // register, which clocks them out on MISO as Pi drives SCK.
    //
    // Direction is reversed from Stream 3: memory → peripheral.
    // No TC interrupt needed — we don't care when TX finishes, only when
    // RX finishes (that's when we know a full transaction occurred).
    //
    // The M0AR (source address) gets reloaded on every NSS falling edge to
    // point at whichever telem_buf slot is NOT currently being written by the ISR.
    // =========================================================================

    DMA1_Stream4->CR = 0;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);

    DMA1_Stream4->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |  // Channel 0 — required for SPI2_TX on Stream 4
        DMA_SxCR_MINC               |  // Memory INCrement — advance through telem_buf each byte
        (1u << DMA_SxCR_DIR_Pos);     // Direction: 01 = memory → peripheral
                                       // No TCIE — don't need TX complete interrupt

    DMA1_Stream4->NDTR = 24;                           // send 24 bytes per transaction
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;          // destination: SPI2 data register
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[0];      // initial source — reloaded on NSS

    DMA1_Stream4->CR |= DMA_SxCR_EN;


    // =========================================================================
    // STEP 6 — CONFIGURE EXTI12 ON PB12 (NSS falling edge)
    //
    // When Pi asserts NSS (drives PB12 low), we need to immediately reload the
    // TX DMA pointer to the freshest telem slot. This is a GPIO interrupt.
    //
    // EXTI (External Interrupt) lines 0–15 map to GPIO pins 0–15.
    // Each EXTI line can be connected to one GPIO port via SYSCFG->EXTICR.
    // EXTI12 can come from PA12, PB12, PC12, etc. We wire it to PB12.
    //
    // SYSCFG->EXTICR[3] controls EXTI12–15. Each field is 4 bits.
    // EXTI12 field = bits [3:0] of EXTICR[3]. Value 0x1 = Port B.
    // =========================================================================

    SYSCFG->EXTICR[3] &= ~(0xFu << 0);   // clear EXTI12 source
    SYSCFG->EXTICR[3] |=  (0x1u << 0);   // connect EXTI12 to Port B (PB12)

    EXTI->FTSR |= (1u << 12);  // Falling edge Trigger Select Register — trigger on NSS low
    EXTI->RTSR &= ~(1u << 12); // No rising edge trigger
    EXTI->IMR  |= (1u << 12);  // Interrupt Mask Register — unmask EXTI12 (allow interrupt)

    // Priority 1 — higher priority than DMA complete (2), lower than TIM1 FOC (0)
    // NSS must reload DMA before first SCK edge, so it needs to be responsive
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // DMA1 Stream 3 complete interrupt — lower priority, just decodes opcode
    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}


// =============================================================================
// EXTI15_10_IRQHandler  —  NSS falling edge (Pi starting a transaction)
//
// WHAT HAPPENS HERE:
//   Pi just pulled NSS low. A SPI transaction is starting.
//   We have a very short window (~1 µs at 8 MHz SPI) before the first SCK edge.
//   We must reload the TX DMA to point at the freshest telem slot NOW.
//
// WHY DISABLE/RELOAD/ENABLE:
//   DMA hardware latches M0AR when the stream is enabled. You cannot change
//   M0AR while the stream is running — the write is ignored. So:
//   1. Disable stream (CR EN=0)
//   2. Wait for hardware to confirm (EN bit clears)
//   3. Write new NDTR and M0AR
//   4. Re-enable (CR EN=1)
//   All 4 steps happen in ~200 ns — well within the NSS-to-SCK window.
// =============================================================================
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1u << 12)) {
        EXTI->PR = (1u << 12);  // clear pending flag by writing 1 (not 0 — counterintuitive)

        // Disable TX DMA stream
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream4->CR & DMA_SxCR_EN);  // wait for hardware confirm

        // Reload: point at the read slot (whichever slot ISR is NOT writing to)
        // telem_write_idx is the slot ISR is ABOUT TO write next
        // telem_write_idx ^ 1 is the slot ISR just FINISHED writing — safe to read
        DMA1_Stream4->NDTR = 24;
        DMA1_Stream4->M0AR = (uint32_t)&telem_buf[telem_write_idx ^ 1];

        // Re-enable — DMA is now armed to clock out the fresh telem frame
        DMA1_Stream4->CR |= DMA_SxCR_EN;
    }
}


// =============================================================================
// DMA1_Stream3_IRQHandler  —  SPI2 RX complete (full 24-byte transaction done)
//
// WHAT HAPPENS HERE:
//   DMA has moved all 24 bytes from SPI2->DR to spi2_rx_buf[].
//   The transaction is complete. Decode the opcode and act.
//   Then re-arm the RX DMA for the next transaction.
//
// NOTE ON RE-ARMING:
//   After a transfer completes, NDTR is 0 and the stream stops.
//   You must explicitly disable, reload NDTR, and re-enable.
//   If you don't re-arm before the next NSS edge, the next transaction
//   is lost — bytes go into SPI2->DR with nobody to receive them.
// =============================================================================
void DMA1_Stream3_IRQHandler(void) {
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;  // clear transfer complete flag

        uint8_t opcode = spi2_rx_buf[0];

        switch (opcode) {

            case 0x06:  // TELEM_REQ — Pi just wants telem, nothing to do on RX side
                break;

            case 0x04:  // DATA packet — trajectory sample, push to ring buffer
                // TrajSample is at bytes 1–8, CRC at byte 9
                // ring_buffer_push((TrajSample*)&spi2_rx_buf[1]);
                // TODO: implement when ring buffer is ready
                break;

            case 0x03:  // BLOCK_HDR — start of 2048-sample refill block
                // TODO: extract sample_count, validate CRC, set CommSM state
                break;

            case 0x05:  // READY_ACK — Pi acknowledged READY signal
                // TODO: clear PC13 READY flag
                break;

            case 0x00:  // NOP / idle
            default:
                break;
        }

        // Re-arm RX DMA for next transaction
        DMA1_Stream3->CR  &= ~DMA_SxCR_EN;
        while (DMA1_Stream3->CR & DMA_SxCR_EN);
        DMA1_Stream3->NDTR = 24;
        DMA1_Stream3->CR  |= DMA_SxCR_EN;
    }
}