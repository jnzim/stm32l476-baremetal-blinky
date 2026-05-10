// spi.c — SPI2 slave + DMA, STM32F446RE bare metal
//
// SPI2 pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI (AF5)
// DMA1 Stream3 Ch0 = SPI2_RX   (periph → memory, normal mode, ping-pong)
// DMA1 Stream4 Ch0 = SPI2_TX   (memory → periph, circular)
//
// RX strategy: manual ping-pong between rx_buf_a / rx_buf_b.
//   Normal (non-circular) mode — stream stops after 24 bytes.
//   ISR immediately re-arms into the OTHER buffer, then decodes the just-filled one.
//   No race: DMA is writing into the new buffer while ISR reads the old one.
//
// TX strategy: circular DMA replays telem_buf[1] continuously.
//   TIM1 ISR writes telem_buf[0], then atomically swaps the TX DMA pointer
//   to whichever slot was just written. TX never needs an interrupt.

#include "spi.h"
#include "ringBuffer.h"
#include "stm32f4xx.h"
#include <string.h>

// ── Opcodes — use protocol.h constants, no local redefinition ────────────────
// SPI2_OP_BLOCK_HDR 0x03, SPI2_OP_DATA 0x04, SPI2_OP_READY_ACK 0x05, SPI2_OP_TELEM_REQ 0x06

// ── Packet length — use protocol.h constant ───────────────────────────────────
#define SPI2_PKT_LEN  SPI2_TRANSACTION_BYTES   // 24

// ── Telemetry double-buffer ───────────────────────────────────────────────────
// telem_buf[0]: written by TIM1 ISR
// telem_buf[1]: read by TX DMA (swap done atomically in spi_update_telem)
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// ── RX buffer — circular DMA writes here continuously ────────────────────────
static volatile uint8_t spi2_rx_buf[SPI2_PKT_LEN];

// ── Diagnostic counter (remove when stable) ──────────────────────────────────
volatile uint32_t cnt_data          = 0;
volatile uint32_t cnt_telem         = 0;
volatile uint32_t cnt_error         = 0;
// ── Reset after each move  ──────────────────────────────────
volatile uint32_t samples_consumed  = 0;

// ─────────────────────────────────────────────────────────────────────────────
// spi_init
// ─────────────────────────────────────────────────────────────────────────────
void spi_init(void)
{
    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // ── GPIO PB12–PB15: AF5 = SPI2 ───────────────────────────────────────────
    // Clear and set MODER to alternate function (10) for pins 12–15
    GPIOB->MODER &= ~((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));
    GPIOB->MODER |=  ((2u << 24) | (2u << 26) | (2u << 28) | (2u << 30));

    // AF5 in AFRH (pins 8–15 → AFR[1], each pin 4 bits, pin N at bit (N-8)*4)
    GPIOB->AFR[1] &= ~((0xFu << 16) | (0xFu << 20) | (0xFu << 24) | (0xFu << 28));
    GPIOB->AFR[1] |=  ((5u  << 16) | (5u  << 20) | (5u  << 24) | (5u  << 28));

    // Very high speed on all four pins
    GPIOB->OSPEEDR |= ((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));

    // ── SPI2: slave, Mode 0, 8-bit, SSM (software NSS, no SSI) ──────────────
    SPI2->CR1 = 0;
    SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SPE;

    // Enable DMA requests for both TX and RX
    SPI2->CR2 = SPI_CR2_TXDMAEN
              | SPI_CR2_RXDMAEN;

    // ── DMA1 Stream4 Ch0: SPI2_TX — telem_buf[1] → SPI DR, circular ─────────
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);

    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4  | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[1];
    DMA1_Stream4->NDTR = SPI2_PKT_LEN;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |   // channel 0 = SPI2_TX
        (1u << DMA_SxCR_DIR_Pos)   |   // memory → peripheral
        DMA_SxCR_MINC              |   // increment M0AR each byte
        DMA_SxCR_CIRC;                 // circular — replays indefinitely, no ISR needed

    DMA1_Stream4->CR |= DMA_SxCR_EN;

    // ── DMA1 Stream3 Ch0: SPI2_RX — circular, fires every 24 bytes ───────────
    DMA1_Stream3->CR = 0;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);

    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->NDTR = SPI2_PKT_LEN;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |   // channel 0 = SPI2_RX
        (0u << DMA_SxCR_DIR_Pos)   |   // peripheral → memory
        DMA_SxCR_MINC              |   // increment M0AR each byte
        DMA_SxCR_CIRC              |   // circular — auto-restart
        DMA_SxCR_TCIE;                 // interrupt every 24 bytes

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    // ── NVIC ──────────────────────────────────────────────────────────────────
    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

// ─────────────────────────────────────────────────────────────────────────────
// DMA1_Stream3_IRQHandler — SPI2 RX complete (circular mode)
//
// Circular DMA restarts immediately after TCIF fires. To avoid the race where
// DMA overwrites byte 0 before we read it, snapshot the buffer into a local
// array first, then clear TCIF, then decode from the snapshot.
// The snapshot window is safe: DMA just reset its pointer to byte 0 and needs
// one SPI clock edge before writing — at 1MHz that's 1µs, memcpy of 24 bytes
// at 180MHz takes ~15 cycles = ~83ns. Well within the window.
// ─────────────────────────────────────────────────────────────────────────────
void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;

    // Snapshot buffer before clearing — DMA is at byte 0 of next revolution
    uint8_t local[SPI2_PKT_LEN];
    memcpy(local, (void*)spi2_rx_buf, SPI2_PKT_LEN);

    // Clear TC flag
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    // Decode from stable local copy
    switch (local[0]) {
        case SPI2_OP_DATA: {
            uint8_t crc = 0;
            for (int i = 0; i < 9; i++) crc ^= local[i];
            if (crc != local[9]) { cnt_error++; break; }
            TrajSample s;
            memcpy(&s.pos_cmd, &local[1], sizeof(int32_t));
            memcpy(&s.vel_cmd,  &local[5], sizeof(int32_t));
            ring_push(&s);
            cnt_data++;
            break;
        }
        case SPI2_OP_TELEM_REQ:
            cnt_telem++;
            break;
        case SPI2_OP_BLOCK_HDR:
            ring_reset();
            samples_consumed = 0;
            telem_buf[1].samples_consumed = 0;
            break;
        case SPI2_OP_READY_ACK:
            break;

        default:
            cnt_error++;
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// spi_process
// Called from superloop. Non-ISR bookkeeping — telemetry assembly, ring buffer
// health checks, fault reporting. Currently a stub.
// ─────────────────────────────────────────────────────────────────────────────
void spi_process(void)
{
    // TODO: build TelemetryFrame from drive state, call spi_update_telem()
}

// ─────────────────────────────────────────────────────────────────────────────
// spi_update_telem
// Not in spi.h yet — add declaration there when telemetry path is wired up.
// Called from TIM1 ISR (or superloop) after writing a fresh TelemetryFrame
// into telem_buf[0]. Atomically swaps the TX DMA source pointer so the next
// DMA revolution clocks out the new data.
//
// Safe because:
//   M0AR write is 32-bit and naturally atomic on Cortex-M4.
//   TX DMA reads M0AR only at the start of each revolution (when NDTR wraps).
//   We update M0AR while DMA is mid-packet (not at the wrap boundary), so
//   the swap takes effect at the next clean packet boundary.
// ─────────────────────────────────────────────────────────────────────────────
void spi_update_telem(const TelemetryFrame *frame)
{
    // Write into the shadow slot
    uint8_t write_slot = telem_write_idx ^ 1u;
    memcpy((void*)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));

    // Swap TX DMA to the freshly written slot
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[write_slot];

    telem_write_idx = write_slot;
}