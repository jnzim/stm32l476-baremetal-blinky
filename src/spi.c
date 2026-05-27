// spi.c — SPI2 slave + DMA, STM32F411 bare metal
//
// SPI2 pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI (all AF5)
// DMA1 Stream3 Ch0 = SPI2_RX (peripheral → memory, circular)
// DMA1 Stream4 Ch0 = SPI2_TX (memory → peripheral, circular)
//
// Architecture (from working baseline):
//   Hardware NSS (no SSM) — Pi's CS line synchronizes the SPI shift register
//   per frame. NSS rising edge resets bit alignment, so no per-frame slide.
//
//   TX DMA — circular, never stops. Replays telem_buf[1] forever.
//     Atomic M0AR pointer swap in spi_update_telem() to swap slots.
//
//   RX DMA — circular, never stops. ISR fires every 32 bytes.
//     Snapshots buffer to a local copy before clearing TC flag, so DMA
//     can immediately resume writing without race.

#include "spi.h"
#include "protocol.h"
#include "ringBuffer.h"
#include "drive.h"
#include "loops.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdint.h>

#define SPI_FRAME_BYTES SPI2_TRANSACTION_BYTES

// ── Diagnostic counters ──────────────────────────────────────────────────────
volatile uint32_t cnt_data      = 0;
volatile uint32_t cnt_error     = 0;
volatile uint32_t cnt_telem     = 0;
volatile uint32_t cnt_block_hdr = 0;

volatile uint8_t  ready_asserted = 0;

// ── Telemetry double-buffer ──────────────────────────────────────────────────
// telem_buf[0]: shadow slot, written by SysTick (or whatever updates telem)
// telem_buf[1]: active slot, read by TX DMA
// spi_update_telem() writes new data into the shadow slot then atomically
// swaps the TX DMA M0AR to point at the freshly written slot.
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// ── RX buffer — circular DMA writes here continuously ────────────────────────
static volatile uint8_t spi2_rx_buf[SPI_FRAME_BYTES];

typedef char TelemetryFrame_must_be_32_bytes[
    (sizeof(TelemetryFrame) == SPI_FRAME_BYTES) ? 1 : -1
];

typedef char TrajSample_must_be_8_bytes[
    (sizeof(TrajSample) == 8u) ? 1 : -1
];

void spi_init(void)
{
    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // ── PC13 READY output (active-low, idle high) ────────────────────────────
    GPIOC->MODER   &= ~(3u << 26);
    GPIOC->MODER   |=  (1u << 26);
    GPIOC->OTYPER  &= ~(1u << 13);
    GPIOC->OSPEEDR &= ~(3u << 26);
    GPIOC->OSPEEDR |=  (1u << 26);
    GPIOC->PUPDR   &= ~(3u << 26);
    GPIOC->BSRR     =  (1u << 13);

    // ── GPIO PB12–PB15: AF5 = SPI2 (hardware NSS) ───────────────────────────
    GPIOB->MODER &= ~((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));
    GPIOB->MODER |=  ((2u << 24) | (2u << 26) | (2u << 28) | (2u << 30));

    GPIOB->AFR[1] &= ~((0xFu << 16) | (0xFu << 20) | (0xFu << 24) | (0xFu << 28));
    GPIOB->AFR[1] |=  ((5u  << 16) | (5u  << 20) | (5u  << 24) | (5u  << 28));

    GPIOB->OSPEEDR |= ((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));

    // ── SPI2: slave, Mode 0, 8-bit, HARDWARE NSS ─────────────────────────────
    // CR2 (DMA requests) configured FIRST, then DMA streams enabled,
    // SPE set LAST so peripheral comes alive with everything ready.
    SPI2->CR1 = 0;
    SPI2->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // ── DMA1 Stream4 Ch0: SPI2_TX — telem_buf[1] → SPI DR, circular ─────────
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[1];
    DMA1_Stream4->NDTR = SPI_FRAME_BYTES;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |   // channel 0 = SPI2_TX
        (1u << DMA_SxCR_DIR_Pos)   |   // memory → peripheral
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC;                 // circular, no IRQ

    DMA1_Stream4->CR |= DMA_SxCR_EN;

    // ── DMA1 Stream3 Ch0: SPI2_RX — circular, IRQ every 32 bytes ────────────
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->NDTR = SPI_FRAME_BYTES;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |   // channel 0 = SPI2_RX
        (0u << DMA_SxCR_DIR_Pos)   |   // peripheral → memory
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_TCIE;

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    // ── SPE last — peripheral comes online with DMA already armed ───────────
    //SPI2->CR1 |= SPI_CR1_SPE;
    SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SPE;
}

// ─────────────────────────────────────────────────────────────────────────────
// spi_update_telem
// Called whenever fresh telemetry is ready. Writes into the shadow slot,
// then atomically swaps the TX DMA pointer.
// ─────────────────────────────────────────────────────────────────────────────
void spi_update_telem(const TelemetryFrame *frame)
{
    uint8_t write_slot = telem_write_idx ^ 1u;

    memcpy((void *)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));

    __DMB();

    // M0AR write is naturally atomic on Cortex-M4.
    // DMA will pick up the new pointer at the next circular wrap.
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[write_slot];

    telem_write_idx = write_slot;
}

// ─────────────────────────────────────────────────────────────────────────────
// DMA1_Stream3_IRQHandler — SPI2 RX complete (circular mode)
//
// Snapshot the buffer immediately, then clear TC flag, then decode.
// Window is safe: DMA just reset to byte 0 of next revolution and needs
// at least one SPI byte time before writing.
// ─────────────────────────────────────────────────────────────────────────────
void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;

    uint8_t local[SPI_FRAME_BYTES];
    memcpy(local, (void *)spi2_rx_buf, SPI_FRAME_BYTES);

    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    uint8_t crc = 0;

    switch (local[0]) {
        case SPI2_OP_DATA:
            // [0]   opcode
            // [1-4] int32 pos_cmd
            // [5-8] int32 vel_cmd
            // [9]   CRC over bytes 0-8
            for (int i = 0; i < 9; i++) crc ^= local[i];
            if (crc != local[9]) {
                cnt_error++;
                break;
            }
            {
                TrajSample s;
                memcpy(&s.pos_cmd, &local[1], sizeof(int32_t));
                memcpy(&s.vel_cmd, &local[5], sizeof(int32_t));
                ring_push(&s);
                cnt_data++;
            }
            if (ready_asserted) {
                GPIOC->BSRR    = (1u << 13);   // PC13 HIGH = deasserted
                ready_asserted = 0;
            }
            break;

        case SPI2_OP_BLOCK_HDR:
            for (int i = 0; i < 3; i++) crc ^= local[i];
            if (crc != local[3]) {
                cnt_error++;
                break;
            }
            first_sample_ready = 0;
            ring_reset();
            samples_consumed = 0;
            cnt_block_hdr++;

            GPIOC->BSRR    = (1u << 13);
            ready_asserted = 0;

            drive_request_servo_on();
            break;

        case SPI2_OP_OPEN_LOOP:
            for (int i = 0; i < 9; i++) crc ^= local[i];
            if (crc != local[9]) {
                cnt_error++;
                break;
            }
            {
                float v_mag, d_theta;
                memcpy(&v_mag,   &local[1], sizeof(float));
                memcpy(&d_theta, &local[5], sizeof(float));
                drive_request_open_loop(v_mag, d_theta);
            }
            break;

        case SPI2_OP_STOP:
            crc = local[0];
            if (crc != local[1]) {
                cnt_error++;
                break;
            }
            drive_request_stop();
            break;

        case SPI2_OP_TELEM_REQ:
            cnt_telem++;
            break;

        case SPI2_OP_READY_ACK:
        case SPI2_OP_NOP:
            break;

        default:
            cnt_error++;
            break;
    }
}