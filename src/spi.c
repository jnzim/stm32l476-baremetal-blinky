// spi.c — SPI2 slave + DMA, STM32F446RE bare metal
//
// APPROACH: both RX and TX DMA run in circular mode — no manual re-arming,
// no EXTI handler, no NSS interrupt. DMA runs continuously.
// TX always clocks out the latest telem_buf contents.
// RX always receives into spi2_rx_buf, IRQ fires every 24 bytes to decode opcode.

#include "spi.h"
#include "stm32f4xx.h"

// telem double-buffer — written by TIM1 ISR, read by SPI2 DMA TX
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// SPI2 RX buffer — DMA writes here, IRQ reads opcode
static volatile uint8_t spi2_rx_buf[24];

void spi_init(void) {

    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;    // DMA1 on AHB1 bus
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;    // SPI2 on APB1 bus
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   // GPIOB for SPI2 pins

    // ── GPIO PB12-PB15 — AF5 = SPI2 ──────────────────────────────────────────
    // PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI
    GPIOB->MODER &= ~( (3u<<24)|(3u<<26)|(3u<<28)|(3u<<30) );
    GPIOB->MODER |=  ( (2u<<24)|(2u<<26)|(2u<<28)|(2u<<30) );  // alternate function

    GPIOB->AFR[1] &= ~( (0xFu<<16)|(0xFu<<20)|(0xFu<<24)|(0xFu<<28) );
    GPIOB->AFR[1] |=  ( (5u<<16)|(5u<<20)|(5u<<24)|(5u<<28) );  // AF5 = SPI2

    GPIOB->OSPEEDR |= ( (3u<<24)|(3u<<26)|(3u<<28)|(3u<<30) );  // very high speed

    // ── SPI2 — slave, Mode 0, 8-bit, hardware NSS, DMA enabled ──────────────
    SPI2->CR1 = 0;
    SPI2->CR1 = SPI_CR1_SPE;               // slave mode, CPOL=0, CPHA=0, 8-bit
    SPI2->CR2 = SPI_CR2_TXDMAEN
              | SPI_CR2_RXDMAEN;           // DMA requests for TX and RX

    // ── DMA1 Stream 3 — SPI2 RX → spi2_rx_buf, circular ────────────────────
    DMA1_Stream3->CR = 0;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);

    DMA1_Stream3->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |      // channel 0 = SPI2_RX on Stream 3
        DMA_SxCR_MINC               |      // increment memory address each byte
        (0u << DMA_SxCR_DIR_Pos)    |      // peripheral → memory
        DMA_SxCR_CIRC               |      // circular — auto restart, no re-arm needed
        DMA_SxCR_TCIE;                     // interrupt when 24 bytes received

    DMA1_Stream3->NDTR = 24;
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->CR  |= DMA_SxCR_EN;

    // ── DMA1 Stream 4 — telem_buf → SPI2 TX, circular ───────────────────────
    DMA1_Stream4->CR = 0;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);

    DMA1_Stream4->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |      // channel 0 = SPI2_TX on Stream 4
        DMA_SxCR_MINC               |      // increment memory address each byte
        (1u << DMA_SxCR_DIR_Pos)    |      // memory → peripheral
        DMA_SxCR_CIRC;                     // circular — auto restart, no re-arm needed

    DMA1_Stream4->NDTR = 24;
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[1];  // always send from slot 1
    DMA1_Stream4->CR  |= DMA_SxCR_EN;

    // ── NVIC — DMA1 Stream 3 RX complete interrupt ───────────────────────────
    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

// DMA1 Stream 3 complete — 24 bytes received, decode opcode
void DMA1_Stream3_IRQHandler(void) {
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;

        uint8_t opcode = spi2_rx_buf[0];

        switch (opcode) {
            case 0x06:  // TELEM_REQ — nothing to do, telem already clocked out
                break;
            case 0x04:  // DATA packet — push to ring buffer
                // ring_buffer_push((TrajSample*)&spi2_rx_buf[1]);
                break;
            case 0x03:  // BLOCK_HDR
                break;
            case 0x05:  // READY_ACK
                break;
            case 0x00:
            default:
                break;
        }
        // circular mode — no re-arm needed
    }
}