// spi.c — SPI2 slave RX DMA, STM32F411 bare metal
//
// SPI2 pins:
//   PB12 = NSS / CS from Pi, AF5 hardware NSS
//   PB13 = SCK, AF5
//   PB14 = MISO, AF5
//   PB15 = MOSI, AF5
//
// DMA1 Stream3 Ch0 = SPI2_RX
// DMA1 Stream4 Ch0 = SPI2_TX, currently unused
//
// Current baseline:
//   Pi TX / MOSI -> STM SPI2 RX -> DMA -> local[] -> CRC -> ring_push()
//   MISO telemetry is intentionally disabled/deferred.

#include "spi.h"
#include "protocol.h"
#include "ringBuffer.h"
#include "drive.h"
#include "loops.h"
#include "stm32f4xx.h"

#include <string.h>
#include <stdint.h>



volatile uint32_t cnt_data      = 0;
volatile uint32_t cnt_error     = 0;
volatile uint32_t cnt_telem     = 0;
volatile uint32_t cnt_block_hdr = 0;

volatile uint8_t ready_asserted = 0;

volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

static volatile uint8_t     spi2_rx_buf[SPI2_TRANSACTION_BYTES];
volatile        uint16_t    expected_samples;


typedef char TelemetryFrame_must_be_32_bytes[
    (sizeof(TelemetryFrame) == SPI2_TRANSACTION_BYTES) ? 1 : -1
];

extern uint16_t crc16_calc(const uint8_t* data, size_t len);

static void spi2_dma_clear_flags(void)
{
    // DMA1 Stream3 RX flags are in LIFCR.
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    // DMA1 Stream4 TX flags are in HIFCR.
    DMA1->HIFCR = DMA_HIFCR_CTCIF4  |
                  DMA_HIFCR_CHTIF4  |
                  DMA_HIFCR_CTEIF4  |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;
}

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // PC13 READY output, active-low, idle high.
    GPIOC->MODER   &= ~(3u << 26);
    GPIOC->MODER   |=  (1u << 26);
    GPIOC->OTYPER  &= ~(1u << 13);
    GPIOC->OSPEEDR &= ~(3u << 26);
    GPIOC->OSPEEDR |=  (1u << 26);
    GPIOC->PUPDR   &= ~(3u << 26);
    GPIOC->BSRR     =  (1u << 13);

    // PB12/PB13/PB14/PB15 all AF5 SPI2.
    GPIOB->MODER &= ~((3u << 24) |
                      (3u << 26) |
                      (3u << 28) |
                      (3u << 30));

    GPIOB->MODER |=  ((2u << 24) |   // PB12 NSS
                      (2u << 26) |   // PB13 SCK
                      (2u << 28) |   // PB14 MISO
                      (2u << 30));   // PB15 MOSI

    GPIOB->AFR[1] &= ~((0xFu << 16) |
                       (0xFu << 20) |
                       (0xFu << 24) |
                       (0xFu << 28));

    GPIOB->AFR[1] |=  ((5u << 16) |
                       (5u << 20) |
                       (5u << 24) |
                       (5u << 28));

    GPIOB->OSPEEDR |= ((3u << 24) |
                       (3u << 26) |
                       (3u << 28) |
                       (3u << 30));

    GPIOB->PUPDR &= ~(3u << 24);
    GPIOB->PUPDR |=  (1u << 24);

    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    spi2_dma_clear_flags();

    // RX DMA: SPI2->DR -> spi2_rx_buf (Stream3, Channel 0).
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_TCIE;
    DMA1_Stream3->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    // TX DMA: telem_buf[1] -> SPI2->DR (Stream4, Channel 0).
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[1];
    DMA1_Stream4->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0             |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC;
    DMA1_Stream4->CR |= DMA_SxCR_EN;

    // SPI2 slave, Mode 1, software NSS (SSM=1, SSI=0 — slave always selected).
    // CPOL=0, CPHA=1, MSTR=0.
    SPI2->CR1 = SPI_CR1_CPHA | SPI_CR1_SSM;

    // RXDMAEN: SPI2 requests DMA each time it receives a byte (Stream3 services it).
    // TXDMAEN: SPI2 requests DMA each time its transmit register is empty (Stream4 services it).
    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    __DMB();

    SPI2->CR1 |= SPI_CR1_SPE;

    // EXTI12 — PB12 CS falling edge -> rearm RX DMA for frame alignment.
    // Armed after SPE so it cannot fire before SPI2 is ready.
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (0x1u << 0);
    EXTI->FTSR |=  (1u << 12);
    EXTI->RTSR &= ~(1u << 12);
    EXTI->IMR  |=  (1u << 12);
    EXTI->PR    =  (1u << 12);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void spi_update_telem(const TelemetryFrame *frame)
{
    uint8_t write_slot = telem_write_idx ^ 1u;
    memcpy((void*)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[write_slot];
    telem_write_idx = write_slot;
}


void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) {
        return;
    }

    uint8_t local[SPI2_TRANSACTION_BYTES];

    // Copy completed 32-byte RX frame before parsing.
    memcpy(local, (void *)spi2_rx_buf, SPI2_TRANSACTION_BYTES);

    // Clear RX DMA flags.
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    uint8_t opcode = local[0];

    switch (opcode) {
        case SPI2_OP_DATA:
        {
            TrajSlot *s = (TrajSlot *)local;
            uint16_t crc_calc = crc16_calc(local, TRAJ_CRC_LEN);
            uint16_t crc_recv = s->crc16;
            
            if (crc_calc != crc_recv) {
                cnt_error++;
                break;
            }

            ring_push(s);
            cnt_data++;

            if (ready_asserted) {
                GPIOC->BSRR    = (1u << 13);
                ready_asserted = 0;
            }
        }
        break;

        case SPI2_OP_BLOCK_HDR:
        {
            uint16_t crc_calc = crc16_calc(local, 14);
            uint16_t crc_recv = (uint16_t)local[14] | ((uint16_t)local[15] << 8);
            
            if (crc_calc != crc_recv) 
            {
                cnt_error++;
                break;
            }

            expected_samples   = (uint16_t)(local[1] | ((uint16_t)local[2] << 8));
            first_sample_ready = 0;
            ring_reset();
            samples_consumed = 0;
            cnt_block_hdr++;

            GPIOC->BSRR    = (1u << 13);
            ready_asserted = 0;

            drive_request_servo_on();
        }
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



volatile uint32_t cnt_cs = 0;
// void EXTI15_10_IRQHandler(void)
// {
//     if (EXTI->PR & (1u << 12))
//     {
//         EXTI->PR = (1u << 12);
//         EXTI->IMR &= ~(1u << 12);  /* disable retrigger */
//         cnt_cs++;
//         DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;
//         EXTI->IMR |=  (1u << 12);  /* re-enable */
//     }
// }

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 12))
    {
        EXTI->PR = (1u << 12);
        cnt_cs++;
        // DMA1_Stream3->NDTR = SPI2_TRANSACTION_BYTES;  /* COMMENT OUT */
    }
}
