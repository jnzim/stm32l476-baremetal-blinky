#include "spi.h"
#include "protocol.h"
#include "ringBuffer.h"
#include "drive.h"
#include "loops.h"
#include "stm32f4xx.h"
#include <string.h>

volatile uint32_t cnt_data      = 0;
volatile uint32_t cnt_error     = 0;
volatile uint32_t cnt_isr       = 0;
volatile uint32_t cnt_cs        = 0;
volatile uint8_t  dbg_rx0       = 0;
volatile uint32_t cnt_block_hdr = 0;

static uint8_t rx_buf[32];
static uint8_t tx_buf[32];   /* legacy, unused — kept for spi_set_tx() ABI */

// ── Telemetry double-buffer ───────────────────────────────────────────────────
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

static uint8_t crc8_xor(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

static void rx_dma_rearm(void)
{
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);
    DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    DMA1_Stream3->M0AR = (uint32_t)rx_buf;
    DMA1_Stream3->NDTR = 32;
    DMA1_Stream3->CR  |= DMA_SxCR_EN;
}

static void tx_dma_rearm(void)
{
    /* Snapshot which slot is currently published. Single-byte read of a
     * volatile uint8_t is atomic on Cortex-M4, so no race with the
     * SysTick producer's flip. */
    uint8_t slot = telem_write_idx;

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[slot];
    DMA1_Stream4->NDTR = 32;
    DMA1_Stream4->CR  |= DMA_SxCR_EN;
}

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PB12 NSS, PB13 SCK, PB14 MISO, PB15 MOSI — AF5 */
    GPIOB->MODER  &= ~((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));
    GPIOB->MODER  |=  ((2u<<24)|(2u<<26)|(2u<<28)|(2u<<30));
    GPIOB->AFR[1] &= ~((0xFu<<16)|(0xFu<<20)|(0xFu<<24)|(0xFu<<28));
    GPIOB->AFR[1] |=  ((5u<<16)|(5u<<20)|(5u<<24)|(5u<<28));
    GPIOB->OSPEEDR|=  ((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));

    /* EXTI12 on CS falling edge */
    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (1u   << 0);
    EXTI->FTSR |=  (1u << 12);
    EXTI->RTSR &= ~(1u << 12);
    EXTI->IMR  |=  (1u << 12);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    SPI2->CR1 = 0;
    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;   /* both directions on DMA */

    /* ── SPI2 RX DMA — Stream 3, Channel 0, peripheral → memory ───────── */
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);
    DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)rx_buf;
    DMA1_Stream3->NDTR = 32;
    DMA1_Stream3->CR   = (0u << DMA_SxCR_CHSEL_Pos) |
                         (0u << DMA_SxCR_DIR_Pos)   |   /* P → M */
                         DMA_SxCR_MINC              |
                         DMA_SxCR_TCIE;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /* ── SPI2 TX DMA — Stream 4, Channel 0, memory → peripheral ───────── *
     * No TCIE — we re-arm from EXTI on the next CS edge.                 */
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[0];
    DMA1_Stream4->NDTR = 32;
    DMA1_Stream4->CR   = (0u << DMA_SxCR_CHSEL_Pos) |
                         (1u << DMA_SxCR_DIR_Pos)   |   /* M → P */
                         DMA_SxCR_MINC;
    DMA1_Stream4->CR |= DMA_SxCR_EN;

    memset(tx_buf, 0xAB, 32);
    SPI2->CR1 = SPI_CR1_SPE;
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 12)) {
        EXTI->PR = (1u << 12);
        rx_dma_rearm();
        tx_dma_rearm();
        cnt_cs++;
    }
}

void spi_set_tx(const uint8_t *data)
{
    memcpy(tx_buf, data, 32);   /* legacy, no-op for TX path now */
}

/* Producer side of the telemetry double-buffer. Called from SysTick.
 * memcpy into the non-active slot first, then publish via index flip.
 * EXTI's tx_dma_rearm() reads telem_write_idx atomically; if it lands
 * mid-update it sees the old slot (old but coherent data), never torn. */
void spi_update_telem(const TelemetryFrame *frame)
{
    uint8_t write_slot = telem_write_idx ^ 1u;
    memcpy((void*)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));
    telem_write_idx = write_slot;
}

// ─────────────────────────────────────────────────────────────────────────────
// DMA1_Stream3_IRQHandler — SPI2 RX DMA transfer complete, NVIC priority 2
//
// Owns (writes):  ring (push side), drive request flags, samples_consumed
//                 (zero on BLOCK_HDR), first_sample_ready, cnt_data, cnt_error
// Reads:          rx_buf (DMA-filled, atomic snapshot via memcpy)
// Preempts:       SysTick, anything at lower priority
// Preempted by:   TIM1 (priority 1), EXTI15_10 (priority 0)
// ─────────────────────────────────────────────────────────────────────────────
void DMA1_Stream3_IRQHandler(void)
{
    cnt_isr++;
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    if (DMA1_Stream3->NDTR != 0) {
        rx_dma_rearm();
        return;
    }

    uint8_t local[32];
    memcpy(local, rx_buf, 32);
    rx_dma_rearm();

    dbg_rx0 = local[0];

    switch (local[0]) {
        case SPI2_OP_DATA:
            if (crc8_xor(local, 9) != local[9]) { cnt_error++; break; }
            {
                TrajSample s;
                memcpy(&s.pos_cmd, &local[1], sizeof(int32_t));
                memcpy(&s.vel_cmd, &local[5], sizeof(int32_t));
                ring_push(&s);
                cnt_data++;
            }
            break;

        case SPI2_OP_BLOCK_HDR:
            if (crc8_xor(local, 3) != local[3]) { cnt_error++; break; }
            first_sample_ready = 0;
            ring_reset();
            samples_consumed = 0;
            cnt_block_hdr++;
            memset((void*)&telem_buf[1], 0, sizeof(TelemetryFrame));
            drive_request_servo_on();
            break;

        case SPI2_OP_OPEN_LOOP:
            if (crc8_xor(local, 9) != local[9]) { cnt_error++; break; }
            {
                float v_mag, d_theta;
                memcpy(&v_mag,   &local[1], sizeof(float));
                memcpy(&d_theta, &local[5], sizeof(float));
                drive_request_open_loop(v_mag, d_theta);
            }
            break;

        case SPI2_OP_STOP:
            if (crc8_xor(local, 1) != local[1]) { cnt_error++; break; }
            drive_request_stop();
            break;

        case SPI2_OP_TELEM_REQ:
            /* No-op — telemetry is returned on every full-duplex
             * transaction regardless of opcode. */
            break;

        case SPI2_OP_READY_ACK:
            break;

        default:
            cnt_error++;
            break;
    }
}