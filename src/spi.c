// spi.c — SPI2 slave TX/RX DMA sysid latest-sample bring-up
//
// Double-buffer fix:
//   spi_tx_buf[2] holds two complete SysIdSample frames.
//   TIM1 ISR writes to the inactive buffer (index ^ 1).
//   EXTI12 (CS falling edge) snapshots the write index and
//   rearms TX DMA to point at the just-completed buffer.
//   DMA never reads a buffer while the ISR is writing it.

#include "spi.h"
#include "board_f411.h"
#include "protocol.h"
#include "stm32f4xx.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* =============================================================================
 * Legacy globals
 * =============================================================================*/

volatile uint32_t cnt_data      = 0;
volatile uint32_t cnt_error     = 0;
volatile uint32_t cnt_telem     = 0;
volatile uint32_t cnt_block_hdr = 0;
volatile uint32_t cnt_cs        = 0;

volatile uint8_t ready_asserted = 0;

volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

volatile uint16_t expected_samples = 0;


/* =============================================================================
 * SPI DMA buffers
 * =============================================================================*/

static volatile uint8_t spi2_rx_buf[SPI2_TRANSACTION_BYTES];

/*
 * Ping-pong TX buffers.
 *
 * spi_tx_buf[spi_tx_write_idx ^ 1] = buffer TIM1 ISR is currently writing.
 * spi_tx_buf[spi_tx_write_idx]     = buffer DMA is currently reading.
 *
 * EXTI12 flips spi_tx_read_idx to match spi_tx_write_idx at CS falling edge,
 * then rearms TX DMA to point at that buffer for the new transaction.
 */
static SysIdSample spi_tx_buf[2];
static volatile uint8_t spi_tx_write_idx = 0;   /* index last fully written */


/* =============================================================================
 * Sysid sequence
 * =============================================================================*/

static volatile uint32_t sysid_seq = 0;


typedef char SysIdSample_must_be_32_bytes[
    (sizeof(SysIdSample) == SPI2_TRANSACTION_BYTES) ? 1 : -1
];

typedef char TelemetryFrame_must_be_32_bytes[
    (sizeof(TelemetryFrame) == SPI2_TRANSACTION_BYTES) ? 1 : -1
];


/* =============================================================================
 * DMA flag clear
 * =============================================================================*/

static void spi2_dma_clear_flags(void)
{
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    DMA1->HIFCR = DMA_HIFCR_CTCIF4  |
                  DMA_HIFCR_CHTIF4  |
                  DMA_HIFCR_CTEIF4  |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;
}


/* =============================================================================
 * Public sysid update API
 *
 * Called from TIM1 ISR at 20 kHz.
 * Writes to the inactive buffer (index ^ 1).
 * __DMB() (Data Memory Barrier) ensures all field writes are visible
 * before the index flip — prevents the compiler or CPU from reordering
 * the index write ahead of the data writes.
 * =============================================================================*/

void spi_sysid_update_latest(int16_t ia_mA,
                             int16_t ib_mA,
                             int16_t ic_mA,
                             int16_t id_mA,
                             int16_t iq_mA,
                             int16_t vd_mV,
                             int16_t vq_mV,
                             int16_t theta_mrad,
                             uint16_t adc_a,
                             uint16_t adc_b,
                             uint16_t adc_c,
                             uint16_t flags)
{
    uint8_t idx = spi_tx_write_idx ^ 1u;   /* inactive buffer */
    SysIdSample *s = &spi_tx_buf[idx];

    s->t          = sysid_seq++;
    s->ia_mA      = ia_mA;
    s->ib_mA      = ib_mA;
    s->ic_mA      = ic_mA;
    s->id_mA      = id_mA;
    s->iq_mA      = iq_mA;
    s->vd_mV      = vd_mV;
    s->vq_mV      = vq_mV;
    s->theta_mrad = theta_mrad;
    s->adc_a      = adc_a;
    s->adc_b      = adc_b;
    s->adc_c      = adc_c;
    s->flags      = flags;
    s->crc        = 0;
    s->pad        = 0;

    __DMB();                        /* all writes visible before index flip */
    spi_tx_write_idx = idx;         /* publish: this buffer is now complete */
}


/* =============================================================================
 * SPI init
 * =============================================================================*/

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PC13 READY output, idle high */
    GPIOC->MODER   &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->MODER   |=  (1u << (READY_RING_REFILL * 2));
    GPIOC->OTYPER  &= ~(1u << READY_RING_REFILL);
    GPIOC->OSPEEDR &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->OSPEEDR |=  (1u << (READY_RING_REFILL * 2));
    GPIOC->PUPDR   &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->BSRR     =  (1u << READY_RING_REFILL);

    /* SPI2 pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI */
    GPIOB->MODER &= ~((3u << (PIN_RPI_NSS  * 2)) |
                      (3u << (PIN_RPI_SCK  * 2)) |
                      (3u << (PIN_RPI_MISO * 2)) |
                      (3u << (PIN_RPI_MOSI * 2)));

    GPIOB->MODER |=  ((2u << (PIN_RPI_NSS  * 2)) |
                      (2u << (PIN_RPI_SCK  * 2)) |
                      (2u << (PIN_RPI_MISO * 2)) |
                      (2u << (PIN_RPI_MOSI * 2)));

    GPIOB->AFR[1] &= ~((0xFu << ((PIN_RPI_NSS  - 8u) * 4u)) |
                       (0xFu << ((PIN_RPI_SCK  - 8u) * 4u)) |
                       (0xFu << ((PIN_RPI_MISO - 8u) * 4u)) |
                       (0xFu << ((PIN_RPI_MOSI - 8u) * 4u)));

    GPIOB->AFR[1] |=  ((5u << ((PIN_RPI_NSS  - 8u) * 4u)) |
                       (5u << ((PIN_RPI_SCK  - 8u) * 4u)) |
                       (5u << ((PIN_RPI_MISO - 8u) * 4u)) |
                       (5u << ((PIN_RPI_MOSI - 8u) * 4u)));

    GPIOB->OSPEEDR |= ((3u << (PIN_RPI_NSS  * 2)) |
                       (3u << (PIN_RPI_SCK  * 2)) |
                       (3u << (PIN_RPI_MISO * 2)) |
                       (3u << (PIN_RPI_MOSI * 2)));

    GPIOB->PUPDR &= ~(3u << (PIN_RPI_NSS * 2));
    GPIOB->PUPDR |=  (1u << (PIN_RPI_NSS * 2));

    /* Disable SPI and DMA before reconfig */
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    spi2_dma_clear_flags();

    /*
     * Boot sample in buf[0].
     * flags = 0x1234 marks boot/fake data.
     * spi_tx_write_idx starts at 0 so EXTI12 will hand buf[0] to DMA
     * on the first CS edge.
     */
    spi_sysid_update_latest(100, 200, 300,
                            400, 500,
                            600, 700,
                            800,
                            2048, 2050, 2046,
                            0x1234);

    /*
     * RX DMA: Pi MOSI -> SPI2->DR -> spi2_rx_buf
     * CIRC (circular) is fine for RX — we just count transactions.
     */
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

    /*
     * TX DMA: spi_tx_buf[spi_tx_write_idx] -> SPI2->DR -> Pi MISO
     *
     * No CIRC — each transaction is rearmed explicitly by EXTI12.
     * M0AR is set here to buf[0] (boot sample) and updated each CS edge.
     */
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&spi_tx_buf[0];
    DMA1_Stream4->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0             |   /* memory -> peripheral */
        DMA_SxCR_MINC;                 /* no CIRC — rearmed per transaction */

    DMA1_Stream4->CR |= DMA_SxCR_EN;

    /* SPI2 slave, Mode 1: CPOL=0, CPHA=1, SSM=1, SSI=0 */
    SPI2->CR1 = SPI_CR1_CPHA | SPI_CR1_SSM;

    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    __DMB();

    SPI2->CR1 |= SPI_CR1_SPE;

    /* CS edge IRQ on PB12 / EXTI12, falling edge only */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (0x1u << 0);   /* PB12 -> EXTI12 */

    EXTI->FTSR |=  (1u << PIN_RPI_NSS);
    EXTI->RTSR &= ~(1u << PIN_RPI_NSS);
    EXTI->IMR  |=  (1u << PIN_RPI_NSS);
    EXTI->PR    =  (1u << PIN_RPI_NSS);

    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);


}


/* =============================================================================
 * Legacy telemetry API — kept for link compatibility
 * =============================================================================*/

void spi_update_telem(const TelemetryFrame *frame)
{
    (void)frame;
}


/* =============================================================================
 * SPI RX complete IRQ (DMA1 Stream3)
 *
 * Fires after Pi clocks 32 bytes in.
 * Just counts transactions. TX is rearmed by EXTI12.
 * =============================================================================*/

void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) {
        return;
    }

    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    cnt_telem++;
}


/* =============================================================================
 * CS falling edge IRQ (EXTI12)
 *
 * Fires when Pi asserts CS (PB12 falls).
 * Snapshots spi_tx_write_idx and rearms TX DMA to point at that buffer.
 * This must happen before the first SCK edge, so EXTI12 stays at priority 0.
 *
 * Rearm sequence (mandatory order):
 *   1. Disable Stream4.
 *   2. Wait for EN to clear.
 *   3. Clear TX DMA flags.
 *   4. Set M0AR to the completed buffer.
 *   5. Reload NDTR (Number of Data items to Transfer).
 *   6. Re-enable Stream4.
 * =============================================================================*/

 volatile uint32_t cnt_tx_rearm = 0;
void EXTI15_10_IRQHandler(void)
{
    
    if (!(EXTI->PR & (1u << PIN_RPI_NSS))) {
        return;
    }
    cnt_tx_rearm++;
    EXTI->PR = (1u << PIN_RPI_NSS);
    cnt_cs++;

    /* Snapshot the last fully-written buffer index */
    uint8_t idx = spi_tx_write_idx;

    /* Rearm TX DMA (DMA1 Stream4) */
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    DMA1->HIFCR = DMA_HIFCR_CTCIF4  |
                  DMA_HIFCR_CHTIF4  |
                  DMA_HIFCR_CTEIF4  |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;

    DMA1_Stream4->M0AR = (uint32_t)&spi_tx_buf[idx];
    DMA1_Stream4->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream4->CR  |= DMA_SxCR_EN;
}