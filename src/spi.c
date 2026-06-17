// spi.c — SPI2 slave TX/RX DMA sysid latest-sample bring-up
//
// Current goal:
//   RPi master clocks 32 bytes.
//   STM slave returns one packed 32-byte SysIdSample on MISO.
//   TIM1 ISR calls spi_sysid_update_latest(...).
//
// SPI2 pins:
//   PB12 = NSS / CS from Pi
//   PB13 = SCK from Pi
//   PB14 = MISO to Pi
//   PB15 = MOSI from Pi
//
// DMA:
//   DMA1 Stream3 Ch0 = SPI2_RX
//   DMA1 Stream4 Ch0 = SPI2_TX

#include "spi.h"
#include "board_f411.h"
#include "protocol.h"
#include "stm32f4xx.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* =============================================================================
 * Legacy globals kept so old code still links
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
static volatile uint8_t spi_test_tx[SPI2_TRANSACTION_BYTES];


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


/* =============================================================================
 * Public sysid update API
 *
 * Call this from TIM1 ISR.
 *
 * Values are fixed-point:
 *   currents: mA
 *   voltages: mV
 *   theta:    mrad
 *
 * Bring-up behavior:
 *   Writes directly into spi_test_tx[], the active TX DMA source buffer.
 *
 * This can theoretically tear if the RPi clocks SPI while memcpy is happening.
 * For latest-sample bring-up, that is acceptable. We can double-buffer later.
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
    SysIdSample s;

    s.t          = sysid_seq++;
    s.ia_mA      = ia_mA;
    s.ib_mA      = ib_mA;
    s.ic_mA      = ic_mA;
    s.id_mA      = id_mA;
    s.iq_mA      = iq_mA;
    s.vd_mV      = vd_mV;
    s.vq_mV      = vq_mV;
    s.theta_mrad = theta_mrad;
    s.adc_a      = adc_a;
    s.adc_b      = adc_b;
    s.adc_c      = adc_c;
    s.flags      = flags;
    s.crc        = 0;
    s.pad        = 0;

    memcpy((void *)spi_test_tx,
           (const void *)&s,
           sizeof(SysIdSample));

    __DMB();
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

    /*
     * PC13 READY_RING_REFILL output, active-low, idle high.
     * Kept from old working branch.
     */
    GPIOC->MODER   &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->MODER   |=  (1u << (READY_RING_REFILL * 2));
    GPIOC->OTYPER  &= ~(1u << READY_RING_REFILL);
    GPIOC->OSPEEDR &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->OSPEEDR |=  (1u << (READY_RING_REFILL * 2));
    GPIOC->PUPDR   &= ~(3u << (READY_RING_REFILL * 2));
    GPIOC->BSRR     =  (1u << READY_RING_REFILL);

    /*
     * SPI2 pins:
     *   PB12 = NSS / CS from Pi
     *   PB13 = SCK from Pi
     *   PB14 = MISO to Pi
     *   PB15 = MOSI from Pi
     */
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

    /*
     * Keep pull-up on NSS.
     */
    GPIOB->PUPDR &= ~(3u << (PIN_RPI_NSS * 2));
    GPIOB->PUPDR |=  (1u << (PIN_RPI_NSS * 2));

    /*
     * Disable SPI and DMA before reconfig.
     */
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {
    }

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {
    }

    spi2_dma_clear_flags();

    /*
     * Boot sample.
     *
     * This should only appear before TIM1 starts calling
     * spi_sysid_update_latest().
     *
     * flags = 0x1234 marks boot/fake data.
     * Your TIM1 ISR should use a different flag, for example 0x8000/0x8001.
     */
    spi_sysid_update_latest(100, 200, 300,
                            400, 500,
                            600, 700,
                            800,
                            2048, 2050, 2046,
                            0x1234);

    /*
     * RX DMA:
     *   Pi MOSI -> SPI2->DR -> spi2_rx_buf
     *
     * Kept mostly for transaction counting/debug.
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
     * TX DMA:
     *   spi_test_tx[0..31] -> SPI2->DR -> Pi MISO
     *
     * The buffer contents are updated directly by spi_sysid_update_latest().
     */
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)spi_test_tx;
    DMA1_Stream4->NDTR = SPI2_TRANSACTION_BYTES;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0             |   // memory -> peripheral
        DMA_SxCR_MINC              |   // walk through spi_test_tx[]
        DMA_SxCR_CIRC;                 // repeat 32-byte latest sample

    DMA1_Stream4->CR |= DMA_SxCR_EN;

    /*
     * SPI2 slave, Mode 1:
     *   CPOL = 0
     *   CPHA = 1
     *   MSTR = 0
     *
     * Keep old software NSS behavior.
     */
    SPI2->CR1 = SPI_CR1_CPHA | SPI_CR1_SSM;

    /*
     * Enable DMA requests from SPI2.
     */
    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    __DMB();

    SPI2->CR1 |= SPI_CR1_SPE;

    /*
     * Keep old CS edge IRQ/counter.
     */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (0x1u << 0);   // PB12 -> EXTI12

    EXTI->FTSR |=  (1u << PIN_RPI_NSS);
    EXTI->RTSR &= ~(1u << PIN_RPI_NSS);
    EXTI->IMR  |=  (1u << PIN_RPI_NSS);
    EXTI->PR    =  (1u << PIN_RPI_NSS);

    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/* =============================================================================
 * Legacy telemetry API
 *
 * Kept so other code can still link.
 * Not used for sysid bring-up.
 * =============================================================================*/

void spi_update_telem(const TelemetryFrame *frame)
{
    (void)frame;
}


/* =============================================================================
 * SPI RX complete IRQ
 *
 * Fires after the Pi clocks 32 bytes.
 * No TX reload here anymore.
 * TX buffer is updated directly by spi_sysid_update_latest().
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
 * CS falling edge IRQ
 * =============================================================================*/

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << PIN_RPI_NSS))
    {
        EXTI->PR = (1u << PIN_RPI_NSS);
        cnt_cs++;
    }
}