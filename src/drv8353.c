// drv8353.c — DRV8353RS SPI driver
// STM32F411 bare metal
//
// Uses board_f411.h pin map:
//
// SPI1:
//   PA5 = DRV SCK
//   PA6 = DRV MISO
//   PA7 = DRV MOSI
//
// GPIO:
//   PC8 = DRV nSCS   (moved from PB6 for LA probe access, CN10 pin 2)
//   PB0 = DRV ENABLE
//   PB1 = DRV nFAULT
//
// DRV8353 SPI:
//   16-bit frames
//   SPI mode 1: CPOL=0, CPHA=1
//   MSB first
//   bit 15     = R/W, 1 = read, 0 = write
//   bits 14:11 = address
//   bits 10:0  = data
//
//   Read data returns in bits 10:0 of the SAME frame (not pipelined).

#include "drv8353.h"
#include "board_f411.h"
#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>

// =============================================================================
// GPIO mapping
// =============================================================================

#define DRV_CS_PORT        GPIOC
#define DRV_CS_PIN         PIN_DRV_CS

#define DRV_EN_PORT        GPIOB
#define DRV_EN_PIN         PIN_DRV_ENABLE

#define DRV_NFAULT_PORT    GPIOB
#define DRV_NFAULT_PIN     PIN_DRV_NFAULT

// =============================================================================
// Bit helpers
// =============================================================================

#define BIT(n)             (1u << (n))

#define DRV_SPI_READ       0x8000u
#define DRV_SPI_WRITE      0x0000u
#define DRV_ADDR_SHIFT     11u
#define DRV_DATA_MASK      0x07FFu

// Driver Control register, address 0x02.
// Bit 0 = CLR_FLT.
#define DRV8353_CLR_FLT    BIT(0)

// =============================================================================
// Debug globals for SPI write/read test
// =============================================================================
//
// Watch these in debugger or export them through telemetry.
//
// Note: DRV8353 reads are same-frame (not pipelined). The *_ignored values
// should always equal their trusted counterparts. If they ever differ,
// suspect a CS-edge or setup-timing problem in the transfer.

volatile uint16_t g_drv_wr_original;
volatile uint16_t g_drv_wr_test_value;
volatile uint16_t g_drv_wr_readback_ignored;
volatile uint16_t g_drv_wr_readback;
volatile uint16_t g_drv_wr_restored_ignored;
volatile uint16_t g_drv_wr_restored;

// =============================================================================
// Local GPIO helpers
// =============================================================================

static inline void drv_cs_low(void)
{
    DRV_CS_PORT->BSRR = (uint32_t)BIT(DRV_CS_PIN) << 16;
}

static inline void drv_cs_high(void)
{
    DRV_CS_PORT->BSRR = BIT(DRV_CS_PIN);
}

static inline void drv_enable_low(void)
{
    DRV_EN_PORT->BSRR = (uint32_t)BIT(DRV_EN_PIN) << 16;
}

static inline void drv_enable_high(void)
{
    DRV_EN_PORT->BSRR = BIT(DRV_EN_PIN);
}

static inline bool drv_nfault_read(void)
{
    return (DRV_NFAULT_PORT->IDR & BIT(DRV_NFAULT_PIN)) != 0u;
}

static void drv_delay_cycles(volatile uint32_t cycles)
{
    while (cycles--)
    {
        __NOP();
    }
}

// =============================================================================
// Public init
// =============================================================================

void drv8353_init(void)
{
    // -------------------------------------------------------------------------
    // Enable clocks
    // -------------------------------------------------------------------------

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;   // PC8 = nSCS
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    __DSB();

    // -------------------------------------------------------------------------
    // PA5 / PA6 / PA7 = SPI1 AF5
    // -------------------------------------------------------------------------

    GPIOA->MODER &= ~((3u << (PIN_DRV_SCK  * 2u)) |
                      (3u << (PIN_DRV_MISO * 2u)) |
                      (3u << (PIN_DRV_MOSI * 2u)));

    GPIOA->MODER |=  ((2u << (PIN_DRV_SCK  * 2u)) |
                      (2u << (PIN_DRV_MISO * 2u)) |
                      (2u << (PIN_DRV_MOSI * 2u)));

    GPIOA->AFR[0] &= ~((0xFu << (PIN_DRV_SCK  * 4u)) |
                       (0xFu << (PIN_DRV_MISO * 4u)) |
                       (0xFu << (PIN_DRV_MOSI * 4u)));

    GPIOA->AFR[0] |=  ((5u << (PIN_DRV_SCK  * 4u)) |
                       (5u << (PIN_DRV_MISO * 4u)) |
                       (5u << (PIN_DRV_MOSI * 4u)));

    // LOW speed edges for SCK/MOSI — critical on breadboard/long jumpers.
    // High-speed (ns) edges ring on unterminated wires; the DRV counts the
    // ringing as extra SCLK edges and discards write frames (!=16 clocks).
    // Low speed is ample for <=1 MHz bus rates. MISO is an input; N/A.
    GPIOA->OSPEEDR &= ~((3u << (PIN_DRV_SCK  * 2u)) |
                        (3u << (PIN_DRV_MISO * 2u)) |
                        (3u << (PIN_DRV_MOSI * 2u)));

    // Push-pull outputs for SCK/MOSI. MISO setting does not matter as input.
    GPIOA->OTYPER &= ~(BIT(PIN_DRV_SCK) | BIT(PIN_DRV_MOSI));

    // DRV8353 SDO is open-drain. External pull-up is preferred.
    // Internal pull-up is enabled as a backup.
    GPIOA->PUPDR &= ~(3u << (PIN_DRV_MISO * 2u));
    GPIOA->PUPDR |=  (1u << (PIN_DRV_MISO * 2u));

    // -------------------------------------------------------------------------
    // PC8 = DRV nSCS, manual chip select, idle high
    // -------------------------------------------------------------------------

    GPIOC->MODER &= ~(3u << (DRV_CS_PIN * 2u));
    GPIOC->MODER |=  (1u << (DRV_CS_PIN * 2u));

    GPIOC->OTYPER &= ~BIT(DRV_CS_PIN);
    // LOW speed — same ringing rationale as SCK/MOSI.
    GPIOC->OSPEEDR &= ~(3u << (DRV_CS_PIN * 2u));
    GPIOC->PUPDR &= ~(3u << (DRV_CS_PIN * 2u));

    drv_cs_high();

    // -------------------------------------------------------------------------
    // PB0 = DRV ENABLE output, start disabled
    // -------------------------------------------------------------------------

    GPIOB->MODER &= ~(3u << (DRV_EN_PIN * 2u));
    GPIOB->MODER |=  (1u << (DRV_EN_PIN * 2u));

    GPIOB->OTYPER &= ~BIT(DRV_EN_PIN);
    GPIOB->OSPEEDR |= (3u << (DRV_EN_PIN * 2u));
    GPIOB->PUPDR &= ~(3u << (DRV_EN_PIN * 2u));

    drv_enable_low();

    // -------------------------------------------------------------------------
    // PB1 = DRV nFAULT input with pull-up
    // -------------------------------------------------------------------------

    GPIOB->MODER &= ~(3u << (DRV_NFAULT_PIN * 2u));

    GPIOB->PUPDR &= ~(3u << (DRV_NFAULT_PIN * 2u));
    GPIOB->PUPDR |=  (1u << (DRV_NFAULT_PIN * 2u));

    // -------------------------------------------------------------------------
    // SPI1 setup
    // -------------------------------------------------------------------------
    //
    // DRV8353:
    //   CPOL = 0
    //   CPHA = 1
    //   16-bit frame
    //   MSB first
    //
    // STM32F411 SPI1 is on APB2.
    // BR = 111 means f_PCLK / 256.
    // If APB2 = 84 MHz, SPI clock ≈ 328 kHz.
    // Conservative for breadboard + long jumpers + LA capacitance with
    // 4.7k pull-up on open-drain SDO. Bump to /128 or /64 once verified.
    // -------------------------------------------------------------------------

    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    SPI1->CR1 =
        SPI_CR1_MSTR |     // master
        SPI_CR1_SSM  |     // software slave management
        SPI_CR1_SSI  |     // internal NSS high
        SPI_CR1_CPHA |     // mode 1: CPHA=1, CPOL=0
        SPI_CR1_BR_2 |
        SPI_CR1_BR_1 |
        SPI_CR1_BR_0;

    // 16-bit data frame.
    SPI1->CR1 |= SPI_CR1_DFF;

    // Enable SPI.
    SPI1->CR1 |= SPI_CR1_SPE;

    // Enable the DRV after SPI/GPIO init.
    drv_enable_high();

    // Give DRV time to wake up.
    drv_delay_cycles(100000);
}

// =============================================================================
// SPI transfer
// =============================================================================

uint16_t drv8353_transfer16(uint16_t tx)
{
    uint16_t rx;

    // Clear any stale RX data before starting.
    if ((SPI1->SR & SPI_SR_RXNE) != 0u)
    {
        volatile uint16_t dummy = *((volatile uint16_t *)&SPI1->DR);
        (void)dummy;
    }

    // DRV wants SCLK low when nSCS changes.
    // SPI mode 1 with CPOL=0 gives idle-low clock.
    drv_cs_low();

    // Setup delay (nSCS low to first SCLK edge).
    drv_delay_cycles(50);

    while ((SPI1->SR & SPI_SR_TXE) == 0u)
    {
    }

    *((volatile uint16_t *)&SPI1->DR) = tx;

    while ((SPI1->SR & SPI_SR_RXNE) == 0u)
    {
    }

    rx = *((volatile uint16_t *)&SPI1->DR);

    // Critical: wait for the frame to fully clock out before raising CS.
    while ((SPI1->SR & SPI_SR_BSY) != 0u)
    {
    }

    // Hold delay before CS high.
    drv_delay_cycles(50);

    drv_cs_high();

    // nSCS minimum high time between words (>= 400 ns).
    drv_delay_cycles(100);

    return rx;
}

// =============================================================================
// Register access
// =============================================================================

uint16_t drv8353_read_reg(uint8_t addr)
{
    uint16_t tx;

    tx = DRV_SPI_READ |
         (((uint16_t)(addr & 0x0Fu)) << DRV_ADDR_SHIFT);

    return drv8353_transfer16(tx) & DRV_DATA_MASK;
}

uint16_t drv8353_write_reg(uint8_t addr, uint16_t data)
{
    uint16_t tx;

    tx = DRV_SPI_WRITE |
         (((uint16_t)(addr & 0x0Fu)) << DRV_ADDR_SHIFT) |
         (data & DRV_DATA_MASK);

    return drv8353_transfer16(tx) & DRV_DATA_MASK;
}

// =============================================================================
// Fault/status helpers
// =============================================================================

void drv8353_clear_faults(void)
{
    uint16_t ctrl;

    ctrl = drv8353_read_reg(DRV8353_REG_DRIVER_CONTROL);
    ctrl |= DRV8353_CLR_FLT;

    drv8353_write_reg(DRV8353_REG_DRIVER_CONTROL, ctrl);
}

Drv8353Status drv8353_read_status(void)
{
    Drv8353Status s;

    s.fault_status_1 = drv8353_read_reg(DRV8353_REG_FAULT_STATUS_1);
    s.vgs_status_2   = drv8353_read_reg(DRV8353_REG_VGS_STATUS_2);
    s.driver_control = drv8353_read_reg(DRV8353_REG_DRIVER_CONTROL);
    s.n_fault_pin    = drv_nfault_read();

    return s;
}

bool drv8353_spi_self_test(void)
{
    uint16_t fs1;
    uint16_t vgs;
    uint16_t ctl;

    fs1 = drv8353_read_reg(DRV8353_REG_FAULT_STATUS_1);
    vgs = drv8353_read_reg(DRV8353_REG_VGS_STATUS_2);
    ctl = drv8353_read_reg(DRV8353_REG_DRIVER_CONTROL);

    // Common broken-bus case:
    // SDO/MISO pulled high or floating high gives all ones.
    if ((fs1 == 0x07FFu) &&
        (vgs == 0x07FFu) &&
        (ctl == 0x07FFu))
    {
        return false;
    }

    // All zeros can be a valid no-fault state, but it can also mean MISO stuck low.
    // So we do not fail that here.
    return true;
}

void drv8353_enable(bool enable)
{
    if (enable)
    {
        drv_enable_high();
    }
    else
    {
        drv_enable_low();
    }
}

bool drv8353_fault_pin_ok(void)
{
    return drv_nfault_read();
}

bool drv8353_fault_pin_active(void)
{
    return !drv_nfault_read();
}

bool drv8353_write_read_test(void)
{
    const uint8_t reg = DRV8353_REG_CSA_CONTROL;

    uint16_t original;
    uint16_t test_value;
    uint16_t readback_ignored;
    uint16_t readback;
    uint16_t restored_ignored;
    uint16_t restored;

    original = drv8353_read_reg(reg) & DRV_DATA_MASK;

    // Flip bit 6 for temporary write test.
    test_value = original ^ (1u << 6);

    drv8353_write_reg(reg, test_value);
    drv_delay_cycles(1000);

    // Reads are same-frame on the DRV8353; these pairs should always match.
    readback_ignored = drv8353_read_reg(reg) & DRV_DATA_MASK;
    readback         = drv8353_read_reg(reg) & DRV_DATA_MASK;

    drv8353_write_reg(reg, original);
    drv_delay_cycles(1000);

    restored_ignored = drv8353_read_reg(reg) & DRV_DATA_MASK;
    restored         = drv8353_read_reg(reg) & DRV_DATA_MASK;

    // Save debug values so we can inspect what happened.
    g_drv_wr_original         = original;
    g_drv_wr_test_value       = test_value;
    g_drv_wr_readback_ignored = readback_ignored;
    g_drv_wr_readback         = readback;
    g_drv_wr_restored_ignored = restored_ignored;
    g_drv_wr_restored         = restored;

    if (readback != test_value)
    {
        return false;
    }

    if (restored != original)
    {
        return false;
    }

    return true;
}

// =============================================================================
// Deliberate register configuration
// =============================================================================
//
// Field encodings below are from the DRV8353 datasheet (SLVSDJ3) register
// tables. IDRIVE/VDS_LVL codes: VERIFY against your datasheet revision's
// tables once before first powered FET switching. Harmless until PWM wired.
//
// Configuration intent:
//   0x02  3x PWM mode (INHx = PWM, INLx = per-phase enable)
//   0x03  unlocked, IDRIVE HS 150 mA source / 300 mA sink (EVM-mandated max)
//   0x04  CBC on, TDRIVE 4000 ns, IDRIVE LS 150/300 mA
//   0x05  latched OCP for bring-up, 100 ns dead time, VDS at default 0.75 V
//   0x06  bidirectional CSA, GAIN = 40 V/V  <-- i_q scaling reference
//
// Current scaling with this config:
//   I_phase = (V_SOx - VREF/2) / (40 V/V * R_shunt)
//   Read R_shunt off the EVM schematic (RSx) before trusting absolute amps.
//   With 12-bit ADC @ 3.3 V: amps_per_count = 3.3/4096 / (40 * R_shunt)

#define DRV_CFG_DRIVER_CONTROL   0x0020u  // PWM_MODE=01 (3x), all else 0
#define DRV_CFG_GATE_DRIVE_HS    0x0333u  // LOCK=011, IDRIVEP=0011(150mA), IDRIVEN=0011(300mA)
#define DRV_CFG_GATE_DRIVE_LS    0x0733u  // CBC=1, TDRIVE=11(4000ns), 150/300mA
#define DRV_CFG_OCP_CONTROL      0x0119u  // TRETRY=0, DEAD_TIME=01(100ns), OCP_MODE=00(latched), OCP_DEG=01(4us), VDS_LVL=1001(0.75V)
#define DRV_CFG_CSA_CONTROL      0x02C3u  // VREF_DIV=1, CSA_GAIN=11(40V/V), SEN_LVL=11

bool drv8353_configure(void)
{
    // Order: gate drive + protection first, mode last.
    static const struct { uint8_t addr; uint16_t value; } cfg[] = {
        { DRV8353_REG_GATE_DRIVE_HS, DRV_CFG_GATE_DRIVE_HS },
        { DRV8353_REG_GATE_DRIVE_LS, DRV_CFG_GATE_DRIVE_LS },
        { DRV8353_REG_OCP_CONTROL,   DRV_CFG_OCP_CONTROL   },
        { DRV8353_REG_CSA_CONTROL,   DRV_CFG_CSA_CONTROL   },
        { DRV8353_REG_DRIVER_CONTROL, DRV_CFG_DRIVER_CONTROL },
    };

    bool ok = true;

    for (uint32_t i = 0; i < (sizeof(cfg) / sizeof(cfg[0])); i++)
    {
        drv8353_write_reg(cfg[i].addr, cfg[i].value);

        uint16_t rb = drv8353_read_reg(cfg[i].addr);

        // CLR_FLT (0x02 bit 0) self-clears; mask it out of comparison.
        uint16_t mask = (cfg[i].addr == DRV8353_REG_DRIVER_CONTROL)
                            ? (DRV_DATA_MASK & ~DRV8353_CLR_FLT)
                            : DRV_DATA_MASK;

        if ((rb & mask) != (cfg[i].value & mask))
        {
            ok = false;   // keep going; caller inspects full picture
        }
    }

    return ok;
}