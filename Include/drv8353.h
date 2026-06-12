#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DRV8353_REG_FAULT_STATUS_1   0x00u
#define DRV8353_REG_VGS_STATUS_2     0x01u
#define DRV8353_REG_DRIVER_CONTROL   0x02u
#define DRV8353_REG_GATE_DRIVE_HS    0x03u
#define DRV8353_REG_GATE_DRIVE_LS    0x04u
#define DRV8353_REG_OCP_CONTROL      0x05u
#define DRV8353_REG_CSA_CONTROL      0x06u
#define DRV8353_REG_RESERVED_CAL     0x07u

typedef struct
{
    uint16_t fault_status_1;
    uint16_t vgs_status_2;
    uint16_t driver_control;
    bool n_fault_pin;
} Drv8353Status;

void drv8353_init(void);

uint16_t drv8353_transfer16(uint16_t tx);
uint16_t drv8353_read_reg(uint8_t addr);
uint16_t drv8353_write_reg(uint8_t addr, uint16_t data);

bool drv8353_configure(void);
void drv8353_clear_faults(void);
Drv8353Status drv8353_read_status(void);
bool drv8353_spi_self_test(void);
bool drv8353_write_read_test(void);

void drv8353_enable(bool enable);
bool drv8353_fault_pin_ok(void);
bool drv8353_fault_pin_active(void);
