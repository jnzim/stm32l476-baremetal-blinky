#include "ringBuffer.h"
#include "stm32f4xx.h"

RingBuffer ring;

void ring_init(void)
{
    ring.write_idx = 0;
    ring.read_idx  = 0;
}

/* Writer-side only. The Pi protocol guarantees the reader path is dormant
 * here: BLOCK_HDR arrives while drive SM is in IDLE, so drive_is_servo_on()
 * is false and SysTick will not pop. Setting write_idx = read_idx empties
 * the ring atomically from the writer's perspective without touching the
 * reader's counter. */
void ring_reset(void)
{
    ring.write_idx = ring.read_idx;
}

/* Single writer (DMA1_Stream3 ISR, priority 2). */
/* DMA ISR — called when SPI block arrives */
/* ringBuffer.c */
static uint8_t traj_crc_error = 0;


uint32_t cnt_push = 0;
uint32_t cnt_pop = 0;
uint32_t cnt_pop_fail = 0;
uint32_t cnt_crc_fail = 0;

void ring_push(const TrajSlot* s)
{
    uint32_t w = ring.write_idx;
    uint32_t r = ring.read_idx;
    if ((w - r) >= RING_BUFFER_SIZE) return;
    
    /* Validate opcode and CRC */
    if (s->opcode != SPI2_OP_DATA)
    {
        cnt_crc_fail++;
        return;
    }
    
    uint16_t crc_calc = crc16_calc((uint8_t*)s, TRAJ_CRC_LEN);
    if (crc_calc != s->crc16)
    {
        cnt_crc_fail++;
        return;
    }
    
    ring.buf[w & RING_MASK] = *s;
    __DMB();
    ring.write_idx = w + 1u;
    cnt_push++;
}

//Single reader (SysTick, priority 15). 

int ring_pop(TrajSlot* s)
{
    uint32_t r = ring.read_idx;
    uint32_t w = ring.write_idx;
    if (w == r)
    {
        cnt_pop_fail++;
        return 0;
    }
    __DMB();
    *s = ring.buf[r & RING_MASK];
    __DMB();
    ring.read_idx = r + 1u;
    cnt_pop++;
    return 1;
}


/* Slightly stale by definition; correct for READY-threshold and telemetry.
 * Monotonic uint32_t subtraction works across wrap as long as the live
 * difference is well below BUFFER_SIZE. */
uint32_t ring_count(void)
{
    uint32_t w = ring.write_idx;
    uint32_t r = ring.read_idx;
    return w - r;
}

uint8_t ring_get_crc_error(void)
{
    uint8_t err = traj_crc_error;
    traj_crc_error = 0;
    return err;
}