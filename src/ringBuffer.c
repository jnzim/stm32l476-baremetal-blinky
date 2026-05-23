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
void ring_push(const TrajSample* s)
{
    uint32_t w = ring.write_idx;
    uint32_t r = ring.read_idx;                  /* snapshot reader counter */
    if ((w - r) >= RING_BUFFER_SIZE) {
        /* TODO: wire cnt_push_dropped++ here if you want overflow telemetry */
        return;
    }
    ring.buf[w & RING_MASK] = *s;
    __DMB();                                     /* data committed before index publish */
    ring.write_idx = w + 1u;
}

/* Single reader (SysTick, priority 15). */
int ring_pop(TrajSample* s)
{
    uint32_t r = ring.read_idx;
    uint32_t w = ring.write_idx;                 /* snapshot writer counter */
    if (w == r) return 0;                        /* empty */
    __DMB();                                     /* see data committed before reading buf */
    *s = ring.buf[r & RING_MASK];
    __DMB();                                     /* read finishes before publishing new read_idx */
    ring.read_idx = r + 1u;
    return 1;
}

/* Slightly stale by definition; correct for READY-threshold and telemetry.
 * Monotonic uint32_t subtraction works across wrap as long as the live
 * difference is well below 2^31 — always true here (max 4096). */
uint32_t ring_count(void)
{
    uint32_t w = ring.write_idx;
    uint32_t r = ring.read_idx;
    return w - r;
}