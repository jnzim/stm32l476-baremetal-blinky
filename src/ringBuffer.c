#include "ringBuffer.h"
#include "stm32f4xx.h"

RingBuffer ring;

void ring_init(void)
{
}

void ring_reset(void) {
    __disable_irq();
    ring.write_idx = 0;
    ring.read_idx  = 0;
    ring.count     = 0;
    __enable_irq();
}

void ring_push(const TrajSample* s)
{
    if (ring.count >= RING_BUFFER_SIZE) { return ; } // overflow TODO: disable, errror

    ring.buf[ring.write_idx]    = *s;
    ring.write_idx              = (ring.write_idx + 1) % RING_BUFFER_SIZE;
    ring.count++;
}
int ring_pop(TrajSample* s)   // returns 1 if sample available, 0 if empty
{
    if (ring.count < 1) { return 0; } // verflow TODO: disable, errror
   
    *s = ring.buf[ring.read_idx];
    ring.read_idx = (ring.read_idx + 1) % RING_BUFFER_SIZE;
    ring.count--;
    return 1;

}
uint32_t ring_count(void)
{
    return ring.count;
}