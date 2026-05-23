#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include "protocol.h"

#define RING_BUFFER_SIZE 4096u                  /* MUST be a power of two */
#define RING_MASK        (RING_BUFFER_SIZE - 1u)

typedef struct {
    TrajSample        buf[RING_BUFFER_SIZE];
    volatile uint32_t write_idx;   /* monotonic, owned by writer (DMA ISR) */
    volatile uint32_t read_idx;    /* monotonic, owned by reader (SysTick) */
} RingBuffer;

extern RingBuffer ring;

void     ring_init(void);
void     ring_reset(void);           /* writer-side only — see comment in .c */
void     ring_push(const TrajSample* s);
int      ring_pop(TrajSample* s);    /* 1 = got sample, 0 = empty */
uint32_t ring_count(void);
#endif