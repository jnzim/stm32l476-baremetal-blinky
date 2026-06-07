#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include "protocol.h"

#define RING_BUFFER_SIZE 256u  /* slots: 128 × 32 bytes = 4096 bytes total */
#define RING_MASK 0xFFu  /* 256 - 1 in binary */

typedef struct {
    TrajSlot        buf[RING_BUFFER_SIZE];
    volatile uint32_t write_idx;   /* monotonic, owned by writer (DMA ISR) */
    volatile uint32_t read_idx;    /* monotonic, owned by reader (SysTick) */
} RingBuffer;

extern RingBuffer ring;
extern uint32_t cnt_push;
extern uint32_t cnt_pop;
extern uint32_t cnt_pop_fail;

void     ring_init(void);
void     ring_reset(void);           /* writer-side only — see comment in .c */
void     ring_push(const TrajSlot* s);
int      ring_pop(TrajSlot* s);    /* 1 = got sample, 0 = empty */
uint32_t ring_count(void);
uint8_t  ring_get_crc_error(void);

#endif