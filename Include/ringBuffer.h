#pragma once
#include <stdint.h>
#include "protocol.h"

#define RING_BUFFER_SIZE 4096u

typedef struct {
    TrajSample  buf[RING_BUFFER_SIZE];
    uint32_t    write_idx;
    uint32_t    read_idx;
    uint32_t    count;
} RingBuffer;

extern RingBuffer ring;

void     ring_init(void);
void     ring_reset(void);
void     ring_push(const TrajSample* s);
int      ring_pop(TrajSample* s);   // returns 1 if sample available, 0 if empty
uint32_t ring_count(void);