#ifndef MOVING_AVERAGE_FILTER_H
#define MOVING_AVERAGE_FILTER_H

#include <stdint.h>

#define MOVING_AVERAGE_WINDOW 8U

typedef struct
{
    uint32_t samples[MOVING_AVERAGE_WINDOW];
    uint64_t sum;
    uint8_t index;
    uint8_t count;
} MovingAverageFilter_t;

uint32_t MovingAverageFilter_Update(MovingAverageFilter_t *filter, uint32_t sample);

#endif
