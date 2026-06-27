#include "moving_average_filter.h"

uint32_t MovingAverageFilter_Update(MovingAverageFilter_t *filter, uint32_t sample)
{
    filter->sum -= filter->samples[filter->index];
    filter->samples[filter->index] = sample;
    filter->sum += sample;
    filter->index = (filter->index + 1U) % MOVING_AVERAGE_WINDOW;

    if (filter->count < MOVING_AVERAGE_WINDOW)
    {
        filter->count++;
    }

    return (uint32_t)(filter->sum / filter->count);
}
