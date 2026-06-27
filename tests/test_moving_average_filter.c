#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "moving_average_filter.h"

static void test_initial_samples(void)
{
    MovingAverageFilter_t filter = {0};

    assert(MovingAverageFilter_Update(&filter, 100U) == 100U);
    assert(MovingAverageFilter_Update(&filter, 200U) == 150U);
    assert(MovingAverageFilter_Update(&filter, 300U) == 200U);
}

static void test_sliding_window(void)
{
    MovingAverageFilter_t filter = {0};
    uint32_t output = 0U;

    for (uint32_t sample = 1U; sample <= 9U; sample++)
    {
        output = MovingAverageFilter_Update(&filter, sample);
    }

    assert(output == 5U);
    assert(filter.count == MOVING_AVERAGE_WINDOW);
}

static void test_independent_axes(void)
{
    MovingAverageFilter_t x_filter = {0};
    MovingAverageFilter_t y_filter = {0};

    assert(MovingAverageFilter_Update(&x_filter, 1000U) == 1000U);
    assert(MovingAverageFilter_Update(&y_filter, 3000U) == 3000U);
    assert(MovingAverageFilter_Update(&x_filter, 2000U) == 1500U);
    assert(MovingAverageFilter_Update(&y_filter, 1000U) == 2000U);
}

int main(void)
{
    test_initial_samples();
    test_sliding_window();
    test_independent_axes();

    puts("moving_average_filter: all tests passed");
    return 0;
}
