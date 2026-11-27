#ifndef CHIRP_TX_SCHEDULE_H
#define CHIRP_TX_SCHEDULE_H

#include <math.h>
#include <stdint.h>

/* Phase-defined linear-in-time FM: phase(t) = f0*t + slope*t*t/2.
   Consecutive whole-cycle crossings have frequencies sqrt(f0^2+2*slope*n).
   The stable crossing interval is 2/(f(n)+f(n+1)), avoiding subtraction of
   nearly equal times. Round to the HRTIM counter grid, not integer Hz. */
static inline uint32_t chirp_tx_period_ticks(uint32_t cycle, uint32_t start_hz,
    uint32_t end_hz, uint32_t duration_us, uint32_t timer_clock_hz)
{
    if ((start_hz == 0u) || (end_hz <= start_hz) || (duration_us == 0u)) return 0u;
    const float slope = (float)(end_hz - start_hz) * (1000000.0f / (float)duration_us);
    const float f0_squared = (float)start_hz * (float)start_hz;
    const float left = sqrtf(f0_squared + 2.0f * slope * (float)cycle);
    const float right = sqrtf(f0_squared + 2.0f * slope * (float)(cycle + 1u));
    return (uint32_t)(2.0f * (float)timer_clock_hz / (left + right) + 0.5f);
}

#endif
