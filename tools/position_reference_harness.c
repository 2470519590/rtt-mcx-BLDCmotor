#include "position_reference.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    position_reference_t state;
    position_reference_reset(&state, -2, 0);
    position_reference_target(&state, 0, 500, 12);
    float previous = state.position, peak_speed = 0, peak_accel = 0;
    for (unsigned i = 0; i < 500; ++i) {
        float command = position_reference_update(&state, .002f, state.position,
                                                  state.speed, 12, 12);
        assert(isfinite(command) && fabsf(command) <= 12);
        assert(state.position >= previous - 1e-5f && state.position <= 1e-5f);
        previous = state.position;
        peak_speed = fmaxf(peak_speed, fabsf(state.speed));
        peak_accel = fmaxf(peak_accel, fabsf(state.acceleration));
    }
    assert(fabsf(state.position) < 2e-5f && state.speed == 0 && state.acceleration == 0);
    assert(peak_speed < 7.51f && peak_accel < 50.1f);
    /* Sensor-scale jitter retains holding torque; a real displacement exits. */
    assert(state.holding);
    assert(position_reference_update(&state, .002f, .002f, .02f, 12, 12) == 0);
    assert(state.holding);
    assert(position_reference_update(&state, .002f, .02f, .02f, 12, 12) < 0);
    assert(!state.holding);

    position_reference_reset(&state, 80, 0);
    position_reference_target(&state, 82, 500, 8);
    for (unsigned i = 0; i < 100; ++i)
        position_reference_update(&state, .002f, state.position, state.speed, 12, 8);
    float position = state.position, speed = state.speed, acceleration = state.acceleration;
    position_reference_target(&state, 79, 500, 8);
    assert(state.position == position && state.speed == speed &&
           state.acceleration == acceleration);
    for (unsigned i = 0; i < 2500; ++i) {
        float command = position_reference_update(&state, .002f, state.position,
                                                  state.speed, 12, 8);
        assert(isfinite(command) && fabsf(command) <= 8);
    }
    assert(fabsf(state.position - 79) < .00005f && state.speed == 0);
    /* A long move stretches its timing instead of exceeding nominal speed. */
    position_reference_reset(&state, 0, 0);
    position_reference_target(&state, 10, 100, 2);
    assert(state.duration >= 9.375f);
    for (unsigned i = 0; i < 5000; ++i) {
        position_reference_update(&state, .002f, state.position, state.speed, 12, 2);
        assert(fabsf(state.speed) <= 2.001f);
    }
    puts("position reference checks passed");
    return 0;
}
