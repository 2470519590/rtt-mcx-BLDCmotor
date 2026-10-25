#include "speed_estimator.h"
#include <math.h>
#include <string.h>

void speed_estimator_reset(speed_estimator_t *state)
{
    memset(state, 0, sizeof(*state));
    /* Initial position at t=0 is a real sample, not a synthetic velocity. */
    state->count = 1;
}

void speed_estimator_update(speed_estimator_t *state, float delta_rad, float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s >= 0.1f || !isfinite(delta_rad)) {
        speed_estimator_reset(state);
        return;
    }
    state->legacy_delta += delta_rad;
    state->legacy_dt += dt_s;
    if (state->legacy_dt >= 0.009999f) {
        state->legacy += 0.35f * (state->legacy_delta / state->legacy_dt - state->legacy);
        state->legacy_delta = state->legacy_dt = 0.0f;
    }

    /* Local coordinates are rebased every sample: no float precision loss
       after hours of unwrapped travel. Actual sample spacing is retained. */
    for (unsigned i = 0; i < state->count; ++i) {
        state->times[i] -= dt_s;
        state->positions[i] -= delta_rad;
    }
    while (state->count && (state->times[0] < -0.020001f ||
                            state->count == SPEED_ESTIMATOR_HISTORY)) {
        --state->count;
        memmove(state->times, state->times + 1, state->count * sizeof(float));
        memmove(state->positions, state->positions + 1, state->count * sizeof(float));
    }
    state->times[state->count] = state->positions[state->count] = 0.0f;
    ++state->count;
    float mean_t = 0.0f, mean_p = 0.0f, numerator = 0.0f, denominator = 0.0f;
    for (unsigned i = 0; i < state->count; ++i) {
        mean_t += state->times[i];
        mean_p += state->positions[i];
    }
    mean_t /= state->count;
    mean_p /= state->count;
    for (unsigned i = 0; i < state->count; ++i) {
        float t = state->times[i] - mean_t;
        numerator += t * (state->positions[i] - mean_p);
        denominator += t * t;
    }
    state->regression = denominator > 1e-12f ? numerator / denominator : 0.0f;

    /* Alpha-beta observer, two poles at exp(-2*pi*16*dt). Position error
       coordinates avoid integrating a large absolute float position. */
    float pole = expf(-100.5309649f * dt_s);
    float innovation = delta_rad - state->observer_position - state->observer * dt_s;
    state->observer_position += state->observer * dt_s - delta_rad;
    state->observer_position += (1.0f - pole * pole) * innovation;
    state->observer += (1.0f - pole) * (1.0f - pole) * innovation / dt_s;
}

float speed_estimator_get(const speed_estimator_t *state, unsigned mode)
{
    return mode == 1 ? state->regression : mode == 2 ? state->observer : state->legacy;
}
