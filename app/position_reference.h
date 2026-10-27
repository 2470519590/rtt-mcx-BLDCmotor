#ifndef POSITION_REFERENCE_H
#define POSITION_REFERENCE_H

#include <math.h>
#include <string.h>

/* Optional position-only quintic trajectory. No RTOS, I/O, or speed-mode state. */
typedef struct {
    float position, speed, acceleration;
    float start, c[6], elapsed, duration;
    unsigned holding;
} position_reference_t;

static inline void position_reference_reset(position_reference_t *state,
                                            float position, float speed)
{
    memset(state, 0, sizeof(*state));
    state->position = state->start = position;
    state->speed = speed;
}

static inline void position_reference_target(position_reference_t *state,
        float target, unsigned move_ms, float speed_limit)
{
    float distance = target - state->position;
    float duration = move_ms * .001f;
    /* Minimum-jerk profile: peak normalized velocity=1.875,
       peak normalized acceleration=10/sqrt(3). */
    float speed_time = 1.875f * fabsf(distance) / fmaxf(speed_limit, .1f);
    float accel_time = sqrtf(5.773502692f * fabsf(distance) / 50.0f);
    duration = fmaxf(duration, fmaxf(speed_time, accel_time));
    duration = fmaxf(duration, .002f);
    float velocity = state->speed * duration;
    float acceleration = state->acceleration * duration * duration;
    state->start = state->position;
    state->c[0] = 0;
    state->c[1] = velocity;
    state->c[2] = .5f * acceleration;
    state->c[3] = 10*distance - 6*velocity - 1.5f*acceleration;
    state->c[4] = -15*distance + 8*velocity + 1.5f*acceleration;
    state->c[5] = 6*distance - 3*velocity - .5f*acceleration;
    state->duration = duration;
    state->elapsed = 0;
    state->holding = 0;
}

static inline float position_reference_update(position_reference_t *state,
        float dt, float measured, float speed, float gain, float limit)
{
    state->elapsed = fminf(state->elapsed + dt, state->duration);
    float u = state->duration > 0 ? state->elapsed / state->duration : 1;
    const float *c = state->c;
    state->position = state->start +
        u*(c[1] + u*(c[2] + u*(c[3] + u*(c[4] + u*c[5]))));
    if (u >= 1) {
        state->speed = state->acceleration = 0;
    } else {
        state->speed = (c[1] + u*(2*c[2] + u*(3*c[3] +
                            u*(4*c[4] + u*5*c[5])))) / state->duration;
        state->acceleration = (2*c[2] + u*(6*c[3] + u*(12*c[4] +
                            u*20*c[5]))) / (state->duration * state->duration);
    }
    float error = state->position - measured;
    /* About two AS5600 counts to enter, four to leave. Preserve the holding
       torque rather than repeatedly integrating encoder quantization noise. */
    if (state->holding && (fabsf(error) > .006135923f || fabsf(speed) > .3f))
        state->holding = 0;
    if (!state->holding && u >= 1 && fabsf(error) <= .003067962f &&
        fabsf(speed) < .15f)
        state->holding = 1;
    float target = state->holding ? 0 : state->speed + gain*error;
    return fmaxf(-limit, fminf(limit, target));
}

#endif
