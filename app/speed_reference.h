#ifndef SPEED_REFERENCE_H
#define SPEED_REFERENCE_H

/* Backward-Euler first-order reference filter, ONLY for the proportional
   setpoint path. Measured-speed feedback, I and feedforward remain unfiltered. */
static inline float speed_proportional_reference(float previous, float target,
                                                  float dt, unsigned tau_ms)
{
    if (tau_ms == 0U) return target;
    if (dt <= 0.0f) return previous;
    float alpha = dt / (tau_ms * 0.001f + dt);
    return previous + alpha * (target - previous);
}

#endif
