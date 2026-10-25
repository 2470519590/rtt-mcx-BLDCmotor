#ifndef SPEED_TRANSITION_H
#define SPEED_TRANSITION_H

/* Keep only the learned integral on a speed-to-speed reference change.
   This intentionally does not cancel the new P/feedforward reference step.
   Starting from stop/fault/another control mode always starts at zero. */
static inline float speed_transition_integral(unsigned previous_mode,
                                               unsigned keep,
                                               float integral)
{
    return previous_mode == 2U && keep == 1U ? integral : 0.0f;
}

#endif
