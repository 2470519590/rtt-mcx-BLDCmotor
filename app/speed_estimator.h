#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdint.h>

#define SPEED_ESTIMATOR_HISTORY 16
typedef struct {
    float legacy, regression, observer;
    float legacy_delta, legacy_dt;
    float observer_position;
    float times[SPEED_ESTIMATOR_HISTORY], positions[SPEED_ESTIMATOR_HISTORY];
    uint8_t count;
} speed_estimator_t;

void speed_estimator_reset(speed_estimator_t *state);
void speed_estimator_update(speed_estimator_t *state, float delta_rad, float dt_s);
float speed_estimator_get(const speed_estimator_t *state, unsigned mode);
#endif
