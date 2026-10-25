#include "speed_estimator.h"
#include <stdio.h>

int main(void)
{
    speed_estimator_t state;
    speed_estimator_reset(&state);
    float delta, dt;
    while (scanf("%f %f", &delta, &dt) == 2) {
        speed_estimator_update(&state, delta, dt);
        printf("%.9g %.9g %.9g\n", speed_estimator_get(&state, 0),
            speed_estimator_get(&state, 1), speed_estimator_get(&state, 2));
    }
    return 0;
}
