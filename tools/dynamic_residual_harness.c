/* Host execution only: no hardware or RTOS. */
#include <stdio.h>
#include "nn_controller.h"
int main(void)
{
    int mode, alpha;
    float target, speed;
    while (scanf("%d %d %f %f", &mode, &alpha, &target, &speed) == 4) {
        nn_controller_init();
        nn_controller_select_ff_weight(8);
        nn_controller_set_ff_weight(1000); /* known +0.1 V static component */
        if (nn_controller_set_mode(mode) || nn_controller_set_alpha_milli(alpha)) return 2;
        nn_features_t f = {target, speed, target-speed, 0, 0, 0.2f};
        float delta = nn_controller_predict(&f);
        printf("%.9g %.9g\n", delta, nn_controller_apply(0.2f, delta));
    }
    return 0;
}
