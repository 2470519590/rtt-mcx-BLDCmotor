/* Host validation only; never linked into firmware and has no probe/UART I/O. */
#include <stdio.h>
#include "nn_controller.h"

int main(void)
{
    int mode, alpha;
    float angle, target, baseline;
    while (scanf("%d %f %f %f %d", &mode, &angle, &target, &baseline, &alpha) == 5) {
        nn_features_t features = {target, 0, 0, 0, angle, baseline};
        nn_controller_init();
        if (nn_controller_set_mode(mode) != 0 || nn_controller_set_alpha_milli(alpha) != 0)
            return 2;
        float prediction = nn_controller_predict(&features);
        printf("%.9g %.9g\n", prediction, nn_controller_apply(baseline, prediction));
    }
    return 0;
}
