/* Host-only execution of the actual inference modules; no MCU access. */
#include "nn_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    nn_controller_init();
    float row[ANGLE_MODEL_INPUTS + 1U] = {0};
    for (unsigned neuron = 0; neuron < ANGLE_MODEL_HIDDEN; ++neuron) {
        for (unsigned j = 1; j <= ANGLE_MODEL_INPUTS; ++j)
            row[j] = (float)(j * 10 + neuron) / 100.0f;
        assert(nn_controller_set_angle_model(row, 0.25f, neuron) == 0);
        for (unsigned j = 1; j <= ANGLE_MODEL_INPUTS; ++j)
            assert(abs(nn_controller_angle_coefficient(neuron, j) -
                       (int)(row[j] * 10000.0f)) <= 1);
    }
    assert(nn_controller_set_angle_model(row, 0, ANGLE_MODEL_HIDDEN) == -1);
    assert(nn_controller_set_angle_model(NULL, 0, 0) == -1);
    assert(nn_controller_angle_coefficient(0, 999) == 0);
    assert(nn_controller_angle_coefficient(999, 0) == 0);
    assert(nn_controller_set_angle_norm(0, 0, 1.0f) == 0);
    assert(fabsf(angle_shadow_phase(0) - 1.0f) < 1e-6f);
    assert(nn_controller_set_angle_norm(1, 10, 2.0f) == 0);
    assert(nn_controller_set_angle_norm(0, 20, 0.5f) == -2);
    assert(fabsf(angle_shadow_phase(0) - 1.0f) < 1e-6f);
    assert(nn_controller_set_angle_norm(3, 30, 0.5f) == -2);
    assert(angle_shadow_norm_count() == 2);
    assert(angle_shadow_predict(NAN, 0) == 0);
    assert(angle_shadow_predict(INFINITY, 0) == 0);
    assert(angle_shadow_predict(0, INFINITY) == 0);
    assert(angle_shadow_predict(3e6f, 0) == 0);
    assert(isfinite(angle_shadow_predict(1.0f, 1.0f)));
    puts("native angle checks passed (no hardware)");
    return 0;
}
