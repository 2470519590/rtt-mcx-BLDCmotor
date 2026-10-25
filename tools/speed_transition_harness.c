#include <assert.h>
#include "speed_transition.h"
#include "speed_reference.h"
#include <math.h>

int main(void)
{
    for (unsigned mode = 0; mode <= 8; ++mode) {
        for (unsigned keep = 0; keep <= 1; ++keep) {
            for (int i = -10; i <= 10; ++i) {
                float integral = i * 0.1f;
                float expected = mode == 2 && keep ? integral : 0.0f;
                assert(speed_transition_integral(mode, keep, integral) == expected);
            }
        }
    }
    assert(speed_proportional_reference(-1.0f, 2.0f, 0.002f, 0) == 2.0f);
    assert(speed_proportional_reference(-1.0f, 2.0f, 0.0f, 50) == -1.0f);
    for (unsigned tau = 1; tau <= 200; ++tau) {
        float value = -2.0f;
        for (unsigned i = 0; i < 2000; ++i) {
            float next = speed_proportional_reference(value, 1.0f, .002f, tau);
            assert(next >= value && next <= 1.0f);
            value = next;
        }
        assert(fabsf(value - 1.0f) < .0001f);
    }
    return 0;
}
