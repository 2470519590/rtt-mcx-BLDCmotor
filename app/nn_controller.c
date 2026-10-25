#include "nn_controller.h"

#include <math.h>
#include <string.h>

#include "nn_model.h"
#include "angle_shadow.h"
#include "dynamic_residual_model.h"

#define NN_INPUTS NN_MODEL_INPUTS
#define NN_HIDDEN NN_MODEL_HIDDEN

static nn_mode_t s_mode;
static uint16_t s_alpha_milli;
static uint8_t s_learning_enabled;
/* Fixed ReLU basis in normalized RPM; learn only nine output coefficients.
   Runtime upload preserves encoder calibration during iterative experiments. */
static const float s_ff_knots[8] = {-1.0f, -0.6666667f, -0.3333333f,
    -0.05f, 0.05f, 0.3333333f, 0.6666667f, 1.0f};
static float s_ff_weights[9];
static unsigned s_ff_index;
static float s_dynamic_ff;

static float ff_predict(float target)
{
    float x = target / 18.84955592f;
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    float value = s_ff_weights[8];
    for (unsigned i = 0; i < 8U; ++i) {
        float h = x - s_ff_knots[i];
        if (h > 0.0f) value += s_ff_weights[i] * h;
    }
    if (value > 0.3f) value = 0.3f;
    if (value < -0.3f) value = -0.3f;
    return isfinite(value) ? value : 0.0f;
}

static float dynamic_predict(float target, float measured, int use_linear)
{
    /* Exact same milli-unit inputs as L telemetry and training. No hidden
       time derivative or host-cadence state is copied into the 2ms loop. */
    target = truncf(target * 1000.0f) / 1000.0f;
    measured = truncf(measured * 1000.0f) / 1000.0f;
    float x[3] = {target / 1.57079632679f, measured / 1.57079632679f,
                  target * measured < 0.0f ? 1.0f : 0.0f};
    if (x[0] > 2.0f) x[0] = 2.0f;
    if (x[0] < -2.0f) x[0] = -2.0f;
    if (x[1] > 4.0f) x[1] = 4.0f;
    if (x[1] < -4.0f) x[1] = -4.0f;
    float output = dynamic_b2;
    float linear = dynamic_linear[3];
    for (unsigned j = 0; j < DYNAMIC_INPUTS; ++j)
        linear += dynamic_linear[j] * x[j];
    for (unsigned i = 0; i < DYNAMIC_HIDDEN; ++i) {
        float value = dynamic_b1[i];
        for (unsigned j = 0; j < DYNAMIC_INPUTS; ++j)
            value += dynamic_w1[i][j] * x[j];
        if (value > 0.0f) output += dynamic_w2[i] * value;
    }
    if (use_linear) output = linear;
    if (output > 0.03f) output = 0.03f;
    if (output < -0.03f) output = -0.03f;
    return isfinite(output) ? output : 0.0f;
}

int nn_controller_set_angle_phase(int milli_rad)
{
    if (milli_rad < -3142 || milli_rad > 3142) return -1;
    /* Phase mode 0: one phase for every speed (bit-identical shadow parity).
       Mode 1 uses the uploaded speed normalization first. */
    angle_shadow_set_interp(0U);
    return angle_shadow_set_norm(0U, 0.0f, (float)milli_rad / 1000.0f);
}

int nn_controller_set_angle_norm(unsigned point, float rpm, float phase_rad)
{
    if (point >= ANGLE_SHADOW_NORM_POINTS) return -1;
    if (!isfinite(phase_rad) || phase_rad < -3.142f || phase_rad > 3.142f) return -1;
    return angle_shadow_set_norm(point, rpm, phase_rad);
}

int nn_controller_set_angle_interp(int mode)
{
    return angle_shadow_set_interp((unsigned)mode);
}

int nn_controller_set_angle_model(const float *row, float output_weight, unsigned neuron)
{
    if (row == NULL || neuron >= ANGLE_MODEL_HIDDEN) return -1;
    return angle_shadow_set_hidden(neuron, row, output_weight);
}

int nn_controller_set_angle_hidden_bias(unsigned neuron, int milli)
{
    return angle_shadow_set_hidden_bias(neuron, (float)milli / 10000.0f);
}

int nn_controller_set_angle_bias(int milli_volt)
{
    return angle_shadow_set_bias((float)milli_volt / 1000.0f);
}

unsigned nn_controller_input_std_milli(unsigned index)
{
    return (unsigned)roundf(angle_shadow_debug_input(index) * 1000.0f);
}

int nn_controller_angle_coefficient(unsigned neuron, unsigned slot)
{
    return angle_shadow_coefficient(neuron, slot);
}

int nn_controller_select_ff_weight(int index)
{
    if (index < 0 || index > 8) return -1;
    s_ff_index = (unsigned)index;
    return 0;
}

int nn_controller_set_ff_weight(int value)
{
    if (value < -100000 || value > 100000) return -1;
    s_ff_weights[s_ff_index] = (float)value / 10000.0f;
    return 0;
}

void nn_controller_init(void)
{
    s_mode = NN_MODE_OFF;
    s_alpha_milli = 0U;
    s_learning_enabled = 0U;
    memset(s_ff_weights, 0, sizeof(s_ff_weights));
    s_ff_index = 0U;
    s_dynamic_ff = 0.0f;
    /* Restore the compiled angle candidate; RAM uploads do not survive reset. */
    angle_shadow_reset();
}

int nn_controller_set_mode(int value)
{
    if (value < NN_MODE_OFF || value > NN_MODE_DYNAMIC_LINEAR) return -1;
    s_mode = (nn_mode_t)value;
    return 0;
}

int nn_controller_set_alpha_milli(int value)
{
    if (value < 0 || value > 1000) return -1;
    s_alpha_milli = (uint16_t)value;
    return 0;
}

int nn_controller_set_learning(int value)
{
    if (value != 0 && value != 1) return -1;
    s_learning_enabled = (uint8_t)value;
    return 0;
}

nn_mode_t nn_controller_get_mode(void)
{
    return s_mode;
}

uint16_t nn_controller_get_alpha_milli(void)
{
    return s_alpha_milli;
}

uint8_t nn_controller_learning_enabled(void)
{
    return s_learning_enabled;
}

float nn_controller_apply(float baseline_uq, float predicted_delta)
{
    if (s_mode >= NN_MODE_DYNAMIC_SHADOW)
        return baseline_uq + s_dynamic_ff +
            (s_mode != NN_MODE_DYNAMIC_SHADOW ?
             ((float)s_alpha_milli / 1000.0f) * predicted_delta : 0.0f);
    if (s_mode == NN_MODE_ANGLE_BLENDED) {
        /* Bounded application. The 80 mV prediction clamp and the 0.25 alpha
           cap were sized for the old AC-only experiment; the DC trim needs
           about 0.11 V of offset, so both limits are now explicit, still
           bounded, and unchanged for small predictions. */
        float alpha = (float)s_alpha_milli / 1000.0f;
        if (alpha > NN_ANGLE_ALPHA_MAX) alpha = NN_ANGLE_ALPHA_MAX;
        float limit = NN_ANGLE_DELTA_MAX;
        if (predicted_delta > limit) predicted_delta = limit;
        if (predicted_delta < -limit) predicted_delta = -limit;
        return baseline_uq + alpha * predicted_delta;
    }
    if (s_mode == NN_MODE_BLENDED || s_mode == NN_MODE_FEEDFORWARD)
        return baseline_uq + ((float)s_alpha_milli / 1000.0f) * predicted_delta;
    /* All shadow modes, especially angle shadow, cannot actuate through here. */
    return baseline_uq;
}

float nn_controller_predict(const nn_features_t *features)
{
    float input[NN_INPUTS];
    float hidden[NN_HIDDEN];
    float output = nn_model_output_bias;

    if (features == NULL) {
        s_dynamic_ff = 0.0f;
        return 0.0f;
    }
    if (s_mode >= NN_MODE_DYNAMIC_SHADOW) {
        s_dynamic_ff = ff_predict(features->target_speed_rad_s);
        return dynamic_predict(features->target_speed_rad_s, features->measured_speed_rad_s,
                               s_mode == NN_MODE_DYNAMIC_LINEAR);
    }
    if (s_mode == NN_MODE_ANGLE_SHADOW || s_mode == NN_MODE_ANGLE_BLENDED)
        return angle_shadow_predict(features->measured_position_rad,
                                    features->target_speed_rad_s);
    if (s_mode == NN_MODE_FEEDFORWARD || s_mode == NN_MODE_FEEDFORWARD_SHADOW) {
        return ff_predict(features->target_speed_rad_s);
    }
    input[0] = features->target_speed_rad_s;
    input[1] = features->measured_speed_rad_s;
    input[2] = features->speed_error_rad_s;
    input[3] = features->target_position_rad;
    input[4] = features->measured_position_rad;
    input[5] = features->base_uq;
    for (unsigned j = 0; j < NN_INPUTS; ++j) {
        float std = nn_model_input_std[j];
        if (!isfinite(std) || std < 1.0e-6f) std = 1.0f;
        input[j] = (input[j] - nn_model_input_mean[j]) / std;
        if (input[j] > 8.0f) input[j] = 8.0f;
        if (input[j] < -8.0f) input[j] = -8.0f;
    }
    for (unsigned i = 0; i < NN_HIDDEN; ++i) {
        float value = nn_model_hidden_bias[i];
        for (unsigned j = 0; j < NN_INPUTS; ++j) {
            value += nn_model_hidden_weights[i][j] * input[j];
        }
        hidden[i] = value > 0.0f ? value : 0.0f;
        output += nn_model_output_weights[i] * hidden[i];
    }
    if (output > nn_model_output_limit) output = nn_model_output_limit;
    if (output < -nn_model_output_limit) output = -nn_model_output_limit;
    return isfinite(output) ? output : 0.0f;
}
