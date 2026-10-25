#include "angle_shadow.h"
#include "angle_shadow_model.h"
#include <math.h>
#include <string.h>

/* Uploaded candidate, RAM only. The compiled table in angle_shadow_model.h
   stays the default, so a reset returns to the verified model and refitting
   never requires burning a new calibration. */
#define ANGLE_ROW (ANGLE_MODEL_INPUTS + 1U) /* hidden row: bias, then 12 inputs */

typedef struct
{
    float rpm;   /* normalization speed, ascending */
    float phase; /* applied to mechanical position, radians */
} angle_norm_t;

static angle_norm_t s_norm[ANGLE_SHADOW_NORM_POINTS];
static float s_hidden[ANGLE_MODEL_HIDDEN][ANGLE_ROW]; /* weight * std, bias in slot 0 */
static float s_output[ANGLE_MODEL_HIDDEN];
static float s_bias;
static unsigned s_norm_count;
static uint8_t s_interp; /* 0 nearest (step), 1 linear between norm points */

/* The compiled candidate has one phase for all speeds, so phase mode 0 is
   bit-identical to it and is what existing shadow parity refers to. */
static float phase_for(float rpm)
{
    float value;
    if (s_norm_count == 0U) return 0.0f;
    if (s_interp == 0U) {
        unsigned nearest = 0U;
        for (unsigned i = 1U; i < s_norm_count; ++i)
            if (fabsf(rpm - s_norm[i].rpm) < fabsf(rpm - s_norm[nearest].rpm)) nearest = i;
        return s_norm[nearest].phase;
    }
    if (rpm <= s_norm[0].rpm) return s_norm[0].phase;
    if (rpm >= s_norm[s_norm_count - 1U].rpm) return s_norm[s_norm_count - 1U].phase;
    for (unsigned i = 1U; i < s_norm_count; ++i) {
        if (rpm <= s_norm[i].rpm) {
            float span = s_norm[i].rpm - s_norm[i - 1U].rpm;
            float t = span > 1.0e-6f ? (rpm - s_norm[i - 1U].rpm) / span : 0.0f;
            value = s_norm[i - 1U].phase + t * (s_norm[i].phase - s_norm[i - 1U].phase);
            return value;
        }
    }
    return s_norm[s_norm_count - 1U].phase;
}

static int finite_value(float value)
{
    /* Rejects NaN and infinities; round-trips a float to keep -0.0 harmless. */
    return isfinite(value);
}

void angle_shadow_reset(void)
{
    s_norm_count = 0U;
    s_interp = 0U;
    memset(s_norm, 0, sizeof(s_norm));
    for (unsigned i = 0U; i < ANGLE_MODEL_HIDDEN; ++i) {
        s_hidden[i][0] = angle_hidden_bias[i];
        /* angle_hidden_weights already contains weight * std, and predict()
           normalizes the inputs by std, so the product is the intended
           weight * input. Multiplying by std here (as an earlier version did)
           divided the weights a second time and skewed every prediction. */
        for (unsigned j = 0U; j < ANGLE_MODEL_INPUTS; ++j)
            s_hidden[i][1U + j] = angle_hidden_weights[i][j];
        s_output[i] = angle_output_weights[i];
    }
    s_bias = angle_output_bias;
}

int angle_shadow_set_norm(unsigned point, float rpm, float phase_rad)
{
    if (point >= ANGLE_SHADOW_NORM_POINTS) return -1;
    if (!finite_value(rpm) || !finite_value(phase_rad)) return -1;
    if (point > s_norm_count) return -2;
    if (point > 0U && !(rpm > s_norm[point - 1U].rpm)) return -2;
    if (point + 1U < s_norm_count && !(rpm < s_norm[point + 1U].rpm)) return -2;
    s_norm[point].rpm = rpm;
    s_norm[point].phase = phase_rad;
    if (point == s_norm_count) ++s_norm_count;
    return 0;
}

int angle_shadow_set_hidden(unsigned neuron, const float row[ANGLE_ROW], float output_weight)
{
    /* row[1..12] are the checkpoint weights (already the ones the compiled
       table encodes); the bias is compiled in and is not uploaded. */
    if (neuron >= ANGLE_MODEL_HIDDEN || row == NULL) return -1;
    if (!finite_value(output_weight)) return -1;
    for (unsigned j = 1U; j < ANGLE_ROW; ++j) {
        if (!finite_value(row[j])) return -1;
        s_hidden[neuron][j] = row[j];
    }
    s_output[neuron] = output_weight;
    return 0;
}

int angle_shadow_set_hidden_bias(unsigned neuron, float bias)
{
    if (neuron >= ANGLE_MODEL_HIDDEN || !finite_value(bias)) return -1;
    s_hidden[neuron][0] = bias;
    return 0;
}

int angle_shadow_set_bias(float bias)
{
    if (!finite_value(bias)) return -1;
    s_bias = bias;
    return 0;
}

int angle_shadow_set_interp(unsigned mode)
{
    if (mode > 1U) return -1;
    s_interp = (uint8_t)mode;
    return 0;
}

unsigned angle_shadow_norm_count(void)
{
    return s_norm_count;
}

float angle_shadow_debug_input(unsigned index)
{
    if (index >= ANGLE_MODEL_INPUTS) return 0.0f;
    return angle_input_std[index];
}

/* Upload verification: report one stored coefficient in 1e-4 units. The read
   is a plain float compare and convert, no scaling, so a query cannot alter
   the model it is checking. */
int angle_shadow_coefficient(unsigned neuron, unsigned slot)
{
    float value;
    if (neuron >= ANGLE_MODEL_HIDDEN) return 0;
    /* Row layout is [0]=bias, [1..12]=inputs, so slot n maps to row n while
       slots 13 and 14 address the output weight and the network bias. Using
       row index n directly is what makes slot 1 read the input weight. */
    if (slot <= ANGLE_MODEL_INPUTS) value = s_hidden[neuron][slot];
    else if (slot == ANGLE_ROW) value = s_output[neuron];
    else if (slot == ANGLE_ROW + 1U) value = s_bias;
    else return 0;
    return (int)(value * 10000.0f);
}

float angle_shadow_phase(float target_rad_s)
{
    if (!isfinite(target_rad_s)) return 0.0f;
    return phase_for(target_rad_s * 60.0f / 6.28318530718f);
}

float angle_shadow_predict(float position_rad, float target_rad_s)
{
    static const float harmonics[5] = {1.0f, 2.0f, 7.0f, 14.0f, 21.0f};
    float input[ANGLE_MODEL_INPUTS];
    float rpm;
    float output = s_bias;
    /* Telemetry truncates position and target to milli-units, so the host can
       only reproduce the inference if the same truncation is applied here. */
    if (!isfinite(position_rad) || !isfinite(target_rad_s) ||
        fabsf(position_rad) > 2147483.0f || fabsf(target_rad_s) > 2147483.0f)
        return 0.0f;
    position_rad = (float)((int32_t)(position_rad * 1000.0f)) / 1000.0f;
    target_rad_s = (float)((int32_t)(target_rad_s * 1000.0f)) / 1000.0f;
    rpm = target_rad_s * 60.0f / 6.28318530718f;
    input[0] = rpm / 40.0f;
    input[1] = rpm > 0.0f ? 1.0f : (rpm < 0.0f ? -1.0f : 0.0f);
    /* Harmonic k shifted by phase p needs k*p, so the phase is added per
       harmonic. With an empty table every sine argument is unchanged and the
       compiled model's behavior is preserved exactly. */
    for (unsigned i = 0U; i < 5U; ++i) {
        float angle = harmonics[i] * position_rad + angle_shadow_phase(target_rad_s);
        input[2U + 2U * i] = sinf(angle);
        input[3U + 2U * i] = cosf(angle);
    }
    for (unsigned j = 0U; j < ANGLE_MODEL_INPUTS; ++j)
        input[j] = (input[j] - angle_input_mean[j]) / angle_input_std[j];
    for (unsigned i = 0U; i < ANGLE_MODEL_HIDDEN; ++i) {
        float hidden = s_hidden[i][0];
        for (unsigned j = 0U; j < ANGLE_MODEL_INPUTS; ++j)
            hidden += s_hidden[i][1U + j] * input[j];
        if (hidden > 0.0f) output += s_output[i] * hidden;
    }
    return isfinite(output) ? output : 0.0f;
}
