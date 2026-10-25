#ifndef NN_CONTROLLER_H
#define NN_CONTROLLER_H

#include <stdint.h>
#include "angle_shadow.h"

typedef enum
{
    NN_MODE_OFF = 0,
    NN_MODE_SHADOW = 1,
    NN_MODE_BLENDED = 2,
    NN_MODE_FEEDFORWARD = 3,
    NN_MODE_FEEDFORWARD_SHADOW = 4,
    NN_MODE_ANGLE_SHADOW = 5,
    NN_MODE_ANGLE_BLENDED = 6,
    NN_MODE_DYNAMIC_SHADOW = 7, /* static ff applied, dynamic prediction only */
    NN_MODE_DYNAMIC_BLENDED = 8, /* static ff plus alpha * dynamic prediction */
    NN_MODE_DYNAMIC_LINEAR = 9 /* same features/teacher, explicit linear control */
} nn_mode_t;

/* G6 application bounds. The old 80 mV / 0.25 limits came from the AC-only
   experiment and cap the applied voltage at 20 mV, which cannot trim the
   measured +/-110 mV DC offset. Still bounded, now explicit. */
#define NN_ANGLE_DELTA_MAX 0.20f  /* V, applied prediction clamp */
#define NN_ANGLE_ALPHA_MAX 1.0f

typedef struct
{
    float target_speed_rad_s;
    float measured_speed_rad_s;
    float speed_error_rad_s;
    float target_position_rad;
    float measured_position_rad;
    float base_uq;
} nn_features_t;

void nn_controller_init(void);
int nn_controller_set_mode(int value);
int nn_controller_set_alpha_milli(int value);
int nn_controller_set_learning(int value);
nn_mode_t nn_controller_get_mode(void);
uint16_t nn_controller_get_alpha_milli(void);
uint8_t nn_controller_learning_enabled(void);
float nn_controller_predict(const nn_features_t *features);
int nn_controller_select_ff_weight(int index);
int nn_controller_set_ff_weight(int value);
int nn_controller_set_angle_phase(int milli_rad);
/* Speed-dependent phase table (RAM only, reset restores the compiled model).
   phase is in radians; rpm must be strictly ascending across points. */
int nn_controller_set_angle_norm(unsigned point, float rpm, float phase_rad);
int nn_controller_set_angle_interp(int mode);
/* One hidden neuron: 13 values, slot 0 is the bias and slots 1..12 are the
   normalized input weights, in the same order as nn_model.h. */
int nn_controller_set_angle_model(const float *row, float output_weight, unsigned neuron);
int nn_controller_set_angle_hidden_bias(unsigned neuron, int milli);
/* Read-only upload diagnostics: compiled input std in milli-units. */
unsigned nn_controller_input_std_milli(unsigned index);
/* Read-only: one uploaded coefficient in 1e-4 units, for upload verification. */
int nn_controller_angle_coefficient(unsigned neuron, unsigned slot);
int nn_controller_set_angle_bias(int milli_volt);
/* One centralized output gate, shared by control and host-test harnesses. */
float nn_controller_apply(float baseline_uq, float predicted_delta);

#endif
