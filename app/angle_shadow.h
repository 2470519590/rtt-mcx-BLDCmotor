#ifndef ANGLE_SHADOW_H
#define ANGLE_SHADOW_H

#include <stdint.h>
#include "angle_shadow_model.h"

/* Normalization points for the RAM-uploadable angle model. The compiled
   candidate in angle_shadow_model.h stays the default; uploads are RAM only
   and a reset restores the compiled table. */
#define ANGLE_SHADOW_NORM_POINTS (8U)

/* Pure inference, no peripheral/RTOS/output access. Voltage is AC residual only. */
float angle_shadow_predict(float position_rad, float target_rad_s);
/* Speed-dependent phase currently in effect (radians), for parity checks. */
float angle_shadow_phase(float target_rad_s);

void angle_shadow_reset(void);
/* 0 accepted, -1 invalid value, -2 normalization not strictly ascending. */
int angle_shadow_set_norm(unsigned point, float rpm, float phase_rad);
/* slot 1..12 are the normalized input weights; the hidden bias can be
   overridden separately (upload) or left at the compiled value. */
int angle_shadow_set_hidden(unsigned neuron, const float row[ANGLE_MODEL_INPUTS + 1U],
                            float output_weight);
int angle_shadow_set_hidden_bias(unsigned neuron, float bias);
int angle_shadow_set_bias(float bias);
int angle_shadow_set_interp(unsigned mode);
unsigned angle_shadow_norm_count(void);
/* Read-only: compiled input std for feature index 0..11 (diagnostics only). */
float angle_shadow_debug_input(unsigned index);
/* Read-only: one stored coefficient in 1e-4 units. slot 0 bias, 1..12 input
   weights, 13 output weight, 14 network output bias. Never modifies state. */
int angle_shadow_coefficient(unsigned neuron, unsigned slot);

#endif
