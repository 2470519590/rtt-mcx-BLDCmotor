#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#define MOTOR_MAX_RPM 1000
#define MOTOR_OPENLOOP_MAX_RPM 250

typedef struct
{
    uint16_t raw_angle;
    float angle_rad;
    float speed_rad_s;
    uint8_t sensor_ok;
    uint8_t mode; /* 0 stop, 1 calibration, 2 run, 3 fault, 4 openloop, 5 PWM test, 6 ramp, 7 identification, 8 position */
    uint8_t calibrated;
    float target_rad_s;
    float target_position_rad;
    float uq;
    uint8_t fault; /* 1 I2C, 2 heartbeat, 3 alignment, 4 overspeed */
    int32_t calibration_mrad;
    uint32_t sample_tick;
    float position_rad; /* Unwrapped, calibrated-direction mechanical position. */
    float feedback_speed_rad_s;
    float excitation_v;
    float identification_hz;
    float excitation_phase;
    uint8_t identification_point;
    uint8_t identification_stage; /* 0 warm-up, 1 settling, 2 recording */
    uint8_t voltage_clipped;
    float speed_kp;
    float voltage_limit;
    float position_kp;
    float position_speed_limit;
    float base_uq;
    float nn_delta_uq;
    uint8_t nn_mode;
    uint16_t nn_alpha_milli;
    uint8_t nn_learning;
    /* Exact inputs of the last inference. Telemetry truncates these, so this
       is the only way to compare the MCU prediction at full precision. */
    float nn_input_position_rad;
    float nn_input_speed_rad_s;
} motor_state_t;

/* SysTick-reconstructed wall cycles include preemption, not pure CPU time. */
typedef struct {
    uint32_t clock_hz, samples, period_cycles, max_period_cycles;
    uint32_t sensor_cycles, max_sensor_cycles, loop_cycles, max_loop_cycles;
    uint32_t late_periods, over_budget_loops;
    uint8_t estimator_mode;
    float legacy_speed, regression_speed, observer_speed;
} motor_timing_t;
extern volatile motor_timing_t g_motor_timing;
/* Incremented when a host command cannot take the control lock in 50 ms;
   non-zero therefore means contention rather than a dead UART. */
extern volatile unsigned g_motor_lock_timeouts;
/* Command-path trace for the D1 diagnostic: last op/value handed to the
   handler, how many reached it, and the exit marker (0xE0 entered, 0xF0 lock
   taken, 0xFF completed, 0xFE lock timeout). */
uint32_t motor_control_trace_op(void);
uint32_t motor_control_trace_enter(void);
unsigned motor_control_trace_exit(void);
void motor_control_get_timing(motor_timing_t *timing);

void motor_control_init(void);
void motor_control_thread(void *parameter);
void motor_control_get_state(motor_state_t *state);
int motor_control_command(char op, int value);
/* RAM-only angle-model upload state, for the read-only x0 query. */
unsigned angle_upload_point(void);
unsigned angle_upload_count(void);
int angle_upload_row_valid(void);
unsigned angle_upload_interp(void);
unsigned angle_upload_g_stage(void);

#endif
