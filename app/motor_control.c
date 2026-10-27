#include "motor_control.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "as5600.h"
#include "rtthread.h"
#include "simplefoc_pwm.h"
#include "nn_controller.h"
#include "speed_estimator.h"
#include "speed_transition.h"
#include "speed_reference.h"
#include "position_reference.h"
#include "fsl_device_registers.h"

#define MOTOR_TWO_PI (6.28318530718f)
#define ENCODER_SAMPLE_PERIOD_MS (2U) /* identification trace control period */
#define SPEED_ESTIMATION_WINDOW_MS (10U)
#define SPEED_ESTIMATION_FILTER_ALPHA (0.35f)
#define CALIBRATION_ZERO_SETTLE_MS (750U)
#define CALIBRATION_ALIGN_VOLTAGE (1.5f)
#define MOTOR_MODEL_K (44.18f)       /* (rad/s)/V, from 0.2..3 Hz identification */
#define MOTOR_SPEED_KP_DEFAULT (0.0078f) /* 2 Hz PI design, V/(rad/s) */
#define MOTOR_SPEED_KI_DEFAULT (0.284f)  /* 2 Hz PI design, V/(rad/s*s) */
#define MOTOR_POSITION_KP_DEFAULT (4.0f) /* rad/s per rad, outer position loop */
#define MOTOR_POSITION_MAX_RPM_DEFAULT (60.0f)

static motor_state_t s_state;
static uint16_t s_last_raw;
static uint8_t s_have_last;
static float s_speed_delta_rad;
static rt_tick_t s_speed_window_ticks;
static speed_estimator_t s_speed_estimator;
static unsigned s_estimator_mode; /* Default remains the measured legacy baseline. */
static uint32_t s_sample_cycles, s_loop_cycles;
static uint8_t s_have_sample_cycles, s_have_loop_cycles;
volatile motor_timing_t g_motor_timing;

/* MCXA346's DWT CYCCNT did not advance on this board. Use the running
   RT-Thread SysTick plus its sub-tick countdown; no extra timer allocation. */
static uint32_t motor_cycles(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    uint32_t ticks = rt_tick_get();
    uint32_t value = SysTick->VAL;
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        ++ticks;
        value = SysTick->VAL;
    }
    uint32_t period = SysTick->LOAD + 1U;
    uint32_t result = ticks * period + period - 1U - value;
    rt_hw_interrupt_enable(level);
    return result;
}

static void timing_finish(uint32_t start)
{
    uint32_t elapsed = motor_cycles() - start;
    g_motor_timing.loop_cycles = elapsed;
    if (elapsed > g_motor_timing.max_loop_cycles) g_motor_timing.max_loop_cycles = elapsed;
    if (elapsed > SystemCoreClock / 500U) ++g_motor_timing.over_budget_loops;
}
static rt_mutex_t s_lock;
static float s_kp = MOTOR_SPEED_KP_DEFAULT;
static float s_ki = MOTOR_SPEED_KI_DEFAULT;
static float s_position_kp = MOTOR_POSITION_KP_DEFAULT;
static float s_position_speed_limit = MOTOR_POSITION_MAX_RPM_DEFAULT * MOTOR_TWO_PI / 60.0f;
static float s_target_position_rad;
static position_reference_t s_position_reference;
static unsigned s_position_move_ms; /* 0 = original position loop, t500 = smooth demo. */
static float s_integral_v;
static unsigned s_speed_transition_keep; /* RAM only; default legacy reset. */
static unsigned s_proportional_tau_ms; /* zero preserves original controller */
static float s_proportional_reference;
static float s_limit = 1.0f;
static float s_zero, s_travel, s_cal_start;
static int s_direction = 1;
static rt_tick_t s_cal_tick, s_last_command, s_sample_tick;
static rt_tick_t s_cal_zero_tick;
static uint8_t s_cal_zero_pending;
static unsigned s_sensor_fail_streak;
static float s_openloop_angle;
static int s_sweep_start_rpm = 10, s_sweep_end_rpm = 100;
static uint32_t s_sweep_seconds = 20;
static rt_tick_t s_sweep_tick, s_control_tick;
static int s_id_rpm = 60;
static float s_id_amplitude = 0.05f, s_id_first_hz = 0.2f, s_id_last_hz = 5.0f;
static unsigned s_id_points = 8;
static rt_tick_t s_id_tick;
static unsigned s_id_point;
static uint8_t s_id_warmed;
static float s_id_hz;

/* RAM-only angle-model upload staging. A point or neuron is committed only
   after every field arrived, so one rejected value cannot half-update the
   model. Reset clears all of it. */
static unsigned s_upload_point;
static unsigned s_upload_count;
static float s_upload_phase;
static float s_upload_row[ANGLE_MODEL_INPUTS + 1U];
static float s_upload_output;
static uint8_t s_upload_valid;
static uint8_t s_upload_interp;
static uint8_t s_upload_q28;
static volatile uint8_t s_upload_g_stage; /* last g-path outcome, x0 only */
static unsigned s_verify_neuron;
/* Published contention counter: 0 means the control lock was never stuck. */
volatile unsigned g_motor_lock_timeouts;
/* Command trace: op in bits 31..24, value in 23..0, plus entry/exit markers. */
static volatile uint32_t s_cmd_trace;
static volatile uint32_t s_cmd_enter;
static volatile uint8_t s_cmd_exit;
uint32_t motor_control_trace_op(void) { return s_cmd_trace; }
uint32_t motor_control_trace_enter(void) { return s_cmd_enter; }
unsigned motor_control_trace_exit(void) { return s_cmd_exit; }

static void stop_output(uint8_t mode)
{
    s_state.mode = mode;
    if (mode == 0) s_state.fault = 0;
    s_state.uq = 0;
    s_state.target_rad_s = 0;
    s_target_position_rad = s_state.position_rad;
    s_state.target_position_rad = s_target_position_rad;
    s_state.excitation_v = 0;
    s_state.voltage_clipped = 0;
    s_state.base_uq = 0;
    s_state.nn_delta_uq = 0;
    s_integral_v = 0;
    s_proportional_reference = 0;
    position_reference_reset(&s_position_reference, s_state.position_rad, 0);
    simplefoc_pwm_set_all(0, 0, 0);
}

static void fault_output(uint8_t reason)
{
    s_state.fault = reason;
    stop_output(3);
}

static void voltage(float ud, float uq, float theta)
{
    float alpha = ud * cosf(theta) - uq * sinf(theta);
    float beta = ud * sinf(theta) + uq * cosf(theta);
    simplefoc_pwm_set_all(0.5f + alpha / 12.0f,
        0.5f + (-0.5f * alpha + 0.8660254f * beta) / 12.0f,
        0.5f + (-0.5f * alpha - 0.8660254f * beta) / 12.0f);
}

int motor_control_command(char op, int value)
{
    int result = 0;
    /* A bare take would block the CLI thread forever if the control task ever
       failed to release, and the symptom would look like "UART died" while
       telemetry kept running. Bounded wait + explicit timeout result so the
       CLI can still answer and the debugger can see contention instead of
       silence. */
    static unsigned s_lock_timeouts;
    /* A nested take would be fine here (RT-Thread tracks an owner) only if
       both takes and both releases are paired. Every take below is therefore
       followed by exactly one release, and `taken` guards the tail so an early
       return cannot skip it. */
    /* Diagnostics do not touch control state, so they never take this lock.
       Before, they took it and released it, which meant a command arriving
       while the loop already held the lock nested a take/release pair on the
       same thread; any imbalance there leaves the loop waiting forever and
       looks exactly like a dead UART with live telemetry. */
    if (op == 'x' && value == 0) return 0;
    if (op == 'z' && value == 0) return 3006; /* optional position trajectory */
    /* Stage trace: shows whether motor_control_command was entered at all and
       where it stopped. Without it a hung command is indistinguishable from a
       lost byte in the UART path. */
    s_cmd_trace = ((unsigned)(unsigned char)op << 24) | (unsigned)(value & 0xFFFFFFu);
    ++s_cmd_enter;
    s_cmd_exit = 0xE0u;
    if (rt_mutex_take(s_lock, rt_tick_from_millisecond(50)) != RT_EOK) {
        ++s_lock_timeouts;
        g_motor_lock_timeouts = s_lock_timeouts;
        s_cmd_exit = 0xFEu;
        return -3;
    }
    s_cmd_exit = 0xF0u;
    int idle = s_state.mode == 0 || s_state.mode == 3;
    if (op == 'S') stop_output(0);
    else if (op == 'H') s_last_command = rt_tick_get();
    else if (op == 'W' && value == 50 &&
             (s_state.mode == 0 || s_state.mode == 3 || s_state.mode == 5) &&
             g_pwm_debug.init_status == 0) {
        /* Logic-only scope test: disconnect driver power before requesting. */
        stop_output(0);
        s_state.mode = 5;
        s_last_command = rt_tick_get();
        simplefoc_pwm_set_all(0.5f, 0.5f, 0.5f);
    }
    else if (op == 'C' && g_pwm_debug.init_status == 0 && s_state.sensor_ok && s_state.mode != 2) {
        stop_output(0);
        s_state.calibrated = 0;
        s_state.mode = 1;
        s_cal_tick = rt_tick_get();
        s_travel = 0;
        s_cal_start = 0;
        s_last_command = s_cal_tick;
    } else if (op == 'R' && value != 0 && value >= -MOTOR_MAX_RPM && value <= MOTOR_MAX_RPM &&
               (nn_controller_get_mode() != NN_MODE_ANGLE_BLENDED || (abs(value) >= 15 && abs(value) <= 30)) &&
               (nn_controller_get_mode() < NN_MODE_DYNAMIC_SHADOW || abs(value) <= 15) &&
               g_pwm_debug.init_status == 0 && s_state.calibrated && s_state.sensor_ok && s_state.mode != 1) {
        if (s_state.mode != 2) s_proportional_reference = 0.0f;
        s_integral_v = speed_transition_integral(s_state.mode,
                                                s_speed_transition_keep,
                                                s_integral_v);
        s_state.target_rad_s = value * MOTOR_TWO_PI / 60.0f;
        s_state.mode = 2;
        s_control_tick = rt_tick_get();
        s_last_command = rt_tick_get();
    } else if (op == 'j' && idle && (value == 0 || value == 1)) {
        s_speed_transition_keep = (unsigned)value;
    } else if (op == 'j' && value == 2) {
        result = (int)s_speed_transition_keep;
    } else if (op == 'r' && idle && value >= 0 && value <= 200) {
        s_proportional_tau_ms = (unsigned)value;
    } else if (op == 't' && idle && (value == 0 || (value >= 100 && value <= 2000))) {
        s_position_move_ms = (unsigned)value;
    } else if (op == 'X' && value >= -100000 && value <= 100000 &&
               nn_controller_get_mode() != NN_MODE_ANGLE_BLENDED &&
               nn_controller_get_mode() < NN_MODE_DYNAMIC_SHADOW &&
               g_pwm_debug.init_status == 0 && s_state.calibrated && s_state.sensor_ok &&
               s_state.mode != 1) {
        unsigned keep = s_position_move_ms != 0 && s_state.mode == 8;
        if (!keep)
            position_reference_reset(&s_position_reference, s_state.position_rad,
                                     s_state.feedback_speed_rad_s);
        s_target_position_rad = value / 1000.0f;
        if (s_position_move_ms)
            position_reference_target(&s_position_reference, s_target_position_rad,
                                      s_position_move_ms, s_position_speed_limit);
        s_state.target_position_rad = s_target_position_rad;
        s_state.mode = 8;
        s_state.fault = 0;
        if (!keep) s_integral_v = 0;
        s_control_tick = rt_tick_get();
        s_last_command = s_control_tick;
    } else if (op == 'O' && value != 0 && value >= -MOTOR_OPENLOOP_MAX_RPM && value <= MOTOR_OPENLOOP_MAX_RPM &&
               g_pwm_debug.init_status == 0) {
        s_state.target_rad_s = value * MOTOR_TWO_PI / 60.0f;
        s_state.mode = 4;
        s_state.fault = 0;
        s_control_tick = rt_tick_get();
        s_last_command = s_control_tick;
    } else if ((op == 'B' || op == 'E') && (s_state.mode == 0 || s_state.mode == 3) &&
               value != 0 && value >= -MOTOR_OPENLOOP_MAX_RPM && value <= MOTOR_OPENLOOP_MAX_RPM) {
        if (op == 'B') s_sweep_start_rpm = value;
        else s_sweep_end_rpm = value;
    } else if (op == 'D' && (s_state.mode == 0 || s_state.mode == 3) && value >= 1 && value <= 600) {
        s_sweep_seconds = (uint32_t)value;
    } else if (op == 'F' && value == 1 && (s_state.mode == 0 || s_state.mode == 3) &&
               g_pwm_debug.init_status == 0 &&
               ((s_sweep_start_rpm > 0) == (s_sweep_end_rpm > 0))) {
        s_state.mode = 6;
        s_state.fault = 0;
        s_state.target_rad_s = s_sweep_start_rpm * MOTOR_TWO_PI / 60.0f;
        s_sweep_tick = s_control_tick = rt_tick_get();
        s_last_command = s_sweep_tick;
    } else if (op == 'P' && value >= 0 && value <= 200 &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        s_kp = value / 1000.0f;
    } else if (op == 'Q' && value >= 0 && value <= 2000 &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        s_ki = value / 1000.0f;
    } else if (op == 'Z' && value >= 0 && value <= 20000 &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        s_position_kp = value / 1000.0f;
    } else if (op == 'Y' && value >= 1 && value <= MOTOR_MAX_RPM &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        s_position_speed_limit = value * MOTOR_TWO_PI / 60.0f;
    } else if (op == 'G' && value >= NN_MODE_OFF && value <= NN_MODE_DYNAMIC_LINEAR &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        result = nn_controller_set_mode(value);
    } else if (op == 'K' && value >= 0 && value <= 1000 &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        result = nn_controller_set_alpha_milli(value);
    } else if (op == 'T' && (value == 0 || value == 1) &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        result = nn_controller_set_learning(value);
    } else if (op == 'f' && idle) {
        result = nn_controller_select_ff_weight(value);
    } else if (op == 'w' && idle) {
        result = nn_controller_set_ff_weight(value);
    } else if (op == 'a' && idle) {
        result = nn_controller_set_angle_phase(value);
    } else if (op == 'M' && value >= 0 && value < (int)ANGLE_SHADOW_NORM_POINTS &&
               idle) {
        /* Select normalization point; 0..7.  Re-select resets the row so an
           aborted upload cannot leave a stale coefficient behind. */
        s_upload_point = (unsigned)value;
        s_upload_count = 0U;
        s_upload_output = 0.0f;
        memset(s_upload_row, 0, sizeof(s_upload_row));
        s_upload_valid = 1U;
    } else if (op == 'u' && idle && s_upload_valid &&
               value >= -3142 && value <= 3142) {
        /* Phase for the selected point, milli-radians. */
        s_upload_phase = (float)value / 1000.0f;
    } else if (op == 'g' && idle && s_upload_valid) {
        /* One normalized weight per call: value = 20000 + index*4000 + code.
           The payload uses 0..1999 for positive magnitudes
           and 2000..3999 for negative. A 4000 stride keeps sign and index
           fields disjoint; the largest encoded value is below 68000. */
        s_upload_g_stage = 11U;
        if (abs(value) < 20000 || abs(value) > 67999) {
            s_upload_g_stage = 12U;
        } else {
            unsigned field = (unsigned)abs(value);
            unsigned index = (field - 20000U) / 4000U;
            unsigned digit = (field - 20000U) % 4000U;
            float magnitude = (float)(digit >= 2000U ? digit - 2000U : digit) / 1000.0f;
            if (digit >= 2000U) magnitude = -magnitude;
            if (index > 11U || digit > 3999U) s_upload_g_stage = 13U;
            else if (index != s_upload_count) s_upload_g_stage = 14U;
            else {
                s_upload_row[index + 1U] = magnitude;
                ++s_upload_count;
                s_upload_g_stage = 15U;
            }
        }
        if (s_upload_g_stage != 15U) result = -1;
    } else if (op == 'i' && idle && s_upload_valid && value >= -999999 && value <= 999999) {
        /* Output weight of the selected neuron, 1e-4 units. The old +/-50
           limit truncated a checkpoint weight of 0.0949 to 0.005. */
        s_upload_output = (float)value / 10000.0f;
    } else if (op == 'h' && idle && s_upload_valid && value >= -99000 && value <= 99000) {
        /* Hidden bias of the selected neuron, signed 1e-4 units. The neuron
           comes from the last M, and the largest compiled bias (1.834) fits
           the 1e5 argument limit, so no packing is needed. */
        result = nn_controller_set_angle_hidden_bias(s_upload_point, value);
    } else if (op == 'p' && idle && s_upload_valid) {
        /* Commit point: normalization speed (milli-RPM) + the stored phase. */
        result = nn_controller_set_angle_norm(
            s_upload_point, (float)value / 1000.0f, s_upload_phase);
    } else if (op == 'n' && idle && s_upload_valid) {
        /* Points walk the neuron table in the same order as the compiled
           model: point 0 is neuron 0, point 5 neuron 5. Mixing the two uses
           of M (select phase point vs select neuron) silently wrote neuron 0
           repeatedly while phase point 5 received it, which made every upload
           of a zero-weight network keep the compiled weights. */
        if (s_upload_count != 12U || s_upload_point >= ANGLE_MODEL_HIDDEN) result = -1;
        else {
            result = nn_controller_set_angle_model(s_upload_row, s_upload_output,
                                                   s_upload_point);
            s_upload_count = 0U;
        }
    } else if (op == 'z' && idle && value == 0) {
        /* Unreachable for z: handled before the lock for diagnostics. */
        result = 3003;
    } else if (op == 'm' && idle && value >= 0 && value <= 3) {
        /* 0/2 nearest-point hold, 1/3 linear interpolation between points;
           2/3 additionally declare that the g fields use the compact field.
           Bit 0 selects interpolation exactly as before, so 0 stays
           bit-identical to the compiled single-phase model. */
        s_upload_interp = (uint8_t)(value & 1);
        result = nn_controller_set_angle_interp(value & 1);
        if (result == 0) s_upload_q28 = (uint8_t)(value >> 1);
        else result = -1;
    } else if (op == 'x' && idle && value == 0) {
        /* Read-only: the response carries the staged upload state. */
        result = 0;
    } else if (op == 's' && idle && value >= 0 && value <= 11) {
        /* Read-only: compiled input std for feature index 0..11, in milli. */
        result = (int)nn_controller_input_std_milli((unsigned)value);
    } else if (op == 'y' && idle) {
        /* Read-only upload verification, one query per coefficient:
           y<neuron*100+slot> answers the stored value in 1e-4 units. */
        s_verify_neuron = (unsigned)(value / 100);
        result = nn_controller_angle_coefficient(s_verify_neuron, (unsigned)(value % 100));
    } else if (op == 'q' && idle && value >= -1000 && value <= 1000) {
        result = nn_controller_set_angle_bias(value);
    } else if (op == 'v' && idle && value >= 0 && value <= 2) {
        s_estimator_mode = (unsigned)value;
        g_motor_timing.estimator_mode = (uint8_t)value;
    } else if (op == 'e' && value == 0) {
        /* Read-only timing query; host_cli serializes its snapshot. */
    } else if (op == 'd' && idle && value == 0) {
        memset((void *)&g_motor_timing, 0, sizeof(g_motor_timing));
        g_motor_timing.clock_hz = SystemCoreClock;
        g_motor_timing.estimator_mode = (uint8_t)s_estimator_mode;
        s_have_loop_cycles = 0;
    } else if (op == 'V' && value >= 100 && value <= 1500 &&
               (s_state.mode == 0 || s_state.mode == 3)) {
        s_limit = value / 1000.0f;
    } else if (op == 'J' && idle && value != 0 && abs(value) <= MOTOR_MAX_RPM) {
        s_id_rpm = value;
    } else if (op == 'A' && idle && value >= 5 && value <= 300) {
        s_id_amplitude = value / 1000.0f;
    } else if ((op == 'L' || op == 'U') && idle && value >= 100 && value <= 10000) {
        if (op == 'L') s_id_first_hz = value / 1000.0f;
        else s_id_last_hz = value / 1000.0f;
    } else if (op == 'N' && idle && value >= 2 && value <= 24) {
        s_id_points = (unsigned)value;
    } else if (op == 'I' && value == 1 && idle &&
               g_pwm_debug.init_status == 0 && s_state.sensor_ok &&
               s_state.calibrated && s_kp > 0 &&
               s_id_last_hz >= s_id_first_hz && s_id_amplitude < s_limit) {
        s_state.mode = 7;
        s_state.fault = 0;
        s_state.target_rad_s = s_id_rpm * MOTOR_TWO_PI / 60.0f;
        s_state.excitation_v = 0;
        s_state.identification_hz = 0;
        s_state.excitation_phase = 0;
        s_state.identification_point = 1;
        s_state.identification_stage = 0;
        s_id_tick = s_last_command = rt_tick_get();
        s_id_point = 0;
        s_id_hz = s_id_first_hz;
        s_id_warmed = 0;
    } else result = -1;
    s_cmd_exit = 0xFFu;
    rt_mutex_release(s_lock);
    return result;
}

void motor_control_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_upload_valid = 0U;
    s_upload_count = 0U;
    s_upload_point = 0U;
    s_upload_phase = 0.0f;
    s_upload_output = 0.0f;
    s_upload_interp = 0U;
    memset(s_upload_row, 0, sizeof(s_upload_row));
    s_lock = rt_mutex_create("mctl", RT_IPC_FLAG_PRIO);
    RT_ASSERT(s_lock != RT_NULL);
    simplefoc_pwm_init();
    nn_controller_init();
    speed_estimator_reset(&s_speed_estimator);
    g_motor_timing.clock_hz = SystemCoreClock;
    simplefoc_pwm_set_all(0, 0, 0);
}

unsigned angle_upload_point(void) { return s_upload_point; }
unsigned angle_upload_count(void) { return s_upload_count; }
int angle_upload_row_valid(void) { return s_upload_valid ? 1 : 0; }
unsigned angle_upload_interp(void) { return s_upload_interp; }
unsigned angle_upload_g_stage(void) { return s_upload_g_stage; }

void motor_control_get_state(motor_state_t *state)
{
    if (state == NULL) return;
    /* Bounded wait: telemetry must keep flowing even if the control task is
       stuck, otherwise a lock bug looks identical to a dead MCU. */
    if (rt_mutex_take(s_lock, rt_tick_from_millisecond(20)) != RT_EOK) {
        ++g_motor_lock_timeouts;
        return;
    }
    *state = s_state;
    state->speed_kp = s_kp;
    state->voltage_limit = s_limit;
    state->position_kp = s_position_kp;
    state->position_speed_limit = s_position_speed_limit;
    state->target_position_rad = s_target_position_rad;
    state->nn_mode = (uint8_t)nn_controller_get_mode();
    state->nn_alpha_milli = nn_controller_get_alpha_milli();
    state->nn_learning = nn_controller_learning_enabled();
    rt_mutex_release(s_lock);
}

void motor_control_get_timing(motor_timing_t *timing)
{
    if (!timing) return;
    rt_mutex_take(s_lock, RT_WAITING_FOREVER);
    *timing = g_motor_timing;
    rt_mutex_release(s_lock);
}

static void update_sensor(void)
{
    uint16_t raw;
    float delta;
    rt_tick_t now = rt_tick_get();
    uint32_t sensor_start = motor_cycles();
    int sensor_result = as5600_read_raw(&raw);
    uint32_t sensor_elapsed = motor_cycles() - sensor_start;
    /* Midpoint of the bounded transaction approximates sample time. It does
       not remove the AS5600 internal conversion/filter delay. */
    uint32_t sample_cycles = sensor_start + sensor_elapsed / 2U;
    g_motor_timing.sensor_cycles = sensor_elapsed;
    if (sensor_elapsed > g_motor_timing.max_sensor_cycles)
        g_motor_timing.max_sensor_cycles = sensor_elapsed;

    if (sensor_result != 0) {
        ++s_sensor_fail_streak;
        s_state.sensor_ok = 0U;
        s_state.speed_rad_s = 0.0f;
        s_have_last = 0;
        s_speed_delta_rad = 0.0f;
        s_speed_window_ticks = 0;
        speed_estimator_reset(&s_speed_estimator);
        s_have_sample_cycles = 0;
        s_state.calibrated = 0;
        /* Encoder failure blocks calibration/closed-loop, but must not stop
           open-loop tests or prevent configuring a sweep while stopped. */
        /* Allow a short I2C transient during calibration/control. The
           bounded streak still faults a genuinely disconnected sensor. */
        if (s_sensor_fail_streak >= 20U &&
            (s_state.mode == 1 || s_state.mode == 2 || s_state.mode == 7 || s_state.mode == 8))
            fault_output(1);
        return;
    }
    s_sensor_fail_streak = 0U;
    s_state.sensor_ok = 1U;
    s_state.raw_angle = raw;
    s_state.angle_rad = as5600_raw_to_rad(raw);
    ++g_motor_timing.samples;
    if (s_have_last) {
        delta = (float)((int32_t)raw - (int32_t)s_last_raw);
        if (delta > 2048.0f) delta -= 4096.0f;
        if (delta < -2048.0f) delta += 4096.0f;
        rt_tick_t dt_ticks = now - s_sample_tick;
        if (dt_ticks > 0 && dt_ticks < RT_TICK_PER_SECOND / 2U) {
            s_speed_delta_rad += delta * MOTOR_TWO_PI / 4096.0f;
            s_speed_window_ticks += dt_ticks;
            if (s_speed_window_ticks >=
                (RT_TICK_PER_SECOND * SPEED_ESTIMATION_WINDOW_MS + 999U) / 1000U) {
                float window_s = (float)s_speed_window_ticks / RT_TICK_PER_SECOND;
                float speed = s_speed_delta_rad / window_s;
                /* Keep the original tick-based estimator bit-for-bit in v0;
                   observer/regression run in shadow on cycle timestamps. */
                s_speed_estimator.legacy += SPEED_ESTIMATION_FILTER_ALPHA *
                    (speed - s_speed_estimator.legacy);
                s_speed_delta_rad = 0.0f;
                s_speed_window_ticks = 0;
            }
        }
        if (s_have_sample_cycles) {
            float dt_s = (float)(uint32_t)(sample_cycles - s_sample_cycles) / SystemCoreClock;
            float legacy = s_speed_estimator.legacy;
            speed_estimator_update(&s_speed_estimator, delta * MOTOR_TWO_PI / 4096.0f, dt_s);
            s_speed_estimator.legacy = legacy;
        }
        s_travel += delta * MOTOR_TWO_PI / 4096.0f;
    }
    s_last_raw = raw;
    s_have_last = 1U;
    s_sample_tick = now;
    s_sample_cycles = sample_cycles;
    s_have_sample_cycles = 1;
    s_state.speed_rad_s = speed_estimator_get(&s_speed_estimator, s_estimator_mode);
    g_motor_timing.legacy_speed = s_direction * s_speed_estimator.legacy;
    g_motor_timing.regression_speed = s_direction * s_speed_estimator.regression;
    g_motor_timing.observer_speed = s_direction * s_speed_estimator.observer;
    s_state.position_rad = s_direction * s_travel;
}

static void identification_excitation(rt_tick_t now)
{
    float elapsed = (float)(now - s_id_tick) / RT_TICK_PER_SECOND;
    if (!s_id_warmed) {
        if (elapsed < 3.0f) return;
        s_id_warmed = 1;
        s_id_tick = now;
        elapsed = 0;
    }
    float hz = s_id_hz;
    if (elapsed * hz >= 7.0f) {
        if (++s_id_point >= s_id_points) {
            stop_output(0);
            return;
        }
        s_id_tick = now;
        elapsed = 0;
        hz = s_id_first_hz * powf(s_id_last_hz / s_id_first_hz,
                    (float)s_id_point / (s_id_points - 1U));
        s_id_hz = hz;
    }
    float cycles = elapsed * hz;
    s_state.identification_point = s_id_point + 1U;
    s_state.identification_hz = hz;
    s_state.identification_stage = cycles < 2.0f ? 1 : 2;
    s_state.excitation_phase = MOTOR_TWO_PI * (cycles - floorf(cycles));
    s_state.excitation_v = s_id_amplitude * sinf(s_state.excitation_phase);
}

void motor_control_thread(void *parameter)
{
    (void)parameter;
    rt_tick_t next = rt_tick_get();
    while (1) {
        uint32_t loop_start = motor_cycles();
        rt_mutex_take(s_lock, RT_WAITING_FOREVER);
        if (s_have_loop_cycles) {
            uint32_t period = loop_start - s_loop_cycles;
            g_motor_timing.period_cycles = period;
            if (period > g_motor_timing.max_period_cycles) g_motor_timing.max_period_cycles = period;
            /* >2.5 ms distinguishes a late period from cycle-level jitter. */
            if (period > SystemCoreClock / 400U) ++g_motor_timing.late_periods;
        }
        s_loop_cycles = loop_start;
        s_have_loop_cycles = 1;
        update_sensor();
        s_state.feedback_speed_rad_s = s_direction * s_state.speed_rad_s;
        rt_tick_t now = rt_tick_get();
        s_state.sample_tick = now;
        if (s_cal_zero_pending &&
            now - s_cal_zero_tick >=
                (RT_TICK_PER_SECOND * CALIBRATION_ZERO_SETTLE_MS + 999U) / 1000U) {
            s_travel = 0.0f;
            s_state.position_rad = 0.0f;
            s_target_position_rad = 0.0f;
            s_state.target_position_rad = 0.0f;
            s_state.speed_rad_s = 0.0f;
            s_speed_delta_rad = 0.0f;
            s_speed_window_ticks = 0;
            speed_estimator_reset(&s_speed_estimator);
            s_cal_zero_pending = 0U;
        }
        if ((s_state.mode == 1 || s_state.mode == 2 || s_state.mode == 4 || s_state.mode == 5 || s_state.mode == 6 || s_state.mode == 7 || s_state.mode == 8) &&
            now - s_last_command > RT_TICK_PER_SECOND) fault_output(2);
        simplefoc_pwm_snapshot();
        if (s_state.mode == 1) {
            float t = (float)(now - s_cal_tick) / RT_TICK_PER_SECOND;
            if (t < 0.5f) {
                voltage(CALIBRATION_ALIGN_VOLTAGE, 0, 0);
                s_cal_start = s_travel;
            } else if (t < 2.5f) {
                voltage(CALIBRATION_ALIGN_VOLTAGE, 0,
                    MOTOR_TWO_PI * (t - 0.5f) / 2.0f);
            } else if (t < 3.0f) voltage(0.6f, 0, 0);
            else {
                float moved = s_travel - s_cal_start;
                s_state.calibration_mrad = (int32_t)(moved * 1000.0f);
                /* One electric revolution should move 2pi / 7 mechanically. */
                if (fabsf(moved) > 0.45f && fabsf(moved) < 1.35f) {
                    s_direction = moved > 0 ? 1 : -1;
                    s_zero = s_direction * 7.0f * s_state.angle_rad;
                    s_state.calibrated = 1;
                    stop_output(0);
                    /* Let the rotor settle after PWM release before defining position zero. */
                    s_cal_zero_tick = now;
                    s_cal_zero_pending = 1U;
                } else fault_output(3);
            }
        } else if (s_state.mode == 2 || s_state.mode == 7 || s_state.mode == 8) {
            if (s_state.mode == 7) identification_excitation(now);
            if (s_state.mode == 0) {
                timing_finish(loop_start);
                rt_mutex_release(s_lock);
                rt_thread_delay_until(&next, ENCODER_SAMPLE_PERIOD_MS);
                continue;
            }
            float speed = s_direction * s_state.speed_rad_s;
            float dt = (float)(now - s_control_tick) / RT_TICK_PER_SECOND;
            if (dt <= 0.0f || dt > 0.05f) dt = ENCODER_SAMPLE_PERIOD_MS / 1000.0f;
            s_control_tick = now;
            if (s_state.mode == 8) {
                if (s_position_move_ms) {
                    s_state.target_rad_s = position_reference_update(&s_position_reference,
                        dt, s_state.position_rad, speed, s_position_kp, s_position_speed_limit);
                } else {
                    float position_error = s_target_position_rad - s_state.position_rad;
                    s_state.target_rad_s = position_error * s_position_kp;
                }
                if (s_state.target_rad_s > s_position_speed_limit)
                    s_state.target_rad_s = s_position_speed_limit;
                if (s_state.target_rad_s < -s_position_speed_limit)
                    s_state.target_rad_s = -s_position_speed_limit;
            }
            float error = s_state.target_rad_s - speed;
            unsigned position_holding = s_state.mode == 8 && s_position_move_ms &&
                                        s_position_reference.holding;
            float base_uq;
            float uq;
            if (s_state.mode == 7) {
                /* Keep identification comparable: P baseline plus injected sine. */
                base_uq = s_kp * error + s_state.excitation_v;
            } else {
                /* Identified-model feed-forward plus PI speed control. */
                if (!position_holding) s_integral_v += s_ki * error * dt;
                if (s_integral_v > s_limit) s_integral_v = s_limit;
                if (s_integral_v < -s_limit) s_integral_v = -s_limit;
                /* Apply only in speed mode: position-loop dynamics unchanged. */
                float p_error = error;
                if (s_state.mode == 2) {
                    s_proportional_reference = speed_proportional_reference(
                        s_proportional_reference, s_state.target_rad_s, dt,
                        s_proportional_tau_ms);
                    if (s_proportional_tau_ms != 0U)
                        p_error = s_proportional_reference - speed;
                }
                base_uq = s_state.target_rad_s / MOTOR_MODEL_K +
                    s_kp * p_error + s_integral_v;
            }
            s_state.base_uq = base_uq;
            nn_features_t features = {
                s_state.target_rad_s, speed, error,
                s_target_position_rad, s_state.position_rad, base_uq
            };
            /* Record the tensor actually handed to inference. The control loop
               can take another sensor sample before the 100 Hz L frame reads
               state->position_rad, which made telemetry disagree with the
               inputs used for this prediction. */
            s_state.nn_input_position_rad = features.measured_position_rad;
            s_state.nn_input_speed_rad_s = features.measured_speed_rad_s;
            s_state.nn_delta_uq = (s_state.mode == 7) ? 0.0f :
                nn_controller_predict(&features);
            uq = nn_controller_apply(base_uq, s_state.nn_delta_uq);
            s_state.voltage_clipped = uq > s_limit || uq < -s_limit;
            if (uq > s_limit) {
                uq = s_limit;
                if (s_state.mode != 7 && !position_holding && error > 0.0f)
                    s_integral_v -= s_ki * error * dt;
            }
            if (uq < -s_limit) {
                uq = -s_limit;
                if (s_state.mode != 7 && !position_holding && error < 0.0f)
                    s_integral_v -= s_ki * error * dt;
            }
            s_state.uq = uq;
            if (fabsf(speed) > 1.2f * MOTOR_MAX_RPM * MOTOR_TWO_PI / 60.0f) fault_output(4);
            else voltage(0, uq, s_direction * 7.0f * s_state.angle_rad - s_zero);
        } else if (s_state.mode == 4 || s_state.mode == 6) {
            if (s_state.mode == 6) {
                float elapsed = (float)(now - s_sweep_tick) / RT_TICK_PER_SECOND;
                if (elapsed >= s_sweep_seconds) {
                    stop_output(0);
                    timing_finish(loop_start);
                    rt_mutex_release(s_lock);
                    rt_thread_delay_until(&next, ENCODER_SAMPLE_PERIOD_MS);
                    continue;
                }
                float rpm = s_sweep_start_rpm +
                    (s_sweep_end_rpm - s_sweep_start_rpm) * elapsed / s_sweep_seconds;
                s_state.target_rad_s = rpm * MOTOR_TWO_PI / 60.0f;
            }
            float dt = (float)(now - s_control_tick) / RT_TICK_PER_SECOND;
            s_control_tick = now;
            s_openloop_angle += 7.0f * s_state.target_rad_s * dt;
            s_openloop_angle = fmodf(s_openloop_angle, MOTOR_TWO_PI);
            s_state.uq = s_limit;
            voltage(0, s_limit, s_openloop_angle);
        }
        timing_finish(loop_start);
        rt_mutex_release(s_lock);
        rt_thread_delay_until(&next, ENCODER_SAMPLE_PERIOD_MS);
    }
}
