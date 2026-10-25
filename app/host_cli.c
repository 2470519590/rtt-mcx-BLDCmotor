#include "host_cli.h"

#include "board.h"
#include "board_uart.h"
#include "fsl_lpuart.h"
#include "motor_control.h"
#include "nn_controller.h"
#include "angle_shadow.h"
#include "simplefoc_pwm.h"
#include "rtthread.h"
#include "rthw.h"
#include "uart_rx_queue.h"
#include <stdlib.h>

/* RAM-only diagnostic state: inspect this directly in debugger Watch. */
volatile uint32_t g_uart_rx_bytes;
volatile uint32_t g_uart_rx_lines;
volatile uint32_t g_uart_tx_frames;
volatile uint32_t g_uart_ack_count;
volatile uint32_t g_uart_rx_overflows;
volatile uint8_t g_uart_last_byte;
volatile char g_uart_last_command[24];
volatile int32_t g_uart_last_result;
volatile uint32_t g_uart_rx_irq_count;
volatile uint32_t g_uart_rx_hardware_errors;
volatile uint32_t g_uart_rx_hardware_overruns;
volatile uint32_t g_uart_rx_ring_overflows;
volatile uint32_t g_uart_rx_last_error_flags;
volatile uint32_t g_uart_tx_failures;
static uart_rx_queue_t s_rx_queue;

#define UART_RX_ERRORS ((uint32_t)(kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | \
                                  kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag))

/* RX exclusively belongs to this ISR; TX/command execution stays in CLI.
   Error flags must be cleared even when RDRF is no longer set. */
void LPUART3_IRQHandler(void)
{
    rt_interrupt_enter();
    ++g_uart_rx_irq_count;
    LPUART_Type *uart = rt_hw_debug_uart();
    for (unsigned budget = 0; budget < 32U; ++budget) {
        uint32_t flags = LPUART_GetStatusFlags(uart);
        if (flags & UART_RX_ERRORS) {
            uint32_t errors = flags & UART_RX_ERRORS;
            ++g_uart_rx_hardware_errors;
            if (errors & kLPUART_RxOverrunFlag) ++g_uart_rx_hardware_overruns;
            g_uart_rx_last_error_flags = errors;
            uart_rx_queue_error(&s_rx_queue);
            (void)LPUART_ClearStatusFlags(uart, errors);
        }
        if (!(flags & kLPUART_RxDataRegFullFlag)) break;
        uint8_t byte = LPUART_ReadByte(uart);
        ++g_uart_rx_bytes;
        g_uart_last_byte = byte;
        uart_rx_queue_push(&s_rx_queue, byte);
    }
    g_uart_rx_ring_overflows = s_rx_queue.overflows;
    rt_interrupt_leave();
    SDK_ISR_EXIT_BARRIER;
}

static void cli_write(LPUART_Type *uart, const uint8_t *bytes, size_t length)
{
    status_t status = LPUART_WriteBlocking(uart, bytes, length);
    g_host_uart_debug.tx_status = status;
    if (status != kStatus_Success) ++g_uart_tx_failures;
}

static void cli_rx_start(LPUART_Type *uart)
{
    uart_rx_queue_reset(&s_rx_queue);
    /* Initialize after RT-Thread has started, before the potentially long banner. */
    LPUART_SetRxFifoWatermark(uart, 0U);
    uint32_t errors = LPUART_GetStatusFlags(uart) & UART_RX_ERRORS;
    if (errors) {
        ++g_uart_rx_hardware_errors;
        if (errors & kLPUART_RxOverrunFlag) ++g_uart_rx_hardware_overruns;
        g_uart_rx_last_error_flags = errors;
        uart_rx_queue_error(&s_rx_queue);
        (void)LPUART_ClearStatusFlags(uart, errors);
    }
    NVIC_SetPriority(LPUART3_IRQn, 5U);
    EnableIRQ(LPUART3_IRQn);
    LPUART_EnableInterrupts(uart, kLPUART_RxDataRegFullInterruptEnable |
        kLPUART_RxOverrunInterruptEnable | kLPUART_NoiseErrorInterruptEnable |
        kLPUART_FramingErrorInterruptEnable | kLPUART_ParityErrorInterruptEnable);
}

static void cli_print_state(void)
{
    static uint32_t sequence;
    char frame[160]; /* T telemetry or I identification trace */
    /* Retain the last valid snapshot if the bounded getter times out. */
    static motor_state_t state;
    int32_t speed_mrad_s;

    motor_control_get_state(&state);
    /* Report the calibrated mechanical direction used by the controller. */
    speed_mrad_s = (int32_t)(state.feedback_speed_rad_s * 1000.0f);
    int length;
    if (state.nn_learning && state.mode != 0) {
        uint32_t tick_ms = (uint32_t)((uint64_t)state.sample_tick * 1000U / RT_TICK_PER_SECOND);
        /* position_mrad and speed_mrad_s are the tensor of the SAME inference
           that produced nn_delta_uq/base_uq, so a host can reproduce the
           prediction exactly; base/target/uq are 10 ms later, which only
           matters for comparisons that need them in the same instant. */
        length = rt_snprintf(frame, sizeof(frame),
            "L,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%d,%d,%u,%u\r\n",
            sequence++, tick_ms,
            (int)(state.target_position_rad * 1000),
            (int)(state.nn_input_position_rad * 1000),
            (int)(state.target_rad_s * 1000),
            (int)(state.nn_input_speed_rad_s * 1000),
            (int)(state.base_uq * 1000), (int)(state.nn_delta_uq * 1000),
            (int)(state.uq * 1000), state.mode, state.sensor_ok,
            state.calibrated, state.fault, state.voltage_clipped,
            (int)(state.speed_kp * 1000), (int)(state.voltage_limit * 1000),
            (unsigned)state.nn_alpha_milli, (unsigned)state.nn_mode);
    } else if (state.mode == 7) {
        /* One frame type at a time keeps 100 Hz acquisition within 115200 baud.
           All values come from one mutex-protected control-loop snapshot. */
        uint32_t tick_ms = (uint32_t)((uint64_t)state.sample_tick * 1000U / RT_TICK_PER_SECOND);
        length = rt_snprintf(frame, sizeof(frame),
            "I,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%d,%d\r\n",
            sequence++, tick_ms, state.identification_point, state.identification_stage,
            (int)(state.identification_hz * 1000), (int)(state.excitation_phase * 1000),
            (int)(state.position_rad * 1000), (int)(state.feedback_speed_rad_s * 1000),
            (int)(state.target_rad_s * 1000), (int)(state.excitation_v * 1000),
            (int)(state.uq * 1000), state.sensor_ok, state.calibrated,
            state.mode, state.fault, state.voltage_clipped, state.raw_angle,
            (int)(state.speed_kp * 1000), (int)(state.voltage_limit * 1000));
    } else {
        length = rt_snprintf(frame, sizeof(frame), "T,%u,%u,%u,%d,%u,%u,%u,%d,%d,%u,%d\r\n",
                            sequence++, rt_tick_get(), state.raw_angle,
                            speed_mrad_s, state.sensor_ok, state.mode,
                            state.calibrated, (int)(state.target_rad_s * 1000),
                            (int)(state.uq * 1000), state.fault,
                            state.calibration_mrad);
    }
    if (length > 0 && (size_t)length < sizeof(frame)) {
        /* Single telemetry writer; polling TX stays preemptible by sampling. */
        cli_write(rt_hw_debug_uart(), (const uint8_t *)frame, (size_t)length);
        rt_hw_debug_uart_snapshot();
        if (g_host_uart_debug.tx_status == kStatus_Success)
            ++g_uart_tx_frames;
    }
}

static void cli_print_timing(void)
{
    motor_timing_t timing;
    char frame[224];
    motor_control_get_timing(&timing);
    int n = rt_snprintf(frame, sizeof(frame),
        "E,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d\r\n",
        timing.estimator_mode, timing.clock_hz, timing.samples,
        timing.period_cycles, timing.max_period_cycles,
        timing.sensor_cycles, timing.max_sensor_cycles,
        timing.loop_cycles, timing.max_loop_cycles,
        timing.late_periods, timing.over_budget_loops,
        (int)(timing.legacy_speed * 1000), (int)(timing.regression_speed * 1000),
        (int)(timing.observer_speed * 1000));
    if (n > 0 && n < (int)sizeof(frame))
        cli_write(rt_hw_debug_uart(), (const uint8_t *)frame, (size_t)n);
}

void host_cli_thread(void *parameter)
{
    (void)parameter;
    /* Keep the task entry point for the existing RT-Thread startup. */
    LPUART_Type *uart = rt_hw_debug_uart();
    cli_rx_start(uart);
    /* Use the same polling transport as the telemetry and command replies. */
    {
        static const char banner[] =
            "encoder telemetry ready; period_ms=10; format=T\r\n"
            "motor_cli fw=nnref1-uartirq1; max_rpm=1000; estimator=v0/1/2; timing=e0; transition=j0/1; pref=r_ms; nn=off\r\n";
        cli_write(uart, (const uint8_t *)banner, sizeof(banner) - 1U);
    }
    rt_tick_t next = rt_tick_get();
    rt_tick_t sent = next;
    char command[24];
    unsigned length = 0;
    uint8_t discard_command = 0;
    uint32_t generation = 0;
    while (1) {
        /* Bound command work so continuous input cannot starve telemetry. */
        for (unsigned budget = 0; budget < 128U; ++budget) {
            uint8_t byte = 0;
            rt_base_t level = rt_hw_interrupt_disable();
            int received = uart_rx_queue_pop(&s_rx_queue, &generation, &byte);
            rt_hw_interrupt_enable(level);
            if (received < 0) {
                length = 0;
                discard_command = 0;
                continue;
            }
            if (!received) break;
            char ch = (char)byte;
            if (discard_command) {
                if (ch == '\n' || ch == '\r') discard_command = 0;
                continue;
            }
            if (ch == '\n' || ch == '\r') {
                if (length) {
                    command[length] = 0;
                    /* Do not process the line first: any fault or unexpected
                       return inside motor_control_command would otherwise skip
                       the reply, and one silent command made the whole session
                       look dead while telemetry kept running. */
                    ++g_uart_rx_lines;
                    for (unsigned i = 0; i < length; ++i)
                        g_uart_last_command[i] = command[i];
                    g_uart_last_command[length] = 0;
                    char *end;
                    long value = strtol(command + 1, &end, 10);
                    int result = -1;
                    int valid = end != command + 1 && *end == 0 &&
                                value >= -1000000 && value <= 1000000;
                    /* D0 replies from this loop only, so the counters stay
                       readable even when motor_control_command misbehaves. */
                    if (valid && command[0] == 'D' && (value == 0 || value >= 100000)) {
                        char dump[120];
                        int m = rt_snprintf(dump, sizeof(dump),
                            "D,%u,%u,%u,%ld,%d,%s\r\n", (unsigned)g_uart_rx_bytes,
                            (unsigned)g_uart_rx_lines, (unsigned)g_uart_rx_overflows,
                            value, (int)g_uart_last_result, (const char *)g_uart_last_command);
                        if (m > 0 && m < (int)sizeof(dump))
                            cli_write(uart, (const uint8_t *)dump, (size_t)m);
                        length = 0;
                        continue;
                    }
                    if (valid && command[0] == 'E' && value == 0) {
                        /* Command-path trace: which op reached the handler, how
                           many did, and where the last one stopped. */
                        char dump[96];
                        int m = rt_snprintf(dump, sizeof(dump),
                            "E,%lx,%u,%u,%u\r\n", (unsigned long)motor_control_trace_op(),
                            (unsigned)motor_control_trace_enter(),
                            motor_control_trace_exit(), (unsigned)g_motor_lock_timeouts);
                        if (m > 0 && m < (int)sizeof(dump))
                            cli_write(uart, (const uint8_t *)dump, (size_t)m);
                        length = 0;
                        continue;
                    }
                    if (valid && command[0] == 'b' && value == 0) {
                        char dump[128];
                        int m = rt_snprintf(dump, sizeof(dump), "B,%u,%u,%u,%u,%u,%u,%u\r\n",
                            g_uart_rx_irq_count, g_uart_rx_bytes, g_uart_rx_hardware_errors,
                            g_uart_rx_hardware_overruns, g_uart_rx_ring_overflows,
                            g_uart_rx_last_error_flags, g_uart_tx_failures);
                        if (m > 0 && m < (int)sizeof(dump))
                            cli_write(uart, (const uint8_t *)dump, (size_t)m);
                        length = 0;
                        continue;
                    }
                    if (valid)
                        result = motor_control_command(command[0], (int)value);
                    g_uart_last_result = result;
                    if (command[0] != 'H') {
                        char reply[80];
                        int n;
                        if (command[0] == 'x') {
                            /* Staged angle-model upload state, read-only.
                               Value is the accepted-result slot, so 0 means
                               the query itself succeeded. */
                            n = rt_snprintf(reply, sizeof(reply),
                                "X,%d,%d,%u,%d,%u,%d,%d\r\n", result,
                                (int)angle_upload_point(), (unsigned)angle_upload_count(),
                                angle_upload_row_valid(), (unsigned)angle_shadow_norm_count(),
                                angle_upload_interp(), (int)angle_upload_g_stage());
                        } else {
                            n = rt_snprintf(reply, sizeof(reply),
                                "ACK,%c,%d,pwm_init=%d\r\n", command[0], result,
                                (int)g_pwm_debug.init_status);
                        }
                        if (n > 0 && n < (int)sizeof(reply)) {
                            cli_write(uart, (const uint8_t *)reply, (size_t)n);
                            if (g_host_uart_debug.tx_status == kStatus_Success)
                                ++g_uart_ack_count;
                        }
                        /* e0 asks for a timing dump; answer it after the ACK
                           so the reply order is stable for the host parser. */
                        if (command[0] == 'e' && result == 0) cli_print_timing();
                    }
                    length = 0;
                }
            } else if (length < sizeof(command) - 1) command[length++] = ch;
            else {
                length = 0;
                discard_command = 1;
                ++g_uart_rx_overflows;
            }
        }
        if (rt_tick_get() - sent >= RT_TICK_PER_SECOND / 100U) {
            sent = rt_tick_get();
            cli_print_state();
        }
        rt_thread_delay_until(&next, 1);
    }
}
