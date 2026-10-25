#include "simplefoc_pwm.h"

#include "fsl_clock.h"
#include "fsl_pwm.h"
#include "fsl_port.h"

#define SIMPLEFOC_PWM_FREQ_HZ (20000U)

static bool s_pwm_ready;
volatile simplefoc_pwm_debug_t g_pwm_debug = {.init_status = -1};

void simplefoc_pwm_snapshot(void)
{
    const uint8_t pins[3] = {0U, 8U, 10U};
    g_pwm_debug.mctrl = FLEXPWM0->MCTRL;
    g_pwm_debug.outen = FLEXPWM0->OUTEN;
    g_pwm_debug.mask = FLEXPWM0->MASK;
    for (unsigned i = 0; i < 3; ++i) {
        g_pwm_debug.counter[i] = FLEXPWM0->SM[i].CNT;
        g_pwm_debug.period[i] = FLEXPWM0->SM[i].VAL1;
        g_pwm_debug.compare[i] = FLEXPWM0->SM[i].VAL3;
        g_pwm_debug.ctrl2[i] = FLEXPWM0->SM[i].CTRL2;
        g_pwm_debug.pin_pcr[i] = PORT3->PCR[pins[i]];
    }
}

void simplefoc_pwm_init(void)
{
    pwm_config_t config;
    pwm_signal_param_t signal = {0};
    /* PWM is configured to use BUS_CLK (45 MHz system clock / 2). */
    const uint32_t source_hz = CLOCK_GetFreq(kCLOCK_BusClk);
    const pwm_submodule_t modules[3] = {
        kPWM_Module_0, kPWM_Module_1, kPWM_Module_2
    };

    s_pwm_ready = false;
    g_pwm_debug.init_status = -1;
    g_pwm_debug.source_hz = source_hz;
    CLOCK_EnableClock(kCLOCK_GateFLEXPWM0);
    RESET_PeripheralReset(kFLEXPWM0_RST_SHIFT_RSTn);
    CLOCK_EnableClock(kCLOCK_GatePORT3);
    /* Route only the three agreed logic inputs, never complementary pins. */
    const port_pin_config_t pin = {
        .pullSelect = kPORT_PullDisable,
        .pullValueSelect = kPORT_LowPullResistor,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainDisable,
        .driveStrength = kPORT_LowDriveStrength,
        .driveStrength1 = kPORT_NormalDriveStrength,
        .mux = kPORT_MuxAlt5,
        .inputBuffer = kPORT_InputBufferEnable,
        .invertInput = kPORT_InputNormal,
        .lockRegister = kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT3, 0U, &pin);
    PORT_SetPinConfig(PORT3, 8U, &pin);
    PORT_SetPinConfig(PORT3, 10U, &pin);
    PWM_GetDefaultConfig(&config);
    config.clockSource = kPWM_BusClock;
    config.pairOperation = kPWM_Independent;
    config.reloadLogic = kPWM_ReloadPwmFullCycle;

    signal.pwmChannel = kPWM_PwmA;
    signal.level = kPWM_HighTrue;
    signal.dutyCyclePercent = 0U;
    signal.deadtimeValue = 0U;
    signal.faultState = kPWM_PwmFaultState0;
    signal.pwmchannelenable = true;

    for (unsigned i = 0U; i < 3U; ++i) {
        /* Same shared-clock arrangement as the SDK three-phase example. */
        config.clockSource = i == 0U ? kPWM_BusClock : kPWM_Submodule0Clock;
        config.initializationControl = (i == 0U)
                                     ? kPWM_Initialize_LocalSync
                                     : kPWM_Initialize_MasterSync;
        status_t status = PWM_Init(FLEXPWM0, modules[i], &config);
        if (status == kStatus_Success)
            status = PWM_SetupPwm(FLEXPWM0, modules[i], &signal, 1U,
                                 kPWM_EdgeAligned, SIMPLEFOC_PWM_FREQ_HZ, source_hz);
        if (status != kStatus_Success) {
            g_pwm_debug.init_status = status;
            PWM_StopTimer(FLEXPWM0, 7U);
            FLEXPWM0->OUTEN = 0U;
            simplefoc_pwm_snapshot();
            return;
        }
        /* No external fault inputs are connected in this 3PWM setup. */
        FLEXPWM0->SM[i].DISMAP[0] = 0U;
        FLEXPWM0->SM[i].DISMAP[1] = 0U;
    }
    PWM_SetPwmLdok(FLEXPWM0, (1UL << 0) | (1UL << 1) | (1UL << 2), true);
    PWM_StartTimer(FLEXPWM0, (1UL << 0) | (1UL << 1) | (1UL << 2));
    s_pwm_ready = true;
    g_pwm_debug.init_status = 0;
    simplefoc_pwm_snapshot();
}

void simplefoc_pwm_set_all(float a, float b, float c)
{
    float duty[3] = {a, b, c};
    if (!s_pwm_ready) return;
    PWM_SetPwmLdok(FLEXPWM0, 7U, false);
    for (unsigned i = 0; i < 3; ++i) {
        if (duty[i] < 0) duty[i] = 0;
        if (duty[i] > 1) duty[i] = 1;
        PWM_UpdatePwmDutycycleHighAccuracy(FLEXPWM0, (pwm_submodule_t)i,
            kPWM_PwmA, kPWM_EdgeAligned, (uint16_t)(duty[i] * 65535.0f));
        g_pwm_debug.duty[i] = (uint16_t)(duty[i] * 65535.0f);
    }
    PWM_SetPwmLdok(FLEXPWM0, 7U, true);
    ++g_pwm_debug.updates;
    simplefoc_pwm_snapshot();
}

void simplefoc_pwm_set(uint8_t phase, uint8_t duty_percent)
{
    static const pwm_submodule_t modules[3] = {
        kPWM_Module_0, kPWM_Module_1, kPWM_Module_2
    };

    if (!s_pwm_ready || phase >= 3U) {
        return;
    }
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }
    PWM_UpdatePwmDutycycle(FLEXPWM0, modules[phase], kPWM_PwmA,
                          kPWM_EdgeAligned, duty_percent);
    PWM_SetPwmLdok(FLEXPWM0, (1UL << phase), true);
}
