#ifndef SIMPLEFOC_PWM_H
#define SIMPLEFOC_PWM_H

#include <stdint.h>

typedef struct {
    int32_t init_status; /* 0 success, -1 not initialized, otherwise SDK status */
    uint32_t source_hz;
    uint32_t updates;
    uint16_t mctrl, outen, mask;
    uint16_t duty[3], counter[3], period[3], compare[3], ctrl2[3];
    uint32_t pin_pcr[3];
} simplefoc_pwm_debug_t;

extern volatile simplefoc_pwm_debug_t g_pwm_debug;
void simplefoc_pwm_snapshot(void);

#ifdef __cplusplus
extern "C" {
#endif

void simplefoc_pwm_init(void);
void simplefoc_pwm_set(uint8_t phase, uint8_t duty_percent);
void simplefoc_pwm_set_all(float a, float b, float c);

#ifdef __cplusplus
}
#endif

#endif
