#pragma once

#include <stdbool.h>

/*
 * 1st-order IIR (exponential moving average):
 *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * Alpha values are tuned from test-flight telemetry noise at ~42 Hz loop rate.
 * Approximate -3 dB cutoff: fc ≈ -fs * ln(1 - alpha) / (2π)
 *
 *   FILT_ALPHA_ALT   = 0.07  → fc ≈ 0.49 Hz  (altitude/pressure quantization)
 *   FILT_ALPHA_ACCEL = 0.30  → fc ≈ 2.4  Hz  (accel σ ≈ 0.07 g static)
 *   FILT_ALPHA_MAG   = 0.15  → fc ≈ 1.1  Hz  (mag σ ≈ 0.01–0.02 G static)
 *   FILT_ALPHA_GYRO  = 0.50  → fc ≈ 4.6  Hz  (gyro σ ≈ 5–10 dps static)
 *   FILT_ALPHA_TEMP  = 0.01  → fc ≈ 0.067 Hz (temp σ ≈ 0.67 °C static)
 */
#define FILT_ALPHA_ALT      0.07f
#define FILT_ALPHA_ACCEL    0.30f
#define FILT_ALPHA_MAG      0.15f
#define FILT_ALPHA_GYRO     0.50f
#define FILT_ALPHA_TEMP     0.01f

typedef struct {
    float alpha;
    float state;
    bool  ready;
} Filter1P_t;

void  Filter1P_init(Filter1P_t *f, float alpha);
float Filter1P_update(Filter1P_t *f, float sample);
