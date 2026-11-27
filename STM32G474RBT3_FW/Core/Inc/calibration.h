#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

typedef struct {
    float vrefint_cal_voltage;
    uint16_t vrefint_cal_code;
    float temperature_slope;
    float temperature_intercept;
} Calibration;

float adc_code_to_voltage(uint16_t code, float vref, uint16_t full_scale);
float calculate_vdda_from_vrefint(uint16_t vrefint_code,
                                  const Calibration *calibration);
float calculate_temperature(float sensor_voltage,
                            const Calibration *calibration);

#endif
