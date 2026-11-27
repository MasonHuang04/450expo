#include "calibration.h"

float adc_code_to_voltage(uint16_t code, float vref, uint16_t full_scale)
{
    if (full_scale == 0) return 0.0f;
    return ((float)code * vref) / (float)full_scale;
}

float calculate_vdda_from_vrefint(uint16_t vrefint_code,
                                  const Calibration *calibration)
{
    if (calibration == 0 || vrefint_code == 0) return 0.0f;
    return calibration->vrefint_cal_voltage *
           ((float)calibration->vrefint_cal_code / (float)vrefint_code);
}

float calculate_temperature(float sensor_voltage,
                            const Calibration *calibration)
{
    if (calibration == 0) return 0.0f;
    return calibration->temperature_slope * sensor_voltage +
           calibration->temperature_intercept;
}
