#include "temperature_compensation.h"
#include <stddef.h>
#include <math.h>

bool temperature_reference_mv(uint16_t vrefint_code, uint16_t factory_vrefint_code,
                              uint32_t *reference_mv)
{
    if ((reference_mv == NULL) || (vrefint_code == 0u) || (vrefint_code > 4095u) ||
        (factory_vrefint_code == 0u) || (factory_vrefint_code > 4095u)) return false;
    const uint32_t measured = (3000u * factory_vrefint_code + vrefint_code / 2u) / vrefint_code;
    if ((measured < 1800u) || (measured > 3600u)) return false;
    *reference_mv = measured;
    return true;
}

TemperatureNtcStatus temperature_ntc_from_adc(uint16_t adc_code, uint32_t adc_reference_mv,
                                            const TemperatureNtcConfig *config,
                                            int32_t *temperature_centi_c)
{
    if ((config == NULL) || (temperature_centi_c == NULL) ||
        (adc_reference_mv < 1800u) || (adc_reference_mv > 3600u) ||
        (config->supply_mv < 1800u) || (config->supply_mv > 3600u) ||
        (config->pull_down_ohm == 0u) || (config->resistance_at_25c_ohm == 0u) ||
        (config->beta_kelvin < 1000u) || (config->beta_kelvin > 10000u) ||
        (config->offset_centi_c < -2000) || (config->offset_centi_c > 2000))
        return TEMPERATURE_NTC_CONFIG_ERROR;
    if ((adc_code <= 15u) || (adc_code >= 4080u)) return TEMPERATURE_NTC_ADC_RAIL;
    const float voltage_mv = (float)adc_code * (float)adc_reference_mv / 4095.0f;
    if (voltage_mv >= (float)config->supply_mv) return TEMPERATURE_NTC_ADC_RAIL;
    const float resistance = (float)config->pull_down_ohm *
        ((float)config->supply_mv / voltage_mv - 1.0f);
    const float inverse_kelvin = 1.0f / 298.15f +
        logf(resistance / (float)config->resistance_at_25c_ohm) / (float)config->beta_kelvin;
    if (!isfinite(inverse_kelvin) || inverse_kelvin <= 0.0f)
        return TEMPERATURE_NTC_OUT_OF_RANGE;
    const float centi_c = (1.0f / inverse_kelvin - 273.15f) * 100.0f +
                          (float)config->offset_centi_c;
    if (!isfinite(centi_c) || centi_c < TEMPERATURE_MIN_CENTI_C ||
        centi_c > TEMPERATURE_MAX_CENTI_C) return TEMPERATURE_NTC_OUT_OF_RANGE;
    *temperature_centi_c = (int32_t)((centi_c >= 0.0f) ? centi_c + 0.5f : centi_c - 0.5f);
    return TEMPERATURE_NTC_OK;
}

bool temperature_die_centi_c(uint16_t adc_code, uint32_t adc_reference_mv,
                             uint16_t factory_cal_30c, uint16_t factory_cal_130c,
                             int32_t *temperature_centi_c)
{
    if ((temperature_centi_c == NULL) || (adc_code == 0u) || (adc_code > 4095u) ||
        (adc_reference_mv < 1800u) || (adc_reference_mv > 3600u) ||
        (factory_cal_30c == 0u) || (factory_cal_30c > 4095u) ||
        (factory_cal_130c == 0u) || (factory_cal_130c > 4095u) ||
        (factory_cal_30c == factory_cal_130c)) return false;
    const int64_t numerator = ((int64_t)adc_code * adc_reference_mv -
                              (int64_t)factory_cal_30c * 3000) * 10000;
    const int64_t denominator = (int64_t)3000 *
        ((int32_t)factory_cal_130c - (int32_t)factory_cal_30c);
    const int64_t measured = 3000 + numerator / denominator;
    if ((measured < -4000) || (measured > 15000)) return false;
    *temperature_centi_c = (int32_t)measured;
    return true;
}

bool temperature_sound_speed(int32_t temperature_centi_c, uint32_t *speed_mm_s)
{
    if ((speed_mm_s == NULL) || (temperature_centi_c < TEMPERATURE_MIN_CENTI_C) ||
        (temperature_centi_c > TEMPERATURE_MAX_CENTI_C)) return false;
    const int64_t numerator = (int64_t)606 * temperature_centi_c;
    const int64_t rounded = (numerator >= 0) ? (numerator + 50) / 100 :
                                                -((-numerator + 50) / 100);
    *speed_mm_s = (uint32_t)(331300 + rounded);
    return true;
}

void temperature_compensation_init(TemperatureCompensation *state)
{
    if (state == NULL) return;
    *state = (TemperatureCompensation){0};
    state->sound_speed_mm_s = TEMPERATURE_DEFAULT_SOUND_SPEED_MM_S;
    state->status = TEMPERATURE_NO_SAMPLE;
}

bool temperature_compensation_due(const TemperatureCompensation *state, uint32_t now_ms)
{
    return (state != NULL) && (!state->attempted ||
           ((uint32_t)(now_ms - state->last_attempt_ms) >= TEMPERATURE_SAMPLE_PERIOD_MS));
}

void temperature_compensation_attempt(TemperatureCompensation *state, uint32_t now_ms)
{
    if (state == NULL) return;
    state->last_attempt_ms = now_ms;
    state->attempted = true;
}

bool temperature_compensation_accept(TemperatureCompensation *state,
                                    int32_t temperature_centi_c, uint32_t now_ms)
{
    if (state == NULL) return false;
    uint32_t speed;
    if (!temperature_sound_speed(temperature_centi_c, &speed)) {
        state->status = TEMPERATURE_OUT_OF_RANGE;
        return false;
    }
    state->temperature_centi_c = temperature_centi_c;
    state->sound_speed_mm_s = speed;
    state->last_sample_ms = now_ms;
    state->status = TEMPERATURE_VALID;
    ++state->accepted_samples;
    return true;
}

void temperature_compensation_fail(TemperatureCompensation *state)
{
    if (state != NULL) state->status = TEMPERATURE_READ_ERROR;
}

TemperatureSnapshot temperature_compensation_snapshot(const TemperatureCompensation *state,
                                                       uint32_t now_ms)
{
    TemperatureSnapshot snapshot = {0};
    snapshot.sound_speed_mm_s = TEMPERATURE_DEFAULT_SOUND_SPEED_MM_S;
    snapshot.status = TEMPERATURE_NO_SAMPLE;
    if (state == NULL) return snapshot;
    snapshot.temperature_centi_c = state->temperature_centi_c;
    snapshot.status = state->status;
    snapshot.age_ms = (uint32_t)(now_ms - state->last_sample_ms);
    if ((snapshot.status == TEMPERATURE_VALID) &&
        (snapshot.age_ms > TEMPERATURE_MAX_AGE_MS)) snapshot.status = TEMPERATURE_STALE;
    snapshot.valid = (snapshot.status == TEMPERATURE_VALID);
    if (snapshot.valid) snapshot.sound_speed_mm_s = state->sound_speed_mm_s;
    return snapshot;
}
