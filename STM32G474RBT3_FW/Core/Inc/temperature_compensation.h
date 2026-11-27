#ifndef TEMPERATURE_COMPENSATION_H
#define TEMPERATURE_COMPENSATION_H

#include <stdbool.h>
#include <stdint.h>

#define TEMPERATURE_SAMPLE_PERIOD_MS 1000u
#define TEMPERATURE_MAX_AGE_MS 125000u
#define TEMPERATURE_DEFAULT_SOUND_SPEED_MM_S 343000u
#define TEMPERATURE_MIN_CENTI_C (-4000)
#define TEMPERATURE_MAX_CENTI_C 8500

typedef enum {
    TEMPERATURE_NO_SAMPLE = 0,
    TEMPERATURE_VALID,
    TEMPERATURE_READ_ERROR,
    TEMPERATURE_OUT_OF_RANGE,
    TEMPERATURE_STALE
} TemperatureStatus;

typedef struct {
    uint32_t last_attempt_ms;
    uint32_t last_sample_ms;
    uint32_t accepted_samples;
    uint32_t sound_speed_mm_s;
    int32_t temperature_centi_c;
    TemperatureStatus status;
    bool attempted;
} TemperatureCompensation;

typedef struct {
    int32_t temperature_centi_c;
    uint32_t sound_speed_mm_s;
    uint32_t age_ms;
    TemperatureStatus status;
    bool valid;
} TemperatureSnapshot;

typedef struct {
    uint32_t pull_down_ohm;
    uint32_t resistance_at_25c_ohm;
    uint32_t beta_kelvin;
    uint32_t supply_mv;
    int32_t offset_centi_c;
} TemperatureNtcConfig;

typedef enum {
    TEMPERATURE_NTC_OK = 0,
    TEMPERATURE_NTC_ADC_RAIL,
    TEMPERATURE_NTC_CONFIG_ERROR,
    TEMPERATURE_NTC_OUT_OF_RANGE
} TemperatureNtcStatus;

/* NewPCB: supply -> NTC -> ADC node -> fixed pull-down -> AGND.
   ADC reference (3V3A) and NTC supply (3V3) are separate rails. */
TemperatureNtcStatus temperature_ntc_from_adc(uint16_t adc_code, uint32_t adc_reference_mv,
                                            const TemperatureNtcConfig *config,
                                            int32_t *temperature_centi_c);
bool temperature_reference_mv(uint16_t vrefint_code, uint16_t factory_vrefint_code,
                              uint32_t *reference_mv);
/* Diagnostic silicon temperature; never selected automatically as air temperature. */
bool temperature_die_centi_c(uint16_t adc_code, uint32_t adc_reference_mv,
                             uint16_t factory_cal_30c, uint16_t factory_cal_130c,
                             int32_t *temperature_centi_c);

/* c = 331.3 + 0.606*T, rounded to 1 mm/s, matching the FPGA formula. */
bool temperature_sound_speed(int32_t temperature_centi_c, uint32_t *speed_mm_s);
void temperature_compensation_init(TemperatureCompensation *state);
bool temperature_compensation_due(const TemperatureCompensation *state, uint32_t now_ms);
void temperature_compensation_attempt(TemperatureCompensation *state, uint32_t now_ms);
bool temperature_compensation_accept(TemperatureCompensation *state,
                                    int32_t temperature_centi_c, uint32_t now_ms);
void temperature_compensation_fail(TemperatureCompensation *state);
TemperatureSnapshot temperature_compensation_snapshot(const TemperatureCompensation *state,
                                                       uint32_t now_ms);

#endif
