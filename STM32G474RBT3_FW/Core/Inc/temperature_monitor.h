#ifndef TEMPERATURE_MONITOR_H
#define TEMPERATURE_MONITOR_H

#include "temperature_compensation.h"

#define APP_TEMPERATURE_ADC_TIMEOUT_MS 10u

enum {
    APP_TEMPERATURE_ERROR_NONE = 0,
    APP_TEMPERATURE_ERROR_INIT,
    APP_TEMPERATURE_ERROR_START,
    APP_TEMPERATURE_ERROR_TIMEOUT,
    APP_TEMPERATURE_ERROR_ADC,
    APP_TEMPERATURE_ERROR_VREFINT,
    APP_TEMPERATURE_ERROR_NTC,
    APP_TEMPERATURE_ERROR_CHANNEL,
    APP_TEMPERATURE_ERROR_DISABLED
};

/* Keil Watch diagnostics. Channel is 1=NTC1/PB13, 2=NTC2/PB14.
   Parameters are captured at the start of the next scan. */
extern volatile uint32_t app_temperature_ntc_channel;
extern volatile uint32_t app_temperature_ntc_supply_mv;
extern volatile uint32_t app_temperature_ntc_r25_ohm;
extern volatile uint32_t app_temperature_ntc_pull_down_ohm;
extern volatile uint32_t app_temperature_ntc_beta_kelvin;
extern volatile int32_t app_temperature_ntc_offset_centi_c;
extern volatile uint32_t app_temperature_status, app_temperature_error;
extern volatile uint32_t app_temperature_valid, app_temperature_busy;
extern volatile uint32_t app_temperature_attempts, app_temperature_samples;
extern volatile uint32_t app_temperature_errors, app_temperature_last_sample_tick;
extern volatile uint32_t app_temperature_age_ms, app_temperature_last_hal_status;
extern volatile uint32_t app_temperature_source, app_sound_speed_mm_s;
extern volatile int32_t app_temperature_centi_c, app_ntc1_centi_c, app_ntc2_centi_c;
extern volatile uint32_t app_ntc1_status, app_ntc2_status;
extern volatile uint32_t app_adc5_die_code, app_adc5_vrefint_code;
extern volatile uint32_t app_adc5_ntc1_code, app_adc5_ntc2_code;
extern volatile uint32_t app_adc_reference_mv, app_die_temperature_valid;
extern volatile int32_t app_die_temperature_centi_c;

void App_TemperatureInit(void);
/* Returns true while the pending four-rank scan must finish before RX/TX.
   Starts a due scan only when may_start is true. */
bool App_TemperatureProcess(bool may_start);
TemperatureSnapshot App_TemperatureSnapshot(uint32_t now_ms);

#endif
