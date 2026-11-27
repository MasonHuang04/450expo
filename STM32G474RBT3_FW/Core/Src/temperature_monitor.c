#include "temperature_monitor.h"
#include "mcu_acquisition_control.h"
#include "adc.h"
#include "opamp.h"
#include "stm32g4xx_ll_adc.h"

volatile uint32_t app_temperature_ntc_channel = 2u;
volatile uint32_t app_temperature_ntc_supply_mv = 3300u;
volatile uint32_t app_temperature_ntc_r25_ohm = 10000u;
volatile uint32_t app_temperature_ntc_pull_down_ohm = 10000u;
volatile uint32_t app_temperature_ntc_beta_kelvin = 3380u;
volatile int32_t app_temperature_ntc_offset_centi_c;
volatile uint32_t app_temperature_status, app_temperature_error;
volatile uint32_t app_temperature_valid, app_temperature_busy;
volatile uint32_t app_temperature_attempts, app_temperature_samples;
volatile uint32_t app_temperature_errors, app_temperature_last_sample_tick;
volatile uint32_t app_temperature_age_ms, app_temperature_last_hal_status;
volatile uint32_t app_temperature_source;
volatile uint32_t app_sound_speed_mm_s = TEMPERATURE_DEFAULT_SOUND_SPEED_MM_S;
volatile int32_t app_temperature_centi_c, app_ntc1_centi_c, app_ntc2_centi_c;
volatile uint32_t app_ntc1_status, app_ntc2_status;
volatile uint32_t app_adc5_die_code, app_adc5_vrefint_code;
volatile uint32_t app_adc5_ntc1_code, app_adc5_ntc2_code;
volatile uint32_t app_adc_reference_mv, app_die_temperature_valid;
volatile int32_t app_die_temperature_centi_c;

static TemperatureCompensation temperature_state;
static TemperatureNtcConfig sampled_config;
static uint32_t sampled_channel, sample_started_ms;
static bool hardware_ready;

TemperatureSnapshot App_TemperatureSnapshot(uint32_t now_ms)
{
    return temperature_compensation_snapshot(&temperature_state, now_ms);
}

static void TemperaturePublish(uint32_t now_ms)
{
    const TemperatureSnapshot snapshot = App_TemperatureSnapshot(now_ms);
    app_temperature_valid = snapshot.valid ? 1u : 0u;
    app_temperature_status = (uint32_t)snapshot.status;
    app_temperature_centi_c = snapshot.temperature_centi_c;
    app_temperature_age_ms = snapshot.age_ms;
    app_sound_speed_mm_s = snapshot.sound_speed_mm_s;
    app_temperature_samples = temperature_state.accepted_samples;
    app_temperature_last_sample_tick = temperature_state.last_sample_ms;
}

static void TemperatureFail(uint32_t error)
{
    temperature_compensation_fail(&temperature_state);
    app_temperature_error = error;
    ++app_temperature_errors;
}

static HAL_StatusTypeDef TemperatureStartOpamp(OPAMP_HandleTypeDef *handle)
{
    /* HAL_OPAMP_Start rejects an already-running opamp. Preserve a successful
       first stage when retrying after OPAMP5 or ADC calibration failed. */
    if (HAL_OPAMP_GetState(handle) == HAL_OPAMP_STATE_BUSY) return HAL_OK;
    return HAL_OPAMP_Start(handle);
}

void App_TemperatureInit(void)
{
    temperature_compensation_init(&temperature_state);
    hardware_ready = false;
    app_temperature_busy = 0u;
    app_temperature_attempts = app_temperature_samples = app_temperature_errors = 0u;
    app_temperature_source = 0u;
    app_temperature_error = APP_TEMPERATURE_ERROR_NONE;
    app_temperature_last_hal_status = HAL_OK;
    app_ntc1_status = app_ntc2_status = TEMPERATURE_NTC_CONFIG_ERROR;
    app_adc5_die_code = app_adc5_vrefint_code = 0u;
    app_adc5_ntc1_code = app_adc5_ntc2_code = 0u;
    app_adc_reference_mv = app_die_temperature_valid = 0u;
    TemperaturePublish(HAL_GetTick());
}

static void TemperatureConsume(uint32_t now_ms)
{
    uint16_t die_code, vref_code, ntc1_code, ntc2_code;
    uint32_t reference_mv;
    int32_t die_centi = 0, ntc1_centi = 0, ntc2_centi = 0;
    App_Adc5Values(&die_code, &vref_code, &ntc1_code, &ntc2_code);
    app_adc5_die_code = die_code;
    app_adc5_vrefint_code = vref_code;
    app_adc5_ntc1_code = ntc1_code;
    app_adc5_ntc2_code = ntc2_code;
    if (!temperature_reference_mv(vref_code, *VREFINT_CAL_ADDR, &reference_mv)) {
        app_adc_reference_mv = app_die_temperature_valid = 0u;
        app_ntc1_status = app_ntc2_status = TEMPERATURE_NTC_CONFIG_ERROR;
        TemperatureFail(APP_TEMPERATURE_ERROR_VREFINT);
        return;
    }
    app_adc_reference_mv = reference_mv;
    app_die_temperature_valid = temperature_die_centi_c(die_code, reference_mv,
        *TEMPSENSOR_CAL1_ADDR, *TEMPSENSOR_CAL2_ADDR, &die_centi) ? 1u : 0u;
    app_die_temperature_centi_c = die_centi;
    app_ntc1_status = (uint32_t)temperature_ntc_from_adc(ntc1_code, reference_mv,
        &sampled_config, &ntc1_centi);
    app_ntc2_status = (uint32_t)temperature_ntc_from_adc(ntc2_code, reference_mv,
        &sampled_config, &ntc2_centi);
    app_ntc1_centi_c = ntc1_centi;
    app_ntc2_centi_c = ntc2_centi;
    const bool ntc_ok = ((sampled_channel == 1u) && (app_ntc1_status == TEMPERATURE_NTC_OK)) ||
                       ((sampled_channel == 2u) && (app_ntc2_status == TEMPERATURE_NTC_OK));
    if (!ntc_ok) {
        TemperatureFail(APP_TEMPERATURE_ERROR_NTC);
        return;
    }
    const int32_t selected = (sampled_channel == 1u) ? ntc1_centi : ntc2_centi;
    if (!temperature_compensation_accept(&temperature_state, selected, now_ms)) {
        TemperatureFail(APP_TEMPERATURE_ERROR_NTC);
        return;
    }
    app_temperature_source = sampled_channel;
    app_temperature_error = APP_TEMPERATURE_ERROR_NONE;
}

bool App_TemperatureProcess(bool may_start)
{
    const uint32_t now_ms = HAL_GetTick();
    if (app_temperature_busy != 0u) {
        if (App_Adc5Error()) {
            App_StopAdc5Monitor();
            hardware_ready = false;
            app_temperature_busy = 0u;
            TemperatureFail(APP_TEMPERATURE_ERROR_ADC);
        } else if (App_Adc5SequenceReady()) {
            App_StopAdc5Monitor();
            app_temperature_busy = 0u;
            TemperatureConsume(now_ms);
        } else if ((uint32_t)(now_ms - sample_started_ms) >= APP_TEMPERATURE_ADC_TIMEOUT_MS) {
            App_StopAdc5Monitor();
            hardware_ready = false;
            app_temperature_busy = 0u;
            TemperatureFail(APP_TEMPERATURE_ERROR_TIMEOUT);
        }
        TemperaturePublish(now_ms);
        return app_temperature_busy != 0u;
    }
    TemperaturePublish(now_ms);
    if (!may_start || !temperature_compensation_due(&temperature_state, now_ms)) return false;
    temperature_compensation_attempt(&temperature_state, now_ms);
    ++app_temperature_attempts;
    sampled_channel = app_temperature_ntc_channel;
    sampled_config = (TemperatureNtcConfig){app_temperature_ntc_pull_down_ohm,
        app_temperature_ntc_r25_ohm, app_temperature_ntc_beta_kelvin,
        app_temperature_ntc_supply_mv, app_temperature_ntc_offset_centi_c};
    if ((sampled_channel != 1u) && (sampled_channel != 2u)) {
        TemperatureFail(APP_TEMPERATURE_ERROR_CHANNEL);
        TemperaturePublish(now_ms);
        return false;
    }
#if APP_ENABLE_ADC5_MONITOR
    HAL_StatusTypeDef status = HAL_OK;
    if (!hardware_ready) {
        status = TemperatureStartOpamp(&hopamp4);
        if (status == HAL_OK) status = TemperatureStartOpamp(&hopamp5);
        if (status == HAL_OK) status = HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
        app_temperature_last_hal_status = (uint32_t)status;
        if (status != HAL_OK) {
            TemperatureFail(APP_TEMPERATURE_ERROR_INIT);
            TemperaturePublish(now_ms);
            return false;
        }
        hardware_ready = true;
    }
    status = App_StartAdc5Monitor();
    app_temperature_last_hal_status = (uint32_t)status;
    if (status != HAL_OK) {
        App_StopAdc5Monitor();
        hardware_ready = false;
        TemperatureFail(APP_TEMPERATURE_ERROR_START);
        TemperaturePublish(now_ms);
        return false;
    }
    sample_started_ms = now_ms;
    app_temperature_busy = 1u;
    return true;
#else
    TemperatureFail(APP_TEMPERATURE_ERROR_DISABLED);
    TemperaturePublish(now_ms);
    return false;
#endif
}
