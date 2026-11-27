#ifndef MCU_ACQUISITION_CONTROL_H
#define MCU_ACQUISITION_CONTROL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "rx_processing.h"
#include "temperature_monitor.h"

#define RX_SAMPLES 16384u
#define ADC5_ACTIVE_RANKS 4u
#define ADC3_TEST_SAMPLES 256u

/* ADC5 scans at boot and every second when the acoustic hardware is idle. */
#define APP_ENABLE_ADC5_MONITOR 1u
#define APP_TX_MAX_CYCLES 128u
#define APP_TX_GAP_NS 500u /* Both-low interval at each half-cycle boundary. */
#define APP_CHIRP_START_HZ 38000u
#define APP_CHIRP_END_HZ 42000u
#define APP_CHIRP_DURATION_US 20000u
/* Set APP_CHIRP_METHOD to one of these values, then rebuild:
 *   CHIRP_METHOD_HANN_BIN    (0)  Hann + FFT peak bin
 *   CHIRP_METHOD_HANN_LOG    (1)  Hann + FFT + three-point log-power interpolation
 *   CHIRP_METHOD_HANN_CZT    (2)  Hann + FFT + local CZT
 *   CHIRP_METHOD_KAISER_BIN  (3)  Kaiser beta=4 + FFT peak bin
 *   CHIRP_METHOD_KAISER_LOG  (4)  Kaiser beta=4 + FFT + three-point log-power interpolation
 *   CHIRP_METHOD_KAISER_CZT  (5)  Kaiser beta=4 + FFT + local CZT
 */
#ifndef APP_CHIRP_METHOD
#define APP_CHIRP_METHOD CHIRP_METHOD_KAISER_LOG
#endif
#if APP_CHIRP_METHOD < 0 || APP_CHIRP_METHOD >= CHIRP_DSP_METHOD_COUNT
#error "APP_CHIRP_METHOD must select one of the six CHIRP_METHOD_* options"
#endif
#define APP_CHIRP_STEPS 800u
#define APP_CHIRP_MAX_RANGE_MM 1500u
#define APP_CHIRP_SIM_TX_MARKER_SAMPLE 8u
#define APP_CHIRP_SIM_ECHO_PEAK_CODES 40

enum {
    APP_CMD_NONE, APP_CMD_RX, APP_CMD_TX, APP_CMD_TX_RX, APP_CMD_STOP,
    APP_CMD_CHIRP_RX, APP_CMD_DSP_SIMULATION
};
enum { APP_STATE_IDLE, APP_STATE_BUSY, APP_STATE_DONE, APP_STATE_ERROR };
enum {
    APP_ERROR_NONE, APP_ERROR_COMMAND, APP_ERROR_TX_CONFIG,
    APP_ERROR_RX_START, APP_ERROR_TX_START, APP_ERROR_RX_DMA,
    APP_ERROR_TX_DMA, APP_ERROR_TIMEOUT, APP_ERROR_DSP_SIMULATION
};

/* Public globals deliberately retained for Keil Watch (no function calls). */
extern volatile uint32_t app_command, app_state, app_error, app_last_hal_status;
extern volatile uint32_t app_tx_frequency_hz, app_tx_cycles;
extern volatile uint32_t app_chirp_tx_start_sample;
extern volatile uint32_t app_chirp_sample_count;
extern volatile uint32_t app_chirp_sim_target_range_mm;
extern volatile uint32_t app_chirp_result_valid, app_chirp_processing_error;
extern volatile uint32_t app_chirp_range_candidate_mm, app_chirp_beat_millihz;
extern volatile uint32_t app_chirp_peak_ratio_x100, app_chirp_dsp_cycles;
extern volatile uint32_t app_chirp_dsp_time_us;
extern volatile int32_t app_chirp_temperature_centi_c;
extern volatile uint32_t app_chirp_sound_speed_mm_s, app_chirp_temperature_valid;
extern volatile uint32_t app_chirp_temperature_status, app_chirp_temperature_age_ms;
extern volatile uint32_t app_chirp_temperature_source;
/* Bring-up default: permit clearly flagged 343 m/s estimates without an NTC.
   Set 0 to require valid external temperature for a range candidate. */
extern volatile uint32_t app_allow_nominal_sound_speed;
/* Indexed by CHIRP_METHOD_*. Live frames populate the selected and coarse results. */
extern volatile uint32_t app_chirp_method_beat_millihz[CHIRP_DSP_METHOD_COUNT];
extern volatile uint32_t app_chirp_method_range_mm[CHIRP_DSP_METHOD_COUNT];
extern volatile uint32_t app_tx_active, app_tx_done, app_tx_dma_done;
extern volatile uint32_t app_tx_dma_elapsed_us;
extern volatile uint32_t app_frame_count, app_rx_valid;
extern volatile uint32_t app_rx_min, app_rx_max, app_rx_mean, app_rx_peak_to_peak;
extern volatile uint32_t app_rx_clipped_count, app_rx_sample_rate_hz;
/* UART4: r=RX, b=one fixed burst+RX, c=chirp with 1024 IQ pairs,
   C=chirp with raw ADC waveform, s=STOP. All TX requests are finite. */
extern volatile uint32_t app_uart_enabled, app_uart_busy, app_uart_last_status;
extern volatile uint32_t app_uart_chirp_raw, app_uart_frame_bytes;
extern volatile uint32_t app_uart_frames_sent, app_uart_send_errors;
extern volatile uint32_t app_uart_rx_requests, app_uart_rx_errors;
/* Diagnostic: 0 normal; 1 raw 0x55 UART; 2 PC10 GPIO level changes every 1 s.
   Disconnect USB-UART signal wires for mode 2; returning to 0/1 restores AF5. */
extern volatile uint32_t app_uart_test_mode, app_uart_test_bytes, app_uart_test_status;
extern volatile uint32_t app_uart_tx_gpio_mode, app_uart_tx_gpio_af;
extern volatile uint32_t app_uart_pin_level, app_uart_pin_readback, app_uart_pin_edges;
extern uint16_t rx_buffer[RX_SAMPLES];

void App_BasicInit(void);
void App_BasicProcess(void);
void App_EmergencyOutputsOff(void);

void App_AcquisitionInit(void);
HAL_StatusTypeDef App_StartRxFrame(void);
void App_StopRxFrame(void);
bool App_RxFrameReady(void);
bool App_RxFrameActive(void);
bool App_RxDmaError(void);
bool App_RxAdcClipped(void);
bool App_VbiasOutOfRange(void);
const uint16_t *App_RxBuffer(void);
HAL_StatusTypeDef App_StartAdc5Monitor(void);
void App_StopAdc5Monitor(void);
bool App_Adc5SequenceReady(void);
bool App_Adc5Error(void);
void App_Adc5Values(uint16_t *temperature, uint16_t *vrefint,
                    uint16_t *ntc1, uint16_t *ntc2);
HAL_StatusTypeDef App_StartAdc3Test(void);
void App_StopAdc3Test(void);
bool App_Adc3TestActive(void);
bool App_Adc3TestDone(void);
bool App_Adc3TestError(void);
const uint16_t *App_Adc3TestBuffer(void);

#endif
