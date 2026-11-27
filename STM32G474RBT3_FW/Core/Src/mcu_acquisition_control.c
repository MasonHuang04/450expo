/*
 * RX acquisition and finite fixed-tone TX bring-up. Application entry points
 * are App_AcquisitionInit, App_BasicInit and App_BasicProcess.
 * Legacy ADC3 test helpers are retained but never started by the basic flow.
 */

#include "mcu_acquisition_control.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "protocol.h"
#include "rx_processing.h"
#include "chirp_tx_schedule.h"

extern DMA_HandleTypeDef hdma_adc1;

uint16_t rx_buffer[RX_SAMPLES];
static volatile bool rx_frame_done;
static volatile bool rx_dma_error;
static volatile bool rx_adc_clipped;
static volatile bool vbias_out_of_range;
static volatile bool acquisition_active;
static volatile bool adc5_sequence_ready;
static volatile bool adc5_error;
static volatile bool adc5_active;
static volatile uint8_t adc5_rank_index;
static volatile uint16_t adc5_values[4];
static uint16_t adc3_buffer[ADC3_TEST_SAMPLES];
static volatile bool adc3_test_active;
static volatile bool adc3_test_done;
static volatile bool adc3_test_error;

void App_AcquisitionInit(void)
{
    rx_frame_done = false;
    rx_dma_error = false;
    rx_adc_clipped = false;
    vbias_out_of_range = false;
    acquisition_active = false;
    adc5_sequence_ready = false;
    adc5_error = false;
    adc5_active = false;
    adc5_rank_index = 0u;
    adc5_values[0] = 0u;
    adc5_values[1] = 0u;
    adc5_values[2] = 0u;
    adc5_values[3] = 0u;
    adc3_test_active = false;
    adc3_test_done = false;
    adc3_test_error = false;
    app_chirp_tx_start_sample = 0u;
    app_chirp_sample_count = RX_SAMPLES;
    for (uint16_t i = 0u; i < ADC3_TEST_SAMPLES; i++) {
        adc3_buffer[i] = 0u;
    }

    /* The ADC channels use the internal OPAMP outputs. */
    if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) Error_Handler();
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) Error_Handler();

    /* Required once after power-up for each ADC that will be used. */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
    /* OPAMP4/5 and ADC5 are prepared by the temperature service. A failed
       temperature read must not prevent RX waveform diagnostics. */
    HAL_Delay(1); /* Allow Vbias buffer and RX PGA to settle before sampling. */
}

HAL_StatusTypeDef App_StartRxFrame(void)
{
    if (acquisition_active) return HAL_BUSY;

    rx_frame_done = false;
    rx_dma_error = false;
    rx_adc_clipped = false;
    vbias_out_of_range = false;
    acquisition_active = true;

    __HAL_TIM_SET_COUNTER(&htim15, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);

    /* Prepare the consumer before enabling the trigger source. */
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(
        &hadc1, (uint32_t *)rx_buffer, RX_SAMPLES);
    if (status != HAL_OK) {
        acquisition_active = false;
        return status;
    }

    /* TIM15 TRGO is the 500 kHz ADC1/ADC2 sample clock. */
    status = HAL_ADC_Start(&hadc2);
    if (status != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc1);
        acquisition_active = false;
        return status;
    }

    status = HAL_TIM_Base_Start(&htim15);
    if (status != HAL_OK) {
        HAL_ADC_Stop(&hadc2);
        HAL_ADC_Stop_DMA(&hadc1);
        acquisition_active = false;
    }
    return status;
}

void App_StopRxFrame(void)
{
    /* Stop the trigger before stopping ADC/DMA, preventing a late request. */
    HAL_TIM_Base_Stop(&htim15);
    HAL_ADC_Stop(&hadc2);
    HAL_ADC_Stop_DMA(&hadc1);
    acquisition_active = false;
}

bool App_RxFrameReady(void) { return rx_frame_done; }
bool App_RxFrameActive(void) { return acquisition_active; }
bool App_RxDmaError(void) { return rx_dma_error; }
bool App_RxAdcClipped(void) { return rx_adc_clipped; }
bool App_VbiasOutOfRange(void) { return vbias_out_of_range; }
const uint16_t *App_RxBuffer(void) { return rx_buffer; }

HAL_StatusTypeDef App_StartAdc5Monitor(void)
{
    if (adc5_active) return HAL_BUSY;

    adc5_sequence_ready = false;
    adc5_error = false;
    adc5_rank_index = 0u;

    /* TIM7 remains stopped. A software update generates exactly one TRGO;
       the foreground scheduler decides when the next scan is due. */
    HAL_TIM_Base_Stop(&htim7);
    __HAL_TIM_SET_COUNTER(&htim7, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    HAL_StatusTypeDef status = HAL_ADC_Start_IT(&hadc5);
    if (status != HAL_OK) return status;
    adc5_active = true;
    status = HAL_TIM_GenerateEvent(&htim7, TIM_EVENTSOURCE_UPDATE);
    if (status != HAL_OK) {
        HAL_ADC_Stop_IT(&hadc5);
        adc5_active = false;
        return status;
    }
    return HAL_OK;
}

void App_StopAdc5Monitor(void)
{
    HAL_TIM_Base_Stop(&htim7);
    HAL_ADC_Stop_IT(&hadc5);
    adc5_active = false;
}

bool App_Adc5SequenceReady(void) { return adc5_sequence_ready; }
bool App_Adc5Error(void) { return adc5_error; }

HAL_StatusTypeDef App_StartAdc3Test(void)
{
    if (adc3_test_active) return HAL_BUSY;

    adc3_test_done = false;
    adc3_test_error = false;
    adc3_test_active = true;
    for (uint16_t i = 0u; i < ADC3_TEST_SAMPLES; i++) {
        adc3_buffer[i] = 0u;
    }

    /* Arm ADC3 DMA before Timer A begins generating Compare 4 triggers. */
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(
        &hadc3, (uint32_t *)adc3_buffer, ADC3_TEST_SAMPLES);
    if (status != HAL_OK) {
        adc3_test_active = false;
        adc3_test_error = true;
        return status;
    }

    /* ADC Trigger 1 is preloaded with MASTER as its update source in hrtim.c.
       Start MASTER and Timer A counters so the trigger selection is latched and
       Timer A Compare 4 can generate ADC3 triggers. No waveform output is enabled. */
    status = HAL_HRTIM_WaveformCountStart(
        &hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A);
    if (status != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc3);
        adc3_test_active = false;
        adc3_test_error = true;
    }
    return status;
}

void App_StopAdc3Test(void)
{
    /* Stop the trigger source before stopping the ADC/DMA consumer. */
    HAL_HRTIM_WaveformCountStop(
        &hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A);
    HAL_ADC_Stop_DMA(&hadc3);
    adc3_test_active = false;
}

bool App_Adc3TestActive(void) { return adc3_test_active; }
bool App_Adc3TestDone(void) { return adc3_test_done; }
bool App_Adc3TestError(void) { return adc3_test_error; }
const uint16_t *App_Adc3TestBuffer(void) { return adc3_buffer; }

void App_Adc5Values(uint16_t *temperature, uint16_t *vrefint,
                    uint16_t *ntc1, uint16_t *ntc2)
{
    if (temperature != NULL) *temperature = adc5_values[0];
    if (vrefint != NULL) *vrefint = adc5_values[1];
    if (ntc1 != NULL) *ntc1 = adc5_values[2];
    if (ntc2 != NULL) *ntc2 = adc5_values[3];

    adc5_sequence_ready = false;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        /* Do not run filtering/FFT in interrupt context. */
        App_StopRxFrame();
        __DMB();
        rx_frame_done = true;
    }
    else if (hadc->Instance == ADC5) {
        /* ADC5 uses EOC_SINGLE_CONV, so callbacks arrive in rank order. */
        if (adc5_active && (adc5_rank_index < ADC5_ACTIVE_RANKS)) {
            adc5_values[adc5_rank_index] = (uint16_t)HAL_ADC_GetValue(hadc);
            adc5_rank_index++;
            if (adc5_rank_index == ADC5_ACTIVE_RANKS) {
                __DMB();
                adc5_sequence_ready = true;
            }
        }
    }
    else if (hadc->Instance == ADC3) {
        /* DMA has collected the requested block; stop further HRTIM triggers. */
        HAL_HRTIM_WaveformCountStop(
            &hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A);
        adc3_test_done = true;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        App_EmergencyOutputsOff();
        rx_dma_error = true;
        App_StopRxFrame();
    }
    else if (hadc->Instance == ADC5) {
        adc5_error = true;
        App_StopAdc5Monitor();
    }
    else if (hadc->Instance == ADC3) {
        HAL_HRTIM_WaveformCountStop(
            &hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A);
        adc3_test_error = true;
        adc3_test_active = false;
    }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        rx_adc_clipped = true;
    }
    if (hadc->Instance == ADC2) {
        vbias_out_of_range = true;
    }
}

/* -------------------------------------------------------------------------
 * Basic RX/TX control. Write one command in Watch, then run without stepping
 * through interrupts. Results are held until the next accepted command.
 * ------------------------------------------------------------------------- */
volatile uint32_t app_command;
volatile uint32_t app_state;
volatile uint32_t app_error;
volatile uint32_t app_last_hal_status;
volatile uint32_t app_tx_frequency_hz = 40000u;
volatile uint32_t app_tx_cycles = 8u;
volatile uint32_t app_chirp_tx_start_sample;
volatile uint32_t app_chirp_sample_count;
volatile uint32_t app_chirp_sim_target_range_mm = 500u;
volatile uint32_t app_chirp_result_valid, app_chirp_processing_error;
volatile uint32_t app_chirp_range_candidate_mm, app_chirp_beat_millihz;
volatile uint32_t app_chirp_peak_ratio_x100, app_chirp_dsp_cycles;
volatile uint32_t app_chirp_dsp_time_us;
volatile int32_t app_chirp_temperature_centi_c;
volatile uint32_t app_chirp_sound_speed_mm_s = TEMPERATURE_DEFAULT_SOUND_SPEED_MM_S;
volatile uint32_t app_chirp_temperature_valid, app_chirp_temperature_status;
volatile uint32_t app_chirp_temperature_age_ms, app_chirp_temperature_source;
volatile uint32_t app_allow_nominal_sound_speed = 1u;
volatile uint32_t app_chirp_method_beat_millihz[CHIRP_DSP_METHOD_COUNT];
volatile uint32_t app_chirp_method_range_mm[CHIRP_DSP_METHOD_COUNT];
volatile uint32_t app_tx_active;
volatile uint32_t app_tx_done;
volatile uint32_t app_tx_dma_done;
volatile uint32_t app_tx_dma_elapsed_us;
volatile uint32_t app_frame_count;
volatile uint32_t app_rx_valid;
volatile uint32_t app_rx_min, app_rx_max, app_rx_mean, app_rx_peak_to_peak;
volatile uint32_t app_rx_clipped_count;
volatile uint32_t app_rx_sample_rate_hz;
volatile uint32_t app_uart_enabled = 1u;
volatile uint32_t app_uart_chirp_raw; /* 0=1024 I/Q pairs, 1=16384 raw ADC codes. */
volatile uint32_t app_uart_frame_bytes;
volatile uint32_t app_uart_busy;
volatile uint32_t app_uart_last_status;
volatile uint32_t app_uart_frames_sent;
volatile uint32_t app_uart_send_errors;
volatile uint32_t app_uart_rx_requests;
volatile uint32_t app_uart_rx_errors;
volatile uint32_t app_uart_test_mode;
volatile uint32_t app_uart_test_bytes;
volatile uint32_t app_uart_test_status;
volatile uint32_t app_uart_tx_gpio_mode;
volatile uint32_t app_uart_tx_gpio_af;
volatile uint32_t app_uart_pin_level;
volatile uint32_t app_uart_pin_readback;
volatile uint32_t app_uart_pin_edges;
static bool uart_pin_test_active;
static uint32_t uart_pin_test_tick;

/* Each HRTIM DMA burst updates PER, CMP1, CMP2, CMP3, CMP4, in that order.
   Chirp pipeline emits row 0 twice, then rows 1..798, then two quiet rows.
   Account for the duplicate startup period when assigning cycle numbers. */
static uint32_t tx_table[APP_CHIRP_STEPS + 2u][5];
static uint32_t tx_table_words;
static uint32_t tx_table_start;
static volatile uint32_t tx_dma_error;
static volatile uint32_t tx_dma_finished_tick;
static uint32_t basic_start_tick;
static uint32_t basic_mode;
static TemperatureSnapshot frame_temperature;
static bool frame_nominal_speed;
static bool frame_raw_payload;
static uint32_t tx_started_cycles, tx_deadline_cycles;

static void BasicPollSerialCommand(void)
{
    uint8_t command;
    /* A disconnected/noisy UART or commands typed during a blocking transfer
       must not leave a sticky error that prevents the next valid request. */
    if ((huart4.Instance->ISR & (USART_ISR_ORE | USART_ISR_FE |
                                 USART_ISR_NE | USART_ISR_PE)) != 0u) {
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                       UART_CLEAR_NEF | UART_CLEAR_PEF);
        __HAL_UART_SEND_REQ(&huart4, UART_RXDATA_FLUSH_REQUEST);
        ++app_uart_rx_errors;
        return;
    }
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) == RESET) return;
    if (HAL_UART_Receive(&huart4, &command, 1u, 0u) != HAL_OK) return;
    if (command == (uint8_t)'s') {
        app_command = APP_CMD_STOP;
        return;
    }
    uint32_t requested = (command == (uint8_t)'r') ? APP_CMD_RX :
                         (command == (uint8_t)'b') ? APP_CMD_TX_RX :
                         ((command == (uint8_t)'c') || (command == (uint8_t)'C')) ?
                            APP_CMD_CHIRP_RX : APP_CMD_NONE;
    if ((requested != APP_CMD_NONE) && (app_state != APP_STATE_BUSY) &&
        (app_command == APP_CMD_NONE)) {
        app_command = requested;
        if (requested == APP_CMD_CHIRP_RX) app_uart_chirp_raw = (command == (uint8_t)'C');
        ++app_uart_rx_requests;
    }
}

static void BasicSendRxFrame(void)
{
    RxProtocolHeader header;
    HAL_StatusTypeDef status;
    const uint8_t *payload = (const uint8_t *)rx_buffer;
    uint32_t payload_bytes = RX_SAMPLES * sizeof(rx_buffer[0]);
    uint16_t payload_format = RX_PROTOCOL_PAYLOAD_ADC_U16;

    /* Called only after ADC sampling and TX output have stopped. UART does
       not run inside the ADC/DMA IRQ or consume the acquisition timeout. */
    header.sequence = app_frame_count;
    header.sample_count = RX_SAMPLES;
    header.sample_rate_hz = app_rx_sample_rate_hz;
    header.flags = (app_rx_clipped_count != 0u) ? RX_PROTOCOL_FLAG_CLIPPED : 0u;
    if ((basic_mode == APP_CMD_TX_RX) || (basic_mode == APP_CMD_CHIRP_RX))
        header.flags |= RX_PROTOCOL_FLAG_TX_RX;
    const bool chirp_frame = (basic_mode == APP_CMD_CHIRP_RX) ||
                             (basic_mode == APP_CMD_DSP_SIMULATION);
    if (chirp_frame) header.flags |= RX_PROTOCOL_FLAG_CHIRP;
    if (basic_mode == APP_CMD_DSP_SIMULATION) header.flags |= RX_PROTOCOL_FLAG_SIMULATED;
    if (basic_mode == APP_CMD_CHIRP_RX) {
        header.flags |= frame_temperature.valid ? RX_PROTOCOL_FLAG_TEMP_COMPENSATED :
                                                  RX_PROTOCOL_FLAG_TEMP_UNAVAILABLE;
        if (frame_nominal_speed) header.flags |= RX_PROTOCOL_FLAG_NOMINAL_SPEED;
    }
    header.tx_start_sample = (chirp_frame || basic_mode == APP_CMD_TX_RX) ?
                              app_chirp_tx_start_sample : 0u;
    header.chirp_start_hz = chirp_frame ? APP_CHIRP_START_HZ : 0u;
    header.chirp_end_hz = chirp_frame ? APP_CHIRP_END_HZ : 0u;
    header.chirp_duration_us = chirp_frame ? APP_CHIRP_DURATION_US : 0u;
    header.range_candidate_mm = 0u;
    header.beat_millihz = 0u;
    header.peak_ratio_x100 = 0u;
    header.dsp_cycles = 0u;
    if (chirp_frame) {
        ChirpDspResult result;
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        const uint32_t cycle_start = DWT->CYCCNT;
        bool dsp_ok = chirp_dsp_process_method_mm_s(rx_buffer, RX_SAMPLES, app_rx_sample_rate_hz,
            app_chirp_tx_start_sample, APP_CHIRP_START_HZ, APP_CHIRP_END_HZ,
            APP_CHIRP_DURATION_US, frame_temperature.sound_speed_mm_s,
            100u, APP_CHIRP_MAX_RANGE_MM, APP_CHIRP_METHOD, &result);
        app_chirp_dsp_cycles = DWT->CYCCNT - cycle_start;
        app_chirp_dsp_time_us = (SystemCoreClock != 0u) ?
            (uint32_t)(((uint64_t)app_chirp_dsp_cycles * 1000000u) / SystemCoreClock) : 0u;
        app_chirp_processing_error = dsp_ok ? 0u : 1u;
        const bool temperature_ok = (basic_mode == APP_CMD_DSP_SIMULATION) ||
                                     frame_temperature.valid || frame_nominal_speed;
        app_chirp_result_valid = (dsp_ok && result.valid && temperature_ok) ? 1u : 0u;
        app_chirp_range_candidate_mm = dsp_ok ? result.range_mm : 0u;
        app_chirp_beat_millihz = dsp_ok ? result.beat_millihz : 0u;
        app_chirp_peak_ratio_x100 = dsp_ok ? result.peak_ratio_x100 : 0u;
        for (uint32_t method = 0u; method < CHIRP_DSP_METHOD_COUNT; ++method) {
            app_chirp_method_beat_millihz[method] = dsp_ok ? result.method_beat_millihz[method] : 0u;
            app_chirp_method_range_mm[method] = dsp_ok ? result.method_range_mm[method] : 0u;
        }
        if (app_chirp_result_valid != 0u) header.flags |= RX_PROTOCOL_FLAG_RANGE_CANDIDATE;
        /* Candidate only: NOMINAL_SPEED explicitly distinguishes an estimate
           made without air temperature from a compensated result. */
        header.range_candidate_mm = (app_chirp_result_valid != 0u) ? app_chirp_range_candidate_mm : 0u;
        header.beat_millihz = app_chirp_beat_millihz;
        header.peak_ratio_x100 = app_chirp_peak_ratio_x100;
        header.dsp_cycles = app_chirp_dsp_cycles;
        if (dsp_ok && !frame_raw_payload) {
            payload = (const uint8_t *)chirp_dsp_baseband_iq();
            payload_bytes = CHIRP_DSP_OUTPUT_SAMPLES * 2u * sizeof(int16_t);
            payload_format = RX_PROTOCOL_PAYLOAD_IQ_S16;
            header.sample_count = CHIRP_DSP_OUTPUT_SAMPLES;
            header.sample_rate_hz = app_rx_sample_rate_hz / CHIRP_DSP_DECIMATION;
        }
    }
    if (app_uart_enabled == 0u) return;
    app_uart_busy = 1u;
    uint8_t packet_header[RX_PROTOCOL_V4_HEADER_SIZE];
    uint8_t crc_bytes[RX_PROTOCOL_CRC_SIZE];
    size_t header_length;
    if (chirp_frame) {
        const RxProtocolHeaderV4 extended = {header, app_rx_sample_rate_hz,
            frame_temperature.sound_speed_mm_s, frame_temperature.temperature_centi_c,
            payload_format, APP_CHIRP_METHOD};
        header_length = protocol_encode_header_v4(packet_header, sizeof(packet_header), &extended);
    } else {
        header_length = protocol_encode_header(packet_header, sizeof(packet_header), &header);
    }
    app_uart_frame_bytes = (uint32_t)header_length + payload_bytes + RX_PROTOCOL_CRC_SIZE;
    status = (header_length != 0u) ? HAL_OK : HAL_ERROR;
    if (status == HAL_OK) {
        uint16_t crc = protocol_crc16_update(0xFFFFu, packet_header, header_length);
        /* Cortex-M4 STM32G474 is little-endian; uint16 ADC codes can be sent directly. */
        crc = protocol_crc16_update(crc, payload, payload_bytes);
        crc_bytes[0] = (uint8_t)crc;
        crc_bytes[1] = (uint8_t)(crc >> 8u);
        /* v4 IQ: 4164 bytes / 0.361 s. Raw v3: 32820 bytes / 2.849 s. */
        status = HAL_UART_Transmit(&huart4, packet_header, (uint16_t)header_length, 5000u);
        /* Raw ADC payloads take 2.85 s. Leave a scheduling point every 64 bytes
           (5.6 ms at 115200 baud) and finish a due ADC5 scan in the UART gap.
           Acoustic TX/RX is idle and this frame's temperature is already frozen. */
        const uint32_t payload_start_tick = HAL_GetTick();
        while (status == HAL_OK && payload_bytes > 0u) {
            const uint32_t elapsed = HAL_GetTick() - payload_start_tick;
            if (elapsed >= 5000u) { status = HAL_TIMEOUT; break; }
            if (App_TemperatureProcess(true)) continue;
            const uint16_t count = (payload_bytes > 64u) ? 64u : (uint16_t)payload_bytes;
            status = HAL_UART_Transmit(&huart4, payload, count, 5000u - elapsed);
            payload += count;
            payload_bytes -= count;
        }
        if (status == HAL_OK)
            status = HAL_UART_Transmit(&huart4, crc_bytes, sizeof(crc_bytes), 5000u);
    }
    app_uart_last_status = (uint32_t)status;
    app_uart_busy = 0u;
    if (status == HAL_OK) ++app_uart_frames_sent;
    else ++app_uart_send_errors;
    /* Link failure does not invalidate a successfully acquired RX frame.
       HAL_OK confirms UART transmission, not that a PC received the frame. */
}

void App_EmergencyOutputsOff(void)
{
    /* Direct writes work even if a HAL handle is locked or in error state. */
    if (hhrtim1.Instance != NULL) {
        hhrtim1.Instance->sCommonRegs.ODISR =
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2;
        hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].TIMxDIER = 0u;
        CLEAR_BIT(hhrtim1.Instance->sMasterRegs.MCR,
                  HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    }
    app_tx_active = 0u;
}

static void BasicStopTx(void)
{
    App_EmergencyOutputsOff();
    if ((hhrtim1.hdmaTimerB != NULL) &&
        (HAL_DMA_GetState(hhrtim1.hdmaTimerB) == HAL_DMA_STATE_BUSY)) {
        (void)HAL_DMA_Abort(hhrtim1.hdmaTimerB);
    }
}

static void BasicFail(uint32_t error, HAL_StatusTypeDef status)
{
    BasicStopTx();
    if (acquisition_active) App_StopRxFrame();
    app_error = error;
    app_last_hal_status = (uint32_t)status;
    app_rx_valid = 0u;
    app_state = APP_STATE_ERROR;
}

static HAL_StatusTypeDef BasicConfigureTx(uint32_t frequency, uint32_t cycles)
{
    HRTIM_TimeBaseCfgTypeDef base = {0};
    HRTIM_TimerCfgTypeDef timer = {0};
    HRTIM_CompareCfgTypeDef compare = {0};
    HRTIM_OutputCfgTypeDef output = {0};
    uint32_t period, gap;
    const uint32_t units[4] = {HRTIM_COMPAREUNIT_1, HRTIM_COMPAREUNIT_2,
                               HRTIM_COMPAREUNIT_3, HRTIM_COMPAREUNIT_4};

    /* Current board clock contract: 170 MHz HRTIM input, MUL8 counter. */
    if ((HAL_RCC_GetHCLKFreq() != 170000000u) ||
        (frequency < 25000u) || (frequency > 200000u) ||
        (cycles < 2u) || (cycles > APP_TX_MAX_CYCLES)) return HAL_ERROR;
    period = (1360000000u + frequency / 2u) / frequency;
    /* gap is half of the both-low interval. 340 ticks = 250 ns at MUL8. */
    gap = (uint32_t)(((uint64_t)1360000000u * APP_TX_GAP_NS) / 2000000000u);
    if ((gap < 32u) || (period / 2u <= 2u * gap) || (period > 0xFFDFu)) return HAL_ERROR;

    BasicStopTx();
    /* A previous DMA failure must not leave the HAL handle unusable. */
    hhrtim1.State = HAL_HRTIM_STATE_READY;
    base.Period = period;
    base.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    base.Mode = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &base) != HAL_OK) return HAL_ERROR;

    timer.InterruptRequests = HRTIM_TIM_IT_NONE;
    timer.DMARequests = HRTIM_TIM_DMA_REP;
    timer.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    timer.UpdateGating = HRTIM_UPDATEGATING_DMABURST_UPDATE;
    timer.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    timer.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    timer.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
    /* Four compares create a both-low interval; no complementary DT block. */
    timer.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timer) != HAL_OK) return HAL_ERROR;

    for (uint32_t i = 0u; i < cycles; ++i) {
        tx_table[i][0] = period;
        tx_table[i][1] = gap;
        tx_table[i][2] = period / 2u - gap;
        tx_table[i][3] = period / 2u + gap;
        tx_table[i][4] = period - gap;
        /* Two quiet rows account for preload latency. Set compares beyond
           PER: no new rising edge, even if the CPU is halted after DMA TC. */
        if (i >= cycles - 2u) {
            tx_table[i][1] = 0xFFFFu;
            tx_table[i][3] = 0xFFFFu;
        }
    }
    tx_table_words = cycles * 5u;
    tx_table_start = 0u;

    /* Initial active row precedes the first DMA request. The first request
       is at rollover 1; row 0 becomes active at rollover 2. Quiet row N-2
       becomes active at rollover N under the normal DMA timing budget. */
    const uint32_t initial[4] = {gap, period / 2u - gap,
                                 period / 2u + gap, period - gap};
    for (uint32_t i = 0u; i < 4u; ++i) {
        compare.CompareValue = initial[i];
        compare.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
        if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                           units[i], &compare) != HAL_OK) return HAL_ERROR;
    }

    output.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    output.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    output.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    output.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    output.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2 | HRTIM_OUTPUTRESET_TIMPER;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                      HRTIM_OUTPUT_TB1, &output) != HAL_OK) return HAL_ERROR;
    output.SetSource = HRTIM_OUTPUTSET_TIMCMP3;
    output.ResetSource = HRTIM_OUTPUTRESET_TIMCMP4 | HRTIM_OUTPUTRESET_TIMPER;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                      HRTIM_OUTPUT_TB2, &output) != HAL_OK) return HAL_ERROR;

    /* Rewriting this selection also resets the burst redirection pointer
       after an aborted transfer (RM0440 burst DMA description). */
    if (HAL_HRTIM_BurstDMAConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
          HRTIM_BURSTDMA_PER | HRTIM_BURSTDMA_CMP1 | HRTIM_BURSTDMA_CMP2 |
          HRTIM_BURSTDMA_CMP3 | HRTIM_BURSTDMA_CMP4) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_B) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_B) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
          HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
          HRTIM_OUTPUT_TB2, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) return HAL_ERROR;
    __HAL_HRTIM_TIMER_CLEAR_FLAG(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_FLAG_REP);
    return HAL_OK;
}

static HAL_StatusTypeDef BasicConfigureChirp(void)
{
    HRTIM_TimeBaseCfgTypeDef base = {0};
    HRTIM_TimerCfgTypeDef timer = {0};
    HRTIM_CompareCfgTypeDef compare = {0};
    HRTIM_OutputCfgTypeDef output = {0};
    const uint32_t units[4] = {HRTIM_COMPAREUNIT_1, HRTIM_COMPAREUNIT_2,
                               HRTIM_COMPAREUNIT_3, HRTIM_COMPAREUNIT_4};
    const uint32_t timer_clock_hz = 1360000000u; /* 170 MHz x HRTIM MUL8 */
    const uint32_t gap_ticks = (uint32_t)(((uint64_t)timer_clock_hz * APP_TX_GAP_NS) /
                                          2000000000u);

    if ((HAL_RCC_GetHCLKFreq() != 170000000u) ||
        (APP_CHIRP_STEPS < 2u) ||
        (APP_CHIRP_END_HZ <= APP_CHIRP_START_HZ) ||
        ((APP_CHIRP_START_HZ + APP_CHIRP_END_HZ) / 2u > 45000u)) return HAL_ERROR;

    BasicStopTx();
    hhrtim1.State = HAL_HRTIM_STATE_READY;
    /* Start the very first carrier period at the first table period. */
    base.Period = chirp_tx_period_ticks(0u, APP_CHIRP_START_HZ, APP_CHIRP_END_HZ,
                                       APP_CHIRP_DURATION_US, timer_clock_hz);
    base.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    base.Mode = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &base) != HAL_OK)
        return HAL_ERROR;

    timer.InterruptRequests = HRTIM_TIM_IT_NONE;
    timer.DMARequests = HRTIM_TIM_DMA_REP;
    timer.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    timer.UpdateGating = HRTIM_UPDATEGATING_DMABURST_UPDATE;
    timer.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    timer.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    timer.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
    timer.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timer) != HAL_OK)
        return HAL_ERROR;

    for (uint32_t i = 0u; i < APP_CHIRP_STEPS - 1u; ++i) {
        const uint32_t cycle = (i == 0u) ? 0u : i + 1u;
        const uint32_t period = chirp_tx_period_ticks(cycle, APP_CHIRP_START_HZ,
            APP_CHIRP_END_HZ, APP_CHIRP_DURATION_US, timer_clock_hz);
        if ((period / 2u <= 2u * gap_ticks) || (period > 0xFFDFu)) return HAL_ERROR;
        tx_table[i][0] = period;
        tx_table[i][1] = gap_ticks;
        tx_table[i][2] = period / 2u - gap_ticks;
        tx_table[i][3] = period / 2u + gap_ticks;
        tx_table[i][4] = period - gap_ticks;
    }

    /* Two silent rows allow the final chirp period to pass through both the
       DMA transfer and preload pipeline before the foreground stops Timer B. */
    const uint32_t last_period = tx_table[APP_CHIRP_STEPS - 2u][0];
    for (uint32_t i = APP_CHIRP_STEPS - 1u; i < APP_CHIRP_STEPS + 1u; ++i) {
        tx_table[i][0] = last_period;
        tx_table[i][1] = 0xFFFFu;
        tx_table[i][2] = 0xFFFFu;
        tx_table[i][3] = 0xFFFFu;
        tx_table[i][4] = 0xFFFFu;
    }

    const uint32_t initial[4] = {tx_table[0][1], tx_table[0][2],
                                 tx_table[0][3], tx_table[0][4]};
    for (uint32_t i = 0u; i < 4u; ++i) {
        compare.CompareValue = initial[i];
        compare.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
        if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                           units[i], &compare) != HAL_OK) return HAL_ERROR;
    }

    output.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    output.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    output.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    output.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    output.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2 | HRTIM_OUTPUTRESET_TIMPER;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                      HRTIM_OUTPUT_TB1, &output) != HAL_OK) return HAL_ERROR;
    output.SetSource = HRTIM_OUTPUTSET_TIMCMP3;
    output.ResetSource = HRTIM_OUTPUTRESET_TIMCMP4 | HRTIM_OUTPUTRESET_TIMPER;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                      HRTIM_OUTPUT_TB2, &output) != HAL_OK) return HAL_ERROR;

    if (HAL_HRTIM_BurstDMAConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
          HRTIM_BURSTDMA_PER | HRTIM_BURSTDMA_CMP1 | HRTIM_BURSTDMA_CMP2 |
          HRTIM_BURSTDMA_CMP3 | HRTIM_BURSTDMA_CMP4) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_B) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_B) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
          HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) return HAL_ERROR;
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
          HRTIM_OUTPUT_TB2, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) return HAL_ERROR;
    __HAL_HRTIM_TIMER_CLEAR_FLAG(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_FLAG_REP);

    tx_table_start = 1u; /* Row 0 is installed directly before Timer B starts. */
    tx_table_words = APP_CHIRP_STEPS * 5u;
    return HAL_OK;
}

static HAL_StatusTypeDef BasicStartTx(void)
{
    HAL_StatusTypeDef status;
    tx_dma_error = 0u;
    app_tx_dma_done = 0u;
    app_tx_dma_elapsed_us = 0u;
    app_tx_done = 0u;
    __DMB();
    status = HAL_HRTIM_BurstDMATransfer(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                       (uint32_t)&tx_table[tx_table_start][0], tx_table_words);
    if (status != HAL_OK) return status;
    /* BurstDMATransfer arms DMA only. Enable the REP request explicitly.
       Do not use CountStart_DMA here: it would replace the DMA callbacks. */
    __HAL_HRTIM_TIMER_ENABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_DMA_REP);
    /* With preload/update gating enabled, enabling the request after the last
       software update left CNDTR unchanged on this STM32G474. Latch the final
       configuration AFTER arming DMA and BEFORE enabling the counter/outputs.
       Verified with outputs disabled: update-before-enable failed, reverse passed. */
    status = HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_B);
    if (status != HAL_OK) return status;
    /* Software reset events are ignored while this timing unit is stopped
       on the tested board: CNT retained the previous frame's phase. Write
       CNT explicitly before enabling outputs/counting. Otherwise each chirp
       can start up to one 26-us carrier period early (about 4.5 mm of bias). */
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0u);
    if (__HAL_HRTIM_GETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B) != 0u) return HAL_ERROR;
    status = HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    if (status != HAL_OK) return status;
    app_tx_active = 1u;
    if ((basic_mode == APP_CMD_CHIRP_RX) || (basic_mode == APP_CMD_TX_RX)) {
        /* RX is armed before Timer B, so sample 0 is the software reference
           for the chirp. Exact TX/RX latency remains a hardware calibration. */
        app_chirp_tx_start_sample = RX_SAMPLES - __HAL_DMA_GET_COUNTER(&hdma_adc1);
        __DMB();
    }
    const uint32_t duration_us = (basic_mode == APP_CMD_CHIRP_RX) ? APP_CHIRP_DURATION_US :
        (uint32_t)(((uint64_t)app_tx_cycles * 1000000u + app_tx_frequency_hz - 1u) /
                   app_tx_frequency_hz);
    tx_deadline_cycles = (SystemCoreClock / 1000000u) * (duration_us + 1000u);
    tx_started_cycles = DWT->CYCCNT;
    return HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
}

void HAL_HRTIM_BurstDMATransferCallback(HRTIM_HandleTypeDef *hrtim, uint32_t timer_index)
{
    if ((hrtim->Instance == HRTIM1) && (timer_index == HRTIM_TIMERINDEX_TIMER_B)) {
        __HAL_HRTIM_TIMER_DISABLE_DMA(hrtim, timer_index, HRTIM_TIM_DMA_REP);
        app_tx_dma_elapsed_us = (DWT->CYCCNT - tx_started_cycles) /
                               (SystemCoreClock / 1000000u);
        tx_dma_finished_tick = HAL_GetTick();
        app_tx_dma_done = 1u;
        /* DMA completion alone is not a frame end. Keep counting so the last
           quiet preload can latch; main waits >=2 ms before disabling B. */
    }
}

void HAL_HRTIM_ErrorCallback(HRTIM_HandleTypeDef *hrtim)
{
    if (hrtim->Instance == HRTIM1) {
        App_EmergencyOutputsOff();
        tx_dma_error = 1u;
    }
}

static void BasicSummarizeRx(void)
{
    uint32_t sum = 0u, low = 4095u, high = 0u, clipped = 0u;
    for (uint32_t i = 0u; i < RX_SAMPLES; ++i) {
        uint32_t value = rx_buffer[i];
        if (value < low) low = value;
        if (value > high) high = value;
        if ((value <= 15u) || (value >= 4080u)) ++clipped;
        sum += value;
    }
    app_rx_min = low;
    app_rx_max = high;
    app_rx_mean = sum / RX_SAMPLES;
    app_rx_peak_to_peak = high - low;
    app_chirp_sample_count = RX_SAMPLES;
    app_chirp_result_valid = 0u;
    app_chirp_processing_error = 0u;
    app_chirp_range_candidate_mm = 0u;
    app_chirp_beat_millihz = 0u;
    app_chirp_peak_ratio_x100 = 0u;
    app_chirp_dsp_cycles = 0u;
    app_chirp_dsp_time_us = 0u;
    for (uint32_t method = 0u; method < CHIRP_DSP_METHOD_COUNT; ++method) {
        app_chirp_method_beat_millihz[method] = 0u;
        app_chirp_method_range_mm[method] = 0u;
    }
    app_rx_clipped_count = clipped;
    rx_adc_clipped = (clipped != 0u);
    /* ADC2 watchdog interrupts may be disabled in the generated config. */
    vbias_out_of_range = (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_AWD1) != 0u);
    __DMB();
    app_rx_valid = 1u;
}

void App_BasicInit(void)
{
    BasicStopTx();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* PA8 is a GPIO low in hrtim.c's USER section; no Boost waveform. */
    __HAL_DBGMCU_FREEZE_HRTIM1();
    __HAL_DBGMCU_FREEZE_TIM15();
    uint32_t clock = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0u) clock *= 2u;
    app_rx_sample_rate_hz = clock / (htim15.Init.Prescaler + 1u) / (htim15.Init.Period + 1u);
    app_state = APP_STATE_IDLE;
    app_error = APP_ERROR_NONE;
    app_command = APP_CMD_RX; /* Boot captures RX once; TX requires a command. */
    App_TemperatureInit();
}

void App_BasicProcess(void)
{
    if (app_command == APP_CMD_STOP) app_uart_test_mode = 0u;
    /* Pin locator: change only PC10. Disconnect external signal wires first.
       Changing mode or sending STOP restores the existing UART4 AF5 config. */
    if (uart_pin_test_active && (app_uart_test_mode != 2u)) {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF5_UART4;
        HAL_GPIO_Init(GPIOC, &gpio);
        uart_pin_test_active = false;
    }
    if ((app_uart_test_mode == 2u) && (app_state != APP_STATE_BUSY) &&
        !acquisition_active && !app_tx_active) {
        if (!uart_pin_test_active) {
            GPIO_InitTypeDef gpio = {0};
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            gpio.Pin = GPIO_PIN_10;
            gpio.Mode = GPIO_MODE_OUTPUT_PP;
            gpio.Pull = GPIO_NOPULL;
            gpio.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIOC, &gpio);
            app_uart_pin_level = 0u;
            app_uart_pin_edges = 0u;
            uart_pin_test_tick = HAL_GetTick();
            uart_pin_test_active = true;
        }
        app_command = APP_CMD_NONE;
        if ((uint32_t)(HAL_GetTick() - uart_pin_test_tick) >= 1000u) {
            uart_pin_test_tick = HAL_GetTick();
            app_uart_pin_level ^= 1u;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,
                app_uart_pin_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
            ++app_uart_pin_edges;
        }
        app_uart_pin_readback = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) ? 1u : 0u;
        app_uart_tx_gpio_mode = (GPIOC->MODER >> 20u) & 3u;
        app_uart_tx_gpio_af = (GPIOC->AFR[1] >> 8u) & 15u;
        return;
    }
    /* UART wiring test; never starts acoustic TX. Run only once acquisition
       and Timer B have finished. STOP has priority. 0x55 has five low and
       five high bits including start/stop in 8N1: roughly half-supply DC
       average during sustained transmission. This is not an ADC packet. */
    if ((app_uart_test_mode == 1u) && (app_state != APP_STATE_BUSY) &&
        !acquisition_active && !app_tx_active && (app_command != APP_CMD_STOP)) {
        uint8_t pattern[128];
        for (uint32_t i = 0u; i < sizeof(pattern); ++i) pattern[i] = 0x55u;
        app_command = APP_CMD_NONE;
        app_uart_tx_gpio_mode = (GPIOC->MODER >> 20u) & 3u;
        app_uart_tx_gpio_af = (GPIOC->AFR[1] >> 8u) & 15u;
        app_uart_busy = 1u;
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart4, pattern, sizeof(pattern), 100u);
        app_uart_busy = 0u;
        app_uart_test_status = (uint32_t)status;
        if (status == HAL_OK) app_uart_test_bytes += sizeof(pattern);
        return;
    }
    BasicPollSerialCommand();
    uint32_t command = app_command;
    if (command == APP_CMD_STOP) {
        app_uart_test_mode = 0u;
        app_command = APP_CMD_NONE;
        BasicStopTx();
        if (acquisition_active) App_StopRxFrame();
        app_rx_valid = 0u;
        app_state = APP_STATE_IDLE;
        return;
    }
    /* Temperature ADC activity completes before a new acoustic acquisition.
       Its short wait is outside the RX/TX 50 ms timeout and never delays STOP. */
    const bool temperature_may_start = (app_state != APP_STATE_BUSY) &&
        !acquisition_active && !app_tx_active && !app_uart_busy;
    if (App_TemperatureProcess(temperature_may_start)) return;
    if (app_state != APP_STATE_BUSY) {
        if (command == APP_CMD_NONE) return;
        app_command = APP_CMD_NONE;
        if ((command < APP_CMD_RX) || (command > APP_CMD_DSP_SIMULATION)) {
            BasicFail(APP_ERROR_COMMAND, HAL_ERROR);
            return;
        }
        basic_mode = command;
        frame_temperature = App_TemperatureSnapshot(HAL_GetTick());
        frame_nominal_speed = (command == APP_CMD_CHIRP_RX) &&
                              !frame_temperature.valid && (app_allow_nominal_sound_speed != 0u);
        frame_raw_payload = (app_uart_chirp_raw != 0u);
        if (command == APP_CMD_DSP_SIMULATION) {
            /* Command 6 remains the reproducible 343 m/s software self-test. */
            frame_temperature = (TemperatureSnapshot){0};
            frame_temperature.sound_speed_mm_s = TEMPERATURE_DEFAULT_SOUND_SPEED_MM_S;
        }
        app_chirp_temperature_centi_c = frame_temperature.temperature_centi_c;
        app_chirp_sound_speed_mm_s = frame_temperature.sound_speed_mm_s;
        app_chirp_temperature_valid = frame_temperature.valid ? 1u : 0u;
        app_chirp_temperature_status = (uint32_t)frame_temperature.status;
        app_chirp_temperature_age_ms = frame_temperature.age_ms;
        app_chirp_temperature_source = frame_temperature.valid ? app_temperature_source : 0u;
        app_error = APP_ERROR_NONE;
        app_last_hal_status = HAL_OK;
        app_rx_valid = 0u;
        app_rx_min = app_rx_max = app_rx_mean = app_rx_peak_to_peak = 0u;
        app_rx_clipped_count = 0u;
        app_tx_done = app_tx_dma_done = tx_dma_error = 0u;
        app_chirp_tx_start_sample = 0u;
        app_chirp_result_valid = app_chirp_processing_error = 0u;
        app_chirp_range_candidate_mm = app_chirp_beat_millihz = 0u;
        app_chirp_peak_ratio_x100 = app_chirp_dsp_cycles = app_chirp_dsp_time_us = 0u;
        for (uint32_t method = 0u; method < CHIRP_DSP_METHOD_COUNT; ++method) {
            app_chirp_method_beat_millihz[method] = 0u;
            app_chirp_method_range_mm[method] = 0u;
        }
        rx_frame_done = rx_dma_error = false;
        app_state = APP_STATE_BUSY;
        if (command == APP_CMD_DSP_SIMULATION) {
            /* Safe software self-test: synthesize an echo in SRAM; never start
               HRTIM or enable the MOSFET driver. Change target distance in Watch. */
            if ((app_chirp_sim_target_range_mm < 100u) ||
                (app_chirp_sim_target_range_mm > APP_CHIRP_MAX_RANGE_MM)) {
                BasicFail(APP_ERROR_DSP_SIMULATION, HAL_ERROR);
                return;
            }
            app_chirp_tx_start_sample = APP_CHIRP_SIM_TX_MARKER_SAMPLE;
            if (!chirp_dsp_synthesize_echo(rx_buffer, RX_SAMPLES, 500000u,
                    app_chirp_tx_start_sample, APP_CHIRP_START_HZ, APP_CHIRP_END_HZ,
                    APP_CHIRP_DURATION_US, app_chirp_sim_target_range_mm, 343u,
                    APP_CHIRP_SIM_ECHO_PEAK_CODES, 0x45000000u + app_chirp_sim_target_range_mm)) {
                BasicFail(APP_ERROR_DSP_SIMULATION, HAL_ERROR);
                return;
            }
            BasicSummarizeRx();
            ++app_frame_count;
            BasicSendRxFrame();
            app_state = APP_STATE_DONE;
            return;
        } else if (command == APP_CMD_CHIRP_RX) {
            HAL_StatusTypeDef status = BasicConfigureChirp();
            if (status != HAL_OK) { BasicFail(APP_ERROR_TX_CONFIG, status); return; }
        } else if (command != APP_CMD_RX) {
            HAL_StatusTypeDef status = BasicConfigureTx(app_tx_frequency_hz, app_tx_cycles);
            if (status != HAL_OK) { BasicFail(APP_ERROR_TX_CONFIG, status); return; }
        }
        basic_start_tick = HAL_GetTick();
        if (command != APP_CMD_TX) {
            HAL_StatusTypeDef status = App_StartRxFrame();
            if (status != HAL_OK) { BasicFail(APP_ERROR_RX_START, status); return; }
        }
        if ((command == APP_CMD_TX) || (command == APP_CMD_TX_RX) ||
            (command == APP_CMD_CHIRP_RX)) {
            HAL_StatusTypeDef status = BasicStartTx();
            if (status != HAL_OK) { BasicFail(APP_ERROR_TX_START, status); return; }
        }
    } else if (command != APP_CMD_NONE) {
        /* Do not queue unintended repeated transmissions during an active frame. */
        app_command = APP_CMD_NONE;
    }

    if (rx_dma_error) { BasicFail(APP_ERROR_RX_DMA, HAL_ERROR); return; }
    if (tx_dma_error) { BasicFail(APP_ERROR_TX_DMA, HAL_ERROR); return; }
    if (app_tx_active && !app_tx_dma_done &&
        ((uint32_t)(DWT->CYCCNT - tx_started_cycles) >= tx_deadline_cycles)) {
        BasicFail(APP_ERROR_TX_DMA, HAL_TIMEOUT);
        return;
    }
    if (app_tx_active && app_tx_dma_done &&
        ((uint32_t)(HAL_GetTick() - tx_dma_finished_tick) >= 2u)) {
        BasicStopTx();
        app_tx_done = 1u;
    }
    if (rx_frame_done && !app_rx_valid && (basic_mode != APP_CMD_TX)) BasicSummarizeRx();

    if (((basic_mode == APP_CMD_TX) || app_rx_valid) &&
        ((basic_mode == APP_CMD_RX) || app_tx_done)) {
        ++app_frame_count;
        if (app_rx_valid) BasicSendRxFrame();
        app_state = APP_STATE_DONE; /* Set a breakpoint on this line. */
        return;
    }
    if ((uint32_t)(HAL_GetTick() - basic_start_tick) >= 50u) {
        BasicFail(APP_ERROR_TIMEOUT, HAL_TIMEOUT);
    }
}
