#include "drimiun_drv_battery.h"

#include "nrfx_timer.h"
#include "nrf_drv_ppi.h"
#include "nrfx_pwm.h"
#include "nrf_log.h"

static const nrfx_timer_t     m_timer = NRFX_TIMER_INSTANCE(1);                       // Timer to SAADC read
static nrf_ppi_channel_t      m_ppi_channel_adc;             
nrf_saadc_value_t             m_buffer_pool = 0;                                      //ADC buffer to read battery values.
nrf_saadc_value_t             old_buffer_value = 0;


/* Battery Functions*/

uint16_t getBatterymv(uint16_t adcValue)
{
  uint16_t voltage_mv = adcValue * 2400 * BATT_VOLTAGE_DIVIDER_FACTOR / 4096;

  return(voltage_mv);
}

uint8_t getBatteryPercentage(uint16_t battery_mv)
{
  
    int16_t soc_vector_element = (battery_mv - BATT_MEAS_LOW_BATT_LIMIT_MV)/
                                               BATT_MEAS_VOLTAGE_TO_SOC_DELTA_MV;
    
    // Ensure that only valid vector entries are used.
    if (soc_vector_element < 0) {
        soc_vector_element = 0;
    } else if (soc_vector_element > (BATT_MEAS_VOLTAGE_TO_SOC_ELEMENTS - 1) ) {
        soc_vector_element = (BATT_MEAS_VOLTAGE_TO_SOC_ELEMENTS - 1);
    }

    uint8_t battery_level = BATT_MEAS_VOLTAGE_TO_SOC[soc_vector_element];

    return(battery_level);
}


void saadc_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
 
}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrfx_timer_init(&m_timer, &timer_cfg, saadc_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every XXXms */
    uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, APP_SAADC_READ_INTERVAL);
    nrfx_timer_extended_compare(&m_timer,
                                NRF_TIMER_CC_CHANNEL0,
                                ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);


    nrfx_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrfx_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);

    uint32_t saadc_sample_task_addr   = nrfx_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_adc);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_assign(m_ppi_channel_adc,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);

    
    //Fork to ppi channel a LOW pulse to enable the bridge resistor
    static uint16_t /*const*/ seq_values[] ={1,1,1,0};
    
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };

    uint32_t pwm_start_task_address =
        nrfx_pwm_simple_playback(&m_pwm0, &seq, 1,
            NRFX_PWM_FLAG_STOP |
            NRFX_PWM_FLAG_START_VIA_TASK);
    
    err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_adc, pwm_start_task_address);    
   
    APP_ERROR_CHECK(err_code);

}

void pwm_init(void)
{
    uint32_t err_code;
    //Config to send a pulse to enable the battery resistor bridge
    nrfx_pwm_config_t const config0 =
    {
        .output_pins =
        {
            PIN_BATTERY_MES_ENABLE | NRFX_PWM_PIN_INVERTED,               // channel 0
            NRFX_PWM_PIN_NOT_USED,                                        // channel 1
            NRFX_PWM_PIN_NOT_USED,                                        // channel 2
            NRFX_PWM_PIN_NOT_USED,                                        // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .base_clock   = NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 1,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrfx_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);

}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel_adc);

    APP_ERROR_CHECK(err_code);
}



void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE) {
        ret_code_t err_code;
        //nrf_delay_us(10);
        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Battery Value: %d", *p_event->data.done.p_buffer);
        
        //int16_t diff = old_buffer_value - *p_event->data.done.p_buffer;
        //if(abs(diff) < (*p_event->data.done.p_buffer * 0,1) ){
            //nrfx_gpiote_out_clear(PIN_BATTERY_MES_ENABLE);

        uint16_t mvBattery = getBatterymv(*p_event->data.done.p_buffer);
        NRF_LOG_INFO("Battery mv: %d", mvBattery);

        uint8_t perBattery = getBatteryPercentage(mvBattery);
        NRF_LOG_INFO("Battery Percentage: %d", perBattery);      
        
        old_buffer_value = *p_event->data.done.p_buffer;
        //}
    }
}

void saadc_init(void)
{

    ret_code_t          err_code;
    nrfx_saadc_config_t saadc_config =
        NRFX_SAADC_DEFAULT_CONFIG;
    
    saadc_config.resolution = SAADC_RESOLUTION_VAL_12bit;
    saadc_config.oversample = SAADC_OVERSAMPLE_OVERSAMPLE_Over4x;//SAADC_OVERSAMPLE_OVERSAMPLE_Over4x;
    saadc_config.low_power_mode = true;
    

    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIN_BATTERY_VOLTAGE);

    channel_config.gain = NRF_SAADC_GAIN1_4; 
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(&m_buffer_pool, 1);
    APP_ERROR_CHECK(err_code);

}

