#include "drimiun_drv_hx711.h"
#include "drimiun_drv_spi.h"

#include <string.h>
#include "nrf_log.h"
//#include "nrfx_pwm.h"
//#include "nrfx_spim.h"


uint8_t tx_hx711_buffer_spi[HX711_SPI_BUFFER_SIZE] = {0,0,0};
uint8_t rx_hx711_buffer_spi[HX711_SPI_BUFFER_SIZE] = {0,0,0};

nrfx_spim_xfer_desc_t const transfer_buffers_hx711 = {
		.p_tx_buffer = tx_hx711_buffer_spi,
		.tx_length = HX711_SPI_BUFFER_SIZE,
		.p_rx_buffer = rx_hx711_buffer_spi,
		.rx_length = HX711_SPI_BUFFER_SIZE
};

extern bool spi3initiated;

static nrfx_pwm_t           m_pwm1 = NRFX_PWM_INSTANCE(1);
bool  hx711_spi_transfers_complete = false;

/*
* 
*/
void spmiso_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    hx711_spi_transfers_complete = true;
}

/*
*
*/
void hx711_spim_event_handler(nrfx_spim_evt_t const * p_event,
                              void *                   p_context)
{
    //hx711_spi_transfers_complete = true;
}

void hx711_spi_setup(void)	    
{
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_hx711_buffer_spi, HX711_SPI_BUFFER_SIZE, rx_hx711_buffer_spi, HX711_SPI_BUFFER_SIZE);
    
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_500K;
    spi_config.ss_pin         = SPIM_SS_PIN;
    spi_config.miso_pin       = SPIM_MISO_PIN;
    spi_config.mosi_pin       = SPIM_MOSI_PIN;
    spi_config.sck_pin        = SPIM_SCK_PIN;
    spi_config.mode           = NRF_SPIM_MODE_1; //IMPORTANT: This mode is the only that give correct data
    spi_config.use_hw_ss      = true;
    spi_config.ss_active_high = false;
    
    if(spi3initiated) nrfx_spim_uninit(&spi);
    ret_code_t err_code = nrfx_spim_init(&spi, &spi_config, hx711_spim_event_handler, NULL);
    APP_ERROR_CHECK(err_code);
    spi3initiated = true;

    NRF_LOG_INFO("HX711 Setup %x %x %x", rx_hx711_buffer_spi[0], rx_hx711_buffer_spi[1], rx_hx711_buffer_spi[2]);

}


void hx711_spi_device_config()
{
    uint32_t err_code;
    //Config the pwm to set additional pulses in SPI for hx711 peripheral
    nrfx_pwm_config_t const config1 =
    {
        .output_pins =
        {
            ALT_SPIM_SCK_PIN,                                       // channel 0
            NRFX_PWM_PIN_NOT_USED,                                        // channel 1
            NRFX_PWM_PIN_NOT_USED,                                        // channel 2
            NRFX_PWM_PIN_NOT_USED,                                        // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .base_clock   = NRF_PWM_CLK_2MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 1,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrfx_pwm_init(&m_pwm1, &config1, NULL);
    APP_ERROR_CHECK(err_code);
  
  //Nothing to send to configure this device
}

void hx711_spi_sampling_event_config()
{
    ret_code_t err_code;
 
    uint32_t spi_evt_addr;
    uint32_t spi_task_addr;
    uint32_t data_in_evt_addr;
    uint32_t put_clk_pulses_task_addr;
    uint32_t disable_ppi_channel_start_task_addr;
    uint32_t enable_ppi_channel_group_task_addr;

    
    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_detect_hx711_start_tx); //Detect TX Ready
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_hx711_stop_spi); 
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_group_alloc(&m_ppi_group_hx711);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_include_in_group(m_ppi_channel_detect_hx711_start_tx, NRF_PPI_CHANNEL_GROUP0);
    APP_ERROR_CHECK(err_code);

    data_in_evt_addr = nrfx_gpiote_in_event_addr_get(ALT_SPIM_MISO_PIN); //DATAIN going low detect
   
    static uint16_t seq_values[] ={0};
    
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 1,
        .end_delay       = 0
    };
    
    spi_task_addr = nrfx_spim_start_task_get(&spi); //SPI START task address
    put_clk_pulses_task_addr = nrfx_pwm_simple_playback(&m_pwm1, &seq, 1,
            NRFX_PWM_FLAG_STOP |
            NRFX_PWM_FLAG_START_VIA_TASK);
    disable_ppi_channel_start_task_addr = nrfx_ppi_task_addr_get(NRF_PPI_TASK_CHG0_DIS);
    
    //1ST PPI CHANNEL
    err_code = nrfx_ppi_channel_assign(m_ppi_channel_detect_hx711_start_tx,
                                       data_in_evt_addr,
                                       spi_task_addr); //Start SPI RX                                    
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_detect_hx711_start_tx,
                                            disable_ppi_channel_start_task_addr); //Disable DIN Detect
    APP_ERROR_CHECK(err_code);

    
    //2ND PPI CHANNEL
    spi_evt_addr = nrfx_spim_end_event_get(&spi);
    enable_ppi_channel_group_task_addr = nrfx_ppi_task_addr_get(NRF_PPI_TASK_CHG0_EN);
    err_code = nrfx_ppi_channel_assign(m_ppi_channel_hx711_stop_spi,
                                      spi_evt_addr, //SPI end event
                                      put_clk_pulses_task_addr); //Put Extra pulses when SPI transmission ends
                                       
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_hx711_stop_spi, enable_ppi_channel_group_task_addr); //Renable MISO go down detection
    APP_ERROR_CHECK(err_code);

}

void hx711_spi_prepare_transfer(void)
{
    tx_hx711_buffer_spi[0] = 0xAA;
    tx_hx711_buffer_spi[1] = 0xAA;
    tx_hx711_buffer_spi[2] = 0xFF;
    memset(rx_hx711_buffer_spi,0,HX711_SPI_BUFFER_SIZE);
    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(tx_hx711_buffer_spi, HX711_SPI_BUFFER_SIZE, rx_hx711_buffer_spi, HX711_SPI_BUFFER_SIZE);
    uint32_t flags = NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER | NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
    nrfx_spim_xfer(&spi, &xfer, flags);
}

void hx711_spi_sampling_event_enable()
{
    
    ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel_detect_hx711_start_tx);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_group_enable(m_ppi_group_hx711);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_enable(m_ppi_channel_hx711_stop_spi);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(ALT_SPIM_MISO_PIN, true); 

}

void hx711_spi_sampling_event_disable(void)
{
    nrfx_gpiote_in_event_disable(ALT_SPIM_MISO_PIN);

    ret_code_t err_code = nrfx_ppi_channel_disable(m_ppi_channel_hx711_stop_spi);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_group_disable(m_ppi_group_hx711);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_disable(m_ppi_channel_detect_hx711_start_tx);
    APP_ERROR_CHECK(err_code);

}


void hx711_setup(void)
{
  hx711_spi_device_config();
  hx711_spi_sampling_event_config();
}

void hx711_init(void) 
{
  hx711_spi_setup();
  hx711_spi_prepare_transfer();
  hx711_spi_sampling_event_enable();
}

void hx711_uninit(void)
{
  hx711_spi_sampling_event_disable();
}
