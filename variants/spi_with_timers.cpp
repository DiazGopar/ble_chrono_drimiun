static void hx711_spi_sampling_event_config()
{
    ret_code_t err_code;
    //nrfx_gpiote_in_config_t  gpiote_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
   
   /*DELETE WHEN USE ALL PPI CODE*/
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
  /* DELETE WHEN USE ALL PPI CODE*/

    uint32_t int_evt_addr;
    uint32_t cs_task_addr;
    uint32_t spi_evt_addr;
    uint32_t spi_task_addr;
    uint32_t data_in_task_addr;
    uint32_t disable_data_in_event_task_addr;
    uint32_t enable_data_in_event_task_addr;
    uint32_t put_aditional_pulses_task_addr;

    //CS configured in GPIO_INIT

    //err_code = nrfx_gpiote_in_init(SPIM_MISO_PIN, &gpiote_config, gpiote_evt_handler);
    //APP_ERROR_CHECK(err_code);

    
    //Allocate PPI channels for hardware
    //Two channels, One to Detect Init Transfer and another to detect final transfer
    //err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_start_event); //Detect DIN going LOW, disable Falling detect, Timer Init
    //APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_start_spi); //Detect Timer Start, SPI start
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_stop_spi); //Detect Timer Stop, Keep 1,2 or 3 pulses on SPI CLK and reenable DIN going low detect
    APP_ERROR_CHECK(err_code);

    data_in_task_addr = nrfx_gpiote_in_event_addr_get(ALT_SPIM_MISO_PIN); //DATAIN going low detect
    nrfx_gpiote_in_event_enable(ALT_SPIM_MISO_PIN, true);
    spi_task_addr = nrfx_spim_start_task_get(&spi); //SPI START task address
    disable_data_in_event_task_addr = nrfx_gpiote_clr_task_addr_get(ALT_SPIM_MISO_PIN); //Disable DIN going low detect task address
    
    
    //1ST PPI CHANNEL
    err_code = nrfx_ppi_channel_assign(m_ppi_channel_start_spi,
                                       data_in_task_addr,
                                       nrf_spim_task_address_get(NRF_SPIM3, NRF_SPIM_TASK_START));
                                       //spi_task_addr);
                                       
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_start_spi,
                                            disable_data_in_event_task_addr); //Diable MISO Line going LOW
    APP_ERROR_CHECK(err_code);
    
    /*2ND PPI CHANNEL
    //err_code = nrfx_ppi_channel_assign(m_ppi_channel_start_spi, 
                                       nrfx_timer_event_address_get(&mtimer1,
                                                                    NRF_TIMER_EVENT_COMPARE0), 
                                       nrfx_timer_task_address_get(&m_timer1,
                                                                    NRF_TIMER_TASK_STOP)); //Detect TIMER START, DISABLE TIMER 
    //APP_ERROR_CHECK(err_code);
    //err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_start_spi, spi_task_addr); //start capture data
    //APP_ERROR_CHECK(err_code);*/

    static uint16_t seq_values[] ={0};
    
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };
    
    
    //enable_data_in_event_task_addr = nrfx_gpiote_in_event_enable(SPIM_MISO_PIN, true);
    enable_data_in_event_task_addr = nrfx_gpiote_set_task_addr_get(ALT_SPIM_MISO_PIN); //Enable DIN going LOW detect
    put_aditional_pulses_task_addr = nrfx_pwm_simple_playback(&m_pwm1, &seq, 1,
            NRFX_PWM_FLAG_STOP |
            NRFX_PWM_FLAG_START_VIA_TASK);
    //spi_evt_addr = nrfx_spim_end_event_get(&spi); //SPI END task address
    //spi_evt_addr = nrf_spim_event_address_get(&spi,NRF_SPIM_EVENT_ENDTX);

    err_code = nrfx_ppi_channel_assign(m_ppi_channel_stop_spi, nrf_spim_event_address_get(NRF_SPIM3,NRF_SPIM_EVENT_ENDRX), put_aditional_pulses_task_addr); //Use pwm tu show X adittional pulses on SPI CLK 
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_stop_spi, enable_data_in_event_task_addr); //Renable MISO go down detection
    APP_ERROR_CHECK(err_code);


}
