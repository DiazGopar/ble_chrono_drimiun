#include "drimiun_drv_ls7366.h"
#include "nrf_log.h"
#include "nrfx_gpiote.h"
#include "nrfx_timer.h"
#include "nrf_delay.h"


uint8_t tx_ls7366_buffer_spi[LS7366_SPI_BUFFER_SIZE] = {0,0,0,0,0};
uint8_t rx_ls7366_buffer_spi[LS7366_SPI_BUFFER_SIZE] = {0,0,0,0,0};

nrfx_spim_xfer_desc_t const transfer_buffers = {
		.p_tx_buffer = tx_ls7366_buffer_spi,
		.tx_length = LS7366_SPI_BUFFER_SIZE,
		.p_rx_buffer = rx_ls7366_buffer_spi,
                .rx_length = LS7366_SPI_BUFFER_SIZE
};

extern bool spi3initiated;
volatile bool ls7366deviceconfigured = false;

/* Flag to indicate to the applications main context that SPI data had been transfer */
volatile bool spi_transfers_complete = false;
volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
volatile bool spi_tx_done = false;

static const nrfx_timer_t     m_timer1 = NRFX_TIMER_INSTANCE(2);


//TODO: Check if must be one for each peripheral
void ls7366_spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                   p_context)
{
    //if(p_event->type == NRFX_SPIM_EVENT_DONE) {
     //   NRF_LOG_DEBUG("TxDone\n");
     //   NRF_LOG_HEXDUMP_INFO(rx_ls7366_buffer_spi, LS7366_SPI_BUFFER_SIZE);
        spi_tx_done = true; //used for write and read spi functions
        spi_transfers_complete = true;
    //} else {
    //    NRF_LOG_DEBUG("Wrong Event\n");
        // Something is wrong
    //}
}

/** SPI Functions
*
*/

uint32_t spi_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = LS7366_SPI_TIMEOUT;

    //Select LS7366
    nrfx_gpiote_out_task_trigger(SPIM_SS_PIN);
	
    // Read data over SPI and store incomming data in spi_rx_buffer
    tx_ls7366_buffer_spi[0] = reg;
    err_code = nrfx_spim_xfer(&spi,&transfer_buffers,0);
    if(err_code != NRF_SUCCESS) return err_code;
    NRF_LOG_DEBUG("Waiting for SPI Xfer\n");

    while((!spi_tx_done) && --timeout);
    NRF_LOG_DEBUG("Timeout: %d\n", timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    spi_tx_done = false;
    //nrfx_gpiote_out_set(SPIM_SS_PIN);
    nrfx_gpiote_out_task_trigger(SPIM_SS_PIN);
	
    // Copy data in spi_rx_buffer over to p_data
    memcpy(p_data, transfer_buffers.p_rx_buffer, length);

    return NRF_SUCCESS;
}

uint32_t spi_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    
    if(length > LS7366_SPI_BUFFER_SIZE) { // Must be space for register byte in buffer
        return NRF_ERROR_DATA_SIZE;
    }
    
    uint32_t timeout = LS7366_SPI_TIMEOUT;
    nrfx_gpiote_out_task_trigger(SPIM_SS_PIN);   
    tx_ls7366_buffer_spi[0] = reg;
    memcpy((tx_ls7366_buffer_spi + 1), p_data, length);   
    err_code = nrfx_spim_xfer(&spi, &transfer_buffers, 0);
    if(err_code != NRF_SUCCESS) return err_code;
    NRF_LOG_DEBUG("Waiting for SPI Xfer\n");
    
    while((!spi_tx_done) && --timeout);
    NRF_LOG_DEBUG("Timeout: %d\n", timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    spi_tx_done = false;
    nrfx_gpiote_out_task_trigger(SPIM_SS_PIN);
    
    return err_code;
}

/*
* SPI ppi initialization for ls7366 peripheral
*/
void ls7366_spi_setup(void)	    
{
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_ls7366_buffer_spi, 
                                                         LS7366_SPI_BUFFER_SIZE, 
                                                         rx_ls7366_buffer_spi, 
                                                         LS7366_SPI_BUFFER_SIZE);
    
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_8M;
    spi_config.ss_pin         = SPIM_SS_PIN;
    spi_config.miso_pin       = SPIM_MISO_PIN;
    spi_config.mosi_pin       = SPIM_MOSI_PIN;
    spi_config.sck_pin        = SPIM_SCK_PIN;
    spi_config.use_hw_ss      = true;
    spi_config.ss_active_high = false;
  
    if(spi3initiated) nrfx_spim_uninit(&spi);
    ret_code_t err_code = nrfx_spim_init(&spi, 
                                         &spi_config, 
                                         ls7366_spim_event_handler, 
                                         NULL);
    
    APP_ERROR_CHECK(err_code);
    spi3initiated = true;

    NRF_LOG_INFO("LS7366 Setup %x %x %x %x %x", 
                  rx_ls7366_buffer_spi[0], 
                  rx_ls7366_buffer_spi[1], 
                  rx_ls7366_buffer_spi[2],
                  rx_ls7366_buffer_spi[3],
                  rx_ls7366_buffer_spi[4]);

}

void ls7366_spi_uninit(void)
{
    nrfx_spim_uninit(&spi);
}

/*
* Initial configuration of ls7366 device
*/
void ls7366_spi_device_config(void)
{
  
  if(!ls7366deviceconfigured) {
    uint32_t err_code;
    uint8_t loc_buffer[LS7366_SPI_BUFFER_SIZE];

    /*
    myLS7366.write_mode_register_0(register_0);
          digitalWrite(CS_pin, LOW);
          SPI.transfer(WRITE_MDR0);
          SPI.transfer(val);
          digitalWrite(CS_pin, HIGH);
    myLS7366.write_mode_register_1(register_1);
    myLS7366.clear_counter();
    myLS7366.clear_status_register();
    myLS7366.write_data_register(4);
    */

    // Write Register 0
    loc_buffer[0] = 0x03; //Quadrature Mode x4
    NRF_LOG_INFO("Writing Register Mode 0");
    err_code = spi_write_registers(WRITE_MDR0,loc_buffer,1);
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(100);
    // Write Register 1
    loc_buffer[0] = 0x00; //Default config
    NRF_LOG_INFO("Writing Register Mode 1");
    err_code = spi_write_registers(WRITE_MDR1,loc_buffer,1);
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(100);
    // Clear Counter
    NRF_LOG_INFO("LS7366R Clear Counter");
    err_code = spi_write_registers(CLR_CNTR,loc_buffer,1);
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(100);
    // Clear Status Register
    NRF_LOG_INFO("LS7366R Status Register");
    err_code = spi_write_registers(CLR_STR,loc_buffer,1);
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(100);
    //Init Counter
    NRF_LOG_INFO("Initiating Counter");
    err_code = spi_write_registers(WRITE_DTR,loc_buffer,4);
    APP_ERROR_CHECK(err_code);
    ls7366deviceconfigured = true;
    nrf_delay_ms(100);
  }
}

/*
* PPI configuration for LS7366
*/

void ls7366_spi_sampling_event_config(void)
{
    ret_code_t err_code;
    uint32_t int_evt_addr;
    uint32_t cs_task_addr;
    uint32_t spi_evt_addr;
    uint32_t spi_task_addr;
    uint32_t gpiote_task_addr;

    

    //Configure LS7366R Slave Select PIN
    nrfx_gpiote_out_config_t out_ss_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); //Start HIGH
    err_code = nrfx_gpiote_out_init(SPIM_SS_PIN, &out_ss_config);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_out_task_enable(SPIM_SS_PIN);

    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrfx_timer_init(&m_timer1, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_timer_extended_compare(&m_timer1,
                            NRF_TIMER_CC_CHANNEL0,
                            nrfx_timer_ms_to_ticks(&m_timer1, 1),
                            NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                            false);

    nrfx_timer_enable(&m_timer1);

    uint32_t timer_compare_event_addr = nrfx_timer_compare_event_address_get(&m_timer1,
                                                                                NRF_TIMER_CC_CHANNEL0);

    //Allocate PPI channels for hardware
    //Every 1ms poll SPI device
    //Two channels, One to Put CS LOW and init SPI transfer and another to Put CS High when SPI transfer stop
    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_start_spi);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_stop_spi);
    APP_ERROR_CHECK(err_code);

    cs_task_addr = nrfx_gpiote_out_task_addr_get(SPIM_SS_PIN);
    spi_task_addr = nrfx_spim_start_task_get(&spi);

    err_code = nrfx_ppi_channel_assign(m_ppi_channel_start_spi, timer_compare_event_addr, cs_task_addr);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_start_spi, spi_task_addr);
    APP_ERROR_CHECK(err_code);

    spi_evt_addr = nrfx_spim_end_event_get(&spi);

    err_code = nrfx_ppi_channel_assign(m_ppi_channel_stop_spi, spi_evt_addr, cs_task_addr);
    APP_ERROR_CHECK(err_code);

}

/*
*
*/
void ls7366_spi_prepare_transfer(void)
{
    tx_ls7366_buffer_spi[0] = READ_CNTR;
    memset(rx_ls7366_buffer_spi,0,LS7366_SPI_BUFFER_SIZE);
    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(tx_ls7366_buffer_spi, 1, rx_ls7366_buffer_spi, LS7366_SPI_BUFFER_SIZE);
    uint32_t flags = NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER;// | NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
    nrfx_spim_xfer(&spi, &xfer, flags);
}


/*
* Enable ppi channel and start sampling ls7366
*/

void ls7366_spi_sampling_event_enable(void)
{    
    

    ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel_stop_spi);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_ppi_channel_enable(m_ppi_channel_start_spi);
    APP_ERROR_CHECK(err_code);
}

/*
* Disable ppi channel and stop sampling ls7366
*/

void ls7366_spi_sampling_event_disable(void)
{       
    ret_code_t err_code = nrfx_ppi_channel_disable(m_ppi_channel_start_spi);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_disable(m_ppi_channel_stop_spi);
    APP_ERROR_CHECK(err_code);

    //nrfx_gpiote_out_task_disable(SPIM_SS_PIN);
}

/*
* Timer handler for spi sampling interval
*/
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
 
}

void set_5v_power_on()
{
    nrfx_gpiote_out_set(PIN_SHDN_5V);
}

void set_5v_power_off()
{
  nrfx_gpiote_out_clear(PIN_SHDN_5V);
}

void ls7366_setup()
{
  ls7366_spi_sampling_event_config();
}

void ls7366_init() //Configure and init ls7366 sampling
{
  ls7366_spi_setup();
  ls7366_spi_device_config();
  ls7366_spi_prepare_transfer();
  set_5v_power_on();
  ls7366_spi_sampling_event_enable();
}

void ls7366_uninit() //Unconfigure and stop ls7366 sampling
{
  set_5v_power_off();
  //ls7366_spi_uninit();
  ls7366_spi_sampling_event_disable();
}