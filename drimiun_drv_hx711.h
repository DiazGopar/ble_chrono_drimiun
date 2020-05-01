#ifndef DRIMIUN_DRV_HX711
#define DRIMIUN_DRV_HX711

#include "nrfx_spim.h"
#include "nrf_drv_ppi.h"
#include "nrfx_gpiote.h"
#include "nrfx_pwm.h"
#include "drimiun_drv_spi.h"

 
#define ALT_SPIM_MISO_PIN               NRF_GPIO_PIN_MAP(0,30)     
#define ALT_SPIM_SCK_PIN                NRF_GPIO_PIN_MAP(0,28) 

// Definitions for HX711 SPI device
#define HX711_SPI_BUFFER_SIZE     3
#define HX711_SPI_TIMEOUT         2000
#define HX711_SPI_WRITE_BIT       0x00
#define HX711_SPI_READ_BIT        0x80

extern uint8_t tx_hx711_buffer_spi[HX711_SPI_BUFFER_SIZE];
extern uint8_t rx_hx711_buffer_spi[HX711_SPI_BUFFER_SIZE];

extern nrfx_spim_xfer_desc_t const transfer_buffers_hx711;

extern bool hx711_spi_transfers_complete;

static nrfx_pwm_t             m_pwm1;

static nrf_ppi_channel_t      m_ppi_channel_hx711_stop_spi;

static nrf_ppi_channel_t      m_ppi_channel_start_event;
static nrf_ppi_channel_t      m_ppi_channel_detect_hx711_start_tx;
static nrf_ppi_channel_t      m_ppi_channel_pwm_start;
static nrf_ppi_channel_t      m_ppi_channel_pwm_stop;

static nrf_ppi_channel_group_t  m_ppi_group_hx711;


/**
*
*/
void spmiso_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

void hx711_spi_setup(void);	    

void hx711_spi_device_config(void);

void hx711_spi_sampling_event_config(void);

void hx711_spi_prepare_transfer(void);

void hx711_spi_sampling_event_enable(void);

void hx711_spi_sampling_event_disable(void);

void hx711_setup(void);

void hx711_init(void);

void hx711_uninit(void);



#endif