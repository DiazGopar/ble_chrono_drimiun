#ifndef DRIMIUN_DRV_LS7366
#define DRIMIUN_DRV_LS7366

#include <stdint.h>
#include "nrfx_spim.h"
#include "nrf_drv_ppi.h"
#include "nrf_gpio.h"
#include "nrfx_timer.h"
#include "drimiun_drv_spi.h"

/* LS7366R Definitions */
/* LS7366R op-code list */ 
#define CLR_MDR0                        0x08    
#define CLR_MDR1                        0x10 
#define CLR_CNTR                        0x20 
#define CLR_STR                         0x30 
#define READ_MDR0                       0x48 
#define READ_MDR1                       0x50 
#define READ_CNTR                       0x60 
#define READ_OTR                        0x68 
#define READ_STR                        0x70 
#define WRITE_MDR1                      0x90 
#define WRITE_MDR0                      0x88 
#define WRITE_DTR                       0x98 
#define LOAD_CNTR                       0xE0   
#define LOAD_OTR                        0xE4 

#define SPIM_SS_LS7366_PIN              NRF_GPIO_PIN_MAP(1,9)

#define PIN_SHDN_5V                     NRF_GPIO_PIN_MAP(0,26)



//Definition for LS7366 SPI device
#define LS7366_SPI_BUFFER_SIZE     5
#define LS7366_SPI_TIMEOUT         2000
#define LS7366_SPI_WRITE_BIT       0x00
#define LS7366_SPI_READ_BIT        0x80

extern uint8_t tx_ls7366_buffer_spi[LS7366_SPI_BUFFER_SIZE]; 
extern uint8_t rx_ls7366_buffer_spi[LS7366_SPI_BUFFER_SIZE];

extern nrfx_spim_xfer_desc_t const transfer_buffers;

/* Flag to indicate to the applications main context that SPI data had been transfer */
extern volatile bool spi_transfers_complete;
extern volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
extern volatile bool spi_tx_done;

static nrf_ppi_channel_t      m_ppi_channel_start_spi;
static nrf_ppi_channel_t      m_ppi_channel_stop_spi;

static const nrfx_timer_t     m_timer1;               // Timer to SPI and UART ??

//TODO: Check if must be one for each peripheral
void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                   p_context);

/** SPI Functions
*
*/

uint32_t spi_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length);
uint32_t spi_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length);

/*
* SPI ppi initialization for ls7366 peripheral
*/
void ls7366_spi_setup(void);	    

/*
* Initial configuration of ls7366 device
*/
void ls7366_spi_device_config(void);

void ls7366_spi_sampling_event_config(void);

void ls7366_spi_prepare_transfer(void);

void ls7366_spi_sampling_event_enable(void);

void timer_handler(nrf_timer_event_t event_type, void * p_context);

void set_5v_power_on();

void set_5v_power_off();

void ls7366_setup();

void ls7366_init(); //Init ls7366 sampling

void ls7366_uninit(); //Stop ls7366 sampling

#endif