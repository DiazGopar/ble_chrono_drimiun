#include "drimiun_drv_twim.h"
#include "sdk_config.h"
#include "nrf_drv_twi.h"


/* TWI instance ID. */
#if TWI0_ENABLED
  #define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
  #define TWI_INSTANCE_ID     1
#endif
/* TWI instance. */

nrfx_twim_t      twim = NRFX_TWIM_INSTANCE(TWI_INSTANCE_ID);

/* Indicates if operation on TWI has ended. */
bool m_xfer_done = false;


static void twim_ssd1306_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
  switch (p_event->type) {
    case NRFX_TWIM_XFER_TX:   ///< TX transfer.
      m_xfer_done = true;
      break;
    case NRFX_TWIM_XFER_RX:   ///< RX transfer.
      m_xfer_done = true;
      break;
    case NRFX_TWIM_XFER_TXRX: ///< TX transfer followed by RX transfer with repeated start.
      m_xfer_done = true;
      break;
    case NRFX_TWIM_XFER_TXTX: ///< TX transfer followed by TX transfer with repeated start.
      m_xfer_done = true;
      break;
  }
}

/**
 * @brief TWI initialization.
 */
void twim_init (void)
{
    ret_code_t err_code;

    const nrfx_twim_config_t twim_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWIM_FREQ_400K, //Changed to work with OLED at same time (Old value 100K)
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .hold_bus_uninit    = false
    };

    err_code = nrfx_twim_init(&twim, &twim_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&twim);
}


