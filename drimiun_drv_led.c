#include "drimiun_drv_led.h"


nrfx_i2s_config_t i2s_config = NRFX_I2S_DEFAULT_CONFIG;
nrfx_i2s_buffers_t i2s_buffers;

static uint32_t tx_buffer[I2S_BUFFER_SIZE];

 

// This is the I2S data handler - all data exchange related to the I2S transfers
// is done here.
static void i2s_data_handler(nrfx_i2s_buffers_t const * p_released,
                             uint32_t                   status)
{
    // Non-NULL value in 'p_data_to_send' indicates that the driver needs
    // a new portion of data to send.
    if (p_released->p_tx_buffer != NULL)
    {
        // do nothing - buffer is updated elsewhere
    }
}

void led_init() 
{
  uint32_t err_code;
  i2s_config.sdin_pin  = I2S_SDIN_PIN;
  i2s_config.sdout_pin = I2S_SDOUT_PIN;
  i2s_config.lrck_pin  = I2S_LRCK_PIN;
  i2s_config.sck_pin   = I2S_SCK_PIN;
  i2s_config.mck_setup = NRF_I2S_MCK_32MDIV10; ///< 32 MHz / 10 = 3.2 MHz.
  i2s_config.ratio     = NRF_I2S_RATIO_32X;    ///< LRCK = MCK / 32.
  i2s_config.channels  = NRF_I2S_CHANNELS_STEREO;
  
  err_code = nrfx_i2s_init(&i2s_config, i2s_data_handler);
  APP_ERROR_CHECK(err_code);

  i2s_buffers.p_tx_buffer = tx_buffer;
}

void show_leds()
{
  uint32_t err_code;

  err_code = nrfx_i2s_start(&i2s_buffers, I2S_BUFFER_SIZE, 0);
  APP_ERROR_CHECK(err_code);
}

uint32_t calcChannelValue(uint8_t level)
{
    uint32_t val = 0;

    // 0 
    if(level == 0) {
        val = 0x88888888;
    }
    // 255
    else if (level == 255) {
        val = 0xeeeeeeee;
    }
    else {
        // apply 4-bit 0xe HIGH pattern wherever level bits are 1.
        val = 0x88888888;
        for (uint8_t i = 0; i < 8; i++) {
            if((1 << i) & level) {
                uint32_t mask = ~(0x0f << 4*i);
                uint32_t patt = (0x0e << 4*i);
                val = (val & mask) | patt;
            }
        }

        // swap 16 bits
        val = (val >> 16) | (val << 16);
    }

    return val;
}

void set_led(uint8_t num_led, uint8_t red, uint8_t green, uint8_t blue)
{
  for(int i=0; i<3*NLEDS; i+=3) {
    if(i == num_led*3) {
      tx_buffer[i] = calcChannelValue(green);
      tx_buffer[i+1] = calcChannelValue(red);
      tx_buffer[i+2] = calcChannelValue(blue);
    }
  }
  // reset //FIll the rest of data to reset the line
  for(int i = 3*NLEDS; i < I2S_BUFFER_SIZE; i++) {
    tx_buffer[i] = 0;
  }
}