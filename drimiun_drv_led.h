#ifndef DRIMIUN_DRV_LED
#define DRIMIUN_DRV_LED
#include "nrf_gpio.h"
#include "nrfx_i2s.h"

#define I2S_SDIN_PIN NRF_GPIO_PIN_MAP(1,1); //TODO: Check that nothing is attached to this pin
#define I2S_SDOUT_PIN NRF_GPIO_PIN_MAP(0,16);
#define I2S_SCK_PIN NRF_GPIO_PIN_MAP(1,4);
#define I2S_LRCK_PIN NRF_GPIO_PIN_MAP(1,3);

#define NLEDS 1
#define RESET_BITS 6
#define I2S_BUFFER_SIZE 3*NLEDS + RESET_BITS


void led_init();
void show_leds();
void set_led(uint8_t num_led, uint8_t red, uint8_t green, uint8_t blue);




#endif //DRIMIUN_DRV_LED