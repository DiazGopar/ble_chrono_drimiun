#ifndef _DRIMIUN_DRV_TWIM_H_
#define _DRIMIUN_DRV_TWIM_H_

#include "nrfx_twim.h"
#include "nrf_gpio.h"

#define SDA_PIN                         NRF_GPIO_PIN_MAP(0,12)
#define SCL_PIN                         NRF_GPIO_PIN_MAP(0,11)

//void drimiun_twim_init(void);
void twim_init (void);

#endif