#ifndef DRIMIUN_DRV_SPI
#define DRIMIUN_DRV_SPI

#include "nrfx_gpiote.h"
#include "nrfx_spim.h"

#define SPIM_SS_PIN                     NRF_GPIO_PIN_MAP(1,9)
#define SPIM_MISO_PIN                   NRF_GPIO_PIN_MAP(0,15)    
#define SPIM_MOSI_PIN                   NRF_GPIO_PIN_MAP(0,13)
#define SPIM_SCK_PIN                    NRF_GPIO_PIN_MAP(0,14)

#define SPI_INSTANCE  3 /**< SPI instance index. */

extern const nrfx_spim_t spi;  /**< SPI instance. */

extern bool spi3initiated; 

#endif