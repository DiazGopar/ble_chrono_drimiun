#ifndef DRIMIUN_DRV_BATTERY
#define DRIMIUN_DRV_BATTERY

#include "nrfx_pwm.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_saadc.h"

#define APP_SAADC_READ_INTERVAL         15000                                   //TODO: it works x4 some weird FIX IT //   ms between ADc measures. 
#define PIN_BATTERY_MES_ENABLE          NRF_GPIO_PIN_MAP(0,6)                   /**< P0.06 Pin Enable the battery measure resistor bridge. Put this PIN LOW to measure the battery voltage level. Keep voltage HIGH while not reading, then put LOW, read and come back to HIGH. If we missed to put high, the voltage divider disconnect in about 2ms. */
#define PIN_BATTERY_VOLTAGE             NRF_SAADC_INPUT_AIN5

static const nrfx_pwm_t       m_pwm0 = NRFX_PWM_INSTANCE(0);

/** Converts voltage to state of charge (SoC) [%]. The first element corresponds to the voltage 
BATT_MEAS_LOW_BATT_LIMIT_MV and each element is BATT_MEAS_VOLTAGE_TO_SOC_DELTA_MV higher than the previous.
Numbers are obtained via model fed with experimental data. */
//TODO: Revise this table to be more accurate

#define BATT_MEAS_LOW_BATT_LIMIT_MV      3100 // Cutoff voltage [mV].
#define BATT_MEAS_FULL_BATT_LIMIT_MV     4190 // Full charge definition [mV].
#define BATT_MEAS_VOLTAGE_TO_SOC_ELEMENTS 111 // Number of elements in the state of charge vector.
#define BATT_MEAS_VOLTAGE_TO_SOC_DELTA_MV  10 // mV between each element in the SoC vector.
#define BATT_VOLTAGE_DIVIDER_FACTOR         2 // Voltage divider factor

static const uint8_t BATT_MEAS_VOLTAGE_TO_SOC[] = { 
 0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,
 2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,
 4,  5,  5,  5,  6,  6,  7,  7,  8,  8,  9,  9, 10, 11, 12, 13, 13, 14, 15, 16,
18, 19, 22, 25, 28, 32, 36, 40, 44, 47, 51, 53, 56, 58, 60, 62, 64, 66, 67, 69,
71, 72, 74, 76, 77, 79, 81, 82, 84, 85, 85, 86, 86, 86, 87, 88, 88, 89, 90, 91,
91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 100};

uint8_t getBatteryPercentage(uint16_t);
uint16_t getBatterymv(uint16_t);
void saadc_sampling_event_init(void);
void pwm_init(void);
void saadc_sampling_event_enable(void);
void saadc_callback(nrfx_saadc_evt_t const * p_event);
void saadc_init(void);




#endif