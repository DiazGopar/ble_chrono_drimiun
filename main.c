/**
 * Copyright (c) 2019  Drimiun Wireless Systems S.L.
 *
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NRFX_SPIM_ENABLED 1

#include "nrfx_qdec.h"
#include "nrfx_twim.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_spim.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrfx.h"
#include "nrfx_pwm.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_nus.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
//#include "nrf_libuarte_async.h"
/*#include "app_uart.h"
#include "nrf_drv_uart.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif*/
#include "nrfx_uarte.h"
#include "drimiun_drv_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "drimiun_drv_twim.h"

#include "drimiun_drv_led.h"
#include "drimiun_drv_spi.h"
#include "drimiun_drv_ls7366.h"
#include "drimiun_drv_hx711.h"
#include "drimiun_drv_battery.h"
#include "drimiun_drv_button.h"
#include "drimiun_utils.h"

#include "VL53L1X_api.h"
//LCD module
#include "SSD1306.h"
#include "Adafruit_GFX.h"


#define DEVICE_NAME                     "ChronoDrimiun"                         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Drimiun Wireless Systems"              /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "bleChrono"                             /**< Model Number string. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 0x55AA55AA55                            /**< DUMMY Manufacturer ID. Will be passed to Device Information Service. You shall use the ID for your Company*/
#define ORG_UNIQUE_ID                   0xEEBBEE                                /**< DUMMY Organisation Unique ID. Will be passed to Device Information Service. You shall use the Organisation Unique ID relevant for your Company */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                        NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_WATCHDOG_INTERVAL           APP_TIMER_TICKS(300000)                 /**< The watchdog duration in millisenconds*/
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)          /**< Minimum acceptable connection interval (0.01 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(25, UNIT_1_25_MS)         /**< Maximum acceptable connection interval (0.04 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/* PIN definitions*/
#define PIN_KILL                        NRF_GPIO_PIN_MAP(0,7)                   /**< P0.07 Pin Connected to Power On Button. Keep in HIGH to poweron, LOW to poweroff all the system*/

#define PIN_LED1                        NRF_GPIO_PIN_MAP(1,15)
#define PIN_LED2                        NRF_GPIO_PIN_MAP(1,10)

#define PIN_IN_RACE_JUMP                NRF_GPIO_PIN_MAP(0,27)
#define NEOPIX                          NRF_GPIO_PIN_MAP(0,16)

#define ADVERTISING_LED                 LED_1
//Reserved for LED work
//I2S_SDIN_PIN NRF_GPIO_PIN_MAP(1,1);
//I2S_SCK_PIN NRF_GPIO_PIN_MAP(1,4);
//I2S_LRCK_PIN NRF_GPIO_PIN_MAP(1,3);

#define PIN_USER_BUTTON                 NRF_GPIO_PIN_MAP(1,2)
#define PIN_USER_POWER_BUTTON           NRF_GPIO_PIN_MAP(0,8)
#define PIN_USER_PLATFORM               NRF_GPIO_PIN_MAP(0,27)
#define PIN_BATTERY_CHARGING            NRF_GPIO_PIN_MAP(0,4)

#define PIN_XSHUT                       NRF_GPIO_PIN_MAP(0,5)
#define PIN_GPIO1                       NRF_GPIO_PIN_MAP(1,8)
#define CONFIG_QDEC_A_PIN               NRF_GPIO_PIN_MAP(0,3)
#define CONFIG_QDEC_B_PIN               NRF_GPIO_PIN_MAP(0,2)

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(180000)
#define DATA_SEND_INTERVAL              APP_TIMER_TICKS(10)

/* TWI instance ID. */
#if TWI0_ENABLED
  #define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
  #define TWI_INSTANCE_ID     1
#endif
/* TWI instance. */

//#define QDEC_MODULE_ENABLED 1            // Uncomment this to try QDEC hardware module

//#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

/*Application States*/
#define STATE_JUMP_RACE                 0
#define STATE_ENCODER                   1
#define STATE_FORCE                     2
#define STATE_DISTANCE                  3

//NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 3);
//static uint8_t text[] = "UART example started.\r\n Loopback:\r\n";
//static uint8_t text_size = sizeof(text);

static int encoder_value;
static uint8_t app_state;
static uint8_t platform_state;
static uint8_t old_platform_state;
bool spi3initiated = false;                                                     /*Keep track of SPIM module state*/

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */
/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};


//static nrf_libuarte_async_t   libuarte;

extern nrf_saadc_value_t      m_buffer_pool;                                     //ADC buffer to read battery values.


static const nrfx_rtc_t       m_rtc = NRFX_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
extern nrfx_twim_t twim; //share twim instance for i2c peripherals
extern bool m_xfer_done; //share indicator for transfers end on i2c communications

BLE_BAS_DEF(m_bas);                                                             /**< Structure used to identify the battery service. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                               /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                          /**< Context for the Queued Write module.*/                                                         /**< Context for the Queued Write module.*/
//BLE_ADVERTISING_DEF(m_advertising);                                           /**< Advertising module instance. */

APP_TIMER_DEF(m_battery_timer_id);                                              /**< Battery send values timer. */
APP_TIMER_DEF(m_watchdog_timer_id);                                             /**< Watchdog timer to shutdown device.*/
APP_TIMER_DEF(m_data_send_timer_id);                                            /**< Send data over BLE timer*/

//APP_BUTTON_INIT();

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t old_battery_level = 100;                                                //TODO: Read battery level at startup
static bool first_time_battery_readed = true;
volatile uint16_t rtc_overflow_counter = 0;
volatile static int32_t old_encoder_value = 0;
static bool buart_communication_enabled = true;
static bool bble_communication_enabled = true;


// UUUI from service, it must be in reverse order
//const uint8_t encoderservice[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x38,0x05,0x39,0xad}; //"ad390538-d18b-11e6-bf26-cec0c932ce01"
//const uint8_t encoderreadcharac[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x39,0x05,0x39,0xad}; // Encoder data
//const uint8_t encoderwritecharac[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x3a,0x05,0x39,0xad}; // Command receive
//const uint8_t encoderwritelegacycharac[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x3b,0x05,0x39,0xad}; // Legacy to emulate RFDuino

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{   
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_NUS_SERVICE,                  NUS_SERVICE_UUID_TYPE}, 
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};

static nrfx_uarte_t uarte_instance = NRFX_UARTE_INSTANCE(0);
#define UART_RX_BUF_SIZE 1
static uint8_t my_uart_rx_buffer_assigned[UART_RX_BUF_SIZE] = {0x00};
#define UART_TX_BUF_SIZE 5
static uint8_t my_uart_tx_buffer_assigned[UART_TX_BUF_SIZE] = {0x00};

//Button Stuff



						

#ifdef QDEC_MODULE_ENABLED

int32_t qdec_accumulative_value = 0;
int32_t qdec_accumulative_dbl_value = 0;

static void qdec_nrfx_event_handler(nrfx_qdec_event_t event)
{
  
  switch(event.type) {
    case NRF_QDEC_EVENT_SAMPLERDY:
      NRF_LOG_INFO("QDEC Sample ready.");
      qdec_accumulative_value += event.data.sample.value;
      break;
    case NRF_QDEC_EVENT_REPORTRDY:
      NRF_LOG_INFO("QDEC Report ready.");
      qdec_accumulative_value += event.data.report.acc;
      qdec_accumulative_dbl_value += event.data.report.accdbl;
      break;
    case NRF_QDEC_EVENT_ACCOF:
      NRF_LOG_INFO("QDEC OverFlow.");
      break;   
  }

  //qdec_accumulative_value += event.data.report.acc;
  NRF_LOG_INFO("QDEC value: %d", qdec_accumulative_value);
  NRF_LOG_INFO("QDEC DBL value: %d", qdec_accumulative_dbl_value);
}

static int qdec_nrfx_init()
{
    static const nrfx_qdec_config_t config = {
            .reportper          = QDEC_REPORTPER_REPORTPER_10Smpl,//NRF_QDEC_REPORTPER_DISABLED, //We have to modify nrfx_qdec.c to make this work => https://devzone.nordicsemi.com/f/nordic-q-a/37968/nrfx_qdec-driver-quirks?ReplySortBy=CreatedDate&ReplySortOrder=Ascending
            .sampleper          = NRF_QDEC_SAMPLEPER_128us,
            .psela              = CONFIG_QDEC_A_PIN, //p0.03
            .pselb              = CONFIG_QDEC_B_PIN, //p0.02
            .pselled            = 0xFFFFFFFF, /* disabled */
            .ledpol             = NRF_QDEC_LEPOL_ACTIVE_HIGH,
            .interrupt_priority = NRFX_QDEC_CONFIG_IRQ_PRIORITY,
            .dbfen              = 0, /* disabled */
            .sample_inten       = 0, /* disabled */
    };
    nrfx_err_t nerr;
    nerr = nrfx_qdec_init(&config, qdec_nrfx_event_handler);
    
    APP_ERROR_CHECK(nerr);

    nrfx_qdec_enable();

    return 0;

}
#endif

static void advertising_start(bool);
static void sleep_mode_enter();
void change_state(uint8_t);
static void uart_init(uint8_t);

/*
Link: https://devzone.nordicsemi.com/f/nordic-q-a/27597/extending-rtc-beyond-24-bits
Read ticks_overflow
Read RTC counter value
Read ticks_overflow again
Now, if RTC counter value is less than 2^23, use the second ticks_overflow value. Else use the first ticks_overflow value.*/

uint32_t get_current_time_ms()
{
    uint32_t value;
    uint16_t ov_counter1 = rtc_overflow_counter;
    uint32_t rtc_counter = nrfx_rtc_counter_get(&m_rtc);
    uint16_t ov_counter2 = rtc_overflow_counter;
    if(rtc_counter < (1<<23)) {
      value = (ov_counter2 << 24) + rtc_counter;
    } else {
      value = (ov_counter1 << 24) + rtc_counter;
    }

    return(value * 0.030517);     
}

/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_connected(const ble_gap_evt_t * const p_gap_evt)
{
    ret_code_t  err_code;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);

    // Assign connection handle to available instance of QWR module.
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID) {
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }

    // Update LEDs    
    if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) {
        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
    } else {
        // Continue advertising. More connections can be established because the maximum link count has not been reached.
        advertising_start(false);
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
    ret_code_t  err_code;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason);

    // Delete connection handle instance of QWR module.
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
        if (m_qwr[i].conn_handle == p_gap_evt->conn_handle) {
            m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID;
            //err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
            //APP_ERROR_CHECK(err_code);
            break;
        }
    }


    if (periph_link_cnt < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) { 
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
    }
    
    if (periph_link_cnt == 0) {
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
        APP_ERROR_CHECK(err_code);
    }
    
    if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1)) {
        // Advertising is not running when all connections are taken, and must therefore be started.
        advertising_start(false);
    }
}

/**@brief Function to use in NRG_LOG to stamp time.
*
*/

uint32_t get_rtc_counter(void)
{
    return NRF_RTC0->COUNTER;
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */

static void battery_level_update(uint8_t battery_level)
{
    ret_code_t err_code;

    //TODO: Maybe put SAADC values in a buffer and calculate the Average

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint8_t  battery_level;

    battery_level = getBatteryPercentage(getBatterymv(m_buffer_pool));

    
    //int8_t per = old_battery_level * 0.1;
    if(first_time_battery_readed) { //TODO: until we read battery value in startup
      old_battery_level = battery_level;
      first_time_battery_readed = false;
    }

    int8_t diff = old_battery_level - battery_level;
    if(diff <= 0 || (abs(diff) < (old_battery_level * 0.1))) { //battery charging or decrement below 10%
        if(old_battery_level != battery_level) {
            battery_level_update(battery_level);
            old_battery_level = battery_level;
        }
    }
}

/**@brief Function for handling the Watchdog timer timeout.
 *
 * @details This function will be called each time the watchdog timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void watchdog_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

  //Check how many devices are connected  
    if (periph_link_cnt == 0) {
      //Shutdown the system
      NRF_LOG_INFO("Shuting Down.");
      NRF_LOG_FLUSH();
      sleep_mode_enter();
    } else { //Restart the timer
      NRF_LOG_INFO("Watchdog restart.");
      ret_code_t err_code = app_timer_start(m_watchdog_timer_id, APP_WATCHDOG_INTERVAL, NULL);
      APP_ERROR_CHECK(err_code);
    }

}

static void data_send_timeout_handler(void *p_context)
{
  
  uint32_t            time_ms = get_current_time_ms();
  static uint8_t      data_array[BLE_NUS_MAX_DATA_LEN];
  uint16_t            length = 9;
  ret_code_t          err_code = NRF_SUCCESS;
  
  switch(app_state) {
    case STATE_ENCODER:
    case STATE_FORCE:
      if(old_encoder_value != encoder_value) {
        memcpy(&data_array[0],&time_ms,4);
        memcpy(&data_array[4],&encoder_value,4); 

        ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
        // Try sending notifications to all valid connection handles.
        for (uint32_t i = 0; i < conn_handles.len; i++) {
          if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED) {
            if (err_code == NRF_SUCCESS) {
              err_code =  ble_nus_data_send(&m_nus, (uint8_t *) &data_array, &length, conn_handles.conn_handles[i]);
            } else {
              // Preserve the first non-zero error code
              UNUSED_RETURN_VALUE(ble_nus_data_send(&m_nus, (uint8_t *) &data_array, &length, conn_handles.conn_handles[i]));
            }
          }
        }  
        
        old_encoder_value = encoder_value;
      }
      break;

    case STATE_JUMP_RACE:
      break;

  }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    //Timer to send battery values 
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //Timer to Watchdog
    err_code = app_timer_create(&m_watchdog_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                watchdog_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //Timer to send data over BLE
    err_code = app_timer_create(&m_data_send_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                data_send_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


void in_pin_handler( nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action )
{
  //nrfx_gpiote_out_toggle( BSP_LED_1 );
  NRF_LOG_INFO("PIN CHANGED!!");
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    
    NRF_LOG_INFO("ChronoDrimiun GPIO starting.");
    ret_code_t err_code;
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("ChronoDrimiun GPIO init.");

    //Configure 5V powerON/OFF PIN
    nrfx_gpiote_out_config_t out_5v_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrfx_gpiote_out_init(PIN_SHDN_5V, &out_5v_config);
    APP_ERROR_CHECK(err_code);
    
    //Configure System PowerOff PIN
    nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true); 
    err_code = nrfx_gpiote_out_init(PIN_KILL, &out_config);
    APP_ERROR_CHECK(err_code);

    //Configure SPI MISO Pin to get state change
    nrfx_gpiote_in_config_t in_miso_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    //in_miso_config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrfx_gpiote_in_init(ALT_SPIM_MISO_PIN, &in_miso_config, spmiso_evt_handler);
    APP_ERROR_CHECK(err_code);

    //nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

    nrf_gpio_cfg_output(ALT_SPIM_SCK_PIN);
    //nrf_gpio_cfg_output(SPIM_SCK_PIN);
 
    //nrf_gpio_pin_set(ALT_SPIM_SCK_PIN);
    /*nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrfx_gpiote_in_init(PIN_IN_RACE_JUMP, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);
    nrfx_gpiote_in_event_enable(PIN_IN_RACE_JUMP, true);*/
}

void change_uart_baud(uint8_t mode)
{    
  if(mode == BAUDRATE115200){
    nrf_uarte_baudrate_set(uarte_instance.p_reg, NRF_UARTE_BAUDRATE_115200);
  } else if(mode == BAUDRATE9600) {
    nrf_uarte_baudrate_set(uarte_instance.p_reg, NRF_UARTE_BAUDRATE_9600);
  }
  //nrfx_uarte_uninit(&uarte_instance);
  //uart_init(mode);   
}


void process_uart_data(uint8_t *            p_data,
                       size_t               length)
{
   static uint8_t uart_command_state = UART_WAITING_COMMAND;
   uint32_t err_code;

   //NRF_LOG_HEXDUMP_INFO(p_data,length);
   
   switch(uart_command_state) {
    case UART_WAITING_COMMAND:
      switch(p_data[0]){
        case PORT_SCANNING:
              my_uart_tx_buffer_assigned[0] = PORT_SCANNING;
              err_code = nrfx_uarte_tx(&uarte_instance, (uint8_t*)my_uart_tx_buffer_assigned, 1);
              APP_ERROR_CHECK(err_code);
        break;

        case GET_VERSION:
              my_uart_tx_buffer_assigned[0] = '1';
              my_uart_tx_buffer_assigned[1] = '.';
              my_uart_tx_buffer_assigned[2] = '2';
              err_code = nrfx_uarte_tx(&uarte_instance, (uint8_t*)my_uart_tx_buffer_assigned, 3);
              APP_ERROR_CHECK(err_code);
        break;

        case GET_STATUS:
              my_uart_tx_buffer_assigned[0] = 'E';
              my_uart_tx_buffer_assigned[1] = app_button_is_pushed(3)?0x01:0x00;//Value 0 or 1 of jump/Race connector input
              err_code = nrfx_uarte_tx(&uarte_instance, (uint8_t*)my_uart_tx_buffer_assigned, 2);
              APP_ERROR_CHECK(err_code);
        break;

        case GET_DEBOUNCE_TIME:
              my_uart_tx_buffer_assigned[0] = '0' + buttons_get_debounce()/10;
              err_code = nrfx_uarte_tx(&uarte_instance, (uint8_t*)my_uart_tx_buffer_assigned, 1);
        break;

        case SET_DEBOUNCE_TIME:
          uart_command_state = UART_WAITING_DEBOUNCEVALUE;
        break;

        case CHANGEBAUDRATE9600:
          change_uart_baud(BAUDRATE9600);
        break;

        case CHANGEBAUDRATE115200:
          change_uart_baud(BAUDRATE115200);
        break;

        case C1:
        case C2:
        case C3:
        case C4:
        case C5:
        case C6:
        case C7:
        case C8:
        case C9:
        case C10:
        case C11:
        case C12:
        case C13:
        case C14:
        case C15:
          //Reset debouncer to default time
        break;

        default:
          //-1
              my_uart_tx_buffer_assigned[0] = '-';
              my_uart_tx_buffer_assigned[1] = '1';
              err_code = nrfx_uarte_tx(&uarte_instance, (uint8_t*)my_uart_tx_buffer_assigned, 2);
              APP_ERROR_CHECK(err_code);
        break;
  
      
      }
    break;

    case UART_WAITING_DEBOUNCEVALUE:
      //set debounce value
      buttons_set_debounce(p_data[0]);
      uart_command_state = UART_WAITING_COMMAND;
    break;

    default:
      uart_command_state = UART_WAITING_COMMAND;
    break;

   }
 
}

static void my_uarte_event_handler(nrfx_uarte_event_t const * p_event,
                                   void *                     p_context)
{
    uint32_t  err_code;

    NRF_LOG_INFO("UARTE event handler %d", p_event->type);

    if(p_event->type == NRFX_UARTE_EVT_RX_DONE) {
        NRF_LOG_INFO("%d, bytes received!!", p_event->data.rxtx.bytes);
        NRF_LOG_HEXDUMP_INFO((uint8_t*)my_uart_rx_buffer_assigned, UART_RX_BUF_SIZE);
        process_uart_data((uint8_t*)my_uart_rx_buffer_assigned, UART_RX_BUF_SIZE);
        memset(my_uart_rx_buffer_assigned, 0x00, UART_RX_BUF_SIZE); //Clear buffer
        err_code = nrfx_uarte_rx(&uarte_instance, (uint8_t*)my_uart_rx_buffer_assigned, UART_RX_BUF_SIZE); //Reenable Reception
        APP_ERROR_CHECK(err_code);
    } else if(p_event->type == NRFX_UARTE_EVT_TX_DONE) {
        NRF_LOG_INFO("%d, bytes sended!!", p_event->data.rxtx.bytes);
        NRF_LOG_HEXDUMP_INFO((uint8_t*)my_uart_tx_buffer_assigned, UART_TX_BUF_SIZE);
        memset(my_uart_tx_buffer_assigned, 0x00, UART_TX_BUF_SIZE); //Clear buffer
    } else { //NRFX_UARTE_EVT_ERROR Necesary to baud change
        //Reactivate Reception
        memset(my_uart_rx_buffer_assigned, 0x00, UART_RX_BUF_SIZE); //Clear buffer
        err_code = nrfx_uarte_rx(&uarte_instance, (uint8_t*)my_uart_rx_buffer_assigned, UART_RX_BUF_SIZE); //Reenable Reception
        APP_ERROR_CHECK(err_code);
    }
}




/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(uint8_t mode)
{
    uint32_t                     err_code;
    static nrfx_uarte_config_t my_uarte_config = NRFX_UARTE_DEFAULT_CONFIG;
    
    
    my_uarte_config.pselrxd    = RX_PIN_NUMBER;
    my_uarte_config.pseltxd    = TX_PIN_NUMBER;
    my_uarte_config.hwfc       = NRF_UARTE_HWFC_DISABLED;
    if(mode == BAUDRATE9600) {
      my_uarte_config.baudrate = NRF_UARTE_BAUDRATE_9600;
    } else if(mode  == BAUDRATE115200) {
      my_uarte_config.baudrate = NRF_UARTE_BAUDRATE_115200;
    }
    

    err_code = nrfx_uarte_init(&uarte_instance, &my_uarte_config, my_uarte_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_uarte_rx(&uarte_instance, (uint8_t*)my_uart_rx_buffer_assigned, UART_RX_BUF_SIZE);
    APP_ERROR_CHECK(err_code);


}


static void gpiote_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

/* Initialize ppi peripheral, before use it
*
*/
static void ppi_init()
{
  ret_code_t err_code;
 
  err_code = nrf_drv_ppi_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /*err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);*/ //TODO: it's usefull??

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
    
    /*if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }*/

    //ble_bas_on_gatt_evt(&m_bas, p_evt);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);//gatt_evt_handler);//NULL on multiperipheral example
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        
        //TODO: Accept commands from Mobile device
        change_state(p_evt->params.rx_data.p_data[0]);
        
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler    = nrf_qwr_error_handler;
 
    for (uint32_t i = 0; i < LINK_TOTAL; i++){
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }
 
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, "");
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, "0.1");
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, "0.1");
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, "0.1");

    ble_dis_sys_id_t system_id;
    system_id.manufacturer_id            = MANUFACTURER_ID;
    system_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                    = &system_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    ble_conn_state_init();
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    //TODO: See if that is necesary
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED){
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false; //WARNING: this is true on multiperipheral //value of true disconnect IOS in a couple of minutes
    //cp_init.evt_handler                    = on_conn_params_evt; // disable on multiperipheral
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    // -- Battery Timer
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    //-- Watchdog Timer
    err_code = app_timer_start(m_watchdog_timer_id, APP_WATCHDOG_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    //-- Data Send over BLE Timer
    err_code = app_timer_start(m_data_send_timer_id, DATA_SEND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    if (int_type == NRFX_RTC_INT_OVERFLOW) {
      rtc_overflow_counter++;
      NRF_LOG_INFO("RTC Overflow!! Counter: %d", rtc_overflow_counter);
    }
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
    config.prescaler = RTC_FREQ_TO_PRESCALER(32768);
    err_code = nrfx_rtc_init(&m_rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    //nrfx_rtc_tick_enable(&m_rtc,true);

    //Enable OverFlow event
    nrfx_rtc_overflow_enable(&m_rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    //err_code = nrfx_rtc_cc_set(&m_rtc,0,COMPARE_COUNTERTIME * 8,true);
    //APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrfx_rtc_enable(&m_rtc);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    nrfx_gpiote_out_clear(PIN_KILL); //Disconnect main Power

    //Never reach this point
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
    
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
/*static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Advertising Idle.");
            NRF_LOG_FLUSH();
            sleep_mode_enter();
            break;

        default:
                      // No implementation needed.
            NRF_LOG_INFO("Unhandled Advertising Event %d.",
                          ble_adv_evt);
            break;
    }
}*/


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED: //In nus example
            NRF_LOG_INFO("Connected.");
            on_connected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED: //In nus example
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            on_disconnected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: //In nus example
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST: //In nus example
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING: //In nus example
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, 
                                                 NULL, 
                                                 0, 
                                                 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT: //In nus example
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT: //In nus example
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
               
        /*case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;*/

       /* case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
			err_code=sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, 23);
			APP_ERROR_CHECK(err_code);
            break;*/

        default:
            // No implementation needed.
            NRF_LOG_INFO("Unhandled event %d.",
                          p_ble_evt->header.evt_id);
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");
    //TODO: See why this call wont compile
    //err_code = pm_peers_delete();
    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                //err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{

}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t           err_code;
    ble_advdata_t        advdata;
    ble_advdata_t        srdata;
    ble_gap_adv_params_t adv_params;

    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}}; //TODO: change to m_adv_uuids
    
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.p_peer_addr   = NULL;
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    adv_params.interval      = APP_ADV_INTERVAL;

    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.duration        = APP_ADV_DURATION; //BLE_GAP_EVT_ADV_SET_TERMINATED
    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);


}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    //ret_code_t err_code;

    //err_code = nrf_ble_lesc_request_handler();
    //APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true) {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    } else {
      ret_code_t err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
      APP_ERROR_CHECK(err_code);
      bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
    }
}

void change_state(uint8_t state)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_data_send_timer_id);
    APP_ERROR_CHECK(err_code);

    if(state != app_state) { //is a state change, we need to reconfigure   
        switch(app_state){ //first uninit
            case STATE_JUMP_RACE: 
              bble_communication_enabled = false;
              break;

            case STATE_ENCODER: //Uninit quadrature encoder          
              ls7366_uninit();
              break;

            case STATE_FORCE: //Uninit hx711
              hx711_uninit();
              break;

            case STATE_DISTANCE: //Stop VL5x measurement
              //TODO: stop distance measure
              break;

            default:
              return;
        }      
        switch(state){ //second init the appropiate module
            case STATE_JUMP_RACE: 
              set_led(0,10,0,0); //Set led Red
              change_uart_baud(BAUDRATE9600);
              bble_communication_enabled = true;

              break;

            case STATE_ENCODER: 
              ls7366_init();
              set_led(0,0,10,0); //Set led Green
              change_uart_baud(BAUDRATE115200);
              err_code = app_timer_start(m_data_send_timer_id, DATA_SEND_INTERVAL, NULL);
              APP_ERROR_CHECK(err_code);
              break;

            case STATE_FORCE: 
              hx711_init();
              set_led(0,0,0,10); //Set led Blue
              change_uart_baud(BAUDRATE115200);
              err_code = app_timer_start(m_data_send_timer_id, DATA_SEND_INTERVAL, NULL);
              APP_ERROR_CHECK(err_code);
              break;

            case STATE_DISTANCE: 
              //TODO: Start distance measurement
              set_led(0,10,0,10); //Set led Violet
              change_uart_baud(BAUDRATE9600);
              break;

            default:
              return;
            
        }
        app_state = state;
    }      
}


void race_jump_communicate(uint8_t state, uint32_t time)
{
  uint32_t            time_micros;
  static uint8_t      data_array[BLE_NUS_MAX_DATA_LEN];
  uint16_t            length = 9;
  ret_code_t          err_code = NRF_SUCCESS;
  

  if(bble_communication_enabled) {
    if(old_platform_state != state) {  
      memset(data_array, 0x00, BLE_NUS_MAX_DATA_LEN);
      memcpy(&data_array[0],&time,4);
      memcpy(&data_array[4],&state,1);
      ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
      // Try sending notifications to all valid connection handles.
      for (uint32_t i = 0; i < conn_handles.len; i++) {
        if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED) {
          if (err_code == NRF_SUCCESS) {
            err_code =  ble_nus_data_send(&m_nus, (uint8_t *) &data_array, &length, conn_handles.conn_handles[i]);
          } else {
            // Preserve the first non-zero error code
            UNUSED_RETURN_VALUE(ble_nus_data_send(&m_nus, (uint8_t *) &data_array, &length, conn_handles.conn_handles[i]));
          }
        }
      }
      old_platform_state = state;
    }
  }
  
  if(buart_communication_enabled) {
    //send data over serial port
    time_micros = time * 1000 / 8; //To give on units of 1/8 of microseconds( :( ChronoPic CRAP)
    my_uart_tx_buffer_assigned[0] = 'X';
    my_uart_tx_buffer_assigned[1] = state;
    my_uart_tx_buffer_assigned[2] = (time_micros & 0x00FF0000) >> 16;
    my_uart_tx_buffer_assigned[3] = (time_micros & 0x0000FF00) >> 8;
    my_uart_tx_buffer_assigned[4] = (time_micros & 0x000000FF);
    err_code = nrfx_uarte_tx(&uarte_instance, (uint8_t*)my_uart_tx_buffer_assigned, 5);
    APP_ERROR_CHECK(err_code);
  }



}

void race_jump_evt_handler(uint8_t button, event_t evt)
{
  
  NRF_LOG_INFO("RACE_JUMP_3");
  static uint32_t last_time = 0;
  uint32_t current_time = get_current_time_ms();
  uint32_t difference;

  switch(button) {
    case 3:
      switch(evt) {
        case PUSH_EVENT:
            //
            difference = current_time - last_time;
            last_time = current_time;
            //send difference
            race_jump_communicate(1,difference);
        break;

        case RELEASE_EVENT:
            difference = current_time - last_time;
            last_time = current_time;
            race_jump_communicate(0,difference);
        break;

      }
      break;
  
  }


}


/**@brief Function for handling button menu events.
 */
void menu_button_evt_handler(uint8_t button, event_t evt)
{
    uint32_t err_code;
    static    event_t last_event = 0xFF;
    static    uint8_t last_button = 0xFF;


    switch (button) {
        case 0:
            NRF_LOG_INFO("BUTTON_0");
            switch (evt) {
              
              case PUSH_EVENT:
                last_event = evt;
                break;

              case RELEASE_EVENT:
                if(last_button != button || last_event != LONG_PUSH_EVENT) {
                  uint8_t state = (app_state + 1) % 4; //TODO: Make number state parametric dependent
                  change_state(state);
                }
                last_event = evt;
                break;
              
              case LONG_PUSH_EVENT:
                last_event = evt;
                break;
                           
            }
            last_button = button;
            break;

        case 1:
            NRF_LOG_INFO("BSP EVENT BATTERY CHARGING");
            last_button = button;
            break;

        case 2:
            NRF_LOG_INFO("BSP EVENT POWER BUTTON");
            last_button = button;
            break;

        case 3:
            NRF_LOG_INFO("BSP RACE/JUMP"); //TODO: Revise this settings.
            break;

        default:
            return; // no implementation needed
    }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);

    //*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}



/**@brief Function for application main entry.
 */
int main(void)
{
    uint8_t loc_buffer[LS7366_SPI_BUFFER_SIZE];
    bool erase_bonds = false;
    
    // Initialize.
    log_init();
    timers_init();
    uart_init(BAUDRATE9600);
    gpio_init();
    buttons_leds_init(&erase_bonds);
    buttons_init(menu_button_evt_handler, race_jump_evt_handler);

    power_management_init();
    pwm_init(); 
    saadc_init();
    ppi_init(); //Needed for battery(saadc), hx711(spi), ls7366(spi)
  
    saadc_sampling_event_init();   
    saadc_sampling_event_enable();

    ls7366_setup();
    ls7366_init();
    app_state = STATE_ENCODER;
     
    hx711_setup();

    twim_init(); //TODO: Lets OLED working first to restore this to VL5
    
#ifdef QDEC_MODULE_ENABLED
    qdec_nrfx_init();
#endif

    //LED related
    led_init();
    show_leds();
    set_led(0,0,10,0); //Red State 1 encoder


    rtc_config();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

   

    //OLED
    #define SSD1306_CONFIG_SCL_PIN NRF_GPIO_PIN_MAP(0,11)
    #define SSD1306_CONFIG_SDA_PIN NRF_GPIO_PIN_MAP(0,12)

    //ssd1306_init_i2c(SSD1306_CONFIG_SCL_PIN, SSD1306_CONFIG_SDA_PIN);
    //twim_master_init(SSD1306_CONFIG_SCL_PIN, SSD1306_CONFIG_SDA_PIN);
    SSD1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, false);
    Adafruit_GFX_init(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, SSD1306_drawPixel);
    SSD1306_clearDisplay();
    Adafruit_GFX_drawBitmap(0, 0, drimiun_logo_2, 128, 64, 1);
    //SSD1306_display();
    SH1106_display();
    //nrf_delay_ms(3000);
    //SSD1306_clearDisplay();
    //Adafruit_GFX_drawBitmap(0, 0, drimiun_logo_2, 128, 64, 1);
    //SSD1306_display();
    //SH1106_display();
    //nrf_delay_ms(3000);
    //SSD1306_clearDisplay();
    //SH1106_display();

    /*SSD1306_clearDisplay();
    SH1106_display();

    testdrawcircle();
    nrf_delay_ms(1000);
    testdrawText();
    nrf_delay_ms(1000);
  */  
    // Start execution.
    NRF_LOG_INFO("ChronoDrimiun started.");
    application_timers_start();

    advertising_start(erase_bonds);
    

    #define TWI_ADDRESSES      127
    uint8_t address;
    uint8_t found_address;
    uint8_t sample_data = 0;
    bool detected_device = false;
    VL53L1X_ERROR status;
    uint16_t distance;

/*    for (address = 1; address <= TWI_ADDRESSES; address++) {
        //m_xfer_done = false;
        ret_code_t err_code = nrfx_twim_rx(&twim, address, &sample_data, sizeof(sample_data));
        if ((err_code == NRF_SUCCESS) /*&& (m_xfer_done == true)) {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
            found_address = address;
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device) {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
    }*/

    found_address = 41; //workaround to work with oled connected

    while(sample_data == 0) {
      status = VL53L1X_BootState(found_address, &sample_data);
      nrf_delay_ms(2);
      NRF_LOG_INFO("Boot State status: %d",status);
      NRF_LOG_INFO("Boot State data: %d",sample_data);
      NRF_LOG_FLUSH();
    }
    
    status = VL53L1X_SensorInit(found_address);
    NRF_LOG_INFO("Sensor Init value: %d",status);

    status = VL53L1X_SetDistanceMode(found_address, 1);
    NRF_LOG_INFO("SetDistanceMode: %d",status);

    status = VL53L1X_GetDistanceMode(found_address, &distance);
    NRF_LOG_INFO("GetDistanceMode: %d",status);
    NRF_LOG_INFO("Distance Mode: %d",distance);

    status = VL53L1X_SetTimingBudgetInMs(found_address, 15);
    NRF_LOG_INFO("SetTimingBudget status value: %d",status);

    status = VL53L1X_GetTimingBudgetInMs(found_address, &distance);
    NRF_LOG_INFO("Get Timing Budget ms status: %d",status);
    NRF_LOG_INFO("ms : %d",distance);

    status = VL53L1X_SetInterMeasurementInMs(found_address, 15);
    NRF_LOG_INFO("SetInterMeasurement status value: %d",status);

    status = VL53L1X_GetInterMeasurementInMs(found_address, &distance);
    NRF_LOG_INFO("Get Inter Measurement ms status: %d",status);
    NRF_LOG_INFO("ms : %d",distance);

    status = VL53L1X_SetROI(found_address, 16 , 8);
    NRF_LOG_INFO("SetROI status value: %d",status);

    status = VL53L1X_SetROICenter(found_address, 60);
    NRF_LOG_INFO("SetROICenter status value: %d",status);

    status = VL53L1X_StartRanging(found_address);
    NRF_LOG_INFO("Start Ranging status: %d",status);

    NRF_LOG_FLUSH();
    
    nrf_delay_ms(1200);
       
    SSD1306_clearDisplay();
    SH1106_display();
    initdrawtext();

    // Enter main loop.
    for (;;)
    {
        //nrf_delay_ms(1);
        idle_state_handle();
        uint8_t isDataReady = 0;
        
        while(isDataReady == 0) {
          status = VL53L1X_CheckForDataReady(found_address, &isDataReady);   
          //NRF_LOG_INFO("Check data ready status: %d", status);
          //NRF_LOG_INFO("Is data ready %d", isDataReady);
          //NRF_LOG_FLUSH();
        }
        
        status = VL53L1X_GetDistance(found_address,&distance);
        //NRF_LOG_INFO("Distance: %d", distance);
        //status = VL53L1X_ClearInterrupt(found_address);
        //NRF_LOG_INFO("Status of ClearInterrupt: %d", status);
        //NRF_LOG_FLUSH();
        
        //ret_code_t err_code = app_uart_put('X');
        //NRF_LOG_INFO("uart error code %d",err_code);

        /*char data[6];
        sprintf(data,"%d",distance);

        SSD1306_clearDisplay();
        initdrawtext();
        Adafruit_GFX_puts(data);
        SH1106_display();*/
       
      
        if(spi_tx_done) {
          //memset(&encoder_value,0,sizeof(encoder_value));
          revmemcpy(&encoder_value, &rx_ls7366_buffer_spi[1],4); //TODO: It's critical access????
          NRF_LOG_INFO("ENC value: %d", encoder_value);
          spi_tx_done = false;
        }

        if(hx711_spi_transfers_complete) {
          //memset(&encoder_value,0,sizeof(encoder_value));
          //encoder_value = int24toint32(rx_hx711_buffer_spi);
          encoder_value = 0;

          /*if((rx_hx711_buffer_spi[0] & 0x40) > 0) {
            encoder_value = (0xFF << 24) + ((rx_hx711_buffer_spi[0] | 0x80) << 16) + (rx_hx711_buffer_spi[1] << 8) + (rx_hx711_buffer_spi[2]);
          } else {
            encoder_value = (rx_hx711_buffer_spi[0] << 16) + (rx_hx711_buffer_spi[1] << 8) + (rx_hx711_buffer_spi[2]);
          }*/
          
          encoder_value = (rx_hx711_buffer_spi[0] << 16) + (rx_hx711_buffer_spi[1] << 8) + (rx_hx711_buffer_spi[2]);
          encoder_value = encoder_value & 0x800000 ? 0xFF000000 | encoder_value : encoder_value;      
          //revmemcpy(&encoder_value, &rx_hx711_buffer_spi[0],3); //TODO: It's critical access????
          NRF_LOG_INFO("FORCE value: %d", encoder_value);
          hx711_spi_transfers_complete = false;
        }

        //encoder_value = distance;
        
#ifdef QDEC_MODULE_ENABLED        
        NRF_LOG_INFO("QDEC value: %d", qdec_accumulative_value);

        if(qdec_accumulative_dbl_value > 0) {
          set_led(0,10,0,0);
        }
#endif

    }
}
