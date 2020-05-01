#include "drimiun_drv_button.h"
#include "custom_board.h"
#include "app_button.h"
#include "app_timer.h"
#include "nrf_log.h"

static app_button_cfg_t p_button[] = {{BUTTON_1, APP_BUTTON_ACTIVE_LOW, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
                                      {BUTTON_2, APP_BUTTON_ACTIVE_LOW, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
                                      {BUTTON_3, APP_BUTTON_ACTIVE_LOW, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
                                      {BUTTON_4, APP_BUTTON_ACTIVE_LOW, true, NRF_GPIO_PIN_PULLUP, race_jump_handler}};

static uint8_t debounce_time_ms = 50;

static event_callback_t   m_button_registered_callback         = NULL;
static event_callback_t   m_race_jump_registered_callback       = NULL;

APP_TIMER_DEF(m_app_button_tmr);


void m_app_button_init(void)
{
    app_button_init(p_button, 
                  sizeof(p_button) / sizeof(p_button[0]),
                  APP_TIMER_TICKS(debounce_time_ms));
    app_button_enable();
}


void buttons_init(event_callback_t button_callback, event_callback_t race_jump_callback) {
  ret_code_t err_code = NRF_SUCCESS;

//APP_TIMER_TICKS(30, APP_TIMER_PRESCALER)); // 30ms debounce

  if(button_callback != NULL) {
    m_button_registered_callback = button_callback;
  }
  if(race_jump_callback != NULL) {
    m_race_jump_registered_callback = race_jump_callback;
  }
  
  if (err_code == NRF_SUCCESS) {
    err_code = app_timer_create(&m_app_button_tmr,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                button_timer_handler);
  }
  
  m_app_button_init();
}

uint32_t pin_to_button_idx(uint32_t pin_number)
{
    uint32_t i;
    uint32_t ret = 0xFFFFFFFF;
    for (i = 0; i < BUTTONS_NUMBER; ++i) {
        if (p_button[i].pin_no == pin_number) {
            ret = i;
            break;
        }
    }
    return ret;
}

void button_handler(uint8_t pin_no, uint8_t button_action)
{  
  NRF_LOG_INFO("Button handler Pin %d, Action %d", pin_no, button_action);
}

/**@brief Handle events from button timer.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void button_timer_handler(void * p_context)
{
    button_event_handler(*(uint8_t *)p_context, APP_BUTTON_ACTION_LONG_PUSH);
}

/**@brief Function for handling button events.
 *
 * @param[in]   pin_no          The pin number of the button pressed.
 * @param[in]   button_action   Action button.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    event_t            event  = EVENT_NOTHING;
    uint32_t           button = 0;
    uint32_t           err_code;
    static uint8_t     current_long_push_pin_no;              /**< Pin number of a currently pushed button, that could become a long push if held long enough. */
    
    button = pin_to_button_idx(pin_no);

    if (button < BUTTONS_NUMBER) {
        switch (button_action) {
            case APP_BUTTON_PUSH:
                event = PUSH_EVENT;
                err_code = app_timer_start(m_app_button_tmr, APP_TIMER_TICKS(M_LONG_PUSH_TIMEOUT_MS), (void*)&current_long_push_pin_no);
                if (err_code == NRF_SUCCESS) {
                    current_long_push_pin_no = pin_no;
                }
                break;
            case APP_BUTTON_RELEASE:
                (void)app_timer_stop(m_app_button_tmr);
                event = RELEASE_EVENT;
                break;
            case APP_BUTTON_ACTION_LONG_PUSH:
                event = LONG_PUSH_EVENT;
        }
    }

    NRF_LOG_INFO("Button %d, Pin %d, Event %d", button, pin_no, event);
    
    if ((event != EVENT_NOTHING) && (m_button_registered_callback != NULL)) {
        m_button_registered_callback(button, event);
    }
}



void race_jump_handler(uint8_t pin_no, uint8_t button_action)
{
    static event_t     last_event = 0xFF;
    event_t            event  = EVENT_NOTHING;
    uint32_t           button = 0;
  
    NRF_LOG_INFO("Race handler Pin %d, Action %d", pin_no, button_action);
    
    button = pin_to_button_idx(pin_no);

    switch(button_action) {
        case APP_BUTTON_PUSH:
            event = PUSH_EVENT;
            
            break;
        case APP_BUTTON_RELEASE:
            event = RELEASE_EVENT;
            
            break;
    }
    
    if((event != last_event) && (m_race_jump_registered_callback != NULL)) {
      m_race_jump_registered_callback(button, event);
      last_event = event;
    }
}

void buttons_set_debounce(uint8_t debounce_ms)
{
  ret_code_t err_code;
  debounce_time_ms = debounce_ms;
  err_code = app_button_disable();
  APP_ERROR_CHECK(err_code);
  m_app_button_init();
}

uint8_t buttons_get_debounce()
{
  return debounce_time_ms;
}
