#ifndef DRIMIUN_DRV_BUTTON_H
#define DRIMIUN_DRV_BUTTON_H

#include <stdint.h>

#define APP_BUTTON_ACTION_LONG_PUSH (2)                  /**< Represents pushing and holding a button for @ref BSP_LONG_PUSH_TIMEOUT_MS milliseconds. See also @ref bsp_button_action_t. */
#define M_LONG_PUSH_TIMEOUT_MS      (800)

typedef enum
{
    EVENT_NOTHING = 0,                  /**< Assign this event to an action to prevent the action from generating an event (disable the action). */
    PUSH_EVENT,
    LONG_PUSH_EVENT,
    RELEASE_EVENT
} event_t;


//typedef struct
//{
//    bsp_event_t push_event;      /**< The event to fire on regular button press. */
//    bsp_event_t long_push_event; /**< The event to fire on long button press. */
//    bsp_event_t release_event;   /**< The event to fire on button release. */
//} button_event_cfg_t;

typedef void (* event_callback_t)(uint8_t, event_t);


uint8_t buttons_get_debounce();
void buttons_set_debounce(uint8_t);
void button_handler(uint8_t, uint8_t);
static void button_event_handler(uint8_t, uint8_t);
static void button_timer_handler(void *);
void race_jump_handler(uint8_t, uint8_t);
void buttons_init(event_callback_t,event_callback_t);

#endif