#include "car_keys.h"

#include "car_board.h"

#define CAR_KEY_DEBOUNCE_TICKS          (2U)    /* 10 ms */
#define CAR_KEY_LONG_PRESS_TICKS        (200U)  /* 1 s */

static const gpio_pin_enum key_pins[CAR_KEY_COUNT] =
{
    CAR_BOARD_KEY_1,
    CAR_BOARD_KEY_2,
    CAR_BOARD_KEY_3,
    CAR_BOARD_KEY_4,
};

static uint16 key_pressed_ticks[CAR_KEY_COUNT];
static uint8 key_long_reported[CAR_KEY_COUNT];
static car_key_event_enum key_events[CAR_KEY_COUNT];

void car_keys_init(void)
{
    uint8 index;

    for(index = 0U; index < CAR_KEY_COUNT; index++)
    {
        gpio_init(key_pins[index], GPI, GPIO_HIGH, GPI_PULL_UP);
        key_pressed_ticks[index] = 0U;
        key_long_reported[index] = 0U;
        key_events[index] = CAR_KEY_EVENT_NONE;
    }
}

void car_keys_scan_5ms(void)
{
    uint8 index;

    for(index = 0U; index < CAR_KEY_COUNT; index++)
    {
        if(!gpio_get_level(key_pins[index]))
        {
            if(key_pressed_ticks[index] < 0xFFFFU)
            {
                key_pressed_ticks[index]++;
            }
            if((key_pressed_ticks[index] >= CAR_KEY_LONG_PRESS_TICKS) &&
                !key_long_reported[index])
            {
                key_events[index] = CAR_KEY_EVENT_LONG;
                key_long_reported[index] = 1U;
            }
        }
        else
        {
            if((key_pressed_ticks[index] >= CAR_KEY_DEBOUNCE_TICKS) &&
                !key_long_reported[index])
            {
                key_events[index] = CAR_KEY_EVENT_SHORT;
            }
            key_pressed_ticks[index] = 0U;
            key_long_reported[index] = 0U;
        }
    }
}

uint8 car_keys_is_pressed(car_key_id_enum key)
{
    if(key >= CAR_KEY_COUNT)
    {
        return 0U;
    }
    return (uint8)(key_pressed_ticks[key] >= CAR_KEY_DEBOUNCE_TICKS);
}

car_key_event_enum car_keys_take_event(car_key_id_enum key)
{
    car_key_event_enum event;

    if(key >= CAR_KEY_COUNT)
    {
        return CAR_KEY_EVENT_NONE;
    }
    event = key_events[key];
    key_events[key] = CAR_KEY_EVENT_NONE;
    return event;
}
