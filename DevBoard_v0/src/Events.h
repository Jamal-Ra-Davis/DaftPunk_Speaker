#pragma once

typedef enum {
    VOL_P_SHORT_PRESS,
    VOL_P_LONG_PRESS,
    VOL_M_SHORT_PRESS,
    VOL_M_LONG_PRESS,
    PAIR_SHORT_PRESS,
    PAIR_LONG_PRESS,
    CHARGE_START,
    CHARGE_STOP,
    NUM_EVENTS,
} system_event_t;

typedef void (*event_callback_t)(void *ctx);

int init_event_manager();
int register_event_callback(system_event_t event, event_callback_t cb, void *ctx);
int unregister_event_callback(system_event_t event, event_callback_t cb);
bool event_callback_registered(system_event_t event);
int push_event(system_event_t event, bool isr);