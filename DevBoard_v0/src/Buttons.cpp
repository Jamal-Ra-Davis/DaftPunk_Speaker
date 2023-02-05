#include <Arduino.h>
#include <BluetoothA2DPSink.h>
#include "Buttons.h"
#include "global_defines.h"
#include "Events.h"
#include "Logging.h"

#define VOL_TIMER_PERIOD 500
#define SHORT_PRESS_PERIOD 250
#define DEBOUNCE_PERIOD 50

static void volume_p_button_handler();
static void volume_m_button_handler();
static void pair_button_handler();

typedef enum
{
  VOLUME_PLUS,
  VOLUME_MINUS,
  NUM_VOLUME_KEYS
} volume_key_t;

struct volume_button_data
{
  uint32_t press_time;
  TimerHandle_t timer;
  int pin;
};

struct volume_button_data vol_data[NUM_VOLUME_KEYS];
static char *timer_names[NUM_VOLUME_KEYS] = {"Timer_vol_p", "Timer_vol_m"};

static void volume_timer_func(TimerHandle_t xTimer);

int init_buttons()
{
  pinMode(VOL_P_PIN, INPUT);
  pinMode(VOL_M_PIN, INPUT);
  pinMode(PAIR_PIN, INPUT);

  vol_data[VOLUME_PLUS].timer = xTimerCreate("Timer_vol_p", MS_TO_TICKS(VOL_TIMER_PERIOD), pdFALSE, (void *)VOLUME_PLUS, volume_timer_func);
  vol_data[VOLUME_PLUS].pin = VOL_P_PIN;
  if (vol_data[VOLUME_PLUS].timer == NULL)
  {
    log_err("Failed to create timer for volume plus");
    return -1;
  }

  vol_data[VOLUME_MINUS].timer = xTimerCreate("Timer_vol_m", MS_TO_TICKS(VOL_TIMER_PERIOD), pdFALSE, (void *)VOLUME_MINUS, volume_timer_func);
  vol_data[VOLUME_MINUS].pin = VOL_M_PIN;
  if (vol_data[VOLUME_MINUS].timer == NULL)
  {
    log_err("Failed to create timer for volume minus");
    return -1;
  }

  attachInterrupt(digitalPinToInterrupt(VOL_P_PIN), volume_p_button_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VOL_M_PIN), volume_m_button_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PAIR_PIN), pair_button_handler, CHANGE);
  return 0;
}

static void volume_button_handler(volume_key_t key)
{
  if (digitalRead(vol_data[key].pin) == 0)
  {
    // Button pressed

    // Save press time to compare against release time
    vol_data[key].press_time = millis();

    // Start timer
    if (xTimerStart(vol_data[key].timer, 0) != pdPASS)
    {
      // Failed to start timer
      log_err("Failed to start timer");
    }
  }
  else
  {
    // Button released

    // Stop timer
    if (xTimerStop(vol_data[key].timer, 0) != pdPASS)
    {
      // Failed to stop timer
      log_err("Failed to stop timer");
    }

    // Compare release time to press time to check for short press
    uint32_t delta = millis() - vol_data[key].press_time;
    if (delta < DEBOUNCE_PERIOD)
    {
      // Do nothing
    }
    else if (delta <= SHORT_PRESS_PERIOD)
    {
      // Short press, Push short press events to event queue

      if (key == VOLUME_PLUS)
      {
        push_event(VOL_P_SHORT_PRESS, true);
      }
      else if (key == VOLUME_MINUS)
      {
        push_event(VOL_M_SHORT_PRESS, true);
      }
    }
  }
}
static void volume_p_button_handler()
{
  volume_button_handler(VOLUME_PLUS);
}
static void volume_m_button_handler()
{
  volume_button_handler(VOLUME_MINUS);
}

// Pair button can have normal short press long press behavior - LP: pair action, SP: select action
static void pair_button_handler()
{
  static uint32_t press_time = 0;
  if (digitalRead(PAIR_PIN) == 0)
  {
    // Button pressed

    // Save press time to compare against release time
    press_time = millis();
  }
  else
  {
    // Button released

    // Compare release time to press time to check for short press
    uint32_t delta = millis() - press_time;
    if (delta < DEBOUNCE_PERIOD)
    {
      // Do nothing
    }
    else if (delta <= SHORT_PRESS_PERIOD)
    {
      push_event(PAIR_SHORT_PRESS, true);
    }
    else
    {
      push_event(PAIR_LONG_PRESS, true);
    }
  }
}

static void volume_timer_func(TimerHandle_t xTimer)
{
  uint32_t id;
  id = (uint32_t)pvTimerGetTimerID(xTimer);
  if (id >= NUM_VOLUME_KEYS)
  {
    log_err("Invalid timer ID (%d)", id);
    return;
  }

  // Check if button is still pressed
  if (digitalRead(vol_data[id].pin) == 0)
  {
    if (id == VOLUME_PLUS)
    {
      push_event(VOL_P_SHORT_PRESS, true);
    }
    else if (id == VOLUME_MINUS)
    {
      push_event(VOL_M_SHORT_PRESS, true);
    }

    if (xTimerStart(xTimer, 0) != pdPASS)
    {
      // Failed to start timer
      log_err("Failed to start timer");
    }
  }
}