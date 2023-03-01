#include <Arduino.h>
#include <FastLED.h>
#include "rgb_manager.h"
#include "global_defines.h"

#define RGB_MANAGER_TASK_STACK_SIZE 1024

CRGBArray<1> rgb_led;
static rgb_states_t current_state = RGB_1HZ_CYCLE;
static rgb_states_t cached_state = current_state;
static TaskHandle_t xrgb_manager_task = NULL;

static void rgb_manager_task(void *pvParameters);

static void timer_thread_task(void *pvParameters);
static void led_low_battery();
static void _oneshot_blink();
static void rgb_led_cycle();

struct oneshot_blink_data {
    int cnt; 
    int period;
    uint8_t r;
    uint8_t g; 
    uint8_t b;
};

static struct oneshot_blink_data oneshot_data;

int init_rgb_manager()
{
    FastLED.addLeds<NEOPIXEL,RGB_LED_DATA>(rgb_led, 1);
    rgb_led[0] = CRGB::Black;
    FastLED.show();

    current_state = RGB_1HZ_CYCLE;
    cached_state = current_state;
    xTaskCreate(
        rgb_manager_task,
        "RGB_Manager_Task",   // A name just for humans
        RGB_MANAGER_TASK_STACK_SIZE,        // Stack size
        NULL,
        RGB_MANAGER_TASK_PRIORITY,          // priority
        &xrgb_manager_task
    );
    return 0;
}
TaskHandle_t rgb_manager_task_handle()
{
    return xrgb_manager_task;
}
int set_rgb_state(rgb_states_t state)
{
    current_state = state;
    return 0;
}
rgb_states_t get_rgb_state()
{
    return current_state;
}
int oneshot_blink(int cnt, int period, uint8_t r, uint8_t g, uint8_t b)
{
    if (current_state == BLINK_N) {
        return 0;
    }
    cached_state = current_state;
    current_state = BLINK_N;

    oneshot_data.cnt = cnt;
    oneshot_data.period = period;
    oneshot_data.r = r;
    oneshot_data.g = g;
    oneshot_data.b = b;
    return 0;
}
void set_rgb_led(uint8_t r, uint8_t g, uint8_t b)
{
    rgb_led[0].r = r;
    rgb_led[0].g = g;
    rgb_led[0].b = b;
    FastLED.show();
}
void set_rgb_led(CRGB color)
{
    rgb_led[0] = color;
    FastLED.show();
}

static void rgb_manager_task(void *pvParameters)
{
    while (1) {
        switch (current_state) {
            case RGB_1HZ_CYCLE:
                rgb_led_cycle();    
                delay(1000);
                break;
            case RGB_LOW_BATTERY:
                led_low_battery();
                break;
            case BLINK_N:
                _oneshot_blink();
                break;
            default:
                delay(250);
                break;
        } 
    }
}

static void led_low_battery()
{
    static uint8_t brightness = 0x1;
    static bool increasing = true;

    if (brightness == 0x01) {
        increasing = true;
    }
    else if (brightness == 0xFF) {
        increasing = false;
    }

    if (increasing) {
        brightness = (brightness << 1) | 0x01;
    }
    else {
        brightness = brightness >> 1;
    }

    set_rgb_led(brightness, 0, 0);
    delay(25);
}
static void _oneshot_blink()
{
    for (int i=0; i<oneshot_data.cnt; i++) {
        set_rgb_led(oneshot_data.r, oneshot_data.g, oneshot_data.b);
        delay(oneshot_data.period/2);

        rgb_led[0].r = 0;
        rgb_led[0].g = 0;
        rgb_led[0].b = 0;
        FastLED.show();
        set_rgb_led(CRGB::Black);
        delay(oneshot_data.period/2);
    }
    current_state = cached_state;
}
static void rgb_led_cycle()
{
  static const CRGB colors[3] = {CRGB::Red, CRGB::Green, CRGB::Blue};
  static uint8_t i = 0;
  set_rgb_led(colors[i++]);
  if (i >= 3) {
    i=0;
  }
}