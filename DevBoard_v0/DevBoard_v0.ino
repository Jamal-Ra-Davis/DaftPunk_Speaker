#include <FastLED.h>
#include <Wire.h>
#include "src/I2C_Helper.h"
#include "src/sr_driver.h"
#include "src/FrameBuffer.h"
#include "src/timer_thread.h"

#define RGB_LED_EN 17
#define RGB_LED_DATA 27

CRGBArray<1> rgb_led;

// Function prototypes
void draw_display(void *ctx);
void snake_animation(void *ctx);
void rgb_led_cycle(void *ctx);


struct pos {
  int x;
  int y;
};
#define BODY_LEN 10
struct pos body[BODY_LEN];
volatile uint8_t sr_data[2] = {0xAA, 0xF0};
uint8_t test_data[6] = {0x00};

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 19);
  delay(2000);
  
  pinMode(RGB_LED_EN, OUTPUT);
  pinMode(RGB_LED_DATA, OUTPUT);
  digitalWrite(RGB_LED_EN, HIGH);

  init_shift_registers();

  FastLED.addLeds<NEOPIXEL,RGB_LED_DATA>(rgb_led, 1);
  rgb_led[0] = CRGB::Blue;
  FastLED.show();

  uint8_t i2c_devices[16];
  uint8_t i2c_device_cnt = 0;
  if (i2c_bus_scan(i2c_devices, &i2c_device_cnt, 16) < 0) {
    Serial.println("I2C Bus Scan failed");
  }
  else {
    Serial.println("I2C devices: ");
    for (int i=0; i< i2c_device_cnt; i++) {
      Serial.println(i2c_devices[i], HEX);
    }
  }

  int x = rand() % 8;
  int y = rand() % 8;
  for (int i=0; i<BODY_LEN; i++) {
    body[i].x = x;
    body[i].y = y;
  }

  register_timer_thread(snake_animation, NULL, 33000);
  register_timer_thread(rgb_led_cycle, NULL, 1000000);
  register_timer_thread(draw_display, NULL, 1400);
}


void loop() {
  update_timer_threads();
}

void draw_display(void *ctx)
{
  static int idx = 0;
  test_data[0] = (1 << idx);
  memcpy(&test_data[1], frame_buffer[idx], 5);
  sr_write(test_data, 6);
  idx = (idx + 1) % 8;
}
void snake_animation(void *ctx)
{
  static int prev_dir = 0;
  matrix_clear();
  int dir;
  while (1) {
    dir = rand() % 4;
    if (prev_dir == 0 && (dir != 1)) {
      break;
    }
    if (prev_dir == 1 && (dir != 0)) {
      break;
    }
    if (prev_dir == 2 && (dir != 3)) {
      break;
    }
    if (prev_dir == 3 && (dir != 2)) {
      break;
    }
  }
  int x = body[0].x;
  int y = body[0].y;
  prev_dir = dir;
  switch (dir) {
    case 0:
      x++;
      if (x >= 40) {
        x = 0;
      }
      break;
    case 1:
      x--;
      if (x < 0) {
        x = 39;
      }
      break;
    case 2:
      y++;
      if (y >= 8) {
        y = 0;
      }
      break;
    case 3:
      y--;
      if (y < 0) {
        y = 7;
      }
      break;
  }
  
  for (int i=BODY_LEN-1; i>=1; i--) {
    body[i].x = body[i-1].x;
    body[i].y = body[i-1].y;

    matrix_set_pixel(body[i].x, body[i].y);
  }
  body[0].x = x;
  body[0].y = y;
  matrix_set_pixel(body[0].x, body[0].y);
}
void rgb_led_cycle(void *ctx)
{
  CRGB temp = rgb_led[0];
  rgb_led[0] = CRGB::Black;
  if (temp.r == 0xFF) {
    rgb_led[0].g = 0xFF;
  }
  else if (temp.g == 0xFF) {
    rgb_led[0].b = 0xFF;
  }
  else if (temp.b == 0xFF) {
    rgb_led[0].r = 0xFF;
  }
  FastLED.show();
}