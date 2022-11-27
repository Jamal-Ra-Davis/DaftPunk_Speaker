#include <FastLED.h>
#include <Wire.h>
#include "src/I2C_Helper.h"
#include "src/sr_driver.h"
#include "src/FrameBuffer.h"
#include "src/timer_thread.h"

#include <BluetoothA2DPCommon.h>
#include <BluetoothA2DPSink.h>
#include <BluetoothA2DPSource.h>
#include <SoundData.h>
#include <A2DPVolumeControl.h>

#include "src/global_defines.h"
#include "src/FFT_task.h"
#include "src/Display_task.h"
#include "src/Buttons.h"

#define I2C_BUS_SCAN_MAX 16

// Data struct definitions

// Global Variables
BluetoothA2DPSink a2dp_sink;
CRGBArray<1> rgb_led;

// Function prototypes
void snake_animation(void *ctx);
void rgb_led_cycle(void *ctx);

//void draw_equalizer(float *freq_data, int N);


void fft_task(void *pvParameters);
void display_task(void *pvParameters);
void timer_thread_task(void *pvParameters);

struct pos {
  int x;
  int y;
};
#define BODY_LEN 10
struct pos body[BODY_LEN];

void setup() {
  bool init_success = true;
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(2000);

  pinMode(RGB_LED_EN, OUTPUT);
  pinMode(RGB_LED_DATA, OUTPUT);
  pinMode(AMP_SD_PIN, OUTPUT);
  

  digitalWrite(AMP_SD_PIN, LOW);
  digitalWrite(RGB_LED_EN, HIGH);

  init_buttons();

  if (init_display_task() < 0) {
    Serial.println("Failed it start Display task");
    init_success = false;
  }

  for (int i=0; i<8; i++) {
    double_buffer.setPixel(i, i);
  }
  double_buffer.update();

  FastLED.addLeds<NEOPIXEL,RGB_LED_DATA>(rgb_led, 1);
  rgb_led[0] = CRGB::Blue;
  FastLED.show();

  uint8_t i2c_devices[I2C_BUS_SCAN_MAX];
  uint8_t i2c_device_cnt = 0;
  if (i2c_bus_scan(i2c_devices, &i2c_device_cnt, I2C_BUS_SCAN_MAX) < 0) {
    Serial.println("I2C Bus Scan failed");
    init_success = false;
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

  if (init_fft_task() < 0) {
    Serial.println("Failed it start FFT task");
    init_success = false;
  }

  // Create FreeRTOS Tasks
  xTaskCreate(
    timer_thread_task,
    "Timer_Thread_Task",   // A name just for humans
    1024,        // Stack size
    NULL,
    TIMER_THREAD_TASK_PRIORITY,          // priority
    NULL
  );

  // Init Audio
  a2dp_sink.start("DevBoard_v0");
  a2dp_sink.set_stream_reader(read_data_stream);
}

void loop() {
 delay(1000);
}

void snake_animation(void *ctx)
{
  static int prev_dir = 0;
  double_buffer.clear();
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
      if (x >= FRAME_BUF_COLS) {
        x = 0;
      }
      break;
    case 1:
      x--;
      if (x < 0) {
        x = FRAME_BUF_COLS-1;
      }
      break;
    case 2:
      y++;
      if (y >= FRAME_BUF_ROWS) {
        y = 0;
      }
      break;
    case 3:
      y--;
      if (y < 0) {
        y = FRAME_BUF_ROWS-1;
      }
      break;
  }
  
  for (int i=BODY_LEN-1; i>=1; i--) {
    body[i].x = body[i-1].x;
    body[i].y = body[i-1].y;
    double_buffer.setPixel(body[i].x, body[i].y);
  }
  body[0].x = x;
  body[0].y = y;
  double_buffer.setPixel(body[0].x, body[0].y);

  double_buffer.update();
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

/*
// NOTE: Don't delete with other stuff you moved to FFT_task.cpp
void draw_equalizer(float *freq_data, int N) {
  int buckets[16] = {0};
  int max_height = 8;
  
  for (int i=0; i<N; i+=2) {
    int j;
    bool valid = false;
    for (j=0; j<16; j++) {
      if (freq_data[i] < ranges[j]) {
        valid = true;
        break;
      }
    }
    if (valid) {
      float normal = ((freq_data[i+1]/10000)*2/FFT_N) / 0.2f;
      buckets[j] = (int)(max_height * normal);
    }
  }

  double_buffer.clear();
  for (int i=0; i<16; i++) {
    for (int j=0; j<=buckets[i]; j++) {
      double_buffer.setPixel(i, j);
    }
  }
  double_buffer.update();
  
}
*/
void timer_thread_task(void *pvParameters)
{
  //update_timer_threads();
  while (1) {
    Serial.println("Hello from task 1");
    digitalWrite(RGB_LED_EN, HIGH);
    rgb_led_cycle(NULL);
    delay(1000);
  }
}
