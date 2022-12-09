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
#include <Adafruit_MAX1704X.h>

#include "src/global_defines.h"
#include "src/FFT_task.h"
#include "src/Display_task.h"
#include "src/Buttons.h"
#include "src/Events.h"
#include "src/Logging.h"

#define I2C_BUS_SCAN_MAX 16

// Data struct definitions

// Global Variables
BluetoothA2DPSink a2dp_sink;
CRGBArray<1> rgb_led;
Adafruit_MAX17048 maxlipo;

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

static const uint8_t MAX_VOLUME_LEVEL = 8;
static const uint8_t VOLUME_SCALE = 16;
static int8_t volume_level = 2;
static volatile bool pair_press = false;

static void volume_increase(void *ctx)
{
  log_inf("Volume Increase Pressed");
  volume_level++;
  if (volume_level > MAX_VOLUME_LEVEL)
  {
    volume_level = MAX_VOLUME_LEVEL;
  }
  a2dp_sink.set_volume(volume_level * VOLUME_SCALE);
}
static void volume_decrease(void *ctx)
{
  log_inf("Volume Decrease Pressed");
  volume_level--;
  if (volume_level < 0)
  {
    volume_level = 0;
  }
  a2dp_sink.set_volume(volume_level * VOLUME_SCALE);
}
static void select_action(void *ctx)
{
  log_inf("Select button pressed");
}
static void pair_action(void *ctx)
{
  pair_press = true;
  log_inf("Pair Button Pressed");
}

volatile bool start_busy = false;
volatile bool stop_busy = false;
static void chg_stat_isr()
{
  if (digitalRead(CHG_STAT_PIN)) {
    if (!stop_busy)
      push_event(CHARGE_STOP, true);
  }
  else {
    if (!start_busy)
      push_event(CHARGE_START, true);
  }
}
static void charge_start_action(void *ctx)
{
  log_inf("Charging started");
  start_busy = true;
  for (int i=0; i<10; i++) {
    rgb_led[0] = CRGB::Black;
    FastLED.show();
    delay(50);
    rgb_led[0] = CRGB::Green;
    FastLED.show();
    delay(50);
  }
  start_busy = false;
}
static void charge_stop_action(void *ctx)
{
  log_inf("Charging stopped");
  stop_busy = true;
  for (int i=0; i<10; i++) {
    rgb_led[0] = CRGB::Black;
    FastLED.show();
    delay(50);
    rgb_led[0] = CRGB::Red;
    FastLED.show();
    delay(50);
  }
  stop_busy = false;
}

void setup() {
  int ret = 0;
  bool init_success = true;
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(2000);

  pinMode(RGB_LED_EN, OUTPUT);
  pinMode(RGB_LED_DATA, OUTPUT);
  pinMode(AMP_SD_PIN, OUTPUT);

  digitalWrite(AMP_SD_PIN, LOW);
  digitalWrite(RGB_LED_EN, HIGH);

  if (init_display_task() < 0) {
    Serial.println("Error: Failed to start Display task");
    init_success = false;
  }

  if (init_logger() < 0) {
    Serial.println("Error: Failed to init logger");
    init_success = false;
  }

  if (init_event_manager() < 0) {
    Serial.println("Error: Failed to init event manager");
    init_success = false;
  }
  ret |= register_event_callback(VOL_P_SHORT_PRESS, volume_increase, NULL);
  ret |= register_event_callback(VOL_M_SHORT_PRESS, volume_decrease, NULL);
  ret |= register_event_callback(PAIR_SHORT_PRESS, select_action, NULL);
  ret |= register_event_callback(PAIR_LONG_PRESS, pair_action, NULL);
  ret |= register_event_callback(CHARGE_START, charge_start_action, NULL);
  ret |= register_event_callback(CHARGE_STOP, charge_stop_action, NULL);

  if (ret != 0) {
    Serial.println("Error: Failed to register event callbacks");
    init_success = false;
  }
  pinMode(CHG_STAT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CHG_STAT_PIN), chg_stat_isr, CHANGE);


  if (init_buttons() < 0) {
    Serial.println("Error: Failed to init button handlers");
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

  if (!maxlipo.begin()) {
    Serial.println(F("Error: Unable to init MAX17048"));
    init_success = false;
  }

  // Create FreeRTOS Tasks
  xTaskCreate(
    timer_thread_task,
    "Timer_Thread_Task",   // A name just for humans
    4096,        // Stack size
    NULL,
    TIMER_THREAD_TASK_PRIORITY,          // priority
    NULL
  );

  // Init Audio
  a2dp_sink.start("DevBoard_v0");
  a2dp_sink.set_volume(volume_level * VOLUME_SCALE);
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
    //Serial.println("Hello from task 1");
    log_inf("Hello from task 1");
    float voltage = maxlipo.cellVoltage();
    float soc = maxlipo.cellPercent();
    log_inf("Battery Voltage: %0.2f, Battery SOC: %0.2f %%", voltage, soc);
    digitalWrite(RGB_LED_EN, HIGH);
    rgb_led_cycle(NULL);
    delay(1000);
  }
}
