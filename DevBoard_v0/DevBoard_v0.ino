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
#include "src/cli_task.h"
#include "src/Display_task.h"
#include "src/Buttons.h"
#include "src/Events.h"
#include "src/Logging.h"
#include "src/Font.h"
#include "src/Volume.h"
#include "src/rgb_manager.h"

#define I2C_BUS_SCAN_MAX 16
#define TIMER_TASK_STACK_SIZE 2560

// Data struct definitions

// Global Variables
BluetoothA2DPSink a2dp_sink;
Adafruit_MAX17048 maxlipo;
bool display_stack_wm = true;
bool idle = true;

// Function prototypes
void snake_animation(void *ctx);
void stack_display(void *ctx);

void timer_thread_task(void *pvParameters);

struct pos {
  int x;
  int y;
};
#define BODY_LEN 10
struct pos body[BODY_LEN];

static const uint8_t MAX_VOLUME_LEVEL = 8;
static const uint8_t VOLUME_SCALE = 16;
int8_t volume_level = 2;
static volatile bool pair_press = false;

static void select_action(void *ctx)
{
  log_inf("Select button pressed - Create a2dp sink");
  log_inf("Creating a2dp sink...");
  a2dp_sink.start("DevBoard_v0");
  volume_init();
  a2dp_sink.set_stream_reader(read_data_stream);
}
static void pair_action(void *ctx)
{
  pair_press = true;
  log_inf("Pair Button Pressed - Destroy a2dp sink");
  log_inf("Destroying a2dp sink...");
  a2dp_sink.end(false);
}

volatile int start_cnt = 0;
volatile int stop_cnt = 0;
volatile bool start_busy = false;
volatile bool stop_busy = false;
static void chg_stat_isr()
{
  if (digitalRead(CHG_STAT_PIN)) {
    if (!stop_busy)
      push_event(CHARGE_STOP, true);
      stop_cnt++;
  }
  else {
    if (!start_busy)
      push_event(CHARGE_START, true);
      start_cnt++;
  }
}
static void charge_start_action(void *ctx)
{
  log_inf("Charging started");
  oneshot_blink(10, 100, 0, 128, 32);
}
static void charge_stop_action(void *ctx)
{
  log_inf("Charging stopped");
  oneshot_blink(10, 100, 128, 16, 16);
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

  pinMode(CHG_STAT_PIN, INPUT_PULLUP);

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
  
  ret |= register_event_callback(VOL_P_SHORT_PRESS, volume_increase_cb, NULL);
  ret |= register_event_callback(VOL_M_SHORT_PRESS, volume_decrease_cb, NULL);
  ret |= register_event_callback(PAIR_SHORT_PRESS, select_action, NULL);
  ret |= register_event_callback(PAIR_LONG_PRESS, pair_action, NULL);
  ret |= register_event_callback(CHARGE_START, charge_start_action, NULL);
  ret |= register_event_callback(CHARGE_STOP, charge_stop_action, NULL);

  if (ret != 0) {
    Serial.println("Error: Failed to register event callbacks");
    init_success = false;
  }

  if (init_buttons() < 0) {
    Serial.println("Error: Failed to init button handlers");
    init_success = false;
  }

  if (init_rgb_manager() < 0) {
    Serial.println("Error: Failed to init RGB manager");
    init_success = false;
  }

  for (int i=10; i >= 0; i--) {
    double_buffer.clear();
    draw_int(i, 30, 2, &double_buffer);
    double_buffer.update();
    delay(250);
  }

  int test_str_len = get_str_width("DEVBOARD_V0");
  for (int i=FRAME_BUF_COLS; i >= -test_str_len; i--) {
    double_buffer.clear();
    draw_str("DEVBOARD_V0", i, 2, &double_buffer);
    double_buffer.update();
    delay(20);
  }
  const char *test_str = "HI! \"BYE\", '(NO)' 3*3+5=11. printf(); ^/\\+-%#";
  test_str_len = get_str_width(test_str);
  for (int i=FRAME_BUF_COLS; i >= -test_str_len; i--) {
    double_buffer.clear();
    draw_str(test_str, i, 2, &double_buffer);
    double_buffer.update();
    delay(30);
  }

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
  
  if (init_cli_task() < 0) {
    Serial.println("Failed to init CLI");
    init_success = false;
  }

  init_timer_thread_manager();
  if (register_timer_thread(snake_animation, NULL, MS_TO_US(50)) < 0) {
    log_err("Failed to register snake_animation timer thread");
  }
  else {
    log_inf("Successfully registered snake_animation timer thread");
  }
  if (register_timer_thread(stack_display, NULL, MS_TO_US(1000)) < 0) {
    log_err("Failed to register stack_display timer thread");
  }
  else {
    log_inf("Successfully registered stack_display timer thread");
  }

  // Create FreeRTOS Tasks
  xTaskCreate(
    timer_thread_task,
    "Timer_Thread_Task",   // A name just for humans
    TIMER_TASK_STACK_SIZE,        // Stack size
    NULL,
    TIMER_THREAD_TASK_PRIORITY,          // priority
    NULL
  );

  // Init Audio
  a2dp_sink.start("DevBoard_v0");
  //volume_init();
  a2dp_sink.set_stream_reader(read_data_stream);

  attachInterrupt(digitalPinToInterrupt(CHG_STAT_PIN), chg_stat_isr, CHANGE);
  volatile size_t xFreeStackSpace = xPortGetFreeHeapSize();
  log_inf("Free Heap Size = %d", xFreeStackSpace);
  log_inf("BluetoothA2DPSink size = %d", sizeof(BluetoothA2DPSink));
}

void loop() {
 delay(1000);
}

void snake_animation(void *ctx)
{
  static int prev_dir = 0;
  
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
  }
  body[0].x = x;
  body[0].y = y;

  if (idle) {
    double_buffer.clear();
    for (int i=BODY_LEN-1; i>=0; i--) {
      double_buffer.setPixel(body[i].x, body[i].y);
    }
    double_buffer.update();
  }
}
void stack_display(void *ctx)
{
  if (display_stack_wm) {
      TaskHandle_t fft_task = fft_task_handle();
      TaskHandle_t display_task = display_task_handle();
      TaskHandle_t event_task = event_task_handle();
      TaskHandle_t logger_task = logger_task_handle();
      TaskHandle_t cli_task = cli_task_handle();
      TaskHandle_t rgb_task = rgb_manager_task_handle();
      UBaseType_t fft_task_wm = uxTaskGetStackHighWaterMark(fft_task);
      UBaseType_t display_task_wm = uxTaskGetStackHighWaterMark(display_task);
      UBaseType_t event_task_wm = uxTaskGetStackHighWaterMark(event_task);
      UBaseType_t logger_task_wm = uxTaskGetStackHighWaterMark(logger_task);
      UBaseType_t cli_task_wm = uxTaskGetStackHighWaterMark(cli_task);
      UBaseType_t stack_task_wm = uxTaskGetStackHighWaterMark(NULL);
      UBaseType_t rgb_task_wm = uxTaskGetStackHighWaterMark(rgb_task);
      volatile size_t xFreeStackSpace = xPortGetFreeHeapSize();

      log_inf("fft_task watermark: %d", (int)fft_task_wm);
      log_inf("display_task watermark: %d", (int)display_task_wm);
      log_inf("event_task watermark: %d", (int)event_task_wm);
      log_inf("logger_task watermark: %d", (int)logger_task_wm);
      log_inf("cli_task watermark: %d", (int)cli_task_wm);
      log_inf("stack_task watermark: %d", (int)stack_task_wm);
      log_inf("rgb_task watermark: %d", (int)rgb_task_wm);
      log_inf("Free Heap Size = %d\n", xFreeStackSpace);
    }
}
void timer_thread_task(void *pvParameters)
{
  while (1) {
    update_timer_threads();
    delay(50);
  }
  
  static int cnt = 0;
  //update_timer_threads();
  while (1) {
    //log_inf("Hello from task 1");
    
    /*
    //float voltage = maxlipo.cellVoltage();
    float soc = maxlipo.cellPercent();
    uint16_t ic_version = maxlipo.getICversion();
    log_inf("IC Version = 0x%04X", ic_version);
    */
    /*
    log_inf("Battery Voltage: %0.2f, Battery SOC: %0.2f %%", voltage, soc);
    log_inf("Audio Connected: %d", a2dp_sink.is_connected());
    */

    /*
    double_buffer.clear();
    //draw_int(start_cnt, 10, 2, &double_buffer);
    //draw_int(stop_cnt, 30, 2, &double_buffer);
    //draw_int(cnt++, 30, 2, &double_buffer);
    draw_int((int)soc, 30, 2, &double_buffer);
    double_buffer.update();
    */


    cnt++;
    delay(1000);
  }
}
