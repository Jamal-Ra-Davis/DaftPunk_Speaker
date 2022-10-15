#include <FastLED.h>
#include <Wire.h>
#include "src/I2C_Helper.h"
#include "src/sr_driver.h"
#include "src/FrameBuffer.h"

/*
#define SR_DIN 23
#define SR_CLK 18
#define SR_LAT 5
#define SR_CNT 6
*/
#define RGB_LED_EN 17
#define RGB_LED_DATA 27

CRGBArray<1> rgb_led;

/*
// I2C functions
int i2c_write_read(uint8_t dev_addr, uint8_t *wbuf, uint8_t wlen, uint8_t *rbuf, uint8_t rlen)
{
    int ret;
    ret = i2c_write(dev_addr, wbuf, wlen);
    if (ret < 0) {
        return ret;
    }
    Wire.requestFrom(dev_addr, rlen);
    for (int i=0; i<rlen; i++) {
        if (Wire.available()) {
            rbuf[i] = Wire.read();
        }
        else {
            return -1;
        }
    }
    return 0;
}
int i2c_write(uint8_t dev_addr, uint8_t *wbuf, uint8_t wlen)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(wbuf, wlen);
    if (Wire.endTransmission() != 0) {
        return -1;
    }
    return 0;
}
int i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_val)
{
    return i2c_write_read(dev_addr, &reg_addr, 1, reg_val, 1);
}
int i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_val)
{
    uint8_t wbuf[2] = {
        reg_addr,
        reg_val
    };
    return i2c_write(dev_addr, wbuf, sizeof(wbuf));
}
int i2c_update_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_val, uint8_t mask)
{
    uint8_t old_val;
    int ret = i2c_read_byte(dev_addr, reg_addr, &old_val);
    if (ret < 0) {
        return ret;
    }

    uint8_t new_val = (old_val & ~mask) | (reg_val & mask);
    return i2c_write_byte(dev_addr, reg_addr, new_val);
}
int i2c_bus_scan(uint8_t *addr_list, uint8_t *addr_cnt, uint8_t max_addr_cnt)
{
    int cnt = 0;
    if (addr_list == NULL) {
        return -1;
    }
    if (max_addr_cnt < 1) {
        return -1;
    }
    for (uint8_t i=0; i<128; i++) {
        if (cnt > max_addr_cnt) {
            return -2;
        }
        Wire.beginTransmission(i);
        if (Wire.endTransmission() != 0) {
            continue;
        }
        addr_list[cnt++] = i;
    }
    *addr_cnt = cnt;
    return 0;
}
*/

/*
inline void sr_write_byte(uint8_t val)
{ 
  char buf[64];
  for (int i=7; i>=0; i--) {
    digitalWrite(SR_CLK, LOW);
    bool bit_ = (bool)((1 << i) & val);
    if (bit_)
      digitalWrite(SR_DIN, HIGH);
    else
      digitalWrite(SR_DIN, LOW);
    
    digitalWrite(SR_CLK, HIGH);
  }
}
void sr_write(uint8_t *data, int N)
{
  digitalWrite(SR_LAT, LOW);
  
  for (int i=0; i<N; i++) {
    sr_write_byte(data[i]);
  }
  digitalWrite(SR_LAT, HIGH);
}
void init_shift_registers()
{
  pinMode(SR_DIN, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_LAT, OUTPUT);

  digitalWrite(SR_DIN, LOW);
  digitalWrite(SR_CLK, LOW);
  digitalWrite(SR_LAT, LOW);

  uint8_t clear_data[SR_CNT] = {
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
  sr_write(clear_data, SR_CNT);
}
*/

volatile uint8_t grid[40] = {0x00};
uint8_t animation[] = {
  0xFF,
  0x81,
  0x81,
  0x81,
  0x81,
  0x81,
  0x81,
  0xFF,

  0x00,
  0x7E,
  0x42,
  0x42,
  0x42,
  0x42,
  0x7E,
  0x00,

  0x00,
  0x00,
  0x3C,
  0x24,
  0x24,
  0x3C,
  0x00,
  0x00,

  0x00,
  0x00,
  0x00,
  0x18,
  0x18,
  0x00,
  0x00,
  0x00,

  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
};

struct pos {
  int x;
  int y;
};
#define BODY_LEN 10
struct pos body[BODY_LEN];
volatile uint8_t sr_data[2] = {0xAA, 0xF0};
uint8_t test_data[6] = {0x00};
typedef enum {TIMER_DISPLAY, TIMER_SNAKE, TIMER_RGBW, NUM_TIMERS} timer_type_t;
uint32_t timers[NUM_TIMERS][2];

typedef void (*timer_thread_func_t)(void *ctx);
struct timer_thread{
  bool active;
  uint32_t period;
  uint32_t ts;
  timer_thread_func_t func;
  void *ctx;
};

#define NUM_THREADS 16
struct timer_thread threads[NUM_THREADS];

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 19);
  delay(2000);
  
  // put your setup code here, to run once:
  /*
  pinMode(SR_DIN, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_LAT, OUTPUT);
  */
  pinMode(RGB_LED_EN, OUTPUT);
  pinMode(RGB_LED_DATA, OUTPUT);

  /*
  digitalWrite(SR_DIN, LOW);
  digitalWrite(SR_CLK, LOW);
  digitalWrite(SR_LAT, LOW);
  */
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
  
  test_data[0] = 0x1;
  test_data[1] = ~0xFF;
  test_data[2] = ~0xAA;
  test_data[3] = ~0xF0;
  test_data[4] = ~0xCC;
  test_data[5] = ~0x01;
  sr_write(test_data, 6);

  //Test pattern
  matrix_clear();
  matrix_set_pixel(1, 1);
  matrix_set_pixel(1, 2);
  matrix_set_pixel(1, 3);

  matrix_set_pixel(2, 4);

  matrix_set_pixel(3, 1);
  matrix_set_pixel(3, 2);
  matrix_set_pixel(3, 3);
  matrix_set_pixel(3, 4);
  matrix_set_pixel(3, 5);

  matrix_set_pixel(3, 7);


  matrix_set_pixel(7, 2);
  matrix_set_pixel(8, 2);
  matrix_set_pixel(9, 3);
  matrix_set_pixel(10, 3);
  matrix_set_pixel(11, 2);
  matrix_set_pixel(12, 2);

  matrix_set_pixel(39, 2);

  int x = rand() % 8;
  int y = rand() % 8;
  for (int i=0; i<BODY_LEN; i++) {
    body[i].x = x;
    body[i].y = y;
  }

  uint32_t ts = micros();
  timers[TIMER_DISPLAY][0] = ts;
  timers[TIMER_SNAKE][0] = ts;
  timers[TIMER_RGBW][0] = ts;

  timers[TIMER_DISPLAY][1] = 1400;
  timers[TIMER_SNAKE][1] = 33000;
  timers[TIMER_RGBW][1] = 1000000;
}

/*
volatile uint8_t idx = 0;
ISR(TIMER2_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
  sr_data[0] = (1 << idx);
  for (int i=0; i<5; i++) {
    sr_data[i+1] = ~grid[idx];  
  }
  
  sr_write(sr_data, 2);
  idx++;
  if (idx >= 8) {
    idx = 0;
  }
}
*/

void draw_display()
{
  static int idx = 0;
  test_data[0] = (1 << idx);
  memcpy(&test_data[1], frame_buffer[idx], 5);
  sr_write(test_data, 6);
  idx = (idx + 1) % 8;
}
bool check_timer(timer_type_t t, uint32_t ts) {
  if (ts - timers[t][0] >= timers[t][1]) {
    timers[t][0] = ts;
    return true;
  }
  return false;
}

int animation_idx = 0;
int prev_dir = 0;
int idx = 0;
int cnt = 0;


void loop() {
  uint32_t ts = micros();
  // Snake
  if (check_timer(TIMER_SNAKE, ts)) {
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

  // RGB LED
  if (check_timer(TIMER_RGBW, ts)) {
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

  // Draw line
  if (check_timer(TIMER_DISPLAY, ts)) {
    draw_display();
  }

  cnt++;
}
