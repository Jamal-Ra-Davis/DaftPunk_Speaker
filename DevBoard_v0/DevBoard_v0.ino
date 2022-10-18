#include <FastLED.h>
#include <Wire.h>
#include "src/I2C_Helper.h"
#include "src/sr_driver.h"
#include "src/FrameBuffer.h"
#include "src/timer_thread.h"

#include <BluetoothA2DPCommon.h>
#include <BluetoothA2DPSink.h>
#include <BluetoothA2DPSource.h>
#include <config.h>
#include <SoundData.h>
#include <VolumeControl.h>

#include "FFT.h"

#define RGB_LED_EN 17
#define RGB_LED_DATA 27
#define AMP_SD_PIN 4

#define FFT_N 2048
#define TOTAL_TIME 0.0464399

// Data struct definitions
struct fft_double_buffer {
  float buf0[FFT_N];
  float buf1[FFT_N];
  float *fft_read;
  float *fft_write;
  uint32_t widx;
};

BluetoothA2DPSink a2dp_sink;
CRGBArray<1> rgb_led;
volatile bool data_ready = false;
float fft_output[FFT_N];
float fft_input[FFT_N];
struct fft_double_buffer fft_buf;
fft_config_t *real_fft_plan;
int ranges[16] = {20, 32, 51, 82, 131, 210, 336, 536, 859, 1374, 2199, 3518, 5629, 9007, 14411, 23058};

// Function prototypes
void draw_display(void *ctx);
void snake_animation(void *ctx);
void rgb_led_cycle(void *ctx);
void init_fft_buffer(struct fft_double_buffer *fft_buf);
static inline void swap_fft_buffers(struct fft_double_buffer *fft_buf);
void read_data_stream(const uint8_t *data, uint32_t length);
void process_fft();
void draw_equalizer(float *freq_data, int N);
void update_freq_array(float *freq_data, int N, float freq, float mag);

void fft_task(void *pvParameters);
void display_task(void *pvParameters);
void timer_thread_task(void *pvParameters);

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
  double_buffer.reset();

  for (int i=0; i<8; i++) {
    double_buffer.setPixel(i, i);
  }
  double_buffer.update();

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


  // Init Audio
  pinMode(AMP_SD_PIN, OUTPUT);
  digitalWrite(AMP_SD_PIN, LOW);
  real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
  init_fft_buffer(&fft_buf);
  a2dp_sink.start("DevBoard_v0");
  a2dp_sink.set_stream_reader(read_data_stream);


  //register_timer_thread(snake_animation, NULL, 33000);
  //register_timer_thread(rgb_led_cycle, NULL, 1000000);
  //register_timer_thread(draw_display, NULL, 1400);

  // Create FreeRTOS Tasks
  
  xTaskCreate(
    fft_task,
    "FFT_Task",   // A name just for humans
    10000,        // Stack size
    NULL,
    4,          // priority
    NULL
  );

  xTaskCreate(
    display_task,
    "Display_Task",   // A name just for humans
    4096,        // Stack size
    NULL,
    5,          // priority
    NULL
  );
  

  xTaskCreate(
    timer_thread_task,
    "Timer_Thread_Task",   // A name just for humans
    1024,        // Stack size
    NULL,
    1,          // priority
    NULL
  );
  //vTaskStartScheduler();
}

void loop() {
  /*
  process_fft();
  update_timer_threads();
  */
 delay(1000);
}

void draw_display(void *ctx)
{
  static int idx = 0;
  test_data[0] = (1 << idx);
  frame_buffer_t *rbuf = double_buffer.getReadBuffer();
  memcpy(&test_data[1], rbuf->frame_buffer[idx], FRAME_BUF_COL_BYTES);
  sr_write(test_data, 6);
  idx = (idx + 1) % 8;
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


void init_fft_buffer(struct fft_double_buffer *fft_buf) {
  fft_buf->widx = 0;
  fft_buf->fft_read = fft_buf->buf0;
  fft_buf->fft_write = fft_buf->buf1;
  fft_buf->widx = 0;
  for (int i=0; i<FFT_N; i++) {
    fft_buf->buf0[0] = 0;
    fft_buf->buf0[1] = 0; 
  }
}
static inline void swap_fft_buffers(struct fft_double_buffer *fft_buf)
{
  float *temp = fft_buf->fft_read;
  fft_buf->fft_read = fft_buf->fft_write;
  fft_buf->fft_write = temp;
}
void read_data_stream(const uint8_t *data, uint32_t length)
{
  //Odd samples are left channel, even samples are right channel
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
  
  for (int i=0; i<sample_count; i+=2) {
    fft_buf.fft_write[fft_buf.widx++] = (float)samples[i];
    if (fft_buf.widx >= FFT_N) {
      swap_fft_buffers(&fft_buf);
      data_ready = true;
      fft_buf.widx = 0;
    }
  } 
}
void process_fft()
{
  if (data_ready) {
    data_ready = false;
    int32_t start_time = millis();
    for (int i=0; i<FFT_N; i++) {
      fft_input[i] = fft_buf.fft_read[i];
    }

    float max_magnitude = 0;
    float fundamental_freq = 0;

    float freq_data[10] = {0.0};
    float bucket_data[32] = {0.0};
    float bucket_mag = 0;
    float bucket_freq = 0;
    int fft_idx = 0;
    
    int bucket_idx = 0;
    fft_execute(real_fft_plan);
    for (int k = 1 ; k < real_fft_plan->size / 2 ; k++)
    {
      /*The real part of a magnitude at a frequency is 
        followed by the corresponding imaginary part in the output*/
      float mag = sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2));
      
      float freq = k*1.0/TOTAL_TIME;
      if(mag > max_magnitude)
      {
        max_magnitude = mag;
        fundamental_freq = freq;
        update_freq_array(freq_data, 10, freq, mag);
      }

      if (freq > ranges[bucket_idx]) {
        bucket_data[2*bucket_idx] = freq;
        bucket_data[2*bucket_idx + 1] = mag;
        bucket_idx++;
        bucket_mag = 0;
        bucket_freq = 0;
      }
    }
    int32_t end_time = millis();
    Serial.print("Size: ");
    Serial.println(real_fft_plan->size);

    draw_equalizer(freq_data, 10);
  }
}
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
void update_freq_array(float *freq_data, int N, float freq, float mag)
{
  int idx = -1;
  for (int i=0; i<N; i+=2) {
    if (mag > freq_data[i+1]) {
      //Shift everything down
      idx = i;
      break;
    }
  }
  if (idx == -1) {
    return;
  }

  for (int i=N-2; i > idx; i-=2) {
    freq_data[i] = freq_data[i-2];
    freq_data[i+1] = freq_data[i-1];
  }
  freq_data[0] = freq;
  freq_data[1] = mag;
}
void fft_task(void *pvParameters)
{
  while (1) {
    process_fft();
    delay(1);
  }
}
void display_task(void *pvParameters)
{
  while (1) {
    draw_display(NULL);
    delay(1);
  }
}
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