#include "FFT_task.h"
#include "../FFT.h"
#include "global_defines.h"
#include "FrameBuffer.h"
#include "Logging.h"

#define FFT_TASK_STACK_SIZE 3072
#define FFT_N 2048
#define TOTAL_TIME 0.0464399
#define MAX_FFT_MAG 2000000.0
#define FFT_BUCKETS 40
#define PRINT_DELTA false

// TODO: Investigate why audio stream stops sometimes. Check if it's due to FFT processing/display

struct fft_double_buffer
{
    float buf0[FFT_N];
    float buf1[FFT_N];
    float *fft_read;
    float *fft_write;
    uint32_t widx;
};
typedef enum {FFT_LINEAR, FFT_LOG} fft_display_type_t;

// File Globals
static SemaphoreHandle_t xDataReadySem;
static const uint16_t ranges[FFT_BUCKETS] = {
        50, 100, 141, 185, 233, 285, 343, 406, 476, 553,
        637, 729, 829, 937, 1053, 1179, 1314, 1459,
        1614, 1778, 1954, 2140, 2336, 2544, 2764, 2995,
        3238, 3493, 3760, 4040, 4333, 4638, 4957, 5289,
        5635, 5994, 6367, 6754, 7156, 7572, //8002,
};
static const uint16_t MAX_FREQ = 8447;
static float fft_output[FFT_N];
static float fft_input[FFT_N];
static struct fft_double_buffer fft_buf;
static fft_config_t *real_fft_plan;
static TaskHandle_t xfft_task = NULL;
extern bool idle;
TimerHandle_t idle_timer;
static fft_display_type_t fft_display = FFT_LOG;

// Function Prototypes
static void fft_task(void *pvParameters);
static void process_fft();
static void init_fft_buffer(struct fft_double_buffer *fft_buf);
static inline void swap_fft_buffers(struct fft_double_buffer *fft_buf);
static void update_freq_array(float *freq_data, int N, float freq, float mag);
static void idle_timer_func(TimerHandle_t xTimer);

// Public Functions
int init_fft_task()
{
    real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
    init_fft_buffer(&fft_buf);
    xDataReadySem = xSemaphoreCreateBinary();
    if (xDataReadySem == NULL)
    {
        log_err("Could not allocate data ready semaphore");
        return -1;
    }
    idle_timer = xTimerCreate("Idle_Timer", MS_TO_TICKS(10000), pdFALSE, NULL, idle_timer_func);
    xTaskCreate(
        fft_task,
        "FFT_Task", // A name just for humans
        FFT_TASK_STACK_SIZE,
        NULL,
        FFT_TASK_PRIORITY, // priority
        &xfft_task);
    return 0;
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return;
    }
    // Odd samples are left channel, even samples are right channel
    int16_t *samples = (int16_t *)data;
    uint32_t sample_count = length / 2;

    for (int i = 0; i < sample_count; i += 2)
    {
        fft_buf.fft_write[fft_buf.widx++] = (float)samples[i];
        if (fft_buf.widx >= FFT_N)
        {
            swap_fft_buffers(&fft_buf);
            fft_buf.widx = 0;
            if (xSemaphoreGive(xDataReadySem) != pdTRUE)
            {
                // Failed to give semaphore
                log_err_isr("Failed to give semaphore");
            }
        }
    }
}

TaskHandle_t fft_task_handle()
{
    return xfft_task;
}

// Private Functions
static void init_fft_buffer(struct fft_double_buffer *fft_buf)
{
    fft_buf->widx = 0;
    fft_buf->fft_read = fft_buf->buf0;
    fft_buf->fft_write = fft_buf->buf1;
    fft_buf->widx = 0;
    for (int i = 0; i < FFT_N; i++)
    {
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

static inline void draw_fft_linear(float bucket_mags[])
{
    double_buffer.clear();
    for (int i = 0; i < FFT_BUCKETS; i++)
    {
        if (bucket_mags[i] > MAX_FFT_MAG)
        {
            bucket_mags[i] = MAX_FFT_MAG;
        }
        int height = (int)((bucket_mags[i] / MAX_FFT_MAG) * 8);
        for (int j = 0; j < height; j++)
        {
            double_buffer.setPixel(i, j);
        }
    }
    double_buffer.update();
}
static inline void draw_fft_logarithmic(float bucket_mags[])
{
    static const uint32_t LOG_MIN = 20000;//15265;
    double_buffer.clear();
    for (int i = 0; i < FFT_BUCKETS; i++)
    {
        uint32_t mag = (uint32_t)bucket_mags[i];        
        uint8_t height = 0;
        while (mag > LOG_MIN) {
            mag = mag >> 1;
            height++;
            if (height >= 8) {
                break;
            }
        }

        for (uint8_t j = 0; j < height; j++)
        {
            double_buffer.setPixel(i, j);
        }
    }
    double_buffer.update();
}

static void fft_task(void *pvParameters)
{
    while (1)
    {
        process_fft();
    }
}

void process_fft()
{
    if (xSemaphoreTake(xDataReadySem, portMAX_DELAY) == pdFALSE)
    {
        log_err("Failed to take semaphore");
        return;
    }
    static int cnt = 0;
    int32_t start_time = millis();
    memcpy((void *)fft_input, (void *)fft_buf.fft_read, sizeof(fft_input));

    float max_magnitude = 0;
    float fundamental_freq = 0;

    float freq_data[10] = {0.0};
    float bucket_data[32] = {0.0};
    float bucket_mag = 0;
    float bucket_freq = 0;
    float bucket_mags[FFT_BUCKETS] = {0.0};
    int fft_idx = 0;

    int bucket_idx = 0;
    fft_execute(real_fft_plan);
    for (int k = 1; k < real_fft_plan->size / 2; k++)
    {
        /*The real part of a magnitude at a frequency is
          followed by the corresponding imaginary part in the output*/
        float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2));
        float freq = k * 1.0 / TOTAL_TIME;
        if (freq > MAX_FREQ)
        {
            // Exit loop if current freq exceeds values in FFT bucket range
            break;
        }

        int bidx;
        for (bidx = 0; bidx < FFT_BUCKETS; bidx++)
        {
            if (freq < ranges[bidx])
            {
                break;
            }
        }
        if (bidx >= FFT_BUCKETS) 
        {
            bidx = FFT_BUCKETS - 1;
        }

        if (mag > bucket_mags[bidx])
        {
            bucket_mags[bidx] = mag;
        }
    }
    int32_t end_time = millis();

    if (true && cnt % 10 == 0)
    {
        Serial.println("Mags:");
        for (int i = 0; i < FFT_BUCKETS; i++)
        {
            Serial.print(bucket_mags[i]);
            Serial.print(" ");
        }
        Serial.println();
    }

    idle = false;
    switch (fft_display) {
        case FFT_LINEAR:
            draw_fft_linear(bucket_mags);
            break;
        case FFT_LOG:
            draw_fft_logarithmic(bucket_mags);
            break;
        default:
            break;
    }
    
    cnt++;
    int32_t delta = millis() - start_time;
    if (PRINT_DELTA) {
        log_inf("Delta: %d", delta);
    }
    if (xTimerStart(idle_timer, 0) != pdPASS) {
        log_err("Failed to start idle timer");
    }
}

void update_freq_array(float *freq_data, int N, float freq, float mag)
{
    int idx = -1;
    for (int i = 0; i < N; i += 2)
    {
        if (mag > freq_data[i + 1])
        {
            // Shift everything down
            idx = i;
            break;
        }
    }
    if (idx == -1)
    {
        return;
    }

    for (int i = N - 2; i > idx; i -= 2)
    {
        freq_data[i] = freq_data[i - 2];
        freq_data[i + 1] = freq_data[i - 1];
    }
    freq_data[0] = freq;
    freq_data[1] = mag;
}

static void idle_timer_func(TimerHandle_t xTimer)
{
    idle = true;
}