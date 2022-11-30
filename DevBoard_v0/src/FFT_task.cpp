#include "FFT_task.h"
#include "../FFT.h"
#include "global_defines.h"
#include "FrameBuffer.h"
#include "Logging.h"

#define FFT_TASK_STACK_SIZE 10000
#define FFT_N 2048
#define TOTAL_TIME 0.0464399
#define MAX_FFT_MAG 2000000.0
#define FFT_BUCKETS 40

struct fft_double_buffer
{
    float buf0[FFT_N];
    float buf1[FFT_N];
    float *fft_read;
    float *fft_write;
    uint32_t widx;
};

// File Globals
static SemaphoreHandle_t xDataReadySem;
static uint16_t ranges[FFT_BUCKETS] = {
    100, 301, 506, 716, 932, 1158, 1393, 1639, 1898, 2172,
    2460, 2765, 3088, 3429, 3790, 4173, 4577, 5005, 5456, 5933,
    6436, 6967, 7526, 8113, 8731, 9380, 10061, 10775, 11523, 12305,
    13124, 13978, 14870, 15801, 16770, 17780, 18830, 19922, 21057, 22235};
static float fft_output[FFT_N];
static float fft_input[FFT_N];
static struct fft_double_buffer fft_buf;
static fft_config_t *real_fft_plan;

// Function Prototypes
static void fft_task(void *pvParameters);
static void process_fft();
static void init_fft_buffer(struct fft_double_buffer *fft_buf);
static inline void swap_fft_buffers(struct fft_double_buffer *fft_buf);
static void update_freq_array(float *freq_data, int N, float freq, float mag);

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
    xTaskCreate(
        fft_task,
        "FFT_Task", // A name just for humans
        FFT_TASK_STACK_SIZE,
        NULL,
        FFT_TASK_PRIORITY, // priority
        NULL);
    return 0;
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
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
            }
        }
    }
}

// Priavte Functions
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
    /*
    bool pair_pressed = pair_press;
    if (pair_press)
    {
        pair_press = false;
    }
    */
    for (int k = 1; k < real_fft_plan->size / 2; k++)
    {
        /*
        if (pair_pressed)
        {
            Serial.print(real_fft_plan->output[2 * k]);
            Serial.print(", ");
            Serial.println(real_fft_plan->output[2 * k + 1]);
        }
        */

        /*The real part of a magnitude at a frequency is
          followed by the corresponding imaginary part in the output*/
        float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2));
        float freq = k * 1.0 / TOTAL_TIME;

        int bidx;
        for (bidx = 0; bidx < FFT_BUCKETS; bidx++)
        {
            if (freq < ranges[bidx])
            {
                break;
            }
        }
        if (mag > bucket_mags[bidx])
        {
            bucket_mags[bidx] = mag;
        }

        if (mag > max_magnitude)
        {
            max_magnitude = mag;
            fundamental_freq = 0;
            update_freq_array(freq_data, 10, freq, mag);
        }

        if (freq > ranges[bucket_idx])
        {
            bucket_data[2 * bucket_idx] = freq;
            bucket_data[2 * bucket_idx + 1] = mag;
            bucket_idx++;
            bucket_mag = 0;
            bucket_freq = 0;
        }
    }
    int32_t end_time = millis();

    if (cnt % 10 == 0 && false)
    {
        Serial.println("Mags:");
        for (int i = 0; i < FFT_BUCKETS; i++)
        {
            Serial.print(bucket_mags[i]);
            Serial.print(" ");
        }
        Serial.println();
    }

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
    cnt++;
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