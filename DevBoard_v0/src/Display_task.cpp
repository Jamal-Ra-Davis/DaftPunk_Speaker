#include "Display_task.h"
#include "global_defines.h"
#include "FrameBuffer.h"
#include "sr_driver.h"

#define DISPLAY_TASK_STACK_SIZE 1024

// File Globals
static uint8_t sr_buffer[SR_CNT] = {0x00};
static uint8_t row_idx = 0;
static TaskHandle_t xdisplay_task = NULL;

// Function Prototypes
static void display_task(void *pvParameters);

// Public Functions
/**
 * @brief Initializes display task. Responsible for updating shift registers 
 * with data read from framebuffer
 * @return 0 on success, -1 on failure
 */ 
int init_display_task()
{
    init_shift_registers();
    double_buffer.reset();

    xTaskCreate(
        display_task,
        "Display_Task",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &xdisplay_task);
    return 0;
}
TaskHandle_t display_task_handle()
{
    return xdisplay_task;
}
// Private Functions
static void display_task(void *pvParameters)
{
    /* 
        sr_buffer[0] corresponds to ROW select shift register, 
        while sr_buffer[1:5] corresponds to column data
    */
    while (1)
    {
        sr_buffer[0] = (1 << row_idx);
        frame_buffer_t *rbuf = double_buffer.getReadBuffer();
        memcpy(&sr_buffer[1], rbuf->frame_buffer[row_idx], FRAME_BUF_COL_BYTES);
        sr_write(sr_buffer, SR_CNT);
        row_idx = (row_idx + 1) % FRAME_BUF_ROWS;

        delay(1);
    }
}