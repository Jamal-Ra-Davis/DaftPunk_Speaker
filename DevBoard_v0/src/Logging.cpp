#include <Arduino.h>
#include "Logging.h"
#include "global_defines.h"

#define LOGGER_TASK_SIZE 3072
#define MAX_LOG_CHUNK_LEN 32
#define MAX_LOG_LEN 256
#define MAX_LOGS 64
#define MAX_TASK_NAME_LEN 24
#define SCRATCH_BUF_SIZE 64
#define MUTEX_DELAY 100

struct log_message
{
    bool start;
    bool end;
    uint32_t ts;
    log_level_t level;
#if (configUSE_TRACE_FACILITY == 1)
    char task_name[MAX_TASK_NAME_LEN];
#endif
    char buf[MAX_LOG_CHUNK_LEN];
};

static const char *LOG_LEVEL_NAMES[NUM_LOG_LEVELS] = {
    "LOG_ERR",
    "LOG_WRN",
    "LOG_INF",
    "LOG_DBG",
};
static SemaphoreHandle_t log_mutex = NULL;
static QueueHandle_t log_queue = NULL;
static log_level_t log_level = LOG_INF;
static int dropped_messages = 0;
static TaskHandle_t xlogger_task = NULL;

static void logger_task(void *pvParameters);
static int _log(log_level_t level, bool isr, const char *fmt, va_list args);

// Public functions
int init_logger()
{
    log_queue = xQueueCreate(MAX_LOGS, sizeof(struct log_message));
    if (log_queue == NULL)
    {
        Serial.println("Error: Failed to create logger queue");
        return -1;
    }

    log_mutex = xSemaphoreCreateMutex();
    if (log_mutex == NULL)
    {
        Serial.println("Error: Failed to create logger mutex");
        return -1;
    }

    xTaskCreate(
        logger_task,
        "Logger_Task",
        LOGGER_TASK_SIZE,
        NULL,
        LOGGER_TASK_PRIORITY,
        &xlogger_task);
    return 0;
}

int set_log_level(log_level_t level)
{
    if (log_mutex == NULL)
    {
        return -1;
    }
    if (xSemaphoreTake(log_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
    {
        return -1;
    }
    log_level = level;
    xSemaphoreGive(log_mutex);
    return 0;
}

int log_err(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_ERR, false, fmt, args);
}
int log_wrn(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_WRN, false, fmt, args);
}
int log_inf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_INF, false, fmt, args);
}
int log_dbg(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_DBG, false, fmt, args);
}
int log_err_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_ERR, true, fmt, args);
}
int log_wrn_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_WRN, true, fmt, args);
}
int log_inf_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_INF, true, fmt, args);
}
int log_dbg_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log(LOG_DBG, true, fmt, args);
}
TaskHandle_t logger_task_handle()
{
    return xlogger_task;
}

// Private Functions
int _log(log_level_t level, bool isr, const char *fmt, va_list args)
{
    if (log_queue == NULL)
    {
        return -1;
    }
    if (log_mutex == NULL)
    {
        return -1;
    }

    if (isr)
    {

        if (xSemaphoreTake(log_mutex, 0) != pdTRUE)
        {
            return -1;
        }
    }
    else
    {
        if (xSemaphoreTake(log_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
        {
            return -1;
        }
    }
    if (level > log_level)
    {
        xSemaphoreGive(log_mutex);
        return 0;
    }
    xSemaphoreGive(log_mutex);

    struct log_message msg;
    char buf[MAX_LOG_LEN];
    int ret;
    int bytes_left;
    int bytes_written;
    int offset = 0;

    msg.ts = millis();
    msg.start = true;
    msg.end = false;
    msg.level = level;

#if (configUSE_TRACE_FACILITY == 1)
    // Get calling task name
    TaskStatus_t xTaskDetails;
    vTaskGetInfo(NULL, &xTaskDetails, pdFALSE, eReady);
    snprintf(msg.task_name, MAX_TASK_NAME_LEN, "%s", xTaskDetails.pcTaskName);
#endif

    bytes_left = vsnprintf(buf, MAX_LOG_LEN, fmt, args);
    if (bytes_left >= MAX_LOG_LEN)
    {
        bytes_left = MAX_LOG_LEN - 1;
    }

    if (isr)
    {

        if (xSemaphoreTake(log_mutex, 0) != pdTRUE)
        {
            return -1;
        }
    }
    else
    {
        if (xSemaphoreTake(log_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
        {
            return -1;
        }
    }
    while (offset < bytes_left)
    {
        bytes_written = snprintf(msg.buf, MAX_LOG_CHUNK_LEN, "%s", &buf[offset]);
        if (bytes_written >= MAX_LOG_CHUNK_LEN)
        {
            bytes_written = MAX_LOG_CHUNK_LEN - 1;
        }
        else
        {
            msg.end = true;
        }
        offset += bytes_written;

        // Push to message queue
        if (isr)
        {
            ret = xQueueSendFromISR(log_queue, (void *)&msg, NULL);
        }
        else
        {
            ret = xQueueSend(log_queue, (void *)&msg, (TickType_t)0);
        }

        if (ret == errQUEUE_FULL)
        {
            dropped_messages++;
            break;
        }

        msg.start = false;
    }
    xSemaphoreGive(log_mutex);
    return 0;
}

static void logger_task(void *pvParameters)
{
    if (log_queue == NULL)
    {
        Serial.println("Error: Could not get handle to logging queue");
        vTaskDelete(NULL);
    }
    if (log_mutex == NULL)
    {
        Serial.println("Error: Could not get handle to logging mutex");
        vTaskDelete(NULL);
    }

    struct log_message msg;
    char scratch_buf[SCRATCH_BUF_SIZE];
    while (1)
    {
        xQueueReceive(log_queue, &msg, portMAX_DELAY);

        if (msg.start)
        {
            if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
            {
                if (dropped_messages > 0)
                {
                    int cnt = dropped_messages;
                    dropped_messages = 0;
                    xSemaphoreGive(log_mutex);
                    snprintf(scratch_buf, SCRATCH_BUF_SIZE, "---- %d Messages Dropped ----\n", cnt);
                    Serial.write(scratch_buf);
                }
                else 
                {
                    xSemaphoreGive(log_mutex);
                }
            }

            uint32_t s, ms;
            s = msg.ts / 1000;
            ms = msg.ts % 1000;

            snprintf(scratch_buf, SCRATCH_BUF_SIZE, "[%06d:%03d] <%s>", s, ms, LOG_LEVEL_NAMES[msg.level]);
            Serial.write(scratch_buf);

#if (configUSE_TRACE_FACILITY == 1)
            snprintf(scratch_buf, SCRATCH_BUF_SIZE, " %s: ", msg.task_name);
            Serial.write(scratch_buf);
#else
            Serial.write(": ");
#endif

            Serial.write(msg.buf);
        }
        else
        {
            Serial.write(msg.buf);
        }

        if (msg.end)
        {
            Serial.write("\n");
        }
    }
}