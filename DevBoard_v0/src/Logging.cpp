#include <Arduino.h>
#include "Logging.h"
#include "global_defines.h"

#define LOGGER_TASK_SIZE 2304
#define MAX_LOG_CHUNK_LEN 32
#define MAX_LOG_LEN 256
#define MAX_LOGS 64
#define MAX_TASK_NAME_LEN 24
#define SCRATCH_BUF_SIZE MAX_LOG_LEN//64
#define MUTEX_DELAY 100

#define SHOW_PENDING_LOG_NUM 0

typedef enum {LOG_TYPE_REQ, LOG_TYPE_DATA, LOG_TYPE_DATA_ALT} log_message_type_t;
struct log_request {
    uint32_t ts;
    log_level_t level; 
    const char *fmt;
    va_list args;
};
struct log_data {
    uint32_t ts;
    log_level_t level;
    bool start;
    bool end;
#if (configUSE_TRACE_FACILITY == 1)
    char task_name[MAX_TASK_NAME_LEN];
#endif
    char buf[MAX_LOG_CHUNK_LEN];
};
struct log_data_alt {
    uint32_t ts;
    log_level_t level;
    char *buf;
};
struct log_message
{
    log_message_type_t msg_type;
    union { 
        struct log_request req;
        struct log_data data;
        struct log_data_alt data_alt;
    };
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
static int _log_alt(log_level_t level, bool isr, const char *fmt, va_list args);
static int log_request(log_level_t level, bool isr, const char *fmt, va_list args);
static int handle_log_data(struct log_data *data, char *scratch_buf);

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
log_level_t get_log_level()
{
    return log_level;
}
const char *get_log_level_name(log_level_t level)
{
    return LOG_LEVEL_NAMES[level];
}

int log_err(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_ERR, false, fmt, args);
}
int log_wrn(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_WRN, false, fmt, args);
}
int log_inf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_INF, false, fmt, args);
}
int log_dbg(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_DBG, false, fmt, args);
}
int log_err_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_ERR, true, fmt, args);
}
int log_wrn_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_WRN, true, fmt, args);
}
int log_inf_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_INF, true, fmt, args);
}
int log_dbg_isr(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    va_end(args);
    return _log_alt(LOG_DBG, true, fmt, args);
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
#if SHOW_PENDING_LOG_NUM
    UBaseType_t num_messages = uxQueueMessagesWaiting(log_queue);
    Serial.print("Pending Queue: ");
    Serial.println((int)num_messages);
#endif
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
    msg.msg_type = LOG_TYPE_DATA;
    char *buf = (char *)pvPortMalloc(MAX_LOG_LEN * sizeof(char));
    if (buf == NULL) {
        return -1;
    }
    int ret;
    int bytes_left;
    int bytes_written;
    int offset = 0;

    msg.data.ts = millis();
    msg.data.start = true;
    msg.data.end = false;
    msg.data.level = level;

#if (configUSE_TRACE_FACILITY == 1)
    // Get calling task name
    TaskStatus_t xTaskDetails;
    vTaskGetInfo(NULL, &xTaskDetails, pdFALSE, eReady);
    snprintf(msg.data.task_name, MAX_TASK_NAME_LEN, "%s", xTaskDetails.pcTaskName);
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
            vPortFree(buf);
            return -1;
        }
    }
    else
    {
        if (xSemaphoreTake(log_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
        {
            vPortFree(buf);
            return -1;
        }
    }
    while (offset < bytes_left)
    {
        bytes_written = snprintf(msg.data.buf, MAX_LOG_CHUNK_LEN, "%s", &buf[offset]);
        if (bytes_written >= MAX_LOG_CHUNK_LEN)
        {
            bytes_written = MAX_LOG_CHUNK_LEN - 1;
        }
        else
        {
            msg.data.end = true;
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

        msg.data.start = false;
    }
    vPortFree(buf);
    xSemaphoreGive(log_mutex);
    return 0;
}
int _log_alt(log_level_t level, bool isr, const char *fmt, va_list args)
{
    if (log_queue == NULL)
    {
        return -1;
    }
    if (log_mutex == NULL)
    {
        return -1;
    }
#if SHOW_PENDING_LOG_NUM
    UBaseType_t num_messages = uxQueueMessagesWaiting(log_queue);
    Serial.print("Pending Queue: ");
    Serial.println((int)num_messages);
#endif
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
    msg.msg_type = LOG_TYPE_DATA_ALT;
    msg.data_alt.buf = (char *)pvPortMalloc(MAX_LOG_LEN * sizeof(char));
    if (msg.data_alt.buf == NULL) {
        return -1;
    }
    msg.data_alt.ts = millis();
    msg.data_alt.level = level;

    int ret;
    vsnprintf(msg.data_alt.buf, MAX_LOG_LEN, fmt, args);

    // Push to message queue
    if (isr)
    {
        if (xSemaphoreTake(log_mutex, 0) != pdTRUE)
        {
            vPortFree(msg.data_alt.buf);
            return -1;
        }
        ret = xQueueSendFromISR(log_queue, (void *)&msg, NULL);
    }
    else
    {
        if (xSemaphoreTake(log_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
        {
            vPortFree(msg.data_alt.buf);
            return -1;
        }
        ret = xQueueSend(log_queue, (void *)&msg, (TickType_t)0);
    }
    
    if (ret == errQUEUE_FULL)
    {
        dropped_messages++;
    }
    xSemaphoreGive(log_mutex);
    return 0;
}
static int log_request(log_level_t level, bool isr, const char *fmt, va_list args)
{
    if (log_queue == NULL)
    {
        return -1;
    }
    if (log_mutex == NULL)
    {
        return -1;
    }

    int ret;
    uint32_t ts = millis();
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
    msg.msg_type = LOG_TYPE_REQ;
    msg.req.ts = ts;
    msg.req.level = level; 
    msg.req.fmt = fmt;
    msg.req.args = args;

    // Push to message queue
    if (isr)
    {
        ret = xQueueSendFromISR(log_queue, (void *)&msg, NULL);
    }
    else
    {
        ret = xQueueSend(log_queue, (void *)&msg, (TickType_t)0);
    }
    return (ret == pdTRUE) ? 0 : -1;
}
static int handle_log_data(struct log_data *data, char *scratch_buf)
{
    if (data == NULL) {
        return -1;
    }
    if (data->start)
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
        s = data->ts / 1000;
        ms = data->ts % 1000;

        snprintf(scratch_buf, SCRATCH_BUF_SIZE, "[%06d:%03d] <%s>", s, ms, LOG_LEVEL_NAMES[data->level]);
        Serial.write(scratch_buf);

#if (configUSE_TRACE_FACILITY == 1)
        snprintf(scratch_buf, SCRATCH_BUF_SIZE, " %s: ", msg.task_name);
        Serial.write(scratch_buf);
#else
        Serial.write(": ");
#endif

        Serial.write(data->buf);
    }
    else
    {
        Serial.write(data->buf);
    }

    if (data->end)
    {
        Serial.write("\n");
    }
    return 0;
}
static int handle_log_data_alt(struct log_data_alt *data, char *scratch_buf)
{
    if (data == NULL) {
        return -1;
    }
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
    s = data->ts / 1000;
    ms = data->ts % 1000;

    snprintf(scratch_buf, SCRATCH_BUF_SIZE, "[%06d:%03d] <%s>", s, ms, LOG_LEVEL_NAMES[data->level]);
    Serial.write(scratch_buf);
    Serial.write(": ");
    Serial.write(data->buf);
    Serial.write("\n");

    vPortFree(data->buf);
    data->buf = NULL;
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
        switch (msg.msg_type) {
            case LOG_TYPE_REQ: {
                _log(msg.req.level, false, msg.req.fmt, msg.req.args);
                break;
            }
            case LOG_TYPE_DATA: {
                handle_log_data(&msg.data, scratch_buf);
                break;
            }
            case LOG_TYPE_DATA_ALT: {
                handle_log_data_alt(&msg.data_alt, scratch_buf);
                break;
            }
            default: {
                Serial.println("Error: Invalid log message type");
                break;
            }
        }
    }
}