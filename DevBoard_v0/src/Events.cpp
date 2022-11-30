#include <Arduino.h>
#include "Events.h"
#include "global_defines.h"
#include "Logging.h"

#define MAX_EVENTS 10
#define EVENT_MANAGER_TASK_STACK_SIZE 4096
#define MUTEX_DELAY 100

struct event_callback_data
{
    event_callback_t cb;
    void *ctx;
};

// File Globals
static struct event_callback_data event_callbacks[NUM_EVENTS];
static QueueHandle_t event_queue = NULL;
static SemaphoreHandle_t event_mutex = NULL;

// Function Prototypes
static void event_manager_task(void *pvParameters);

// Public Functions
int init_event_manager()
{
    for (int i = 0; i < NUM_EVENTS; i++)
    {
        event_callbacks[i].cb = NULL;
        event_callbacks[i].ctx = NULL;
    }

    event_queue = xQueueCreate(MAX_EVENTS, sizeof(system_event_t));
    if (event_queue == NULL)
    {
        log_err("Failed to create event queue");
        return -1;
    }

    event_mutex = xSemaphoreCreateMutex();
    if (event_mutex == NULL)
    {
        log_err("Failed to create event mutex");
        return -1;
    }

    xTaskCreate(
        event_manager_task,
        "Event_Manager_Task",
        EVENT_MANAGER_TASK_STACK_SIZE,
        NULL,
        EVENT_MANAGER_TASK_PRIORITY,
        NULL);
    return 0;
}
int register_event_callback(system_event_t event, event_callback_t cb, void *ctx)
{
    if (event >= NUM_EVENTS)
    {
        return -1;
    }
    if (event_mutex == NULL)
    {
        return -1;
    }
    if (xSemaphoreTake(event_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
    {
        return -1;
    }

    event_callbacks[event].cb = cb;
    event_callbacks[event].ctx = ctx;
    xSemaphoreGive(event_mutex);
    return 0;
}
int unregister_event_callback(system_event_t event, event_callback_t cb)
{
    if (event >= NUM_EVENTS)
    {
        return -1;
    }
    if (event_mutex == NULL)
    {
        return -1;
    }
    if (xSemaphoreTake(event_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
    {
        return -1;
    }

    event_callbacks[event].cb = NULL;
    event_callbacks[event].ctx = NULL;
    xSemaphoreGive(event_mutex);
    return 0;
}
bool event_callback_registered(system_event_t event)
{
    if (event >= NUM_EVENTS)
    {
        return false;
    }
    if (event_mutex == NULL)
    {
        return false;
    }
    if (xSemaphoreTake(event_mutex, MS_TO_TICKS(MUTEX_DELAY)) != pdTRUE)
    {
        return false;
    }
    bool result = (event_callbacks[event].cb != NULL);
    xSemaphoreGive(event_mutex);
    return result;
}
int push_event(system_event_t event, bool isr)
{
    if (event_queue == NULL)
    {
        return -1;
    }
    BaseType_t ret;
    if (isr)
    {
        ret = xQueueSendFromISR(event_queue, (void *)&event, NULL);
    }
    else
    {
        ret = xQueueSend(event_queue, (void *)&event, (TickType_t)0);
    }

    return (ret == pdTRUE) ? 0 : -1;
}

// Private Functions
static void event_manager_task(void *pvParameters)
{
    if (event_queue == NULL)
    {
        log_err("Could not get handle to system event queue");
        vTaskDelete(NULL);
    }
    if (event_mutex == NULL)
    {
        log_err("Could not get handle to system event mutex");
        vTaskDelete(NULL);
    }

    system_event_t event;
    while (1)
    {
        xQueueReceive(event_queue, &event, portMAX_DELAY);
        if (event >= NUM_EVENTS)
        {
            log_err("Invalid event");
            continue;
        }

        if (xSemaphoreTake(event_mutex, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }
        if (event_callbacks[event].cb != NULL)
        {
            event_callbacks[event].cb(event_callbacks[event].ctx);
        }
        xSemaphoreGive(event_mutex);
    }
}
