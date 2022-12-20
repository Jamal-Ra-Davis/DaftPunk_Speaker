#pragma once
#include <Arduino.h>

typedef enum {LOG_ERR, LOG_WRN, LOG_INF, LOG_DBG, NUM_LOG_LEVELS} log_level_t;
int init_logger();
int set_log_level(log_level_t level);
int log_err(const char* fmt, ...);
int log_wrn(const char* fmt, ...);
int log_inf(const char* fmt, ...);
int log_dbg(const char* fmt, ...);

int log_err_isr(const char* fmt, ...);
int log_wrn_isr(const char* fmt, ...);
int log_inf_isr(const char* fmt, ...);
int log_dbg_isr(const char* fmt, ...);

TaskHandle_t logger_task_handle();
