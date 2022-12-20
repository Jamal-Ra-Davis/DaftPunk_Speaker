#pragma once
#include <Arduino.h>

int init_fft_task();
void read_data_stream(const uint8_t *data, uint32_t length);
TaskHandle_t fft_task_handle();
