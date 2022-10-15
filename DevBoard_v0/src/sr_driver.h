#pragma once

#include <Arduino.h>

void init_shift_registers();
void sr_write(uint8_t *data, int N);
inline void sr_write_byte(uint8_t val);