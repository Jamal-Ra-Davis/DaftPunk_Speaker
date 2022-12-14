#pragma once
#include <Arduino.h>
#include "FrameBuffer.h"

void draw_char(char c, uint8_t x, uint8_t y, DoubleBuffer *frame_buffer);
void draw_str(const char *s, uint8_t x, uint8_t y, DoubleBuffer *frame_buffer);
void draw_int(int val, uint8_t x, uint8_t y, DoubleBuffer *frame_buffer);