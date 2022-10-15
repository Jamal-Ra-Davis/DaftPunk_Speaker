#pragma once

extern uint8_t frame_buffer[8][5];

void matrix_clear();
int matrix_set_pixel(uint8_t x, uint8_t y);
int matrix_clear_pixel(uint8_t x, uint8_t y);
int matrix_set_byte(uint8_t x, uint8_t y, uint8_t b);