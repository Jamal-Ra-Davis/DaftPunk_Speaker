#include <Arduino.h>

uint8_t frame_buffer[8][5] = {0xFF};
void matrix_clear() {
    for (int i=0; i<8; i++) {
        for (int j=0; j<5; j++) {
            frame_buffer[i][j] = 0xFF;
        }
    }
}
int matrix_set_pixel(uint8_t x, uint8_t y) {
    if (x >= 40) {
        return -1;
    }
    if (y >= 8) {
        return -1;
    }

    int idx = 4 - x/8;
    int bit_idx = x % 8;
    frame_buffer[y][idx] &= ~(1 << bit_idx);
    return 0;
}
int matrix_clear_pixel(uint8_t x, uint8_t y) {
    if (x >= 40) {
        return -1;
    }
    if (y >= 8) {
        return -1;
    }

    int idx = 4 - x/8;
    int bit_idx = x % 8;
    frame_buffer[y][idx] |= (1 << bit_idx);
    return 0;
}
int matrix_set_byte(uint8_t x, uint8_t y, uint8_t b) {
    if (x >= 40) {
        return -1;
    }
    if (y >= 8) {
        return -1;
    }

    int idx = 4 - x/8;
    frame_buffer[y][idx] = b;
    return 0;
}
/*
uint8_t[8][5] matrix_get_frame_buffer()
{
    return frame_buffer;
}
*/