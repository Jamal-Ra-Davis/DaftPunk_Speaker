#include <Arduino.h>
#include "FrameBuffer.h"

#define BITS_PER_BYTE 8

DoubleBuffer double_buffer;

DoubleBuffer::DoubleBuffer()
{
    reset();
}
void DoubleBuffer::reset()
{
    rbuf = &buf0;
    wbuf = &buf1;
    for (int i=0; i<FRAME_BUF_ROWS; i++) {
        for (int j=0; j<FRAME_BUF_COL_BYTES; j++) {
            buf0.frame_buffer[i][j] = 0xFF;
            buf1.frame_buffer[i][j] = 0xFF;
        }
    }
}
void DoubleBuffer::clear()
{
    for (int i=0; i<FRAME_BUF_ROWS; i++) {
        for (int j=0; j<FRAME_BUF_COL_BYTES; j++) {
            wbuf->frame_buffer[i][j] = 0xFF;
        }
    }
}
void DoubleBuffer::copy()
{
    memcpy(wbuf, rbuf, sizeof(frame_buffer_t));
}
int DoubleBuffer::setPixel(uint8_t x, uint8_t y) {
    if (x >= FRAME_BUF_COL_BYTES*BITS_PER_BYTE) {
        return -1;
    }
    if (y >= FRAME_BUF_ROWS) {
        return -1;
    }

    int idx = 4 - x/8;
    int bit_idx = x % 8;
    wbuf->frame_buffer[y][idx] &= ~(1 << bit_idx);
    return 0;
}
int DoubleBuffer::clearPixel(uint8_t x, uint8_t y) {
    if (x >= FRAME_BUF_COL_BYTES*BITS_PER_BYTE) {
        return -1;
    }
    if (y >= FRAME_BUF_ROWS) {
        return -1;
    }

    int idx = 4 - x/8;
    int bit_idx = x % 8;
    wbuf->frame_buffer[y][idx] |= (1 << bit_idx);
    return 0;
}
int DoubleBuffer::setByte(uint8_t x, uint8_t y, uint8_t b) {
    if (x >= FRAME_BUF_COL_BYTES*BITS_PER_BYTE) {
        return -1;
    }
    if (y >= FRAME_BUF_ROWS) {
        return -1;
    }

    int idx = 4 - x/8;
    wbuf->frame_buffer[y][idx] = b;
    return 0;
}
void DoubleBuffer::update()
{
    frame_buffer_t *temp = rbuf;
    rbuf = wbuf;
    wbuf = temp;
}