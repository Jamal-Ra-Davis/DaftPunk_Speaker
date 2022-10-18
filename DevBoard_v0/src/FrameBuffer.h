#pragma once

#define FRAME_BUF_ROWS      8
#define FRAME_BUF_COL_BYTES 5

typedef struct {
    uint8_t frame_buffer[FRAME_BUF_ROWS][FRAME_BUF_COL_BYTES] = {0xFF};    
} frame_buffer_t;

class DoubleBuffer {
    private:
        frame_buffer_t buf0;
        frame_buffer_t buf1;
        frame_buffer_t *rbuf;
        frame_buffer_t *wbuf;
    public:
        DoubleBuffer();
        void reset();
        void clear();
        int setPixel(uint8_t x, uint8_t y);
        int clearPixel(uint8_t x, uint8_t y);
        int setByte(uint8_t x, uint8_t y, uint8_t b);
        void update();
        frame_buffer_t *getReadBuffer() {return rbuf;}
};

extern DoubleBuffer double_buffer;