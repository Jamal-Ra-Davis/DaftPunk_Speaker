#include "Font.h"
#include "FrameBuffer.h"
#include <string.h>

#define FONT_HEIGHT 5
#define FONT_WIDTH  5
#define NUM_OFFSET 0
#define CHAR_OFFSET 10
static uint8_t font_data[] = {
    // 0
    0x1E,
    0x12,
    0x12,
    0x12,
    0x1E,

    // 1
    0x08,
    0x0C,
    0x08,
    0x08,
    0x1C,

    // 2
    0x1E,
    0x10,
    0x1E,
    0x02,
    0x1E,

    // 3
    0x1E,
    0x10,
    0x1E,
    0x10,
    0x1E,

    // 4
    0x12,
    0x12,
    0x1E,
    0x10,
    0x10,

    // 5
    0x1E,
    0x02,
    0x1E,
    0x10,
    0x1E,

    // 6
    0x1E,
    0x02,
    0x1E,
    0x12,
    0x1E,

    // 7
    0x1E,
    0x10,
    0x10,
    0x10,
    0x10,

    // 8
    0x1E,
    0x12,
    0x1E,
    0x12,
    0x1E,

    // 9
    0x1E,   
    0x12,
    0x1E,
    0x10,
    0x1E,
};

static inline int char_to_idx(char c)
{
    if (c >= '0' || c <= '9') {
        return (int)((c - '0')*FONT_HEIGHT);
    }
    /*
    if (c >= 'a' || c <= 'z') {
        c -= ('a' - 'A');
    }
    if (c >= 'A' || c <= 'Z') {
        return (int)((c - 'A' + CHAR_OFFSET)*FONT_HEIGHT);
    }
    */
    return -1;
}
void draw_char(char c, uint8_t x, uint8_t y, DoubleBuffer *frame_buffer)
{
    int font_idx = char_to_idx(c);
    if (font_idx < 0 || font_idx >= sizeof(font_data)) {
        return;
    }

    for (int i=0; i<FONT_HEIGHT; i++) {
         for (int j=0; j<FONT_WIDTH; j++) {
             if (font_data[font_idx + i] & (1 << j)) {
                frame_buffer->setPixel(x + j, i + y);
             }
         }
    }
}
void draw_str(const char *s, uint8_t x, uint8_t y, DoubleBuffer *frame_buffer)
{
    if (s == NULL) {
        return;
    }
    for (int i=0; i<strlen(s); i++) {
        int char_x = x + (i * FONT_WIDTH);
        draw_char(s[i], char_x, y, frame_buffer);
    }
}
void draw_int(int val, uint8_t x, uint8_t y, DoubleBuffer *frame_buffer)
{
    if (val == 0) {
        draw_char((char)('0'), x, y, frame_buffer);
        return;
    }
    int x_loc = x;
    while (val != 0) {
        int i = val % 10;
        val /= 10;
        draw_char((char)(i+'0'), x_loc, y, frame_buffer);
        x_loc -= FONT_WIDTH;
    }
}