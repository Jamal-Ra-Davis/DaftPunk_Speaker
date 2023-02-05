#include "Font.h"
#include "FrameBuffer.h"
#include "Logging.h"
#include <string.h>

#define FONT_HEIGHT 5
#define FONT_WIDTH_DEFAULT  5
#define NUM_OFFSET 0
#define CHAR_OFFSET 10
#define SPECIAL0_OFFSET 36 // ! to /
#define SPECIAL1_OFFSET 51 // : to >
#define SPECIAL2_OFFSET 56 // [ to _
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


    // ------------ Letters-----------
    // A
    0x0C,
    0x12,
    0x1E,
    0x12,
    0x12,

    // B
    0x0E,
    0x12,
    0x0E,
    0x12,
    0x0E,

    // C
    0x0C,
    0x12,
    0x02,
    0x12,
    0x0C,

    // D
    0x0E,
    0x12,
    0x12,
    0x12,
    0x0E,

    // E
    0x1E,
    0x02,
    0x1E,
    0x02,
    0x1E,

    // F
    0x1E,
    0x02,
    0x1E,
    0x02,
    0x02,

    // G
    0x0C,
    0x02,
    0x1A,
    0x12,
    0x0C,

    // H
    0x12,
    0x12,
    0x1E,
    0x12,
    0x12,

    // I
    0x3E,
    0x08,
    0x08,
    0x08,
    0x3E,

    // J
    0x10,
    0x10,
    0x10,
    0x12,
    0x0C,

    // K
    0x12,
    0x0A,
    0x06,
    0x0A,
    0x12,

    // L
    0x02,
    0x02,
    0x02,
    0x02,
    0x1E,

    // M
    0x22,
    0x36,
    0x2A,
    0x22,
    0x22,

    // N
    0x22,
    0x26,
    0x2A,
    0x32,
    0x22,

    // O
    0x0C,
    0x12,
    0x12,
    0x12,
    0x0C,

    // P
    0x0E,
    0x12,
    0x0E,
    0x02,
    0x02,

    // Q
    0x1C,
    0x22,
    0x22,
    0x12,
    0x2C,

    // R
    0x0E,
    0x12,
    0x0E,
    0x12,
    0x12,

    // S
    0x1C,
    0x02,
    0x0C,
    0x10,
    0x0E,

    // T
    0x3E,
    0x08,
    0x08,
    0x08,
    0x08,
    
    // U
    0x12,
    0x12,
    0x12,
    0x12,
    0x0C,

    // V
    0x22,
    0x22,
    0x22,
    0x14,
    0x08,

    // W
    0x22,
    0x22,
    0x2A,
    0x36,
    0x22,

    // X
    0x12,
    0x12,
    0x0C,
    0x12,
    0x12,

    // Y
    0x12,
    0x12,
    0x1E,
    0x10,
    0x1E,

    // Z
    0x1E,
    0x10,
    0x0C,
    0x02,
    0x1E,

    // ------------ Special Chars (! to /) -----------
    // !
    0x02,
    0x02,
    0x02,
    0x00,
    0x02,

    // "
    0x0A,
    0x0A,
    0x00,
    0x00,
    0x00,

    // #
    0x14,
    0x3E,
    0x14,
    0x3E,
    0x14,

    // $
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,

    // %
    0x00,
    0x12,
    0x08,
    0x04,
    0x12,

    // &
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,

    // '
    0x02,
    0x02,
    0x00,
    0x00,
    0x00,

    // (
    0x04,
    0x02,
    0x02,
    0x02,
    0x04,

    // )
    0x02,
    0x04,
    0x04,
    0x04,
    0x02,

    // *
    0x00,
    0x0A,
    0x04,
    0x0A,
    0x00,

    // +
    0x00,
    0x04,
    0x0E,
    0x04,
    0x00,

    // ,
    0x00,
    0x00,
    0x00,
    0x04,
    0x02,

    // -
    0x00,
    0x00,
    0x0E,
    0x00,
    0x00,

    // .
    0x00,
    0x00,
    0x00,
    0x00,
    0x02,

    // /
    0x00,
    0x10,
    0x08,
    0x04,
    0x02,

    // ------------ Special Chars (: to >) -----------
    // :
    0x00,
    0x02,
    0x00,
    0x02,
    0x00,

    // ;
    0x00,
    0x02,
    0x00,
    0x02,
    0x02,

    // <
    0x08,
    0x04,
    0x02,
    0x04,
    0x08,

    // = 
    0x00,
    0x0E,
    0x00,
    0x0E,
    0x00,

    // >
    0x02,
    0x04,
    0x08,
    0x04,
    0x02,

    // ------------ Special Chars ([ to _) -----------
    // [
    0x06,
    0x02,
    0x02,
    0x02,
    0x06,

    // /* \ */
    0x00,
    0x02,
    0x04,
    0x08,
    0x10,

    // ]
    0x06,
    0x04,
    0x04,
    0x04,
    0x06,

    // ^
    0x04,
    0x0A,
    0x00,
    0x00,
    0x00,

    // _
    0x00,
    0x00,
    0x00,
    0x00,
    0x0E,
};

static inline int char_to_idx(char c)
{
    if (c >= '0' && c <= '9') {
        return (int)((c - '0')*FONT_HEIGHT);
    }
    if (c >= '!' && c <= '/') {
        return (int)((c - '!' + SPECIAL0_OFFSET)*FONT_HEIGHT);
    }
    if (c >= ':' && c <= '>') {
        return (int)((c - ':' + SPECIAL1_OFFSET)*FONT_HEIGHT);
    }
    if (c >= '[' && c <= '_') {
        log_err("Special 1: %c", c);
        return (int)((c - '[' + SPECIAL2_OFFSET)*FONT_HEIGHT);
    }

    if (c >= 'a' && c <= 'z') {
        c -= ('a' - 'A');
    }
    if (c >= 'A' && c <= 'Z') {
        return (int)((c - 'A' + CHAR_OFFSET)*FONT_HEIGHT);
    }
    return -1;
}
inline int get_font_width(char c)
{
    static uint8_t FONT_WIDTH_LARGE = 6;
    bool valid_char = false;
    if (c >= '0' && c <= '9') {
        valid_char = true;
    }
    if (c >= '!' && c <= '/') {
        valid_char = true;
    }
    if (c >= ':' && c <= '>') {
        valid_char = true;
    }
    if (c >= '[' && c <= '_') {
        valid_char = true;
    }

    if (c >= 'a' && c <= 'z') {
        c -= ('a' - 'A');
    }
    if (c >= 'A' && c <= 'Z') {
        valid_char = true;
    }
    if (!valid_char) {
        return -1;
    }

    switch (c) {
        case '!':
        case '\'':
        case '.':
        case ':':
        case ';':
            return 2;
        case '(':
        case ')':
        case ',':
        case '[':
        case ']':
            return 3;
        case '\"':
        case '*':
        case '+':
        case '-':
        case '<':
        case '=':
        case '>':
        case '^':
        case '_':
            return 4;
        case 'I':
        case 'M':
        case 'N':
        case 'Q':
        case 'T':
        case 'V':
        case 'W':
        case '#':
            return FONT_WIDTH_LARGE;
        default:
            return FONT_WIDTH_DEFAULT;
    }
    return -1;
}
int draw_char(char c, int x, int y, DoubleBuffer *frame_buffer)
{
    int font_idx = char_to_idx(c);
    int font_width = get_font_width(c);
    if (font_idx < 0 || font_idx >= sizeof(font_data) || font_width < 0) {
        return -1;
    }
    if ((x + font_width) < 0 || x >= FRAME_BUF_COLS) {
        return -1;
    }
    if ((y + FONT_HEIGHT) < 0 || y >= FRAME_BUF_ROWS) {
        return -1;
    }

    for (int i=0; i<FONT_HEIGHT; i++) {
         for (int j=0; j<font_width; j++) {
             if (font_data[font_idx + i] & (1 << j)) {
                frame_buffer->setPixel(x + j, i + y);
             }
         }
    }
    return 0;
}
void draw_str(const char *s, int x, int y, DoubleBuffer *frame_buffer)
{
    if (s == NULL) {
        return;
    }
    int char_x = x;
    for (int i=0; i<strlen(s); i++) {
        int width = get_font_width(s[i]);
        if (width < 0) {
            width = FONT_WIDTH_DEFAULT;
        }
        draw_char(s[i], char_x, y, frame_buffer);
        char_x += width;
    }
}
void draw_int(int val, int x, int y, DoubleBuffer *frame_buffer)
{
    if (val == 0) {
        draw_char((char)('0'), x, y, frame_buffer);
        return;
    }
    int x_loc = x;
    while (val != 0) {
        int i = val % 10;
        val /= 10;
        char c = (char)(i+'0');
        draw_char(c, x_loc, y, frame_buffer);
        x_loc -= get_font_width(c);
    }
}
int get_str_width(const char *str)
{
    if (str == NULL) {
        return 0;
    }
    int len = 0;
    for (int i=0; i<strlen(str); i++) {
        int width = get_font_width(str[i]);
        if (width < 0) {
            width = FONT_WIDTH_DEFAULT;
        }
        len += width;
    }
    return len;
}