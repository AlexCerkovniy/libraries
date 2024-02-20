#include "gfx8lib_fonts.h"

static const uint8_t PixelOperatorBold8x16[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x17, 0x00, 0x00, 0x00,
    0x00, 0x70, 0x70, 0x00, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0xf0, 0xf0, 0x40, 0xf0, 0xf0, 0x40, 0x00, 0x04, 0x1f, 0x1f, 0x04, 0x1f, 0x1f, 0x04, 0x00,
    0xe0, 0xf0, 0x10, 0xfc, 0x10, 0x30, 0x20, 0x00, 0x08, 0x19, 0x11, 0x7f, 0x11, 0x1f, 0x0e, 0x00,
    0x60, 0xf0, 0x90, 0x60, 0x80, 0xc0, 0x60, 0x00, 0x00, 0x0c, 0x06, 0x03, 0x0d, 0x1e, 0x12, 0x0c,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0x30, 0x20, 0x00, 0x00, 0x0e, 0x1f, 0x11, 0x11, 0x1f, 0x1e, 0x00,
    0x00, 0x00, 0x00, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xc0, 0xe0, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0f, 0x18, 0x10, 0x00,
    0x00, 0x10, 0x30, 0xe0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x18, 0x0f, 0x07, 0x00, 0x00, 0x00,
    0x00, 0x60, 0xc0, 0xf0, 0xc0, 0x60, 0x00, 0x00, 0x00, 0x03, 0x01, 0x07, 0x01, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x07, 0x07, 0x01, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x70, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0xf0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x1f, 0x03, 0x00, 0x00, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x90, 0xf0, 0xe0, 0x00, 0x00, 0x0f, 0x1f, 0x13, 0x11, 0x1f, 0x0f, 0x00,
    0x00, 0xc0, 0x60, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x1f, 0x1f, 0x10, 0x10, 0x00,
    0x00, 0x20, 0x30, 0x10, 0x90, 0xf0, 0x60, 0x00, 0x00, 0x1c, 0x1e, 0x13, 0x11, 0x10, 0x10, 0x00,
    0x00, 0x20, 0x30, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x08, 0x18, 0x11, 0x11, 0x1f, 0x0e, 0x00,
    0x00, 0x00, 0x80, 0xc0, 0x60, 0xf0, 0xf0, 0x00, 0x00, 0x03, 0x03, 0x02, 0x02, 0x1f, 0x1f, 0x00,
    0x00, 0xf0, 0xf0, 0x90, 0x90, 0x90, 0x10, 0x00, 0x00, 0x08, 0x18, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0x30, 0x20, 0x00, 0x00, 0x0f, 0x1f, 0x11, 0x11, 0x1f, 0x0e, 0x00,
    0x00, 0x10, 0x10, 0x10, 0x90, 0xf0, 0x70, 0x00, 0x00, 0x1c, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x0e, 0x1f, 0x11, 0x11, 0x1f, 0x0e, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x08, 0x19, 0x11, 0x11, 0x1f, 0x0f, 0x00,
    0x00, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x70, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0xc0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00,
    0x00, 0x20, 0x30, 0x10, 0x90, 0xf0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x17, 0x17, 0x00, 0x00, 0x00,
    0xe0, 0x10, 0x90, 0x50, 0xd0, 0x10, 0xe0, 0x00, 0x0f, 0x10, 0x13, 0x14, 0x17, 0x14, 0x03, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x1f, 0x1f, 0x02, 0x02, 0x1f, 0x1f, 0x00,
    0x00, 0xf0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x1f, 0x1f, 0x11, 0x11, 0x1f, 0x0e, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0x30, 0x20, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x18, 0x08, 0x00,
    0x00, 0xf0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x1f, 0x1f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xf0, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x1f, 0x1f, 0x11, 0x11, 0x10, 0x10, 0x00,
    0x00, 0xf0, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x1f, 0x1f, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0x30, 0x20, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x11, 0x1f, 0x1f, 0x00,
    0x00, 0xf0, 0xf0, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x1f, 0x1f, 0x01, 0x01, 0x1f, 0x1f, 0x00,
    0x00, 0x10, 0x10, 0xf0, 0xf0, 0x10, 0x10, 0x00, 0x00, 0x10, 0x10, 0x1f, 0x1f, 0x10, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x10, 0xf0, 0xf0, 0x10, 0x00, 0x08, 0x18, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xf0, 0xf0, 0x80, 0xc0, 0x70, 0x30, 0x00, 0x00, 0x1f, 0x1f, 0x03, 0x06, 0x1c, 0x18, 0x00,
    0x00, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x10, 0x10, 0x10, 0x10, 0x00,
    0xf0, 0xf0, 0xc0, 0x80, 0xc0, 0xf0, 0xf0, 0x00, 0x1f, 0x1f, 0x00, 0x01, 0x00, 0x1f, 0x1f, 0x00,
    0x00, 0xf0, 0xf0, 0x80, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x1f, 0x1f, 0x01, 0x03, 0x1f, 0x1f, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xf0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x1f, 0x1f, 0x01, 0x01, 0x01, 0x00, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x0f, 0x1f, 0x16, 0x0c, 0x1f, 0x17, 0x00,
    0x00, 0xf0, 0xf0, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x1f, 0x1f, 0x03, 0x07, 0x1d, 0x18, 0x00,
    0x00, 0xe0, 0xf0, 0x10, 0x10, 0x30, 0x20, 0x00, 0x00, 0x08, 0x19, 0x11, 0x11, 0x1f, 0x0e, 0x00,
    0x00, 0x10, 0x10, 0xf0, 0xf0, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00, 0x00,
    0x00, 0xf0, 0xf0, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xf0, 0xf0, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x07, 0x0f, 0x18, 0x18, 0x0f, 0x07, 0x00,
    0xf0, 0xf0, 0x00, 0xc0, 0x00, 0xf0, 0xf0, 0x00, 0x0f, 0x1f, 0x10, 0x0f, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0x70, 0xf0, 0x80, 0x80, 0xf0, 0x70, 0x00, 0x00, 0x1c, 0x1e, 0x03, 0x03, 0x1e, 0x1c, 0x00,
    0x00, 0x70, 0xf0, 0x80, 0x80, 0xf0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x10, 0x10, 0x90, 0xf0, 0x70, 0x00, 0x00, 0x1c, 0x1e, 0x13, 0x11, 0x10, 0x10, 0x00,
    0x00, 0x00, 0x00, 0xf0, 0xf0, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x10, 0x10, 0x00,
    0x00, 0x00, 0x70, 0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1f, 0x1c, 0x00, 0x00,
    0x00, 0x10, 0x10, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x1f, 0x1f, 0x00, 0x00, 0x00,
    0x00, 0xc0, 0x60, 0x30, 0x30, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
    0x00, 0x00, 0x10, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x0c, 0x1e, 0x12, 0x12, 0x1f, 0x1f, 0x00,
    0x00, 0xf0, 0xf0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x1f, 0x1f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x18, 0x08, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xf0, 0xf0, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x1f, 0x1f, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x0f, 0x1f, 0x12, 0x12, 0x1b, 0x0b, 0x00,
    0x00, 0x00, 0x80, 0xe0, 0xf0, 0x90, 0x90, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0xc0, 0x00, 0x00, 0x27, 0x6f, 0x48, 0x48, 0x7f, 0x3f, 0x00,
    0x00, 0xf0, 0xf0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00, 0x1f, 0x1f, 0x00,
    0x00, 0x40, 0x40, 0xd0, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x1f, 0x1f, 0x10, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x40, 0x40, 0xd0, 0xd0, 0x00, 0x00, 0x20, 0x60, 0x40, 0x40, 0x7f, 0x3f, 0x00,
    0x00, 0xf0, 0xf0, 0x00, 0x80, 0xc0, 0x40, 0x00, 0x00, 0x1f, 0x1f, 0x07, 0x0d, 0x18, 0x10, 0x00,
    0x00, 0x10, 0x10, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x1f, 0x1f, 0x10, 0x10, 0x00,
    0xc0, 0xc0, 0x40, 0x80, 0x40, 0xc0, 0x80, 0x00, 0x1f, 0x1f, 0x00, 0x07, 0x00, 0x1f, 0x1f, 0x00,
    0x00, 0xc0, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00, 0x1f, 0x1f, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xc0, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x7f, 0x7f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0xc0, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x7f, 0x7f, 0x00,
    0x00, 0xc0, 0xc0, 0x00, 0x80, 0xc0, 0xc0, 0x00, 0x00, 0x1f, 0x1f, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xc0, 0x40, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x09, 0x1b, 0x12, 0x12, 0x1e, 0x0c, 0x00,
    0x00, 0x00, 0x40, 0xe0, 0xe0, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x00,
    0x00, 0xc0, 0xc0, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0x0f, 0x1f, 0x10, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xc0, 0xc0, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0x07, 0x0f, 0x18, 0x18, 0x0f, 0x07, 0x00,
    0xc0, 0xc0, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x0f, 0x1f, 0x10, 0x0f, 0x10, 0x1f, 0x0f, 0x00,
    0x00, 0xc0, 0xc0, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0x18, 0x1d, 0x07, 0x07, 0x1d, 0x18, 0x00,
    0x00, 0xc0, 0xc0, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0x27, 0x6f, 0x48, 0x48, 0x7f, 0x3f, 0x00,
    0x00, 0x40, 0x40, 0x40, 0x40, 0xc0, 0xc0, 0x00, 0x00, 0x18, 0x1c, 0x16, 0x13, 0x11, 0x10, 0x00,
    0x00, 0x00, 0x00, 0xe0, 0xf0, 0x10, 0x10, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x1e, 0x10, 0x10, 0x00,
    0x00, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x1e, 0x0f, 0x01, 0x00, 0x00,
    0x20, 0x30, 0x10, 0x30, 0x20, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const gfx8_font_t pixel_operator_bold_8x16 = {8, 16, 2, 32, 126,  PixelOperatorBold8x16};