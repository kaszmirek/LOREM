#pragma once
#include "hardware/i2c.h"
#include <cstdarg>
#include <cstring>

// SSD1306 128x32 OLED driver over I2C.
// Call init() once (after i2c_init), then draw and call display() to flush.
//
// Text cells are 6×8 px → 21 columns × 4 rows on a 128×32 screen.
// print(col, row, str) and printf(col, row, fmt, ...) address those cells.
class OLED {
public:
    static constexpr uint8_t ADDR  = 0x3C;
    static constexpr int     W     = 128;
    static constexpr int     H     = 32;
    static constexpr int     PAGES = H / 8;

    OLED(i2c_inst_t* i2c, uint8_t addr = ADDR) : _i2c(i2c), _addr(addr) {}

    void init();
    void clear();
    void set_pixel(int x, int y, bool on = true);
    void print(int col, int row, const char* str);
    void printf(int col, int row, const char* fmt, ...);
    void display();

private:
    i2c_inst_t* _i2c;
    uint8_t     _addr;
    uint8_t     _buf[W * PAGES] = {};

    void _cmd(uint8_t c);
};
