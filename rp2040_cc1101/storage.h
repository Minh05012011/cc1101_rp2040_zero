#pragma once
#include <Arduino.h>
#include "config.h"

struct SavedCode {
    uint64_t payload;
    uint8_t  bits;
    uint16_t T_us;
    bool     inverted;
    uint16_t repeats;
    uint32_t freq_khz;      // ★ v2: kHz (433920, 315000) — không còn truncate
    char     name[MAX_NAME_LEN];
};

namespace Storage {

bool begin();

// Codes
int  count();
bool get(int idx, SavedCode &out);
int  add(const SavedCode &code);       // returns idx, or -1
bool update(int idx, const SavedCode &code);
bool remove(int idx);                  // shifts following
void clear();

// Settings
float    get_frequency();              // MHz (float, đủ thập phân)
void     set_frequency(float mhz);
uint8_t  get_led_brightness();
void     set_led_brightness(uint8_t b);
uint8_t  get_max_bits();               // ★ v2
void     set_max_bits(uint8_t bits);   // ★ v2

} // namespace Storage