#pragma once
#include <Arduino.h>
#include "config.h"

struct SavedCode {
    uint64_t payload;
    uint8_t  bits;
    uint16_t T_us;
    bool     inverted;
    uint16_t repeats;
    uint16_t freq_mhz;      // MHz integer (315)
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
float    get_frequency();              // MHz
void     set_frequency(float mhz);
uint8_t  get_led_brightness();
void     set_led_brightness(uint8_t b);

} // namespace Storage