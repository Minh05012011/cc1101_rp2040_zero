#pragma once
#include <Arduino.h>
#include "config.h"

struct SavedCode {
    uint64_t payload;
    uint8_t  bits;
    uint16_t T_us;
    bool     inverted;
    uint16_t repeats;
    uint32_t freq_khz;
    char     name[MAX_NAME_LEN];
    uint8_t  mode;          // SAVED_MODE_PROTO / SAVED_MODE_RAW
    uint16_t raw_count;     // 0 nếu PROTO
};

namespace Storage {

bool begin();

// PROTO
int  count();
bool get(int idx, SavedCode &out);
int  add(const SavedCode &code);
bool update(int idx, const SavedCode &code);
bool remove(int idx);

// RAW
int  raw_count();
bool get_raw(int idx, SavedCode &out,
             uint32_t* dur_out, uint8_t* lvl_out,
             uint16_t max_n, uint16_t* n_out);
int  add_raw(const SavedCode &info,
             const uint32_t* dur, const uint8_t* lvl, uint16_t n);
bool remove_raw(int idx);
bool rename_raw(int idx, const char* new_name);   // ★ NEW

void clear();

// Settings
float    get_frequency();
void     set_frequency(float mhz);
uint8_t  get_led_brightness();
void     set_led_brightness(uint8_t b);
uint8_t  get_max_bits();
void     set_max_bits(uint8_t bits);

} // namespace Storage