#pragma once
#include <Arduino.h>

namespace Keyboard {

enum KMode {
    KMODE_NONE = 0,
    KMODE_TEXT,
    KMODE_NUMBER
};

bool is_active();

bool input_text(const char* title, const char* initial, char* out, size_t max_len);
bool input_number(const char* title, float initial, float* out,
                  float step, float min, float max, int decimals);

// ★ NEW: nhập 1 số float qua bàn phím QWERTY (hỗ trợ dấu chấm)
bool input_float_text(const char* title, float initial, float* out,
                      float min, float max);

// ★ NEW: nhập range "a-b" qua bàn phím QWERTY. Tự đảo nếu a>b.
bool input_range_text(const char* title, float init_start, float init_stop,
                      float* out_start, float* out_stop, float min, float max);

void begin();
void tick();
int  result();
void close();

} // namespace Keyboard