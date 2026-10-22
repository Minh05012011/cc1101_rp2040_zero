#pragma once
#include <Arduino.h>

namespace Keyboard {

enum KMode {
    KMODE_NONE = 0,
    KMODE_TEXT,
    KMODE_NUMBER
};

// Trạng thái bàn phím
bool is_active();

// Blocking modal dialogs — return true nếu user nhấn OK/SAVE, false nếu Cancel/Exit
bool input_text(const char* title, const char* initial, char* out, size_t max_len);
bool input_number(const char* title, float initial, float* out,
                  float step, float min, float max, int decimals);

// Non-blocking API
void begin();
void tick();
int  result(); // 0 = busy, 1 = OK, -1 = cancel
void close();

} // namespace Keyboard