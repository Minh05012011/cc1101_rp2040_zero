#pragma once
#include <Arduino.h>

namespace Keyboard {

enum KMode { KMODE_NONE = 0, KMODE_TEXT, KMODE_NUMBER };

bool is_active();

// Blocking modal — return true nếu OK, false nếu cancel
bool input_text(const char* title, const char* initial, char* out, size_t max_len);
bool input_number(const char* title, float initial, float* out,
                  float step, float min, float max, int decimals);

// Non-blocking API (dùng nội bộ Menu nếu muốn)
void begin();
void tick();      // xử lý event + render; tự thoát khi xong
int  result();    // 0=busy, 1=OK, -1=cancel
void close();

} // namespace Keyboard