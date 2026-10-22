#include "keyboard.h"
#include "hardware.h"
#include "config.h"
#include <math.h>

namespace Keyboard {

static KMode    _mode  = KMODE_NONE;
static int      _result = 0;

// Text
static char     _text_buf[48];
static size_t   _text_max = 0;
static size_t   _text_len = 0;
static size_t   _text_cur = 0;   // cursor position
static int      _layout_row = 0;
static int      _layout_col = 0;

// Number
static float    _num_val   = 0.0f;
static float    _num_step  = 1.0f;
static float    _num_min   = 0.0f;
static float    _num_max   = 100.0f;
static int      _num_dec   = 0;
static int      _num_field = 0;   // 0=whole, 1=decimal
static char     _num_buf[20];

// Common
static char     _title[24] = {0};
static bool     _redraw = true;

// ---------------------------------------------------------------------------
// Layouts
// ---------------------------------------------------------------------------
// 3 hàng chữ hoa & số đồng đều 13 ký tự mỗi hàng:
static const char* TEXT_ROWS[3] = {
    "ABCDEFGHIJKLM",
    "NOPQRSTUVWXYZ",
    "0123456789.-_"
};

struct KeyRepeat {
    bool held = false;
    bool repeated = false;
    uint32_t started_ms = 0;
    uint32_t last_step_ms = 0;
};
static KeyRepeat _left_repeat, _right_repeat;

static int text_row_count(int row) {
    if (row < 3) return 13;
    return 4; // DEL, SPACE, CLR, OK
}

static bool repeat_navigation(HW::Btn button, HW::Ev event, KeyRepeat& r) {
    const uint32_t now = millis();
    const bool down = HW::is_pressed(button);
    bool move = false;
    if (down) {
        if (!r.held) {
            r.held = true;
            r.repeated = false;
            r.started_ms = now;
            r.last_step_ms = now;
        } else if ((now - r.started_ms) >= 280 && (now - r.last_step_ms) >= 75) {
            r.last_step_ms = now;
            r.repeated = true;
            move = true;
        }
    } else {
        if (event == HW::EV_CLICK && !r.repeated) move = true;
        r.held = false;
        r.repeated = false;
    }
    return move;
}

static void text_move(int direction) {
    if (direction < 0) {
        if (_layout_col > 0) {
            --_layout_col;
        } else if (_layout_row > 0) {
            --_layout_row;
            _layout_col = text_row_count(_layout_row) - 1;
        } else {
            _layout_row = 3;
            _layout_col = 3;
        }
    } else {
        if (_layout_col + 1 < text_row_count(_layout_row)) {
            ++_layout_col;
        } else if (_layout_row < 3) {
            ++_layout_row;
            _layout_col = 0;
        } else {
            _layout_row = 0;
            _layout_col = 0;
        }
    }
    _redraw = true;
}

// Number layout
static const char NUM_KEYS[4][3] = {
    {'7','8','9'},
    {'4','5','6'},
    {'1','2','3'},
    {'0','.','<'}
};

// ---------------------------------------------------------------------------
bool is_active() { return _mode != KMODE_NONE; }
int  result()    { return _result; }

void begin() {
    _mode   = KMODE_NONE;
    _result = 0;
}

// ---------------------------------------------------------------------------
// Text helpers
// ---------------------------------------------------------------------------
static void text_insert(char c) {
    if (_text_len >= _text_max - 1) return;
    if (_text_cur < _text_len) {
        memmove(&_text_buf[_text_cur + 1], &_text_buf[_text_cur],
                _text_len - _text_cur + 1);
    }
    _text_buf[_text_cur] = c;
    _text_len++;
    _text_cur++;
    _text_buf[_text_len] = 0;
    _redraw = true;
}

static void text_backspace() {
    if (_text_cur == 0) return;
    memmove(&_text_buf[_text_cur - 1], &_text_buf[_text_cur],
            _text_len - _text_cur + 1);
    _text_len--;
    _text_cur--;
    _redraw = true;
}

static void text_clear() {
    _text_buf[0] = 0;
    _text_len = 0;
    _text_cur = 0;
    _redraw = true;
}

// ---------------------------------------------------------------------------
// Text Render
// ---------------------------------------------------------------------------
static void text_render() {
    auto& d = HW::oled;
    d.clearDisplay();

    // 1. Text field box (Blue zone y=17..27)
    d.drawRoundRect(2, 17, 124, 11, 2, SSD1306_WHITE);
    const size_t viewChars = 19;
    size_t start = (_text_cur > viewChars - 1) ? _text_cur - (viewChars - 1) : 0;
    if (start > _text_len) start = _text_len;
    size_t end = start + viewChars;
    if (end > _text_len) end = _text_len;

    int x = 5;
    for (size_t i = start; i < end; ++i) {
        if (i == _text_cur) {
            d.fillRect(x - 1, 18, 7, 9, SSD1306_WHITE);
            d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        } else {
            d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        }
        d.setCursor(x, 19);
        d.write((uint8_t)_text_buf[i]);
        x += 6;
    }
    if (_text_cur == _text_len && _text_cur < start + viewChars) {
        d.fillRect(x - 1, 18, 6, 9, SSD1306_WHITE);
        d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        d.setCursor(x, 19);
        d.write(' ');
    }
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // 2. Keyboard keys (y=29..53)
    const int rowY[3] = {29, 37, 45};
    for (int r = 0; r < 3; ++r) {
        const char* keys = TEXT_ROWS[r];
        for (int c = 0; keys[c]; ++c) {
            const int kx = 4 + c * 9;
            const int ky = rowY[r];
            const bool sel = (_layout_row == r && _layout_col == c);
            if (sel) {
                d.fillRoundRect(kx - 1, ky - 1, 9, 8, 1, SSD1306_WHITE);
                d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            } else {
                d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            }
            d.setCursor(kx + 1, ky);
            d.write((uint8_t)keys[c]);
        }
    }

    // 3. Action row (y=54..62)
    const int by = 54;
    const int bx[4] = {3, 31, 68, 97};
    const int bw[4] = {26, 35, 27, 28};
    const char* labels[4] = {"DEL", "SPACE", "CLR", "OK"};
    for (int c = 0; c < 4; ++c) {
        const bool sel = (_layout_row == 3 && _layout_col == c);
        if (sel) {
            d.fillRoundRect(bx[c], by, bw[c], 9, 2, SSD1306_WHITE);
            d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        } else {
            d.drawRoundRect(bx[c], by, bw[c], 9, 2, SSD1306_WHITE);
            d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        }
        int lx = bx[c] + (bw[c] - (int)strlen(labels[c]) * 6) / 2;
        d.setCursor(lx, by + 1);
        d.print(labels[c]);
    }

    // 4. Header vàng vẽ SAU CÙNG để luôn đè lên trên cùng (y=0..15)
    d.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    d.fillRect(0, 0, 128, 15, SSD1306_WHITE);
    d.setTextSize(1);
    d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(3, 4);
    d.print(_title);

    char counter[12];
    snprintf(counter, sizeof(counter), "%u/%u", (unsigned)_text_len, (unsigned)(_text_max > 1 ? _text_max - 1 : 0));
    int cx = 125 - (int)strlen(counter) * 6;
    d.setCursor(cx, 4);
    d.print(counter);
    d.drawFastHLine(0, 15, 128, SSD1306_BLACK);

    d.display();
}

// ---------------------------------------------------------------------------
// Number helpers & render
// ---------------------------------------------------------------------------
static void num_to_buf() {
    if (_num_dec > 0) {
        snprintf(_num_buf, sizeof(_num_buf), "%.*f", _num_dec, _num_val);
    } else {
        snprintf(_num_buf, sizeof(_num_buf), "%d", (int)_num_val);
    }
}

static void num_apply_digit(char k) {
    if (k == '<') {
        if (_num_field == 1 && _num_dec > 0) {
            float m = powf(10.0f, (float)_num_dec);
            _num_val = floorf((_num_val * m) / 10.0f) / m;
        } else {
            _num_val = floorf(_num_val / 10.0f);
        }
        if (_num_val < _num_min) _num_val = _num_min;
        num_to_buf();
        _redraw = true;
        return;
    }
    if (k == '.') {
        if (_num_dec == 0) _num_dec = 2;
        num_to_buf();
        _redraw = true;
        return;
    }
    int d = k - '0';
    if (_num_field == 0) {
        _num_val = _num_val * 10.0f + d;
        if (_num_val > _num_max) _num_val = _num_max;
    } else {
        float m = powf(10.0f, (float)_num_dec);
        _num_val += (float)d / m;
        if (_num_val > _num_max) _num_val = _num_max;
    }
    num_to_buf();
    _redraw = true;
}

static void num_render() {
    auto& d = HW::oled;
    d.clearDisplay();

    // Value display
    d.setTextSize(2);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.setCursor(6, 18);
    d.print(_num_buf);
    d.setTextSize(1);

    d.drawFastHLine(0, 34, 128, SSD1306_WHITE);

    // Keypad (y=36..62)
    for (int r = 0; r < 4; ++r) {
        const int y = 36 + r * 7;
        for (int c = 0; c < 3; ++c) {
            const int x = 24 + c * 38;
            const bool sel = (r == _layout_row && c == _layout_col);
            if (sel) {
                d.fillRoundRect(x - 8, y, 20, 7, 1, SSD1306_WHITE);
                d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            } else {
                d.drawRoundRect(x - 8, y, 20, 7, 1, SSD1306_WHITE);
                d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            }
            d.setCursor(x, y);
            d.write((uint8_t)NUM_KEYS[r][c]);
        }
    }

    // Top Header (Yellow zone) vẽ sau cùng
    d.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    d.fillRect(0, 0, 128, 15, SSD1306_WHITE);
    d.setTextSize(1);
    d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(3, 4);
    d.print(_title);
    d.setCursor(85, 4);
    d.print("[NUM]");
    d.drawFastHLine(0, 15, 128, SSD1306_BLACK);

    d.display();
}

// ---------------------------------------------------------------------------
static void render() {
    if (_mode == KMODE_TEXT)        text_render();
    else if (_mode == KMODE_NUMBER) num_render();
}

static void open_common(const char* title) {
    strncpy(_title, title, sizeof(_title) - 1);
    _title[sizeof(_title) - 1] = 0;
    _layout_row = 0;
    _layout_col = 0;
    _redraw = true;
    _result = 0;
    _left_repeat = KeyRepeat{};
    _right_repeat = KeyRepeat{};
}

void open_text(const char* title, const char* initial, char* out, size_t max_len) {
    open_common(title);
    _mode = KMODE_TEXT;
    _text_max = max_len;
    strncpy(_text_buf, initial ? initial : "", sizeof(_text_buf) - 1);
    _text_buf[sizeof(_text_buf) - 1] = 0;
    _text_len = strlen(_text_buf);
    _text_cur = _text_len;
}

// ---------------------------------------------------------------------------
// Run loop
// ---------------------------------------------------------------------------
static void run_loop() {
    HW::led_set_color(60, 60, 0);
    while (_result == 0) {
        HW::buttons_tick();
        HW::led_tick();

        if (_redraw) {
            render();
            _redraw = false;
        }

        HW::Ev eL = HW::get_event(HW::BTN_LEFT);
        HW::Ev eM = HW::get_event(HW::BTN_MID);
        HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

        if (_mode == KMODE_TEXT) {
            if (repeat_navigation(HW::BTN_LEFT, eL, _left_repeat))   text_move(-1);
            if (repeat_navigation(HW::BTN_RIGHT, eR, _right_repeat)) text_move(+1);

            if (eM == HW::EV_CLICK) {
                if (_layout_row < 3) {
                    text_insert(TEXT_ROWS[_layout_row][_layout_col]);
                } else {
                    if (_layout_col == 0)      text_backspace();
                    else if (_layout_col == 1) text_insert(' ');
                    else if (_layout_col == 2) text_clear();
                    else                       _result = 1; // OK / SAVE
                }
            }
            if (eM == HW::EV_LONG) _result = -1; // Cancel
        }
        else if (_mode == KMODE_NUMBER) {
            if (eL == HW::EV_CLICK) {
                if (_layout_col > 0) { _layout_col--; _redraw = true; }
                else if (_layout_row > 0) { _layout_row--; _layout_col = 2; _redraw = true; }
            }
            if (eR == HW::EV_CLICK) {
                if (_layout_col < 2) { _layout_col++; _redraw = true; }
                else if (_layout_row < 3) { _layout_row++; _layout_col = 0; _redraw = true; }
            }
            if (eM == HW::EV_CLICK) {
                char k = NUM_KEYS[_layout_row][_layout_col];
                num_apply_digit(k);
            }
            if (eL == HW::EV_LONG) { _result = -1; }
            if (eR == HW::EV_LONG) { _result = 1; }
        }

        delay(5);
    }
}

bool input_text(const char* title, const char* initial, char* out, size_t max_len) {
    open_text(title, initial, out, max_len);
    run_loop();

    if (_result == 1 && out) {
        strncpy(out, _text_buf, max_len - 1);
        out[max_len - 1] = 0;
    }
    _mode = KMODE_NONE;
    return _result == 1;
}

void open_number(const char* title, float initial, float* out,
                 float step, float min, float max, int decimals) {
    open_common(title);
    _mode = KMODE_NUMBER;
    _num_val = initial;
    _num_step = step;
    _num_min = min;
    _num_max = max;
    _num_dec = decimals;
    _num_field = 0;
    num_to_buf();
}

bool input_number(const char* title, float initial, float* out,
                  float step, float min, float max, int decimals) {
    open_number(title, initial, out, step, min, max, decimals);
    run_loop();
    if (_result == 1 && out) {
        if (_num_val < min) _num_val = min;
        if (_num_val > max) _num_val = max;
        *out = _num_val;
    }
    _mode = KMODE_NONE;
    return _result == 1;
}

void close() {
    _mode = KMODE_NONE;
    _result = 0;
}

void tick() {}

} // namespace Keyboard