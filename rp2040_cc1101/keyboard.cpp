#include "keyboard.h"
#include "hardware.h"
#include "config.h"
#include <math.h>

namespace Keyboard {

static KMode    _mode  = KMODE_NONE;
static int      _result = 0;

// Text
static char     _text_buf[32];
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
// Compact text keyboard: A-Z, 0-9, then DEL / SPACE / SAVE.
static const char* TEXT_ROWS[3] = {
    "abcdefghijklm",
    "nopqrstuvwxyz",
    "0123456789"
};

struct KeyRepeat {
    bool held = false;
    bool repeated = false;
    uint32_t started_ms = 0;
    uint32_t last_step_ms = 0;
};
static KeyRepeat _left_repeat, _right_repeat;

static int text_row_count(int row) {
    if (row < 3) return (int)strlen(TEXT_ROWS[row]);
    return 3; // DEL, SPACE, SAVE
}

static bool repeat_navigation(HW::Btn button, HW::Ev event, KeyRepeat& r) {
    const uint32_t now = millis();
    const bool down = HW::is_pressed(button);
    bool move = false;
    if (down) {
        if (!r.held) {
            r.held = true; r.repeated = false;
            r.started_ms = now; r.last_step_ms = now;
        } else if ((now - r.started_ms) >= 300 && (now - r.last_step_ms) >= 82) {
            r.last_step_ms = now; r.repeated = true; move = true;
        }
    } else {
        if (event == HW::EV_CLICK && !r.repeated) move = true;
        r.held = false; r.repeated = false;
    }
    return move;
}

static void text_move(int direction) {
    if (direction < 0) {
        if (_layout_col > 0) --_layout_col;
        else if (_layout_row > 0) { --_layout_row; _layout_col = text_row_count(_layout_row) - 1; }
        else { _layout_row = 3; _layout_col = 2; }
    } else {
        if (_layout_col + 1 < text_row_count(_layout_row)) ++_layout_col;
        else if (_layout_row < 3) { ++_layout_row; _layout_col = 0; }
        else { _layout_row = 0; _layout_col = 0; }
    }
    _redraw = true;
}

// Number layout:
//   row 0: 7 8 9
//   row 1: 4 5 6
//   row 2: 1 2 3
//   row 3: 0 . <
static const char NUM_KEYS[4][3] = {
    {'7','8','9'},
    {'4','5','6'},
    {'1','2','3'},
    {'0','.','<'}
};
static const int NUM_COLS[4] = {3, 3, 3, 3};

// ---------------------------------------------------------------------------
bool is_active() { return _mode != KMODE_NONE; }
int  result()    { return _result; }

void begin() {
    _mode   = KMODE_NONE;
    _result = 0;
}

// ---------------------------------------------------------------------------
// Text
// ---------------------------------------------------------------------------
static void text_apply_edit(int delta) {
    if (_text_cur + delta < 0) return;
    if (_text_cur + delta > _text_len) return;
    _text_cur += delta;
    _redraw = true;
}

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

static void text_render() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.fillRect(0, 0, 128, 16, SSD1306_WHITE);
    d.setTextSize(1);
    d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(2, 4); d.print(_title);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // One-line edit field with an inverse cursor; horizontally follows long names.
    d.drawRoundRect(0, 18, 128, 10, 2, SSD1306_WHITE);
    const size_t viewChars = 20;
    size_t start = (_text_cur > viewChars - 1) ? _text_cur - (viewChars - 1) : 0;
    if (start > _text_len) start = _text_len;
    size_t end = start + viewChars;
    if (end > _text_len) end = _text_len;
    int x = 3;
    for (size_t i = start; i < end; ++i) {
        if (i == _text_cur) {
            d.fillRect(x - 1, 19, 7, 8, SSD1306_WHITE);
            d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        }
        d.setCursor(x, 19); d.write((uint8_t)_text_buf[i]);
        if (i == _text_cur) d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        x += 6;
    }
    if (_text_cur == _text_len) {
        if (_text_cur < start + viewChars) {
            d.fillRect(x - 1, 19, 7, 8, SSD1306_WHITE);
        }
    }

    d.drawFastHLine(0, 29, 128, SSD1306_WHITE);
    const int rowY[4] = {31, 39, 47, 55};
    for (int r = 0; r < 3; ++r) {
        const char* keys = TEXT_ROWS[r];
        for (int c = 0; keys[c]; ++c) {
            const int xx = 3 + c * 9;
            const int yy = rowY[r];
            const bool sel = (_layout_row == r && _layout_col == c);
            if (sel) d.fillRoundRect(xx - 2, yy - 1, 8, 9, 1, SSD1306_WHITE);
            d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                           sel ? SSD1306_WHITE : SSD1306_BLACK);
            d.setCursor(xx, yy); d.write((uint8_t)keys[c]);
            d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        }
    }

    // Special-action row: actions are visible and selectable like any other key.
    const int by = rowY[3];
    const int bx[3] = {3, 43, 88};
    const int bw[3] = {29, 39, 37};
    const char* labels[3] = {"DEL", "SPACE", "SAVE"};
    for (int c = 0; c < 3; ++c) {
        const bool sel = (_layout_row == 3 && _layout_col == c);
        if (sel) d.fillRoundRect(bx[c], by - 1, bw[c], 9, 2, SSD1306_WHITE);
        else d.drawRoundRect(bx[c], by - 1, bw[c], 9, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(bx[c] + (bw[c] - (int)strlen(labels[c]) * 6) / 2, by);
        d.print(labels[c]);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    d.display();
}

// ---------------------------------------------------------------------------
// Number
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
        // backspace — giảm digit
        if (_num_field == 1 && _num_dec > 0) {
            // shift decimal, không dễ — giảm 10^dec
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
    // digit
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
    d.fillRect(0, 0, 128, 16, SSD1306_WHITE);
    d.setTextSize(1); d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(2, 4); d.print(_title);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.setTextSize(2); d.setCursor(3, 18); d.print(_num_buf);
    d.setTextSize(1);
    d.drawFastHLine(0, 36, 128, SSD1306_WHITE);
    for (int r = 0; r < 4; ++r) {
        const int y = 38 + r * 6;
        for (int c = 0; c < 3; ++c) {
            const int x = 24 + c * 38;
            const bool sel = (r == _layout_row && c == _layout_col);
            if (sel) d.fillRoundRect(x - 8, y - 1, 20, 8, 2, SSD1306_WHITE);
            else d.drawRoundRect(x - 8, y - 1, 20, 8, 2, SSD1306_WHITE);
            d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                           sel ? SSD1306_WHITE : SSD1306_BLACK);
            d.setCursor(x, y); d.write((uint8_t)NUM_KEYS[r][c]);
            d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        }
    }
    d.display();
}

// ---------------------------------------------------------------------------
// Common render
// ---------------------------------------------------------------------------
static void render() {
    if (_mode == KMODE_TEXT)   text_render();
    else if (_mode == KMODE_NUMBER) num_render();
}

// ---------------------------------------------------------------------------
// Open
// ---------------------------------------------------------------------------
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
    strncpy(_text_buf, initial ? initial : "", max_len - 1);
    _text_buf[max_len - 1] = 0;
    _text_len = strlen(_text_buf);
    _text_cur = _text_len;
    // store out pointer statically? No — must use blocking API
    // (out được capture trong input_text())
}

// ---------------------------------------------------------------------------
// Blocking wrappers
// ---------------------------------------------------------------------------
static char* _out_text_ptr = nullptr;
static float* _out_num_ptr = nullptr;

static void run_loop() {
    HW::led_set_color(80, 80, 0);
    while (_result == 0) {
        HW::buttons_tick();
        HW::led_tick();

        if (_redraw) { render(); _redraw = false; }

        // Check events
        HW::Ev eL = HW::get_event(HW::BTN_LEFT);
        HW::Ev eM = HW::get_event(HW::BTN_MID);
        HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

        if (_mode == KMODE_TEXT) {
            if (repeat_navigation(HW::BTN_LEFT, eL, _left_repeat)) text_move(-1);
            if (repeat_navigation(HW::BTN_RIGHT, eR, _right_repeat)) text_move(+1);
            if (eM == HW::EV_CLICK) {
                if (_layout_row < 3) {
                    text_insert(TEXT_ROWS[_layout_row][_layout_col]);
                } else if (_layout_col == 0) {
                    text_backspace();
                } else if (_layout_col == 1) {
                    text_insert(' ');
                } else {
                    _result = 1; // visible SAVE key
                }
            }
            // A deliberate middle-button hold cancels; L/R remain available for repeat-scroll.
            if (eM == HW::EV_LONG) _result = -1;
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
    _out_text_ptr = out;
    run_loop();

    if (_result == 1) {
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
    _out_num_ptr = out;
    run_loop();
    if (_result == 1 && out) {
        // clamp
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

void tick() {}   // no-op when using blocking

} // namespace Keyboard