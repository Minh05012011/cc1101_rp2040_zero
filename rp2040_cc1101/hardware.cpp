#include "hardware.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

namespace HW {

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_NeoPixel led(1, PIN_LED, NEO_GRB + NEO_KHZ800);

// LED state
static LedMode  _lmode          = LED_OFF;
static uint8_t  _lr = 0, _lg = 0, _lb = 0;
static uint32_t _lstart         = 0;
static uint32_t _lflash_ms      = 0;
static uint8_t  _lbright        = DEFAULT_LED_BRIGHT;
static uint32_t _btn_pop_start  = 0;

// Button state
struct BtnS {
    uint8_t  pin;
    bool     last_reading;
    bool     stable;
    uint32_t last_change;
    uint32_t press_start;
    bool     long_sent;
    Ev       event;
};
static BtnS _btns[3];

// ---------------------------------------------------------------------------
bool begin() {
    // I2C
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setClock(400000);
    Wire.begin();

    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[HW] OLED init FAIL");
        return false;
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.display();

    // LED
    led.begin();
    led.setBrightness(_lbright);
    led.setPixelColor(0, 0, 0, 0);
    led.show();

    // Buttons
    _btns[0] = {PIN_BTN_LEFT,  true, true, 0, 0, false, EV_NONE};
    _btns[1] = {PIN_BTN_MID,   true, true, 0, 0, false, EV_NONE};
    _btns[2] = {PIN_BTN_RIGHT, true, true, 0, 0, false, EV_NONE};
    for (int i = 0; i < 3; i++) pinMode(_btns[i].pin, INPUT_PULLUP);

    return true;
}

// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------
void led_set_mode(LedMode m) {
    _lmode = m;
    _lstart = millis();
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    _lr = r; _lg = g; _lb = b;
    led.setPixelColor(0, r, g, b);
    led.show();
}

void led_flash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
    _lmode = LED_FLASH;
    _lr = r; _lg = g; _lb = b;
    _lflash_ms = ms;
    _lstart = millis();
    led.setPixelColor(0, r, g, b);
    led.show();
}

void led_off() {
    _lmode = LED_OFF;
    led.setPixelColor(0, 0, 0, 0);
    led.show();
}

void led_set_brightness(uint8_t b) {
    _lbright = b;
    led.setBrightness(b);
}

void led_btn_pop() {
    _btn_pop_start = millis();
}

void led_set_theme(uint8_t r, uint8_t g, uint8_t b) {
    _lr = r; _lg = g; _lb = b;
    led_set_mode(LED_BREATHE);
}

void led_rainbow() {
    led_set_mode(LED_RAINBOW);
}

void led_jam_continuous() {
    if (_lmode != LED_JAM_CONT) led_set_mode(LED_JAM_CONT);
}

void led_jam_delay_wait() {
    if (_lmode != LED_JAM_DELAY_WAIT) {
        _lr = 255; _lg = 120; _lb = 0;
        led_set_mode(LED_JAM_DELAY_WAIT);
    }
}

void led_jam_delay_burst() {
    led_flash(255, 0, 220, 180);
}

void led_tick() {
    uint32_t now = millis();

    // 1. Reactive button pop feedback (40ms)
    if (_btn_pop_start > 0 && (now - _btn_pop_start < 40)) {
        led.setPixelColor(0, 180, 230, 255);
        led.show();
        return;
    }

    switch (_lmode) {
        case LED_OFF:
            led.setPixelColor(0, 0, 0, 0);
            led.show();
            break;

        case LED_SOLID:
            led.setPixelColor(0, _lr, _lg, _lb);
            led.show();
            break;

        case LED_BREATHE: {
            float t = (now - _lstart) / 2200.0f;
            float val = (sinf(t * 2.0f * PI) + 1.0f) / 2.0f;
            // Floor of 0.15 so it never goes completely black
            float factor = 0.15f + 0.85f * val;
            led.setPixelColor(0,
                (uint8_t)(_lr * factor),
                (uint8_t)(_lg * factor),
                (uint8_t)(_lb * factor));
            led.show();
            break;
        }

        case LED_BLINK: {
            bool on = ((now - _lstart) / 220) % 2 == 0;
            if (on) led.setPixelColor(0, _lr, _lg, _lb);
            else    led.setPixelColor(0, 0, 0, 0);
            led.show();
            break;
        }

        case LED_FLASH:
            if (now - _lstart > _lflash_ms) {
                _lmode = LED_OFF;
                led.setPixelColor(0, 0, 0, 0);
                led.show();
            }
            break;

        case LED_RAINBOW: {
            uint16_t hue = (uint16_t)(((now - _lstart) * 25) % 65536);
            led.setPixelColor(0, led.ColorHSV(hue, 240, 255));
            led.show();
            break;
        }

        case LED_JAM_CONT: {
            // Aggressive alternating fiery red & bright amber strobe
            bool phase = ((now - _lstart) / 60) % 2 == 0;
            if (phase) led.setPixelColor(0, 255, 10, 0);
            else       led.setPixelColor(0, 255, 150, 0);
            led.show();
            break;
        }

        case LED_JAM_DELAY_WAIT: {
            // Gentle breathing amber
            float t = (now - _lstart) / 1000.0f;
            float val = 0.2f + 0.8f * ((sinf(t * 2.0f * PI) + 1.0f) / 2.0f);
            led.setPixelColor(0, (uint8_t)(255 * val), (uint8_t)(110 * val), 0);
            led.show();
            break;
        }

        case LED_JAM_DELAY_BURST: {
            bool on = ((now - _lstart) / 45) % 2 == 0;
            if (on) led.setPixelColor(0, 255, 0, 220);
            else    led.setPixelColor(0, 0, 0, 0);
            led.show();
            break;
        }
    }
}

void led_idle()    { led_set_theme(0, 80, 180); }               // Neon Blue
void led_scan()    { led_set_theme(0, 200, 200); }              // Cyan
void led_capture() { led_set_theme(0, 220, 50); }               // Green
void led_confirm() { led_flash(255, 220, 0, 350); }             // Amber flash
void led_tx()      { led_flash(255, 0, 80, 100); }              // Red flash
void led_error()   { led_flash(255, 0, 0, 800); }               // Red long
void led_saved()   { led_flash(0, 255, 120, 400); }             // Emerald green flash
void led_tx_ok()   { led_flash(180, 0, 255, 200); }             // Purple flash
void led_jammer()  { led_jam_continuous(); }

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------
void buttons_tick() {
    uint32_t now = millis();
    for (int i = 0; i < 3; i++) {
        BtnS &b = _btns[i];
        bool r = digitalRead(b.pin);

        if (r != b.last_reading) {
            b.last_change = now;
            b.last_reading = r;
        }

        if (now - b.last_change > BTN_DEBOUNCE_MS && r != b.stable) {
            b.stable = r;
            if (b.stable == LOW) {
                b.press_start = now;
                b.long_sent = false;
                b.event = EV_PRESS;
                led_btn_pop(); // Haptic visual pop on button press!
            } else {
                if (!b.long_sent) b.event = EV_CLICK;
                else              b.event = EV_RELEASE;
            }
        }

        if (b.stable == LOW && !b.long_sent &&
            (now - b.press_start) > BTN_LONG_MS) {
            b.long_sent = true;
            b.event = EV_LONG;
        }
    }
}

Ev get_event(Btn b) {
    int idx = (int)b;
    if (idx < 0 || idx > 2) return EV_NONE;
    Ev e = _btns[idx].event;
    _btns[idx].event = EV_NONE;
    return e;
}

bool is_long_pressed(Btn b, uint32_t ms) {
    int idx = (int)b;
    if (idx < 0 || idx > 2) return false;
    BtnS &bs = _btns[idx];
    if (bs.stable != LOW) return false;
    return (millis() - bs.press_start) >= ms;
}

bool is_pressed(Btn b) {
    int idx = (int)b;
    if (idx < 0 || idx > 2) return false;
    return _btns[idx].stable == LOW;
}

} // namespace HW
