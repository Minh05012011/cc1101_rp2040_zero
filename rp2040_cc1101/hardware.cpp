#include "hardware.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

namespace HW {

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_NeoPixel led(1, PIN_LED, NEO_GRB + NEO_KHZ800);

// LED state
static LedMode  _lmode      = LED_OFF;
static uint8_t  _lr = 0, _lg = 0, _lb = 0;
static uint32_t _lstart     = 0;
static uint32_t _lflash_ms  = 0;
static uint8_t  _lbright    = DEFAULT_LED_BRIGHT;

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

void led_tick() {
    uint32_t now = millis();
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
            float t = (now - _lstart) / 3000.0f;
            float val = (sinf(t * 2.0f * PI) + 1.0f) / 2.0f;
            led.setPixelColor(0,
                (uint8_t)(_lr * val),
                (uint8_t)(_lg * val),
                (uint8_t)(_lb * val));
            led.show();
            break;
        }
        case LED_BLINK: {
            bool on = ((now - _lstart) / 250) % 2 == 0;
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
    }
}

void led_idle()    { led_set_color(0, 0, 30);   led_set_mode(LED_BREATHE); }
void led_scan()    { led_set_color(0, 60, 60);  led_set_mode(LED_BLINK);   }
void led_capture() { led_set_color(0, 40, 0);   led_set_mode(LED_BREATHE); }
void led_confirm() { led_flash(255, 200, 0, 400); }
void led_tx()      { led_flash(255, 0, 0, 100);   }
void led_error()   { led_flash(255, 0, 0, 800);   }
void led_saved()   { led_flash(255, 255, 255, 200); }

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
}void led_tx_ok()   { led_flash(128, 0, 255, 150); }   // TÍM
void led_jammer()  { led_set_color(255, 200, 0); led_set_mode(LED_SOLID); }  // VÀNG
bool is_pressed(Btn b) {
    int idx = (int)b;
    if (idx < 0 || idx > 2) return false;
    return _btns[idx].stable == LOW;
}

} // namespace HW