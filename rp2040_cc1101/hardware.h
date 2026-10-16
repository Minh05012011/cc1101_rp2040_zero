#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

namespace HW {

extern Adafruit_SSD1306 oled;
extern Adafruit_NeoPixel led;

bool begin();

// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------
enum LedMode {
    LED_OFF = 0,
    LED_SOLID,
    LED_BREATHE,
    LED_BLINK,
    LED_FLASH
};

void led_set_mode(LedMode m);
void led_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_flash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms);
void led_off();
void led_set_brightness(uint8_t b);
void led_tick();
void led_tx_ok();   // tím — phát OK
void led_jammer();  // vàng solid — đang jammer
// Preset colors
void led_idle();
void led_scan();
void led_capture();
void led_confirm();
void led_tx();
void led_error();
void led_saved();

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------
enum Btn { BTN_LEFT = 0, BTN_MID, BTN_RIGHT };
enum Ev  { EV_NONE = 0, EV_PRESS, EV_RELEASE, EV_CLICK, EV_LONG };
// Kiểm tra nút đang giữ liên tục >= ms (không debounce reset)
bool is_long_pressed(Btn b, uint32_t ms);
void buttons_tick();
Ev   get_event(Btn b);
bool is_pressed(Btn b);

} // namespace HW