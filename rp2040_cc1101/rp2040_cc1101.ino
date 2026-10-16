/*
 * ============================================================================
 *  RF Gateway — RP2040 Zero + CC1101 + OLED + WS2812B
 *  ---------------------------------------------------------------------------
 *  Wiring:
 *    I2C OLED : SDA=GP0   SCL=GP1
 *    CC1101   : SCK=GP2  MOSI=GP3  MISO=GP4  CSN=GP5  GDO0=GP6
 *    Buttons  : LEFT=GP9  MID=GP11  RIGHT=GP13   (chân kia -> GND)
 *    WS2812B  : GP16 (onboard)
 *
 *  Libraries (cài từ Library Manager):
 *    - Adafruit SSD1306
 *    - Adafruit GFX Library
 *    - Adafruit NeoPixel
 *
 *  Files trong project:
 *    main.ino
 *    config.h
 *    hardware.h / hardware.cpp
 *    storage.h  / storage.cpp
 *    keyboard.h / keyboard.cpp
 *    menu.h     / menu.cpp
 *    rf_listen.h / rf_listen.cpp
 *    rf_replay.h / rf_replay.cpp
 * ============================================================================
 */

#include <Arduino.h>
#include "config.h"
#include "hardware.h"
#include "storage.h"
#include "menu.h"

void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println(F("=== RF Gateway boot ==="));

    if (!HW::begin()) {
        Serial.println(F("[FATAL] HW init failed"));
        // Blink onboard LED as error indicator
        for (;;) {
            digitalWrite(PIN_LED, HIGH);
            delay(150);
            digitalWrite(PIN_LED, LOW);
            delay(150);
        }
    }

    Storage::begin();
    HW::led_set_brightness(Storage::get_led_brightness());

    Serial.printf("[OK] Storage: %d codes\n", Storage::count());
    Serial.printf("[OK] Freq:    %.3f MHz\n", Storage::get_frequency());

    Menu::begin();

    Serial.println(F("[OK] Menu running"));
}

void loop() {
    Menu::update();
}