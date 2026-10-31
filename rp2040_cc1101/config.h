#pragma once
#include <Arduino.h>

// Pins
#define PIN_I2C_SDA    0
#define PIN_I2C_SCL    1
#define PIN_CC_SCK     2
#define PIN_CC_MOSI    3
#define PIN_CC_MISO    4
#define PIN_CC_CSN     5
#define PIN_CC_GDO0    6
#define PIN_BTN_LEFT   9
#define PIN_BTN_MID    11
#define PIN_BTN_RIGHT  13
#define PIN_LED        16

// RF defaults
#define DEFAULT_FREQ_MHZ     315.000f
#define DEFAULT_REPEATS      15
#define DEFAULT_LED_BRIGHT   40

// De Bruijn
#define DEBRUIJN_DEFAULT_BITS    12
#define DEBRUIJN_DEFAULT_T_US    350
#define DEBRUIJN_MAX_BITS        16

#define MIN_MAX_BITS         8
#define MAX_MAX_BITS         64
#define DEFAULT_MAX_BITS     32

// OLED
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

// ★ Storage — 65536 bytes, sector-aligned slots
#define MAX_SAVED_CODES  20         // PROTO codes
#define MAX_RAW_CODES    7          // RAW codes
#define RAW_SLOT_SIZE    8192       // 2 sectors per raw slot (aligned)
#define MAX_NAME_LEN     12
#define EEPROM_SIZE      65536      // ★ 64 KB
#define STORAGE_MAGIC    0x52464459UL  // ★ "RFDY" — bump lần nữa để reset layout cũ

// SavedCode mode
#define SAVED_MODE_PROTO  0
#define SAVED_MODE_RAW    1

// Buttons
#define BTN_DEBOUNCE_MS  25
#define BTN_LONG_MS      600