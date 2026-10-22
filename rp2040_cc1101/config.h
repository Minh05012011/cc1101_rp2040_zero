#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin map
// ---------------------------------------------------------------------------
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
// ---------------------------------------------------------------------------
// RF defaults
// ---------------------------------------------------------------------------
#define DEFAULT_FREQ_MHZ     315.000f
#define DEFAULT_REPEATS      15
#define DEFAULT_LED_BRIGHT   40
// ---------------------------------------------------------------------------
// ★ De Bruijn Sequence
// ---------------------------------------------------------------------------
#define DEBRUIJN_DEFAULT_BITS    12      // Mặc định 12-bit cho cổng fixed-code
#define DEBRUIJN_DEFAULT_T_US    350     // 1T = 350µs (EV1527-like)
#define DEBRUIJN_MAX_BITS        16      // Giới hạn an toàn (65536 bit)

#define MIN_MAX_BITS         8
#define MAX_MAX_BITS         64
#define DEFAULT_MAX_BITS     32
// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

// ---------------------------------------------------------------------------
// Storage
// ---------------------------------------------------------------------------
#define MAX_SAVED_CODES  20
#define MAX_NAME_LEN     12
#define EEPROM_SIZE      1024
// ★ v2: layout mới (freq_khz uint32_t + CODE_SIZE=32) → bump magic để tự reset
#define STORAGE_MAGIC    0x52464455UL   // "RFDU"

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------
#define BTN_DEBOUNCE_MS  25
#define BTN_LONG_MS      600

// ---------------------------------------------------------------------------
// RF defaults
// ---------------------------------------------------------------------------
#define DEFAULT_FREQ_MHZ     315.000f
#define DEFAULT_REPEATS      15
#define DEFAULT_LED_BRIGHT   40