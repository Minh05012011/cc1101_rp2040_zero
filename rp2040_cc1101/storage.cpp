#include "storage.h"
#include <EEPROM.h>

namespace Storage {

// ---------------------------------------------------------------------------
// Layout (bytes):
//   0..3   magic
//   4..5   count
//   6..9   freq_mhz_x1000 (uint32)
//   10     led_brightness
//   11     max_bits               ★ v2
//   12..15 reserved
//   16..   codes (32 bytes each)
//
// StoredCode layout (32 bytes, align 8):
//   offset  0..7   payload (uint64)
//   offset  8      bits
//   offset  9      flags
//   offset 10..11  T_us
//   offset 12..13  repeats
//   offset 14..15  (auto-padding)
//   offset 16..19  freq_khz (uint32)
//   offset 20..31  name[12]
// ---------------------------------------------------------------------------
#define HDR_SIZE     16
#define CODE_SIZE    32

struct StoredCode {
    uint64_t payload;
    uint8_t  bits;
    uint8_t  flags;        // bit0 = inverted
    uint16_t T_us;
    uint16_t repeats;
    uint32_t freq_khz;     // ★ v2
    char     name[MAX_NAME_LEN];
};
static_assert(sizeof(StoredCode) == CODE_SIZE,
              "StoredCode must match CODE_SIZE — adjust _pad or CODE_SIZE");

// ---------------------------------------------------------------------------
// State — khai báo MỘT LẦN duy nhất
// ---------------------------------------------------------------------------
static int      _count      = 0;
static float    _freq_mhz   = DEFAULT_FREQ_MHZ;
static uint8_t  _led_bright = DEFAULT_LED_BRIGHT;
static uint8_t  _max_bits   = DEFAULT_MAX_BITS;   // ★ v2

// ---------------------------------------------------------------------------
static int addr_of(int idx) { return HDR_SIZE + idx * CODE_SIZE; }

static void write_header() {
    uint32_t magic = STORAGE_MAGIC;
    EEPROM.put(0, magic);
    EEPROM.put(4, (uint16_t)_count);
    uint32_t fq1000 = (uint32_t)(_freq_mhz * 1000.0f + 0.5f);
    EEPROM.put(6, fq1000);
    EEPROM.put(10, _led_bright);
    EEPROM.put(11, _max_bits);        // ★ v2
    EEPROM.commit();
}

static void read_header() {
    uint32_t magic = 0;
    EEPROM.get(0, magic);
    if (magic != STORAGE_MAGIC) {
        _count = 0;
        _freq_mhz = DEFAULT_FREQ_MHZ;
        _led_bright = DEFAULT_LED_BRIGHT;
        _max_bits = DEFAULT_MAX_BITS;   // ★ v2
        return;
    }
    uint16_t c = 0;
    EEPROM.get(4, c);
    if (c > MAX_SAVED_CODES) c = 0;
    _count = c;

    uint32_t fq1000 = 0;
    EEPROM.get(6, fq1000);
    _freq_mhz = fq1000 / 1000.0f;
    if (_freq_mhz < 280.0f || _freq_mhz > 928.0f) _freq_mhz = DEFAULT_FREQ_MHZ;

    EEPROM.get(10, _led_bright);
    if (_led_bright > 255) _led_bright = DEFAULT_LED_BRIGHT;

    // ★ v2: byte 11 — có thể 0xFF nếu EEPROM cũ chưa từng set
    _max_bits = EEPROM.read(11);
    if (_max_bits < MIN_MAX_BITS || _max_bits > MAX_MAX_BITS)
        _max_bits = DEFAULT_MAX_BITS;
}

// ---------------------------------------------------------------------------
bool begin() {
    EEPROM.begin(EEPROM_SIZE);
    read_header();

    // Nếu flash trống (hoặc layout cũ) → ghi header mặc định
    uint32_t magic = 0;
    EEPROM.get(0, magic);
    if (magic != STORAGE_MAGIC) {
        _count = 0;
        _freq_mhz = DEFAULT_FREQ_MHZ;
        _led_bright = DEFAULT_LED_BRIGHT;
        _max_bits = DEFAULT_MAX_BITS;   // ★ v2: nhất quán với read_header
        write_header();
    }

    // ★ Debug: verify layout một lần lúc boot
    Serial.printf("[STORAGE] sizeof(StoredCode)=%u  CODE_SIZE=%u  codes=%d  max_bits=%u\n",
                  (unsigned)sizeof(StoredCode), (unsigned)CODE_SIZE,
                  _count, _max_bits);
    return true;
}

int count() { return _count; }

bool get(int idx, SavedCode &out) {
    if (idx < 0 || idx >= _count) return false;
    StoredCode sc;
    EEPROM.get(addr_of(idx), sc);

    out.payload  = sc.payload;
    out.bits     = sc.bits;
    out.T_us     = sc.T_us;
    out.inverted = (sc.flags & 0x01) != 0;
    out.repeats  = sc.repeats;
    out.freq_khz = sc.freq_khz;             // ★ v2
    strncpy(out.name, sc.name, MAX_NAME_LEN - 1);
    out.name[MAX_NAME_LEN - 1] = 0;
    return true;
}

static void code_to_stored(const SavedCode &in, StoredCode &sc) {
    memset(&sc, 0, sizeof(sc));
    sc.payload  = in.payload;
    sc.bits     = in.bits;
    sc.flags    = in.inverted ? 0x01 : 0;
    sc.T_us     = in.T_us;
    sc.repeats  = in.repeats;
    sc.freq_khz = in.freq_khz;              // ★ v2
    strncpy(sc.name, in.name, MAX_NAME_LEN - 1);
    sc.name[MAX_NAME_LEN - 1] = 0;
}

int add(const SavedCode &code) {
    if (_count >= MAX_SAVED_CODES) return -1;
    StoredCode sc;
    code_to_stored(code, sc);
    EEPROM.put(addr_of(_count), sc);
    _count++;
    write_header();
    return _count - 1;
}

bool update(int idx, const SavedCode &code) {
    if (idx < 0 || idx >= _count) return false;
    StoredCode sc;
    code_to_stored(code, sc);
    EEPROM.put(addr_of(idx), sc);
    EEPROM.commit();
    return true;
}

bool remove(int idx) {
    if (idx < 0 || idx >= _count) return false;
    for (int i = idx; i < _count - 1; i++) {
        StoredCode sc;
        EEPROM.get(addr_of(i + 1), sc);
        EEPROM.put(addr_of(i), sc);
    }
    _count--;
    write_header();
    return true;
}

void clear() {
    _count = 0;
    write_header();
}

float get_frequency() { return _freq_mhz; }

void set_frequency(float mhz) {
    if (mhz < 280.0f || mhz > 928.0f) return;
    _freq_mhz = mhz;
    write_header();
}

uint8_t get_led_brightness() { return _led_bright; }

void set_led_brightness(uint8_t b) {
    _led_bright = b;
    write_header();
}

uint8_t get_max_bits() { return _max_bits; }

void set_max_bits(uint8_t bits) {
    if (bits < MIN_MAX_BITS || bits > MAX_MAX_BITS) return;
    _max_bits = bits;
    write_header();
}

} // namespace Storage