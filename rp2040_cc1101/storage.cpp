#include "storage.h"
#include <EEPROM.h>

namespace Storage {

// Layout (65536 bytes, sector-aligned):
//   0..15       header
//   16..655     PROTO (20 × 32 = 640)
//   656..4095   spare
//   4096..      RAW (7 × 8192 = 57344) → ends at 61440
//   61440..     spare

#define HDR_SIZE      16
#define PROTO_SIZE    32
#define PROTO_BASE    HDR_SIZE
#define RAW_BASE      4096

// ★ NEW: bump magic → invalidate data cũ (13-bit → 3-byte/edge format)
#define STORAGE_MAGIC_LOCAL 0x5246445AUL  // "RFDZ" (was "RFDY")

struct StoredProto {
    uint64_t payload;
    uint32_t freq_khz;
    uint16_t T_us;
    uint16_t repeats;
    uint8_t  bits;
    uint8_t  flags;
    uint8_t  _pad[2];
    char     name[MAX_NAME_LEN];
};
static_assert(sizeof(StoredProto) == PROTO_SIZE, "StoredProto must be 32B");

struct StoredRawHdr {
    uint32_t freq_khz;
    uint16_t edge_count;
    uint16_t repeats;
    char     name[MAX_NAME_LEN];
    uint8_t  mode;
    uint8_t  _pad[3];
};
static_assert(sizeof(StoredRawHdr) == 24, "StoredRawHdr must be 24B");

#define RAW_DATA_OFF  24
#define RAW_DATA_MAX  (RAW_SLOT_SIZE - RAW_DATA_OFF)   // 8168 bytes
// ★ NEW: 3 bytes/edge → max_edges = 8168 / 3 = 2722 edges

static int      _count      = 0;
static int      _raw_count  = 0;
static float    _freq_mhz   = DEFAULT_FREQ_MHZ;
static uint8_t  _led_bright = DEFAULT_LED_BRIGHT;
static uint8_t  _max_bits   = DEFAULT_MAX_BITS;

static int addr_proto(int idx) { return PROTO_BASE + idx * PROTO_SIZE; }
static int addr_raw(int idx)   { return RAW_BASE   + idx * RAW_SLOT_SIZE; }

// =========================================================================
// ★ NEW: 3-byte/edge packing (level + 23-bit duration, max 8388607µs ≈ 8.4s)
//   Byte 0: bits 0-7   = duration low
//   Byte 1: bits 8-15  = duration mid
//   Byte 2: bit 7 = level, bits 0-6 = duration high (7 bits)
// → duration max = 0x7FFFFF = 8.388.607 µs
// → capacity: 8168 / 3 = 2722 edges (đủ cho EV1527 × 55 frames)
// =========================================================================
static void pack_edges(const uint32_t* dur, const uint8_t* lvl, uint16_t n,
                       uint8_t* out) {
    memset(out, 0, (size_t)n * 3);
    for (uint16_t i = 0; i < n; i++) {
        uint32_t d = dur[i];
        if (d > 0x7FFFFF) d = 0x7FFFFF;   // max 8.388.607 µs
        out[i*3 + 0] = (uint8_t)(d & 0xFF);
        out[i*3 + 1] = (uint8_t)((d >> 8) & 0xFF);
        uint8_t hi = (uint8_t)((d >> 16) & 0x7F);
        if (lvl[i]) hi |= 0x80;
        out[i*3 + 2] = hi;
    }
}

static void unpack_edges(const uint8_t* in, uint16_t n,
                         uint32_t* dur, uint8_t* lvl) {
    for (uint16_t i = 0; i < n; i++) {
        uint32_t d = (uint32_t)in[i*3 + 0]
                   | ((uint32_t)in[i*3 + 1] << 8)
                   | (((uint32_t)(in[i*3 + 2] & 0x7F)) << 16);
        lvl[i] = (in[i*3 + 2] >> 7) & 1;
        dur[i] = d;
    }
}

static void write_header() {
    EEPROM.put(0, (uint32_t)STORAGE_MAGIC_LOCAL);
    EEPROM.put(4, (uint16_t)_count);
    EEPROM.put(6, (uint32_t)(_freq_mhz * 1000.0f + 0.5f));
    EEPROM.put(10, _led_bright);
    EEPROM.put(11, _max_bits);
    EEPROM.put(12, (uint16_t)_raw_count);
    EEPROM.commit();
}

static void read_header() {
    uint32_t magic = 0;
    EEPROM.get(0, magic);
    if (magic != STORAGE_MAGIC_LOCAL) {
        _count = 0; _raw_count = 0;
        _freq_mhz = DEFAULT_FREQ_MHZ;
        _led_bright = DEFAULT_LED_BRIGHT;
        _max_bits = DEFAULT_MAX_BITS;
        return;
    }
    uint16_t c = 0; EEPROM.get(4, c);
    _count = (c > MAX_SAVED_CODES) ? 0 : c;

    uint32_t fq1000 = 0; EEPROM.get(6, fq1000);
    _freq_mhz = fq1000 / 1000.0f;
    if (_freq_mhz < 280.0f || _freq_mhz > 928.0f) _freq_mhz = DEFAULT_FREQ_MHZ;

    EEPROM.get(10, _led_bright);
    _max_bits = EEPROM.read(11);
    if (_max_bits < MIN_MAX_BITS || _max_bits > MAX_MAX_BITS)
        _max_bits = DEFAULT_MAX_BITS;

    uint16_t rc = 0; EEPROM.get(12, rc);
    _raw_count = (rc > MAX_RAW_CODES) ? 0 : rc;
}

bool begin() {
    EEPROM.begin(EEPROM_SIZE);
    read_header();
    uint32_t magic = 0; EEPROM.get(0, magic);
    if (magic != STORAGE_MAGIC_LOCAL) {
        _count = 0; _raw_count = 0;
        _freq_mhz = DEFAULT_FREQ_MHZ;
        _led_bright = DEFAULT_LED_BRIGHT;
        _max_bits = DEFAULT_MAX_BITS;
        write_header();
        Serial.println(F("[STORAGE] Format changed → cleared old data"));
    }
    Serial.printf("[STORAGE] proto=%d raw=%d freq=%.3f max_bits=%u eeprom=%u\n",
                  _count, _raw_count, _freq_mhz, _max_bits, (unsigned)EEPROM_SIZE);
    return true;
}

// ---- PROTO ----
int count() { return _count; }

bool get(int idx, SavedCode &out) {
    if (idx < 0 || idx >= _count) return false;
    StoredProto sp;
    EEPROM.get(addr_proto(idx), sp);
    out.payload  = sp.payload;
    out.freq_khz = sp.freq_khz;
    out.T_us     = sp.T_us;
    out.repeats  = sp.repeats;
    out.bits     = sp.bits;
    out.inverted = (sp.flags & 0x01) != 0;
    out.mode     = SAVED_MODE_PROTO;
    out.raw_count= 0;
    strncpy(out.name, sp.name, MAX_NAME_LEN - 1);
    out.name[MAX_NAME_LEN - 1] = 0;
    return true;
}

static void code_to_proto(const SavedCode& in, StoredProto& sp) {
    memset(&sp, 0, sizeof(sp));
    sp.payload  = in.payload;
    sp.freq_khz = in.freq_khz;
    sp.T_us     = in.T_us;
    sp.repeats  = in.repeats;
    sp.bits     = in.bits;
    sp.flags    = in.inverted ? 0x01 : 0;
    strncpy(sp.name, in.name, MAX_NAME_LEN - 1);
}

int add(const SavedCode &code) {
    if (_count >= MAX_SAVED_CODES) return -1;
    StoredProto sp;
    code_to_proto(code, sp);
    EEPROM.put(addr_proto(_count), sp);
    _count++;
    write_header();
    return _count - 1;
}

bool update(int idx, const SavedCode &code) {
    if (idx < 0 || idx >= _count) return false;
    StoredProto sp;
    code_to_proto(code, sp);
    EEPROM.put(addr_proto(idx), sp);
    EEPROM.commit();
    return true;
}

bool remove(int idx) {
    if (idx < 0 || idx >= _count) return false;
    for (int i = idx; i < _count - 1; i++) {
        StoredProto sp;
        EEPROM.get(addr_proto(i + 1), sp);
        EEPROM.put(addr_proto(i), sp);
    }
    _count--;
    write_header();
    return true;
}

void clear() {
    _count = 0;
    _raw_count = 0;
    write_header();
}

// ---- RAW ----
int raw_count() { return _raw_count; }

bool get_raw(int idx, SavedCode &out,
             uint32_t* dur_out, uint8_t* lvl_out,
             uint16_t max_n, uint16_t* n_out) {
    if (idx < 0 || idx >= _raw_count) return false;
    int base = addr_raw(idx);
    StoredRawHdr h;
    EEPROM.get(base, h);

    out.payload  = 0;
    out.bits     = 0;
    out.T_us     = 0;
    out.inverted = false;
    out.repeats  = h.repeats;
    out.freq_khz = h.freq_khz;
    out.mode     = SAVED_MODE_RAW;
    out.raw_count= h.edge_count;
    strncpy(out.name, h.name, MAX_NAME_LEN - 1);
    out.name[MAX_NAME_LEN - 1] = 0;

    if (n_out) *n_out = 0;
    if (!dur_out || !lvl_out || max_n == 0) return true;

    uint16_t n = h.edge_count;
    if (n > max_n) n = max_n;

    // ★ NEW: 3 bytes/edge
    uint16_t need_bytes = n * 3;
    if (need_bytes > RAW_DATA_MAX) {
        n = RAW_DATA_MAX / 3;
        need_bytes = n * 3;
    }

    static uint8_t buf[RAW_DATA_MAX];
    for (uint16_t i = 0; i < need_bytes; i++)
        buf[i] = EEPROM.read(base + RAW_DATA_OFF + i);

    unpack_edges(buf, n, dur_out, lvl_out);
    if (n_out) *n_out = n;
    return true;
}

int add_raw(const SavedCode &info,
            const uint32_t* dur, const uint8_t* lvl, uint16_t n) {
    if (_raw_count >= MAX_RAW_CODES) return -1;
    if (!dur || !lvl || n == 0) return -1;

    // ★ NEW: max_edges = 8168 / 3 = 2722
    uint16_t max_edges = RAW_DATA_MAX / 3;
    if (n > max_edges) {
        Serial.printf("[STORAGE] raw truncate %u -> %u edges\n", n, max_edges);
        n = max_edges;
    }

    uint16_t need_bytes = n * 3;
    static uint8_t buf[RAW_DATA_MAX];
    memset(buf, 0, need_bytes);
    pack_edges(dur, lvl, n, buf);

    int base = addr_raw(_raw_count);

    StoredRawHdr h;
    memset(&h, 0, sizeof(h));
    h.freq_khz   = info.freq_khz;
    h.edge_count = n;
    h.repeats    = info.repeats ? info.repeats : 5;
    h.mode       = SAVED_MODE_RAW;
    strncpy(h.name, info.name, MAX_NAME_LEN - 1);
    h.name[MAX_NAME_LEN - 1] = 0;

    EEPROM.put(base, h);
    for (uint16_t i = 0; i < need_bytes; i++)
        EEPROM.write(base + RAW_DATA_OFF + i, buf[i]);

    _raw_count++;
    write_header();
    return _raw_count - 1;
}

bool remove_raw(int idx) {
    if (idx < 0 || idx >= _raw_count) return false;
    for (int i = idx; i < _raw_count - 1; i++) {
        int src = addr_raw(i + 1);
        int dst = addr_raw(i);
        for (int b = 0; b < RAW_SLOT_SIZE; b++)
            EEPROM.write(dst + b, EEPROM.read(src + b));
    }
    _raw_count--;
    write_header();
    return true;
}

bool rename_raw(int idx, const char* new_name) {
    if (idx < 0 || idx >= _raw_count) return false;
    int base = addr_raw(idx);
    StoredRawHdr h;
    EEPROM.get(base, h);
    strncpy(h.name, new_name, MAX_NAME_LEN - 1);
    h.name[MAX_NAME_LEN - 1] = 0;
    EEPROM.put(base, h);
    EEPROM.commit();
    return true;
}

// ---- Settings ----
float get_frequency() { return _freq_mhz; }
void set_frequency(float mhz) {
    if (mhz < 280.0f || mhz > 928.0f) return;
    _freq_mhz = mhz;
    write_header();
}
uint8_t get_led_brightness() { return _led_bright; }
void set_led_brightness(uint8_t b) { _led_bright = b; write_header(); }
uint8_t get_max_bits() { return _max_bits; }
void set_max_bits(uint8_t bits) {
    if (bits < MIN_MAX_BITS || bits > MAX_MAX_BITS) return;
    _max_bits = bits;
    write_header();
}

} // namespace Storage