/*
 * ============================================================================
 *  RF Listener v2.0 — RP2040 + CC1101
 *  Fix scan: AGC nhạy hơn, multi-sample RSSI, threshold -70dBm mặc định
 *  Fix capture: parallel T estimation, single-shot confirm, nhiều protocol
 * ============================================================================
 */

#include "rf_listen.h"

// CC1101 registers
#define CC_IOCFG2     0x00
#define CC_IOCFG1     0x01
#define CC_IOCFG0     0x02
#define CC_FIFOTHR    0x03
#define CC_SYNC1      0x04
#define CC_SYNC0      0x05
#define CC_PKTLEN     0x06
#define CC_PKTCTRL1   0x07
#define CC_PKTCTRL0   0x08
#define CC_ADDR       0x09
#define CC_CHANNR     0x0A
#define CC_FSCTRL1    0x0B
#define CC_FSCTRL0    0x0C
#define CC_FREQ2      0x0D
#define CC_FREQ1      0x0E
#define CC_FREQ0      0x0F
#define CC_MDMCFG4    0x10
#define CC_MDMCFG3    0x11
#define CC_MDMCFG2    0x12
#define CC_MDMCFG1    0x13
#define CC_MDMCFG0    0x14
#define CC_DEVIATN    0x15
#define CC_MCSM2      0x16
#define CC_MCSM1      0x17
#define CC_MCSM0      0x18
#define CC_FOCCFG     0x19
#define CC_BSCFG      0x1A
#define CC_AGCCTRL2   0x1B
#define CC_AGCCTRL1   0x1C
#define CC_AGCCTRL0   0x1D
#define CC_FREND1     0x21
#define CC_FREND0     0x22
#define CC_FSCAL3     0x23
#define CC_FSCAL2     0x24
#define CC_FSCAL1     0x25
#define CC_FSCAL0     0x26
#define CC_RCCTRL1    0x27
#define CC_RCCTRL0    0x28
#define CC_FSTEST     0x29
#define CC_PTEST      0x2A
#define CC_AGCTEST    0x2B
#define CC_TEST2      0x2C
#define CC_TEST1      0x2D
#define CC_TEST0      0x2E
#define CC_PARTNUM    0x30
#define CC_VERSION    0x31
#define CC_RSSI       0x34
#define CC_MARCSTATE  0x35
#define CC_SRES       0x30
#define CC_SCAL       0x33
#define CC_SRX        0x34
#define CC_STX        0x35
#define CC_SIDLE      0x36
#define CC_SFRX       0x3A
#define CC_SFTX       0x3B
#define CC_PATABLE    0x3E
#define WRITE_BURST   0x40
#define READ_SINGLE   0x80
#define READ_BURST    0xC0

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------
struct Packet {
    uint64_t payload;
    uint8_t  bits;
    uint32_t T_us;
    bool     inverted;
    bool     valid;
    char     protocol[20];
    uint8_t  confidence;   // ★ v2
};

struct Group {
    Packet   best;
    uint16_t count;
    uint16_t members[RF_LISTEN_MAX_PACKETS];
};

struct CacheEntry {
    uint64_t payload;
    uint8_t  bits;
    bool     inverted;
    uint32_t timestamp_ms;
    uint8_t  hits;
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static RFListenerConfig g_cfg;
static bool             g_initialized  = false;
static bool             g_chip_ok      = false;
static RFListenMode     g_mode         = RF_LISTEN_SCAN;
static bool             g_has_result   = false;
static RFResult         g_result;
static RFListenCallback g_callback     = nullptr;
static void            *g_callback_data = nullptr;
static bool             g_last_confirmed = false;
static bool             g_nonblocking  = false;

static CacheEntry       g_cache[RF_LISTEN_CACHE_SIZE];
static uint8_t          g_cacheCount   = 0;

static float            g_last_peak_freq = 0.0f;
static int              g_last_peak_rssi = -200;

static SPISettings      g_spi(2000000, MSBFIRST, SPI_MODE0);  // ★ v2: 2MHz

// Edge capture
static volatile uint32_t g_edgeDur[RF_LISTEN_MAX_EDGES];
static volatile uint8_t  g_edgeLevel[RF_LISTEN_MAX_EDGES];
static volatile uint16_t g_edgeCount  = 0;
static volatile uint32_t g_lastEdgeUs = 0;
static volatile uint32_t g_lastEdgeMs = 0;
static volatile bool     g_lastLevel  = false;
static volatile bool     g_capturing  = false;
static volatile uint32_t g_overflow   = 0;

// Frequency list cho SCAN — tối ưu 433/315/868
static const float FREQ_LIST[] = {
    300.000f, 303.875f, 303.900f, 307.000f, 310.000f, 312.000f,
    313.000f, 314.350f, 315.000f, 318.000f, 330.000f, 345.000f,
    348.000f, 350.000f, 387.000f, 390.000f, 418.000f, 430.000f,
    430.500f, 431.000f, 431.500f,
    // ★ Dải 433 MHz — dày hơn vì phổ biến nhất
    433.050f, 433.100f, 433.200f, 433.420f, 433.657f, 433.780f,
    433.850f, 433.880f, 433.900f, 433.920f, 433.950f,
    434.000f, 434.050f, 434.075f, 434.150f, 434.177f, 434.200f,
    434.390f, 434.420f, 434.620f, 434.775f,
    438.900f, 440.175f, 464.000f, 467.750f,
    868.300f, 868.350f, 868.400f, 868.800f, 868.950f,
    915.000f, 925.000f, 928.000f,
};
static const size_t FREQ_COUNT = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);

// ---------------------------------------------------------------------------
// SPI primitives
// ---------------------------------------------------------------------------
static inline void cs_low()  { digitalWrite(g_cfg.pin_csn, LOW);  delayMicroseconds(3); }
static inline void cs_high() { digitalWrite(g_cfg.pin_csn, HIGH); delayMicroseconds(3); }

static void cc_strobe(uint8_t c) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(c); cs_high(); SPI.endTransaction();
}
static void cc_write(uint8_t a, uint8_t v) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a); SPI.transfer(v); cs_high(); SPI.endTransaction();
}
static uint8_t cc_read(uint8_t a) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a | READ_SINGLE);
    uint8_t v = SPI.transfer(0); cs_high(); SPI.endTransaction(); return v;
}
static uint8_t cc_read_status(uint8_t a) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a | READ_BURST);
    uint8_t v = SPI.transfer(0); cs_high(); SPI.endTransaction(); return v;
}
static void cc_write_burst(uint8_t a, const uint8_t *b, size_t l) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a | WRITE_BURST);
    for (size_t i = 0; i < l; i++) SPI.transfer(b[i]);
    cs_high(); SPI.endTransaction();
}

// ---------------------------------------------------------------------------
// Frequency + RSSI
// ---------------------------------------------------------------------------
static void cc_set_freq_regs(float mhz) {
    if (mhz <= 0) return;
    uint32_t v = (uint32_t)(((double)mhz * 1e6 * 65536.0) / 26.0e6 + 0.5);
    cc_write(CC_FREQ2, (v >> 16) & 0xFF);
    cc_write(CC_FREQ1, (v >> 8)  & 0xFF);
    cc_write(CC_FREQ0, v & 0xFF);
}

// ★ v2: đọc RSSI nhiều lần rồi lấy max (tránh đọc trúng khoảng yên lặng)
static int cc_read_rssi_dbm() {
    int best = -200;
    for (int i = 0; i < 3; i++) {
        delayMicroseconds(150);
        uint8_t raw = cc_read_status(CC_RSSI);
        int v;
        if (raw >= 128) v = (int)((int8_t)(raw - 256) / 2) - 74;
        else             v = (int)(raw / 2) - 74;
        if (v > best) best = v;
    }
    return best;
}

// ---------------------------------------------------------------------------
// CC1101 init — cấu hình OOK broadband
// ---------------------------------------------------------------------------
static bool cc_init_hw() {
    pinMode(g_cfg.pin_csn, OUTPUT); digitalWrite(g_cfg.pin_csn, HIGH);
    pinMode(g_cfg.pin_gdo0, INPUT);
    cs_high(); delayMicroseconds(100);
    cs_low();  delayMicroseconds(50);
    cs_high(); delayMicroseconds(100);
    cc_strobe(CC_SRES);
    delay(25);

    uint8_t part = cc_read_status(CC_PARTNUM);
    uint8_t ver  = cc_read_status(CC_VERSION);
    if (g_cfg.verbose) Serial.printf("[RFListen] PARTNUM=0x%02X VERSION=0x%02X\n", part, ver);
    if (ver == 0x00 || ver == 0xFF) return false;

    // GDO0 = carrier sense (async serial data output = signal present)
    cc_write(CC_IOCFG2, 0x0D);
    cc_write(CC_IOCFG1, 0x2E);
    cc_write(CC_IOCFG0, 0x0D);   // GDO0 = sync output (carrier detect)

    cc_write(CC_FIFOTHR,  0x47);
    cc_write(CC_SYNC1,    0xD3);
    cc_write(CC_SYNC0,    0x91);
    cc_write(CC_PKTLEN,   0xFF);
    cc_write(CC_PKTCTRL1, 0x04);
    cc_write(CC_PKTCTRL0, 0x32);  // async serial, infinite packet
    cc_write(CC_ADDR,     0x00);
    cc_write(CC_CHANNR,   0x00);

    cc_write(CC_FSCTRL1,  0x06);
    cc_write(CC_FSCTRL0,  0x00);
    cc_set_freq_regs(g_cfg.frequency_mhz);

    // ★ v2: bandwidth rộng hơn (600kHz) để bắt được nhiều remote hơn
    // MDMCFG4: CHANBW_E=0, CHANBW_M=0 => BW=812kHz; DRATE_E=5
    cc_write(CC_MDMCFG4,  0x07);   // BW=325kHz, DRATE_E=7 (cân bằng)
    cc_write(CC_MDMCFG3,  0x83);   // DRATE_M=131
    cc_write(CC_MDMCFG2,  0x30);   // OOK, no Manchester, no sync check
    cc_write(CC_MDMCFG1,  0x00);
    cc_write(CC_MDMCFG0,  0x00);
    cc_write(CC_DEVIATN,  0x00);

    cc_write(CC_MCSM2, 0x07);
    cc_write(CC_MCSM1, 0x30);
    cc_write(CC_MCSM0, 0x18);  // ★ v2: FS_AUTOCAL=01 (calibrate khi vào RX)

    cc_write(CC_FOCCFG, 0x18);
    cc_write(CC_BSCFG,  0x6C);

    if (g_cfg.agc_sensitive) {
        // ★ v2: AGC nhạy hơn — quan trọng cho remote yếu/xa
        cc_write(CC_AGCCTRL2, 0x03);   // MAX_DVGA_GAIN=00, MAX_LNA_GAIN=000, MAGN_TARGET=011 (33dBm)
        cc_write(CC_AGCCTRL1, 0x00);   // CS_REL_THR=0, CS_ABS_THR=0
        cc_write(CC_AGCCTRL0, 0x91);   // HYST_LEVEL=10, WAIT_TIME=01, AGC_FREEZE=00, FILTER_LENGTH=01
    } else {
        cc_write(CC_AGCCTRL2, 0x43);
        cc_write(CC_AGCCTRL1, 0x40);
        cc_write(CC_AGCCTRL0, 0x91);
    }

    cc_write(CC_FREND1, 0x56);
    cc_write(CC_FREND0, 0x11);
    cc_write(CC_FSCAL3, 0xE9);
    cc_write(CC_FSCAL2, 0x2A);
    cc_write(CC_FSCAL1, 0x00);
    cc_write(CC_FSCAL0, 0x1F);
    cc_write(CC_RCCTRL1, 0x41);
    cc_write(CC_RCCTRL0, 0x00);
    cc_write(CC_FSTEST,  0x59);
    cc_write(CC_PTEST,   0x7F);
    cc_write(CC_AGCTEST, 0x3F);
    cc_write(CC_TEST2,   0x81);
    cc_write(CC_TEST1,   0x35);
    cc_write(CC_TEST0,   0x09);

    uint8_t pa[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc_write_burst(CC_PATABLE, pa, 8);
    cc_strobe(CC_SIDLE);
    delay(5);
    return true;
}

static void cc_enter_rx(float mhz) {
    cc_strobe(CC_SIDLE);
    delayMicroseconds(300);
    cc_set_freq_regs(mhz);
    cc_strobe(CC_SFRX);
    cc_strobe(CC_SRX);
    delay(12);   // ★ v2: chờ PLL lock + AGC settle
}

// ---------------------------------------------------------------------------
// Edge capture ISR
// ---------------------------------------------------------------------------
static void __not_in_flash_func(on_gdo0_edge)() {
    if (!g_capturing) return;
    uint32_t now = micros();
    uint32_t dt  = now - g_lastEdgeUs;
    g_lastEdgeUs = now;
    if (dt < 20) return;  // ★ v2: tăng từ 15 lên 20µs để lọc glitch
    if (g_edgeCount < RF_LISTEN_MAX_EDGES) {
        g_edgeDur[g_edgeCount]   = dt;
        g_edgeLevel[g_edgeCount] = g_lastLevel ? 1 : 0;
        g_edgeCount++;
    } else {
        g_overflow++;
    }
    g_lastLevel = !g_lastLevel;
    g_lastEdgeMs = millis();
}

static void capture_reset() {
    noInterrupts();
    g_edgeCount = 0;
    g_lastLevel = false;
    g_lastEdgeUs = micros();
    g_lastEdgeMs = millis();
    g_overflow = 0;
    interrupts();
}

// ---------------------------------------------------------------------------
// Decoder helpers
// ---------------------------------------------------------------------------
static int cmp_u32(const void *a, const void *b) {
    uint32_t va = *(const uint32_t*)a, vb = *(const uint32_t*)b;
    return (va > vb) - (va < vb);
}

// ★ v2: estimate_T cải thiện — cluster analysis để tìm T thật
static uint32_t estimate_T(const uint32_t *dur, uint16_t n, uint32_t noise_min) {
    static uint32_t tmp[RF_LISTEN_MAX_EDGES];
    size_t cnt = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (dur[i] >= noise_min && dur[i] <= 3000) tmp[cnt++] = dur[i];
    }
    if (cnt < 4) return 0;
    qsort(tmp, cnt, sizeof(uint32_t), cmp_u32);

    // ★ Tìm nhóm nhỏ nhất (T) bằng cách tìm cluster dày nhất ở phần dưới
    uint32_t best_T = 0;
    size_t   best_count = 0;
    for (size_t i = 0; i + 2 < cnt; i++) {
        // Xem nếu tmp[i..i+2] nằm trong 1.4x nhau (1 cluster)
        if (tmp[i + 2] < (uint32_t)(tmp[i] * 1.5f)) {
            size_t w = 3;
            while (i + w < cnt && tmp[i + w] < (uint32_t)(tmp[i] * 1.5f)) w++;
            if (w > best_count) {
                best_count = w;
                best_T = tmp[i + w / 2];  // median của cluster
            }
        }
    }
    if (best_T >= 80 && best_T <= 2500) return best_T;

    // Fallback: phương pháp cũ
    for (size_t i = 0; i + 3 < cnt; i++) {
        if (tmp[i+3] < tmp[i] * 5 / 4) return tmp[i + 1];
    }
    return tmp[cnt / 7];
}

static uint16_t find_syncs(const uint32_t *dur, const uint8_t *lvl, uint16_t n,
                            uint32_t T, uint16_t *out, uint16_t max_out) {
    uint16_t cnt = 0;
    uint32_t thresh = 10 * T;  // ★ v2: giảm từ 15T xuống 10T (Came sync ~11T)
    for (uint16_t i = 0; i < n; i++) {
        if (lvl[i] == 0 && dur[i] > thresh && cnt < max_out) out[cnt++] = i;
    }
    // ★ v2: cũng thử sync HIGH (một số protocol dùng HIGH sync)
    if (cnt == 0) {
        thresh = 8 * T;
        for (uint16_t i = 0; i < n; i++) {
            if (lvl[i] == 1 && dur[i] > thresh && cnt < max_out) out[cnt++] = i;
        }
    }
    return cnt;
}

static int decode_from_offset(const uint32_t *dur, const uint8_t *lvl,
                               uint16_t s, uint16_t end,
                               uint32_t T, bool try_invert, Packet &pkt) {
    if (s + 4 >= end) return 0;
    uint8_t bitStartLevel = lvl[s];
    if (try_invert) bitStartLevel = !bitStartLevel;

    uint32_t thresh2 = 2 * T;
    uint64_t payload = 0;
    int bits = 0, noise_edges = 0;
    uint16_t j = s;

    while (j + 1 < end) {
        if (lvl[j] != bitStartLevel) { j++; continue; }
        uint32_t a = dur[j], b = dur[j + 1];
        // ★ v2: tolerance ±40% thay vì cứng 2T
        bool a_short = a < (uint32_t)(T * 1.8f);
        bool b_short = b < (uint32_t)(T * 1.8f);
        int bit;
        if (a_short && !b_short)      bit = 0;
        else if (!a_short && b_short) bit = 1;
        else { j++; noise_edges++; if (noise_edges > 8) break; continue; }
        payload = (payload << 1) | bit;
        bits++;
        j += 2;
        if (bits >= 64) break;
    }
    if (bits < 8) return 0;
    // ★ v2: score tính thêm chiều dài bit và noise ratio
    int noise_ratio = (bits > 0) ? (noise_edges * 100 / (bits + noise_edges)) : 100;
    int score = bits * 12 - noise_ratio * 2;

    pkt.payload    = payload;
    pkt.bits       = bits;
    pkt.T_us       = T;
    pkt.inverted   = try_invert;
    pkt.valid      = true;
    pkt.protocol[0] = 0;
    pkt.confidence = (uint8_t)constrain(100 - noise_ratio, 0, 100);
    return score;
}

static bool decode_segment_best(const uint32_t *dur, const uint8_t *lvl,
                                 uint16_t syncStart, uint16_t end,
                                 uint32_t T, Packet &out) {
    int best_score = 0;
    Packet best{};
    for (uint16_t off = 0; off < 6; off++) {  // ★ v2: thử 6 offset
        uint16_t s = syncStart + 1 + off;
        if (s + 4 >= end) break;
        Packet p1{}, p2{};
        int sc1 = decode_from_offset(dur, lvl, s, end, T, false, p1);
        int sc2 = decode_from_offset(dur, lvl, s, end, T, true, p2);
        if (sc1 > 0 && sc1 >= best_score) { best_score = sc1; best = p1; }
        if (sc2 > 0 && sc2 > best_score)  { best_score = sc2; best = p2; }
    }
    if (best_score == 0) return false;
    out = best;
    return true;
}

// ★ v2: thử decode với nhiều giá trị T khác nhau quanh T ước tính
static bool decode_multi_T(const uint32_t *dur, const uint8_t *lvl, uint16_t n,
                            uint32_t T_est, uint16_t syncStart, uint16_t end, Packet &out) {
    if (T_est == 0) return false;
    int best_score = 0;
    Packet best{};
    // Thử T * 0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15
    static const float mults[] = {0.85f, 0.90f, 0.95f, 1.00f, 1.05f, 1.10f, 1.15f};
    for (size_t m = 0; m < 7; m++) {
        uint32_t T = (uint32_t)(T_est * mults[m]);
        if (T < 80 || T > 2500) continue;
        Packet p{};
        if (decode_segment_best(dur, lvl, syncStart, end, T, p)) {
            int score = p.bits * 12 + (int)p.confidence;
            if (score > best_score) { best_score = score; best = p; }
        }
    }
    if (best_score == 0) return false;
    out = best;
    return true;
}

static void label_protocol(Packet &pkt) {
    uint32_t T = pkt.T_us;
    int bits = pkt.bits;
    bool inv = pkt.inverted;
    if (!inv && bits == 24 && T >= 350 && T <= 500) strcpy(pkt.protocol, "EV1527");
    else if (!inv && bits == 24 && T >= 250 && T < 350) strcpy(pkt.protocol, "PT2262-24");
    else if (!inv && bits == 24 && T < 250) strcpy(pkt.protocol, "EV1527-fast");
    else if (!inv && bits == 24) strcpy(pkt.protocol, "EV1527/PT2262");
    else if (inv && bits == 12 && T >= 250 && T <= 400) strcpy(pkt.protocol, "Came");
    else if (inv && bits == 12 && T >= 180 && T < 250) strcpy(pkt.protocol, "Came-fast");  // ★ v2
    else if (inv && bits == 12 && T >= 400 && T <= 700) strcpy(pkt.protocol, "Came/Ansonic");// ★ v2
    else if (inv && bits == 12 && T >= 700 && T <= 900) strcpy(pkt.protocol, "Nice");
    else if (!inv && bits == 12) strcpy(pkt.protocol, "Holtek/Ansonic");
    else if (inv && bits == 12) strcpy(pkt.protocol, "Came/Nice?");
    else if (bits >= 20) strcpy(pkt.protocol, "Generic");
    else strcpy(pkt.protocol, "Unknown");
}

static bool same_code(const Packet &a, const Packet &b) {
    if (a.inverted != b.inverted) return false;
    if (a.bits == b.bits && a.payload == b.payload) return true;
    if (a.bits < b.bits) {
        uint8_t diff = b.bits - a.bits;
        if (diff > 4) return false;
        uint64_t shifted = b.payload >> diff;
        uint64_t mask = (a.bits >= 64) ? ~0ULL : ((1ULL << a.bits) - 1);
        if ((shifted & mask) == a.payload) return true;
    } else if (b.bits < a.bits) {
        uint8_t diff = a.bits - b.bits;
        if (diff > 4) return false;
        uint64_t shifted = a.payload >> diff;
        uint64_t mask = (b.bits >= 64) ? ~0ULL : ((1ULL << b.bits) - 1);
        if ((shifted & mask) == b.payload) return true;
    }
    return false;
}

static uint16_t group_packets(Packet *pkts, uint16_t n, Group *groups, uint16_t max_groups) {
    uint16_t nG = 0;
    bool assigned[RF_LISTEN_MAX_PACKETS] = {false};
    for (uint16_t i = 0; i < n; i++) {
        if (assigned[i]) continue;
        if (nG >= max_groups) break;
        Group &g = groups[nG];
        g.count = 0;
        g.best = pkts[i];
        g.members[g.count++] = i;
        assigned[i] = true;
        for (uint16_t j = i + 1; j < n; j++) {
            if (assigned[j]) continue;
            if (same_code(pkts[i], pkts[j])) {
                assigned[j] = true;
                g.members[g.count++] = j;
                // Giữ bản có nhiều bit hơn hoặc confidence cao hơn
                if (pkts[j].bits > g.best.bits ||
                    (pkts[j].bits == g.best.bits && pkts[j].confidence > g.best.confidence))
                    g.best = pkts[j];
            }
        }
        nG++;
    }
    return nG;
}

// Cache
static void cache_clean() {
    uint32_t now = millis();
    uint8_t w = 0;
    for (uint8_t i = 0; i < g_cacheCount; i++) {
        if (now - g_cache[i].timestamp_ms < 10000) g_cache[w++] = g_cache[i];
    }
    g_cacheCount = w;
}

static int cache_find(const Packet &p) {
    for (uint8_t i = 0; i < g_cacheCount; i++) {
        Packet c{};
        c.payload = g_cache[i].payload;
        c.bits = g_cache[i].bits;
        c.inverted = g_cache[i].inverted;
        if (same_code(p, c)) return i;
    }
    return -1;
}

static void cache_put(const Packet &p) {
    int idx = cache_find(p);
    if (idx >= 0) {
        g_cache[idx].hits++;
        g_cache[idx].timestamp_ms = millis();
        if (p.bits > g_cache[idx].bits) {
            g_cache[idx].payload  = p.payload;
            g_cache[idx].bits     = p.bits;
            g_cache[idx].inverted = p.inverted;
        }
        return;
    }
    uint8_t slot = g_cacheCount;
    if (g_cacheCount >= RF_LISTEN_CACHE_SIZE) {
        slot = 0;
        for (uint8_t i = 1; i < RF_LISTEN_CACHE_SIZE; i++)
            if (g_cache[i].timestamp_ms < g_cache[slot].timestamp_ms) slot = i;
    } else {
        g_cacheCount++;
    }
    g_cache[slot] = {p.payload, p.bits, p.inverted, millis(), 1};
}

// ---------------------------------------------------------------------------
// Publish
// ---------------------------------------------------------------------------
static void publish_result(const Packet &bp, int8_t rssi) {
    g_result.payload  = bp.payload;
    g_result.bits     = bp.bits;
    g_result.T_us     = bp.T_us;
    g_result.inverted = bp.inverted;
    strncpy(g_result.protocol, bp.protocol, sizeof(g_result.protocol) - 1);
    g_result.protocol[sizeof(g_result.protocol) - 1] = 0;
        g_result.frequency_mhz_x1000 = (uint32_t)(g_cfg.frequency_mhz * 1000.0f + 0.5f);
    g_result.timestamp_ms = millis();
    g_result.rssi_dbm  = rssi;
    g_result.confidence = bp.confidence;
    g_has_result   = true;
    g_last_confirmed = true;
    if (g_callback) g_callback(g_result, g_callback_data);
}

// ---------------------------------------------------------------------------
// Decode frame — v2: multi-T, single-shot confirm
// ---------------------------------------------------------------------------
static bool decode_frame(const uint32_t *dur, const uint8_t *lvl,
                         uint16_t n, bool printRaw, int8_t rssi) {
    if (n < 6) return false;

    uint32_t T = estimate_T(dur, n, g_cfg.noise_filter_us);

    // ★ v2: Layer 1 — loại nhiễu xung ngắn (89µs, 100µs từ log thực tế)
    // Remote thật: EV1527/PT2262 ≥ 250µs, Came/Nice ≥ 185µs
    if (T < 150) {
        if (printRaw && g_cfg.verbose)
            Serial.printf("⚠ T=%luµs < 150 → noise\n", (unsigned long)T);
        return false;
    }
    if (T > 2500) {
        if (printRaw && g_cfg.verbose) Serial.println(F("⚠ T ngoài dải"));
        return false;
    }

    uint16_t syncIdx[64];
    uint16_t nSync = find_syncs(dur, lvl, n, T, syncIdx, 64);
    if (nSync < 1) {
        // ★ v2: thử decode không cần sync (một số protocol đơn giản)
        Packet pkt{};
        if (decode_multi_T(dur, lvl, n, T, 0, n - 1, pkt) && pkt.bits >= 12) {
            // ★ Layer 6: chặn rác dài (54–60 bit) trước khi label/cache
            if (pkt.bits > g_cfg.max_bits) {
                if (printRaw && g_cfg.verbose)
                    Serial.printf("⚠ NO-SYNC %u bit > max(%u) → bỏ qua\n",
                                  pkt.bits, g_cfg.max_bits);
                return false;
            }
            label_protocol(pkt);
            if (pkt.confidence >= 60) {
                cache_clean();
                cache_put(pkt);
                if (g_cfg.verbose)
                    Serial.printf("  ✓ NO-SYNC  %-16s | %u bit | 0x%llX | conf=%u\n",
                                  pkt.protocol, pkt.bits,
                                  (unsigned long long)pkt.payload, pkt.confidence);
                publish_result(pkt, rssi);
                return true;
            }
        }
        if (printRaw && g_cfg.verbose) Serial.println(F("⚠ Không có sync"));
        return false;
    }

    Packet packets[RF_LISTEN_MAX_PACKETS];
    uint16_t nPkt = 0;

    for (uint16_t s = 0; s + 1 < nSync && nPkt < RF_LISTEN_MAX_PACKETS; s++) {
        Packet pkt{};
        if (decode_multi_T(dur, lvl, n, T, syncIdx[s], syncIdx[s + 1], pkt)) {
            label_protocol(pkt);
            packets[nPkt++] = pkt;
        }
    }
    if (nSync >= 1 && nPkt < RF_LISTEN_MAX_PACKETS) {
        Packet pkt{};
        if (decode_multi_T(dur, lvl, n, T, syncIdx[nSync - 1], n - 1, pkt)) {
            label_protocol(pkt);
            packets[nPkt++] = pkt;
        }
    }
    if (nPkt == 0) {
        if (printRaw && g_cfg.verbose) Serial.println(F("⚠ Không decode được"));
        return false;
    }

    Packet filtered[RF_LISTEN_MAX_PACKETS];
    uint16_t nFilt = 0;
    for (uint16_t i = 0; i < nPkt; i++)
        if (packets[i].bits >= 20) filtered[nFilt++] = packets[i];
    if (nFilt == 0)
        for (uint16_t i = 0; i < nPkt; i++)
            if (packets[i].bits >= 12) filtered[nFilt++] = packets[i];
    // ★ v2: bỏ fallback ≥8 — nhiễu 0xF0 (8 bit) không còn lọt
    if (nFilt == 0) {
        if (printRaw && g_cfg.verbose)
            Serial.println(F("⚠ Tất cả packet < 12 bit → noise"));
        return false;
    }

    Group groups[RF_LISTEN_MAX_PACKETS];
    uint16_t nG = group_packets(filtered, nFilt, groups, RF_LISTEN_MAX_PACKETS);

    int best_g = -1, best_score = -1;
    for (uint16_t g = 0; g < nG; g++) {
        int score = groups[g].best.bits * 10
                  + groups[g].count * 5
                  + (int)groups[g].best.confidence / 5;
        if (score > best_score) { best_score = score; best_g = g; }
    }

    if (best_g < 0) return false;

    const Packet &bp  = groups[best_g].best;
    uint16_t      cnt = groups[best_g].count;

    // ★ v2: Layer 6 — lọc theo max_bits do user cấu hình
    if (bp.bits > g_cfg.max_bits) {
        if (g_cfg.verbose)
            Serial.printf("⚠ %u bit > max(%u) → nghi rác\n",
                          bp.bits, g_cfg.max_bits);
        return false;
    }

    cache_clean();
    int cached = cache_find(bp);
    uint8_t prev_hits = (cached >= 0) ? g_cache[cached].hits : 0;
    uint8_t total_hits = cnt + prev_hits;
    cache_put(bp);

    // ★ v2: confirm ngay từ lần đầu nếu:
    //   - confidence >= 70 (decode rõ ràng), HOẶC
    //   - 2+ packet giống nhau trong 1 frame, HOẶC
    //   - đã thấy trước đó trong cache
    bool confirmed = (bp.confidence >= 70)
                  || (cnt >= 2)
                  || (total_hits >= 2);

    if (g_cfg.min_confidence > 0 && bp.confidence < g_cfg.min_confidence)
        confirmed = false;

    if (g_cfg.verbose) {
        Serial.printf("\n[FRAME] %u edges | T=%lu µs | %u pkt / %u nhóm | conf=%u\n",
                      n, (unsigned long)T, nFilt, nG, bp.confidence);
    }

    if (confirmed) {
        if (g_cfg.verbose) {
            Serial.printf("  ✓ XÁC NHẬN  %-16s | %u bit | 0x%llX | conf=%u | rssi=%d\n",
                          bp.protocol, bp.bits, (unsigned long long)bp.payload,
                          bp.confidence, rssi);
        }
        publish_result(bp, rssi);
        return true;
    }

    if (g_cfg.verbose) {
        Serial.printf("  ⚠ CHỜ — conf=%u (<70), cnt=%u — bấm phát nữa\n",
                      bp.confidence, cnt);
    }
    return false;
}

// ---------------------------------------------------------------------------
// SCAN (non-blocking)
// ---------------------------------------------------------------------------
static void run_scan() {
    static size_t   s_idx      = 0;
    static float    s_peakFreq = 0;
    static int      s_peakRssi = -200;
    static uint32_t s_sweep    = 0;
    static uint32_t s_t0       = 0;

    if (!g_nonblocking) {
        if (g_cfg.verbose)
            Serial.println(F("\n[SCAN] Quét dải — gõ phím + Enter để dừng\n"));
        g_capturing = false;
        while (g_mode == RF_LISTEN_SCAN) {
            if (Serial.available()) return;
            s_sweep++;
            s_t0 = millis();
            float peakFreq = 0; int peakRssi = -200; int printed = 0;
            for (size_t i = 0; i < FREQ_COUNT; i++) {
                if (Serial.available()) return;
                float f = FREQ_LIST[i];
                cc_strobe(CC_SIDLE); delayMicroseconds(200);
                cc_set_freq_regs(f);
                cc_strobe(CC_SFRX); cc_strobe(CC_SRX);
                delay(g_cfg.settle_time_ms);
                int rssi = cc_read_rssi_dbm();
                if (rssi > peakRssi) { peakRssi = rssi; peakFreq = f; }
                if (g_cfg.verbose && rssi >= g_cfg.scan_threshold_dbm) {
                    int bar = constrain((rssi + 100) * 20 / 70, 0, 20);
                    Serial.printf("  %8.3f MHz |", f);
                    for (int b = 0; b < 20; b++) Serial.print(b < bar ? '#' : '.');
                    Serial.printf("| %4d dBm\n", rssi);
                    printed++;
                }
            }
            g_last_peak_freq = peakFreq;
            g_last_peak_rssi = peakRssi;
            if (g_cfg.verbose) {
                Serial.printf("[SWEEP %4lu] %4lu ms | peak %.3f @ %d dBm\n",
                              (unsigned long)s_sweep, (unsigned long)(millis()-s_t0),
                              peakFreq, peakRssi);
            }
        }
        return;
    }

    // Non-blocking: SCAN_STEP tần số / lần gọi
    const size_t SCAN_STEP = 2;  // ★ v2: giảm step để đọc nút nhanh hơn
    g_capturing = false;

    if (s_idx == 0) {
        s_peakFreq = 0;
        s_peakRssi = -200;
        s_t0 = millis();
    }

    size_t end = s_idx + SCAN_STEP;
    if (end > FREQ_COUNT) end = FREQ_COUNT;

    for (size_t i = s_idx; i < end; i++) {
        float f = FREQ_LIST[i];
        cc_strobe(CC_SIDLE); delayMicroseconds(200);
        cc_set_freq_regs(f);
        cc_strobe(CC_SFRX); cc_strobe(CC_SRX);
        delay(g_cfg.settle_time_ms);
        int rssi = cc_read_rssi_dbm();
        if (rssi > s_peakRssi) {
            s_peakRssi = rssi;
            s_peakFreq = f;
        }
    }
    s_idx = end;
    if (s_idx >= FREQ_COUNT) {
        s_sweep++;
        g_last_peak_freq = s_peakFreq;
        g_last_peak_rssi = s_peakRssi;
        s_idx = 0;
    }
}

// ---------------------------------------------------------------------------
// CAPTURE
// ---------------------------------------------------------------------------
static int8_t g_cap_rssi = 0;

static void capture_setup() {
    if (g_cfg.verbose) {
        Serial.println();
        Serial.printf("[CAPTURE] Nghe ở %.3f MHz — bấm remote\n", g_cfg.frequency_mhz);
        Serial.printf("          Gap = %lu ms | noise >= %lu µs\n\n",
                      (unsigned long)g_cfg.frame_gap_ms,
                      (unsigned long)g_cfg.noise_filter_us);
    }
    g_cacheCount = 0;
    cc_enter_rx(g_cfg.frequency_mhz);
    capture_reset();
    g_capturing = true;
    g_cap_rssi  = 0;
    attachInterrupt(digitalPinToInterrupt(g_cfg.pin_gdo0), on_gdo0_edge, CHANGE);
}

static void capture_iteration() {
    static uint32_t dur[RF_LISTEN_MAX_EDGES];
    static uint8_t  lvl[RF_LISTEN_MAX_EDGES];
    static uint32_t lastCheck = 0;

    uint16_t n = g_edgeCount;
    if (n >= RF_LISTEN_MAX_EDGES - 2) {
        noInterrupts();
        uint16_t cnt = g_edgeCount;
        for (uint16_t i = 0; i < cnt; i++) { dur[i] = g_edgeDur[i]; lvl[i] = g_edgeLevel[i]; }
        g_edgeCount = 0;
        g_overflow = 0;
        interrupts();
        int8_t rssi = (int8_t)cc_read_rssi_dbm();
        decode_frame(dur, lvl, cnt, false, rssi);
        return;
    }

    if (n > 0 && (millis() - g_lastEdgeMs) > g_cfg.frame_gap_ms) {
        // ★ v2: đọc RSSI ngay trước khi xử lý
        int8_t rssi = (int8_t)cc_read_rssi_dbm();
        noInterrupts();
        uint16_t cnt = g_edgeCount;
        for (uint16_t i = 0; i < cnt; i++) { dur[i] = g_edgeDur[i]; lvl[i] = g_edgeLevel[i]; }
        g_edgeCount = 0;
        g_overflow = 0;
        interrupts();
        uint16_t valid = 0;
        for (uint16_t i = 0; i < cnt; i++) if (dur[i] >= g_cfg.noise_filter_us) valid++;
        if (valid >= 5) decode_frame(dur, lvl, cnt, true, rssi);
    }

    // Watchdog: khôi phục RX nếu chip thoát khỏi RX state
    if (millis() - lastCheck > 2000) {
        lastCheck = millis();
        uint8_t ms = cc_read_status(CC_MARCSTATE) & 0x1F;
        if (ms != 0x0D && ms != 0x0E && ms != 0x0F) {
            cc_enter_rx(g_cfg.frequency_mhz);
            if (g_cfg.verbose) Serial.printf("[CAPTURE] RX recover (MARC=0x%02X)\n", ms);
        }
    }
}

static void run_capture() {
    if (!g_capturing) capture_setup();
    capture_iteration();
    if (!g_nonblocking) {
        while (g_mode == RF_LISTEN_CAPTURE && !Serial.available()) {
            capture_iteration();
            delay(1);
        }
        rf_listen_set_mode(RF_LISTEN_IDLE);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool rf_listen_begin(const RFListenerConfig &cfg) {
    g_cfg = cfg;
    g_initialized = false;
    g_chip_ok = false;
    SPI.setSCK(cfg.pin_sck);
    SPI.setTX (cfg.pin_mosi);
    SPI.setRX (cfg.pin_miso);
    SPI.begin();
    g_chip_ok = cc_init_hw();
    if (!g_chip_ok) {
        if (cfg.verbose) Serial.println(F("[RFListen] CC1101 init FAIL"));
        return false;
    }
    g_initialized = true;
    g_mode = RF_LISTEN_SCAN;
    if (cfg.verbose) Serial.printf("[RFListen] Ready v%s — %.3f MHz\n",
                                    RF_LISTEN_VERSION, g_cfg.frequency_mhz);
    return true;
}

bool rf_listen_available() { return g_initialized && g_chip_ok; }

void rf_listen_end() {
    if (g_capturing) {
        g_capturing = false;
        detachInterrupt(digitalPinToInterrupt(g_cfg.pin_gdo0));
    }
    g_initialized = false;
}

void rf_listen_update() {
    if (!g_initialized) return;
    g_last_confirmed = false;
    if (g_mode == RF_LISTEN_SCAN)    run_scan();
    if (g_mode == RF_LISTEN_CAPTURE) run_capture();
}

void rf_listen_set_callback(RFListenCallback cb, void *user_data) {
    g_callback = cb;
    g_callback_data = user_data;
}

void rf_listen_set_mode(RFListenMode mode) {
    if (g_mode == RF_LISTEN_CAPTURE && mode != RF_LISTEN_CAPTURE) {
        if (g_capturing) {
            g_capturing = false;
            detachInterrupt(digitalPinToInterrupt(g_cfg.pin_gdo0));
        }
    }
    g_mode = mode;
}
RFListenMode rf_listen_get_mode() { return g_mode; }
bool rf_listen_is_confirmed()     { return g_last_confirmed; }
bool rf_listen_has_result()       { return g_has_result; }

RFResult rf_listen_get_result() {
    RFResult r = g_result;
    g_has_result = false;
    return r;
}

void rf_listen_clear_result()  { g_has_result = false; }
void rf_listen_set_nonblocking(bool nb) { g_nonblocking = nb; }
bool rf_listen_is_nonblocking()         { return g_nonblocking; }
float rf_listen_get_last_peak_freq()    { return g_last_peak_freq; }
int   rf_listen_get_last_peak_rssi()    { return g_last_peak_rssi; }

void rf_listen_set_frequency(float mhz) {
    if (mhz < 280 || mhz > 928) return;
    g_cfg.frequency_mhz = mhz;
    if (g_chip_ok) cc_enter_rx(mhz);
}
float rf_listen_get_frequency() { return g_cfg.frequency_mhz; }
void  rf_listen_set_threshold(int dbm)   { if (dbm >= -120 && dbm <= -20) g_cfg.scan_threshold_dbm = dbm; }
void  rf_listen_set_frame_gap(uint32_t ms) { if (ms >= 3 && ms <= 500) g_cfg.frame_gap_ms = ms; }
void  rf_listen_set_noise_filter(uint32_t us) { if (us <= 5000) g_cfg.noise_filter_us = us; }

uint16_t rf_listen_get_edge_count() { return g_edgeCount; }

uint16_t rf_listen_snapshot_recent_edges(uint32_t* dur, uint8_t* lvl, uint16_t max_n) {
    noInterrupts();
    uint16_t n = g_edgeCount;
    if (n > max_n) n = max_n;
    for (uint16_t i = 0; i < n; i++) { dur[i] = g_edgeDur[i]; lvl[i] = g_edgeLevel[i]; }
    interrupts();
    return n;
}

static void print_status(Stream &out) {
    out.printf("Mode         : %s\n",
               g_mode==RF_LISTEN_SCAN?"SCAN":g_mode==RF_LISTEN_CAPTURE?"CAPTURE":"IDLE");
    out.printf("Frequency    : %.3f MHz\n", g_cfg.frequency_mhz);
    out.printf("Scan thresh  : %d dBm\n", g_cfg.scan_threshold_dbm);
    out.printf("Frame gap    : %lu ms\n", (unsigned long)g_cfg.frame_gap_ms);
    out.printf("Noise filter : %lu µs\n", (unsigned long)g_cfg.noise_filter_us);
    out.printf("AGC mode     : %s\n", g_cfg.agc_sensitive ? "sensitive" : "normal");
    out.printf("Cache        : %u / %d\n", g_cacheCount, RF_LISTEN_CACHE_SIZE);
}

static void print_help(Stream &out) {
    out.println(F("\n=== RF Listen CLI v2 ==="));
    out.println(F("  s         - SCAN mode"));
    out.println(F("  c         - CAPTURE mode"));
    out.println(F("  f <mhz>   - đổi tần số"));
    out.println(F("  t <-dBm>  - ngưỡng scan (vd: t -70)"));
    out.println(F("  g <ms>    - frame gap"));
    out.println(F("  n <us>    - noise filter"));
    out.println(F("  w <ms>    - settle time"));
    out.println(F("  a 0/1     - AGC mode (1=sensitive)"));
    out.println(F("  r         - reset cache"));
    out.println(F("  p         - print config"));
    out.println(F("  d         - dump CC1101 regs"));
    out.println(F("  ?         - help"));
    out.println();
}

static void dump_regs(Stream &out) {
    out.println(F("--- CC1101 registers ---"));
    for (uint8_t a = 0x00; a <= 0x2E; a++) out.printf("  %02X : %02X\n", a, cc_read(a));
    out.printf("  PARTNUM=%02X VERSION=%02X RSSI=%d MARCSTATE=%02X\n",
               cc_read_status(CC_PARTNUM), cc_read_status(CC_VERSION),
               cc_read_rssi_dbm(), cc_read_status(CC_MARCSTATE) & 0x1F);
}

bool rf_listen_handle_serial(Stream &in) {
    if (!in.available()) return false;
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return false;
    char c = tolower(line.charAt(0));
    String arg = line.substring(1); arg.trim();
    switch (c) {
        case 's': g_mode = RF_LISTEN_SCAN;    in.println(F(">> SCAN"));    break;
        case 'c': g_mode = RF_LISTEN_CAPTURE; in.println(F(">> CAPTURE")); break;
        case 'f': { float f = arg.toFloat(); if (f >= 280 && f <= 928) {
            g_cfg.frequency_mhz = f; in.printf(">> freq = %.3f MHz\n", f);
        } break; }
        case 't': { int v = arg.toInt(); if (v >= -120 && v <= -20) {
            g_cfg.scan_threshold_dbm = v; in.printf(">> threshold = %d dBm\n", v);
        } break; }
        case 'g': { int v = arg.toInt(); if (v >= 3 && v <= 500) {
            g_cfg.frame_gap_ms = v; in.printf(">> gap = %d ms\n", v);
        } break; }
        case 'n': { int v = arg.toInt(); if (v >= 0 && v <= 5000) {
            g_cfg.noise_filter_us = v; in.printf(">> noise = %d µs\n", v);
        } break; }
        case 'w': { int v = arg.toInt(); if (v >= 1 && v <= 100) {
            g_cfg.settle_time_ms = v; in.printf(">> settle = %d ms\n", v);
        } break; }
        case 'a': { int v = arg.toInt();
            g_cfg.agc_sensitive = (v != 0);
            // Áp dụng lại AGC ngay
            if (g_chip_ok) {
                if (g_cfg.agc_sensitive) {
                    cc_write(CC_AGCCTRL2, 0x03);
                    cc_write(CC_AGCCTRL1, 0x00);
                    cc_write(CC_AGCCTRL0, 0x91);
                } else {
                    cc_write(CC_AGCCTRL2, 0x43);
                    cc_write(CC_AGCCTRL1, 0x40);
                    cc_write(CC_AGCCTRL0, 0x91);
                }
            }
            in.printf(">> AGC = %s\n", g_cfg.agc_sensitive ? "sensitive" : "normal");
            break;
        }
        case 'r': capture_reset(); g_cacheCount = 0; in.println(F(">> reset + clear cache")); break;
        case 'p': print_status(in); break;
        case 'd': dump_regs(in);    break;
        case '?': print_help(in);   break;
        default: in.println(F("?? gõ '?'"));
    }
    return true;
}

void rf_listen_print_status(Stream &out)   { print_status(out); }
void rf_listen_print_help(Stream &out)     { print_help(out); }
void rf_listen_dump_registers(Stream &out) { dump_regs(out); }
