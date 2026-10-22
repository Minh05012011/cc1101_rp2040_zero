/*
 * ============================================================================
 *  Advanced Capture v4 — frame-based filtering (giống normal mode)
 *
 *  Chiến lược:
 *    - ISR: glitch filter 20µs cứng — giống hệt rf_listen.cpp
 *    - Frame buffer 256 edges: mỗi burst nhận vào đây
 *    - Phát hiện frame kết thúc khi yên lặng > frame_gap_ms
 *    - Validate frame (giống decode_frame của normal):
 *        * đủ edges hợp lệ (>= 8)
 *        * ước lượng T trong [150, 2500]µs
 *        * decode thử ít nhất 1 segment → bits ∈ [12, max_bits]
 *      → chỉ frame PASS mới được commit vào session buffer
 *    - Session buffer tích lũy các frame hợp lệ (tối đa theo free heap)
 *    - Auto-stop:
 *        * session buffer đầy
 *        * user bấm MID
 *        * 2.5s không có frame hợp lệ nào (user nhả nút)
 *    - Analyze: pass1/pass2/pass3 trên session buffer
 * ============================================================================
 */

#include "advanced_capture.h"
#include "config.h"
#include <SPI.h>
#include <math.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// CC1101 register addresses
// ---------------------------------------------------------------------------
#define CC_IOCFG2     0x00
#define CC_IOCFG1     0x01
#define CC_IOCFG0     0x02
#define CC_FIFOTHR    0x03
#define CC_SYNC1      0x04
#define CC_SYNC0      0x05
#define CC_PKTLEN     0x06
#define CC_PKTCTRL1   0x07
#define CC_PKTCTRL0   0x08
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
#define CC_SRX        0x34
#define CC_SIDLE      0x36
#define CC_SFRX       0x3A
#define CC_PATABLE    0x3E
#define WRITE_BURST   0x40
#define READ_SINGLE   0x80
#define READ_BURST    0xC0

namespace AdvCapture {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
// Frame buffer — 1 burst đơn lẻ (giống normal mode)
static const uint16_t FRAME_MAX             = 256;
static const uint16_t MIN_EDGES_PER_FRAME   = 8;   // như normal mode (valid >= 5)

// Session buffer — tất cả frame hợp lệ
static const uint16_t ADV_MAX_EDGES   = 8192;
static const uint16_t ADV_MAX_SYNCS   = 256;
static const uint16_t ADV_MAX_CANDS   = 64;
static const uint16_t ADV_MAX_CLUSTERS = 16;

// Analysis
static const uint16_t MIN_EDGES_TOTAL    = 100;
static const uint16_t MIN_T_COUNT        = 20;
static const uint16_t MIN_REPEATS        = 5;
static const uint16_t MIN_CLUSTER_SIZE   = 2;
static const float    T_TOLERANCE        = 0.15f;
static const float    T_STDDEV_MAX       = 8.0f;
static const uint32_t T_MIN_US           = 150;
static const uint32_t T_MAX_US           = 2500;
static const uint16_t BUCKET_US          = 50;
static const uint8_t  MIN_BITS           = 12;
static const uint8_t  MAX_BITS_HARD      = 64;

// Auto-stop: nếu đã có frame hợp lệ mà 2.5s không có frame mới → dừng
static const uint32_t AUTO_STOP_MS       = 10000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static AdvConfig g_cfg;
static AdvState  g_state = ADV_IDLE;
static AdvResult g_result = {};

// --- Frame buffer (một burst, ISR ghi vào đây) ---
static uint32_t          g_frame_dur[FRAME_MAX];
static uint8_t           g_frame_lvl[FRAME_MAX];
static volatile uint16_t g_frame_count      = 0;
static volatile uint32_t g_frame_last_us    = 0;
static volatile uint32_t g_frame_last_ms    = 0;
static volatile bool     g_frame_last_level = false;
static volatile bool     g_capturing        = false;

// --- Session buffer (tích lũy các frame đã validate) ---
static uint32_t* g_session_dur      = nullptr;
static uint8_t*  g_session_lvl      = nullptr;
static uint16_t  g_session_capacity = 0;
static uint16_t  g_session_count    = 0;

static uint16_t g_valid_frames    = 0;
static uint32_t g_last_valid_ms   = 0;
static uint32_t g_begin_ms        = 0;

static SPISettings g_spi(2000000, MSBFIRST, SPI_MODE0);

// ---------------------------------------------------------------------------
// SPI + CC1101 primitives
// ---------------------------------------------------------------------------
static inline void cs_low()  { digitalWrite(PIN_CC_CSN, LOW);  delayMicroseconds(3); }
static inline void cs_high() { digitalWrite(PIN_CC_CSN, HIGH); delayMicroseconds(3); }

static void cc_strobe(uint8_t c) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(c); cs_high(); SPI.endTransaction();
}
static void cc_write(uint8_t a, uint8_t v) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a); SPI.transfer(v);
    cs_high(); SPI.endTransaction();
}
static uint8_t cc_read_status(uint8_t a) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a | READ_BURST);
    uint8_t v = SPI.transfer(0); cs_high(); SPI.endTransaction(); return v;
}
static void cc_write_burst(uint8_t a, const uint8_t* b, size_t l) {
    SPI.beginTransaction(g_spi); cs_low(); SPI.transfer(a | WRITE_BURST);
    for (size_t i = 0; i < l; i++) SPI.transfer(b[i]);
    cs_high(); SPI.endTransaction();
}
static void cc_set_freq_regs(float mhz) {
    if (mhz <= 0) return;
    uint32_t v = (uint32_t)(((double)mhz * 1e6 * 65536.0) / 26.0e6 + 0.5);
    cc_write(CC_FREQ2, (v >> 16) & 0xFF);
    cc_write(CC_FREQ1, (v >> 8)  & 0xFF);
    cc_write(CC_FREQ0, v & 0xFF);
}
static int cc_read_rssi_dbm() {
    int best = -200;
    for (int i = 0; i < 3; i++) {
        delayMicroseconds(150);
        uint8_t raw = cc_read_status(CC_RSSI);
        int v = (raw >= 128) ? (int)((int8_t)(raw - 256) / 2) - 74
                             : (int)(raw / 2) - 74;
        if (v > best) best = v;
    }
    return best;
}

static bool cc_init_rx() {
    pinMode(PIN_CC_CSN, OUTPUT); digitalWrite(PIN_CC_CSN, HIGH);
    pinMode(PIN_CC_GDO0, INPUT);
    cs_high(); delayMicroseconds(100);
    cs_low();  delayMicroseconds(50);
    cs_high(); delayMicroseconds(100);
    cc_strobe(CC_SRES);
    delay(25);

    uint8_t ver = cc_read_status(CC_VERSION);
    if (ver == 0x00 || ver == 0xFF) return false;

    cc_write(CC_IOCFG2, 0x0D);
    cc_write(CC_IOCFG1, 0x2E);
    cc_write(CC_IOCFG0, 0x0D);
    cc_write(CC_FIFOTHR,  0x47);
    cc_write(CC_SYNC1,    0xD3);
    cc_write(CC_SYNC0,    0x91);
    cc_write(CC_PKTLEN,   0xFF);
    cc_write(CC_PKTCTRL1, 0x04);
    cc_write(CC_PKTCTRL0, 0x32);
    cc_write(CC_CHANNR,   0x00);
    cc_write(CC_FSCTRL1,  0x06);
    cc_write(CC_FSCTRL0,  0x00);
    cc_set_freq_regs(g_cfg.frequency_mhz);
    cc_write(CC_MDMCFG4,  0x07);
    cc_write(CC_MDMCFG3,  0x83);
    cc_write(CC_MDMCFG2,  0x30);
    cc_write(CC_MDMCFG1,  0x00);
    cc_write(CC_MDMCFG0,  0x00);
    cc_write(CC_DEVIATN,  0x00);
    cc_write(CC_MCSM2, 0x07);
    cc_write(CC_MCSM1, 0x30);
    cc_write(CC_MCSM0, 0x18);
    cc_write(CC_FOCCFG, 0x18);
    cc_write(CC_BSCFG,  0x6C);
    cc_write(CC_AGCCTRL2, 0x03);
    cc_write(CC_AGCCTRL1, 0x00);
    cc_write(CC_AGCCTRL0, 0x91);
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

    uint8_t pa[8] = {0x00, 0xC5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc_write_burst(CC_PATABLE, pa, 8);
    cc_strobe(CC_SIDLE);
    delay(5);
    return true;
}

static void cc_enter_rx() {
    cc_strobe(CC_SIDLE);
    delayMicroseconds(300);
    cc_set_freq_regs(g_cfg.frequency_mhz);
    cc_strobe(CC_SFRX);
    cc_strobe(CC_SRX);
    delay(12);
}

// ---------------------------------------------------------------------------
// ISR — giống hệt normal mode: filter 20µs cứng
// ---------------------------------------------------------------------------
static void __not_in_flash_func(on_edge)() {
    if (!g_capturing) return;
    uint32_t now = micros();
    uint32_t dt  = now - g_frame_last_us;
    g_frame_last_us = now;
    if (dt < 20) return;                       // ← như normal mode
    if (g_frame_count < FRAME_MAX) {
        g_frame_dur[g_frame_count] = dt;
        g_frame_lvl[g_frame_count] = g_frame_last_level ? 1 : 0;
        g_frame_count++;
    }
    g_frame_last_level = !g_frame_last_level;
    g_frame_last_ms    = millis();
}

// ---------------------------------------------------------------------------
// Decoder helpers (giống rf_listen.cpp)
// ---------------------------------------------------------------------------
struct Packet {
    uint64_t payload;
    uint8_t  bits;
    uint32_t T_us;
    bool     inverted;
    char     protocol[20];
    uint8_t  confidence;
};

static int cmp_u32(const void* a, const void* b) {
    uint32_t va = *(const uint32_t*)a, vb = *(const uint32_t*)b;
    return (va > vb) - (va < vb);
}

// Ước lượng T bằng cách tìm cluster dày nhất (giống rf_listen.cpp)
static uint32_t estimate_T_frame(const uint32_t* dur, uint16_t n) {
    static uint32_t tmp[FRAME_MAX];
    size_t cnt = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (dur[i] >= T_MIN_US && dur[i] <= T_MAX_US) tmp[cnt++] = dur[i];
    }
    if (cnt < 4) return 0;
    qsort(tmp, cnt, sizeof(uint32_t), cmp_u32);

    uint32_t best_T = 0;
    size_t   best_count = 0;
    for (size_t i = 0; i + 2 < cnt; i++) {
        if (tmp[i + 2] < (uint32_t)(tmp[i] * 1.5f)) {
            size_t w = 3;
            while (i + w < cnt && tmp[i + w] < (uint32_t)(tmp[i] * 1.5f)) w++;
            if (w > best_count) {
                best_count = w;
                best_T = tmp[i + w / 2];
            }
        }
    }
    return (best_T >= T_MIN_US && best_T <= T_MAX_US) ? best_T : 0;
}

static uint16_t find_syncs(const uint32_t* dur, const uint8_t* lvl, uint16_t n,
                            uint32_t T, uint16_t* out, uint16_t max_out) {
    uint16_t cnt = 0;
    uint32_t thresh = 10 * T;
    for (uint16_t i = 0; i < n && cnt < max_out; i++)
        if (lvl[i] == 0 && dur[i] > thresh) out[cnt++] = i;
    if (cnt == 0) {
        thresh = 8 * T;
        for (uint16_t i = 0; i < n && cnt < max_out; i++)
            if (lvl[i] == 1 && dur[i] > thresh) out[cnt++] = i;
    }
    return cnt;
}

static int decode_from_offset(const uint32_t* dur, const uint8_t* lvl,
                               uint16_t s, uint16_t end, uint32_t T,
                               bool try_invert, Packet& pkt) {
    if (s + 4 >= end) return 0;
    uint8_t bitStartLevel = lvl[s];
    if (try_invert) bitStartLevel = !bitStartLevel;

    uint64_t payload = 0;
    int bits = 0, noise = 0;
    uint16_t j = s;
    while (j + 1 < end) {
        if (lvl[j] != bitStartLevel) { j++; continue; }
        uint32_t a = dur[j], b = dur[j+1];
        bool a_short = a < (uint32_t)(T * 1.8f);
        bool b_short = b < (uint32_t)(T * 1.8f);
        int bit;
        if (a_short && !b_short)      bit = 0;
        else if (!a_short && b_short) bit = 1;
        else { j++; noise++; if (noise > 8) break; continue; }
        payload = (payload << 1) | bit;
        bits++;
        j += 2;
        if (bits >= 64) break;
    }
    if (bits < 8) return 0;
    int noise_ratio = (bits > 0) ? (noise * 100 / (bits + noise)) : 100;
    pkt.payload    = payload;
    pkt.bits       = bits;
    pkt.T_us       = T;
    pkt.inverted   = try_invert;
    pkt.protocol[0] = 0;
    pkt.confidence = (uint8_t)constrain(100 - noise_ratio, 0, 100);
    return bits * 12 - noise_ratio * 2;
}

static bool decode_segment_best(const uint32_t* dur, const uint8_t* lvl,
                                 uint16_t syncStart, uint16_t end,
                                 uint32_t T, Packet& out) {
    int best_score = 0;
    Packet best{};
    for (uint16_t off = 0; off < 6; off++) {
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

static bool decode_multi_T(const uint32_t* dur, const uint8_t* lvl,
                            uint32_t T_est, uint16_t syncStart, uint16_t end, Packet& out) {
    if (T_est == 0) return false;
    int best_score = 0;
    Packet best{};
    static const float mults[] = {0.90f, 0.95f, 1.00f, 1.05f, 1.10f};
    for (size_t m = 0; m < 5; m++) {
        uint32_t T = (uint32_t)(T_est * mults[m]);
        if (T < T_MIN_US || T > T_MAX_US) continue;
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

static void label_protocol(Packet& pkt) {
    uint32_t T = pkt.T_us;
    int bits = pkt.bits;
    bool inv = pkt.inverted;
    if (!inv && bits == 24 && T >= 350 && T <= 500) strcpy(pkt.protocol, "EV1527");
    else if (!inv && bits == 24 && T >= 250 && T < 350) strcpy(pkt.protocol, "PT2262-24");
    else if (!inv && bits == 24 && T < 250) strcpy(pkt.protocol, "EV1527-fast");
    else if (!inv && bits == 24) strcpy(pkt.protocol, "EV1527/PT2262");
    else if (inv && bits == 12 && T >= 250 && T <= 400) strcpy(pkt.protocol, "Came");
    else if (inv && bits == 12 && T >= 180 && T < 250) strcpy(pkt.protocol, "Came-fast");
    else if (inv && bits == 12 && T >= 400 && T <= 700) strcpy(pkt.protocol, "Came/Ansonic");
    else if (inv && bits == 12 && T >= 700 && T <= 900) strcpy(pkt.protocol, "Nice");
    else if (!inv && bits == 12) strcpy(pkt.protocol, "Holtek/Ansonic");
    else if (inv && bits == 12) strcpy(pkt.protocol, "Came/Nice?");
    else if (bits >= 20) strcpy(pkt.protocol, "Generic");
    else strcpy(pkt.protocol, "Unknown");
}

// ---------------------------------------------------------------------------
// Validation 1 frame — giống hệt decode_frame của normal mode
// ---------------------------------------------------------------------------
static bool frame_looks_decodable(uint16_t n) {
    if (n < MIN_EDGES_PER_FRAME) return false;

    // Đếm edges vượt noise filter (như normal mode)
    uint16_t valid = 0;
    for (uint16_t i = 0; i < n; i++)
        if (g_frame_dur[i] >= g_cfg.noise_filter_us && g_frame_dur[i] <= 5000) valid++;
    if (valid < MIN_EDGES_PER_FRAME) return false;

    // Ước lượng T
    uint32_t T = estimate_T_frame(g_frame_dur, n);
    if (T < T_MIN_US || T > T_MAX_US) return false;

    // Tìm sync
    uint16_t syncs[16];
    uint16_t nSync = find_syncs(g_frame_dur, g_frame_lvl, n, T, syncs, 16);

    // Không có sync → thử decode toàn frame (như normal mode)
    if (nSync < 1) {
        Packet pkt{};
        if (decode_multi_T(g_frame_dur, g_frame_lvl, T, 0, n - 1, pkt)) {
            return (pkt.bits >= MIN_BITS && pkt.bits <= g_cfg.max_bits);
        }
        return false;
    }

    // Có sync → thử decode từng segment, chỉ cần 1 segment OK là đủ
    for (uint16_t s = 0; s < nSync; s++) {
        uint16_t end = (s + 1 < nSync) ? syncs[s + 1] : n;
        Packet pkt{};
        if (decode_multi_T(g_frame_dur, g_frame_lvl, T, syncs[s], end, pkt)) {
            if (pkt.bits >= MIN_BITS && pkt.bits <= g_cfg.max_bits) return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Commit 1 frame hợp lệ vào session buffer
// ---------------------------------------------------------------------------
static void commit_frame(uint16_t n) {
    if (g_session_count + n > g_session_capacity) {
        n = g_session_capacity - g_session_count;
        if (n < MIN_EDGES_PER_FRAME) return;
    }
    for (uint16_t i = 0; i < n; i++) {
        g_session_dur[g_session_count] = g_frame_dur[i];
        g_session_lvl[g_session_count] = g_frame_lvl[i];
        g_session_count++;
    }
    g_valid_frames++;
    g_last_valid_ms = millis();
}

// Reset frame buffer an toàn + reset timing để edge tiếp theo không bị dt lớn
static uint16_t reset_frame_buffer() {
    noInterrupts();
    uint16_t n = g_frame_count;
    g_frame_count      = 0;
    g_frame_last_level = false;
    g_frame_last_us    = micros();
    g_frame_last_ms    = millis();
    interrupts();
    return n;
}

// ---------------------------------------------------------------------------
// Analysis — 3 pass trên SESSION buffer
// ---------------------------------------------------------------------------
struct TCand {
    uint32_t T_median;
    uint16_t count;
};

struct PayloadCand {
    uint64_t payload;
    uint8_t  bits;
    bool     inverted;
    uint32_t T_us;
    char     protocol[20];
};

struct FinalCluster {
    uint64_t payload;
    uint8_t  bits;
    bool     inverted;
    uint32_t T_median;
    uint64_t sum_T;
    uint64_t sum_T2;
    uint16_t count;
    float    T_stddev_pct;
    char     protocol[20];
};

static TCand        g_t_cands[4];
static uint8_t      g_t_count = 0;
static PayloadCand  g_cands[ADV_MAX_CANDS];
static uint8_t      g_cand_count = 0;
static FinalCluster g_finals[ADV_MAX_CLUSTERS];
static uint8_t      g_final_count = 0;

static void pass1_find_T() {
    g_t_count = 0;
    const uint16_t n_buckets = (T_MAX_US - T_MIN_US) / BUCKET_US;
    uint16_t buckets[64] = {0};

    for (uint16_t i = 0; i < g_session_count; i++) {
        uint32_t d = g_session_dur[i];
        if (d < T_MIN_US || d > T_MAX_US) continue;
        uint16_t idx = (d - T_MIN_US) / BUCKET_US;
        if (idx < n_buckets) buckets[idx]++;
    }

    struct Peak { uint16_t idx; uint16_t count; };
    Peak peaks[8];
    uint8_t np = 0;
    for (uint16_t i = 0; i < n_buckets && np < 8; i++) {
        if (buckets[i] < MIN_T_COUNT) continue;
        if (i > 0 && buckets[i-1] > buckets[i]) continue;
        if (i < n_buckets - 1 && buckets[i+1] > buckets[i]) continue;
        peaks[np].idx = i;
        peaks[np].count = buckets[i];
        np++;
    }
    for (uint8_t i = 0; i < np; i++)
        for (uint8_t j = i+1; j < np; j++)
            if (peaks[j].count > peaks[i].count) {
                Peak tmp = peaks[i]; peaks[i] = peaks[j]; peaks[j] = tmp;
            }

    static uint32_t ref[512];
    for (uint8_t p = 0; p < np && g_t_count < 3; p++) {
        uint32_t lo = T_MIN_US + (peaks[p].idx > 0 ? peaks[p].idx - 1 : 0) * BUCKET_US;
        uint32_t hi = T_MIN_US + (peaks[p].idx + 2) * BUCKET_US;
        uint16_t n = 0;
        for (uint16_t i = 0; i < g_session_count && n < 512; i++) {
            uint32_t d = g_session_dur[i];
            if (d >= lo && d <= hi) ref[n++] = d;
        }
        if (n < MIN_T_COUNT) continue;
        qsort(ref, n, sizeof(uint32_t), cmp_u32);
        g_t_cands[g_t_count].T_median = ref[n / 2];
        g_t_cands[g_t_count].count    = n;
        g_t_count++;
    }
}

static void add_candidate(const Packet& pkt) {
    if (g_cand_count >= ADV_MAX_CANDS) return;
    g_cands[g_cand_count].payload  = pkt.payload;
    g_cands[g_cand_count].bits     = pkt.bits;
    g_cands[g_cand_count].inverted = pkt.inverted;
    g_cands[g_cand_count].T_us     = pkt.T_us;
    strncpy(g_cands[g_cand_count].protocol, pkt.protocol, 19);
    g_cands[g_cand_count].protocol[19] = 0;
    g_cand_count++;
}

static void pass2_decode() {
    g_cand_count = 0;
    static uint16_t syncIdx[ADV_MAX_SYNCS];
    const uint8_t max_b = g_cfg.max_bits;

    for (uint8_t t = 0; t < g_t_count; t++) {
        uint32_t T = g_t_cands[t].T_median;
        uint16_t nSync = find_syncs(g_session_dur, g_session_lvl, g_session_count,
                                     T, syncIdx, ADV_MAX_SYNCS);
        if (nSync < 1) {
            Packet pkt{};
            if (decode_multi_T(g_session_dur, g_session_lvl, T,
                               0, g_session_count - 1, pkt)) {
                if (pkt.bits >= MIN_BITS && pkt.bits <= max_b) {
                    label_protocol(pkt);
                    add_candidate(pkt);
                }
            }
            continue;
        }
        for (uint16_t s = 0; s + 1 < nSync && g_cand_count < ADV_MAX_CANDS; s++) {
            Packet pkt{};
            if (decode_multi_T(g_session_dur, g_session_lvl, T,
                               syncIdx[s], syncIdx[s+1], pkt)) {
                if (pkt.bits >= MIN_BITS && pkt.bits <= max_b) {
                    label_protocol(pkt);
                    add_candidate(pkt);
                }
            }
        }
        if (nSync >= 1 && g_cand_count < ADV_MAX_CANDS) {
            Packet pkt{};
            if (decode_multi_T(g_session_dur, g_session_lvl, T,
                               syncIdx[nSync-1], g_session_count - 1, pkt)) {
                if (pkt.bits >= MIN_BITS && pkt.bits <= max_b) {
                    label_protocol(pkt);
                    add_candidate(pkt);
                }
            }
        }
    }
}

static void pass3_merge_and_pick() {
    g_final_count = 0;
    for (uint8_t i = 0; i < g_cand_count; i++) {
        bool merged = false;
        for (uint8_t f = 0; f < g_final_count; f++) {
            if (g_finals[f].payload != g_cands[i].payload) continue;
            if (g_finals[f].bits != g_cands[i].bits) continue;
            if (g_finals[f].inverted != g_cands[i].inverted) continue;
            float diff = fabsf((float)g_finals[f].T_median - (float)g_cands[i].T_us);
            float rel = diff / (float)g_cands[i].T_us;
            if (rel < T_TOLERANCE) {
                g_finals[f].sum_T  += g_cands[i].T_us;
                g_finals[f].sum_T2 += (uint64_t)g_cands[i].T_us * g_cands[i].T_us;
                g_finals[f].count++;
                g_finals[f].T_median = (uint32_t)(g_finals[f].sum_T / g_finals[f].count);
                merged = true;
                break;
            }
        }
        if (!merged && g_final_count < ADV_MAX_CLUSTERS) {
            FinalCluster& f = g_finals[g_final_count];
            f.payload  = g_cands[i].payload;
            f.bits     = g_cands[i].bits;
            f.inverted = g_cands[i].inverted;
            f.T_median = g_cands[i].T_us;
            f.sum_T    = g_cands[i].T_us;
            f.sum_T2   = (uint64_t)g_cands[i].T_us * g_cands[i].T_us;
            f.count    = 1;
            f.T_stddev_pct = 0.0f;
            strncpy(f.protocol, g_cands[i].protocol, 19);
            f.protocol[19] = 0;
            g_final_count++;
        }
    }
    for (uint8_t f = 0; f < g_final_count; f++) {
        float mean = (float)g_finals[f].sum_T / g_finals[f].count;
        float var  = (float)g_finals[f].sum_T2 / g_finals[f].count - mean * mean;
        float sd   = sqrtf(var > 0 ? var : 0);
        g_finals[f].T_stddev_pct = (mean > 0) ? (sd / mean * 100.0f) : 100.0f;
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
uint8_t get_free_heap_kb() {
    uint32_t kb = rp2040.getFreeHeap() / 1024;
    if (kb > 255) kb = 255;
    return (uint8_t)kb;
}

bool begin(const AdvConfig& cfg) {
    if (g_state != ADV_IDLE) end();

    g_cfg = cfg;
    if (g_cfg.max_bits < MIN_BITS)      g_cfg.max_bits = DEFAULT_MAX_BITS;
    if (g_cfg.max_bits > MAX_BITS_HARD) g_cfg.max_bits = MAX_BITS_HARD;
    if (g_cfg.noise_filter_us < 20)     g_cfg.noise_filter_us = 20;
    if (g_cfg.noise_filter_us > 500)    g_cfg.noise_filter_us = 500;

    // Cấp phát session buffer theo free heap
    if (!g_session_dur) {
        uint32_t free_kb = rp2040.getFreeHeap() / 1024;
        uint16_t cap;
        if      (free_kb > 80) cap = ADV_MAX_EDGES;
        else if (free_kb > 50) cap = 4096;
        else if (free_kb > 30) cap = 2048;
        else                   cap = 0;
        if (cap == 0) return false;

        g_session_dur = (uint32_t*)malloc(cap * sizeof(uint32_t));
        g_session_lvl = (uint8_t*)malloc(cap);
        if (!g_session_dur || !g_session_lvl) {
            if (g_session_dur) free(g_session_dur);
            if (g_session_lvl) free(g_session_lvl);
            g_session_dur = nullptr; g_session_lvl = nullptr;
            return false;
        }
        g_session_capacity = cap;
    }

    // Reset tất cả
    noInterrupts();
    g_frame_count      = 0;
    g_frame_last_us    = micros();
    g_frame_last_ms    = millis();
    g_frame_last_level = false;
    g_capturing        = false;
    interrupts();

    g_session_count = 0;
    g_valid_frames  = 0;
    g_last_valid_ms = 0;
    g_begin_ms      = millis();
    memset(&g_result, 0, sizeof(g_result));
    g_t_count = 0; g_cand_count = 0; g_final_count = 0;

    SPI.setSCK(PIN_CC_SCK);
    SPI.setTX(PIN_CC_MOSI);
    SPI.setRX(PIN_CC_MISO);
    SPI.begin();

    if (!cc_init_rx()) return false;
    cc_enter_rx();

    pinMode(PIN_CC_GDO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0), on_edge, CHANGE);
    g_capturing = true;

    g_state = ADV_CAPTURING;

    Serial.printf("[ADV] begin  noise=%luus  max_bits=%u  cap=%u\n",
                  (unsigned long)g_cfg.noise_filter_us, g_cfg.max_bits, g_session_capacity);
    return true;
}

void end() {
    if (g_capturing) {
        g_capturing = false;
        detachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0));
    }
    g_state = ADV_IDLE;
}

void stop_early() {
    if (g_state != ADV_CAPTURING) return;
    g_capturing = false;
    detachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0));

    // Xử lý frame cuối còn dở (nếu có)
    uint16_t n = reset_frame_buffer();
    if (n >= MIN_EDGES_PER_FRAME && frame_looks_decodable(n)) {
        commit_frame(n);
    }
    g_state = ADV_ANALYZING;
}

void cancel() {
    end();
}

// ---------------------------------------------------------------------------
// update() — logic chính: detect frame → validate → commit
// ---------------------------------------------------------------------------
void update() {
    if (g_state != ADV_CAPTURING) return;

    // 1) Frame kết thúc: có yên lặng > frame_gap_ms
    if (g_frame_count > 0 &&
        (millis() - g_frame_last_ms) > g_cfg.frame_gap_ms) {
        uint16_t n = reset_frame_buffer();
        if (n >= MIN_EDGES_PER_FRAME && frame_looks_decodable(n)) {
            commit_frame(n);
            Serial.printf("[ADV] frame OK   edges=%u  total=%u  frames=%u\n",
                          n, g_session_count, g_valid_frames);
        }
        // Frame không decode được → DISCARD (không tăng edges)
    }

    // 2) Frame buffer đầy — chắc user giữ nút lâu, không có gap đủ dài
    else if (g_frame_count >= FRAME_MAX - 5) {
        uint16_t n = reset_frame_buffer();
        if (n >= MIN_EDGES_PER_FRAME && frame_looks_decodable(n)) {
            commit_frame(n);
            Serial.printf("[ADV] frame OK*  edges=%u  total=%u  frames=%u\n",
                          n, g_session_count, g_valid_frames);
        }
    }

    // 3) Auto-stop nếu session buffer gần đầy
    if (g_session_count >= g_session_capacity - 50) {
        Serial.println(F("[ADV] buffer full -> auto stop"));
        stop_early();
        return;
    }

    // 4) Auto-stop nếu đã có frame hợp lệ mà 2.5s không có frame mới
    if (g_valid_frames > 0 && (millis() - g_last_valid_ms) > AUTO_STOP_MS) {
        Serial.println(F("[ADV] inactivity -> auto stop"));
        stop_early();
        return;
    }
}

// ---------------------------------------------------------------------------
// analyze() — chạy 3 pass trên session buffer
// ---------------------------------------------------------------------------
void analyze() {
    if (g_state != ADV_ANALYZING) return;

    int rssi = cc_read_rssi_dbm();
    uint16_t n = g_session_count;

    Serial.printf("[ADV] analyze: %u edges  frames=%u  noise=%luus  max_bits=%u\n",
                  n, g_valid_frames, (unsigned long)g_cfg.noise_filter_us, g_cfg.max_bits);

    if (n < MIN_EDGES_TOTAL || g_valid_frames < 1) {
        Serial.println(F("[ADV] too short"));
        g_result.valid = false;
        g_result.total_bursts = 0;
        g_result.repeats = 0;
        g_state = ADV_DONE;
        return;
    }

    pass1_find_T();
    Serial.printf("[ADV] pass1: %u T candidates\n", g_t_count);
    if (g_t_count == 0) {
        g_result.valid = false; g_result.total_bursts = 0; g_result.repeats = 0;
        g_state = ADV_DONE; return;
    }

    pass2_decode();
    Serial.printf("[ADV] pass2: %u payload candidates\n", g_cand_count);
    if (g_cand_count == 0) {
        g_result.valid = false; g_result.total_bursts = 0; g_result.repeats = 0;
        g_state = ADV_DONE; return;
    }

    pass3_merge_and_pick();
    Serial.printf("[ADV] pass3: %u final clusters\n", g_final_count);

    int8_t winner = -1;
    uint16_t best_count = 0;
    for (uint8_t f = 0; f < g_final_count; f++) {
        if (g_finals[f].count < MIN_CLUSTER_SIZE) continue;
        if (g_finals[f].T_stddev_pct > T_STDDEV_MAX) continue;
        if (g_finals[f].count > best_count) {
            best_count = g_finals[f].count;
            winner = f;
        }
    }

    if (winner < 0) {
        g_result.valid = false;
        g_result.total_bursts = g_cand_count;
        g_result.repeats = 0;
        g_state = ADV_DONE;
        return;
    }

    const FinalCluster& w = g_finals[winner];
    g_result.payload        = w.payload;
    g_result.bits           = w.bits;
    g_result.T_us           = w.T_median;
    g_result.inverted       = w.inverted;
    strncpy(g_result.protocol, w.protocol, 19);
    g_result.protocol[19]   = 0;
    g_result.freq_mhz_x1000 = (uint32_t)(g_cfg.frequency_mhz * 1000.0f + 0.5f);
    g_result.rssi_dbm       = (int8_t)rssi;
    g_result.repeats        = w.count;
    g_result.total_bursts   = g_cand_count;
    g_result.valid          = (w.count >= MIN_REPEATS);

    g_state = ADV_DONE;
}

AdvState get_state()         { return g_state; }
bool     is_running()        { return g_state == ADV_CAPTURING; }
bool     is_done()           { return g_state == ADV_DONE; }

uint32_t get_elapsed_ms() {
    if (g_begin_ms == 0) return 0;
    return millis() - g_begin_ms;
}

// UI chỉ thấy edges của các frame ĐÃ VALIDATE → không auto-tăng khi không bấm
uint16_t get_edge_count()    { return g_session_count; }
uint16_t get_edge_capacity() { return g_session_capacity; }

AdvResult get_result()       { return g_result; }

} // namespace AdvCapture