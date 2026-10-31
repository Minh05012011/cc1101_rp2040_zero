#include "rf_raw_capture.h"
#include "config.h"
#include <SPI.h>

// -----------------------------------------------------------------------------
// THANH GHI CC1101
// -----------------------------------------------------------------------------
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
#define CC_OSC_FREQ   26000000.0

namespace RawCapture {

static Config g_cfg;
static State  g_state = RC_IDLE;

// Runtime tuning
static uint32_t g_glitch_filter_us  = 150;
static uint32_t g_gap_timeout_us    = 12000;
static int      g_rssi_threshold    = -95;
static uint16_t g_min_frame_pulses  = 16;

// ISR active frame buffer
static uint32_t          g_frame_dur[RAW_FRAME_MAX];
static uint8_t           g_frame_lvl[RAW_FRAME_MAX];
static volatile uint16_t g_isr_count        = 0;
static volatile uint32_t g_isr_last_us      = 0;
static volatile bool     g_isr_last_level   = false;
static volatile bool     g_frame_active     = false;
static volatile bool     g_capturing        = false;
static volatile uint32_t g_isr_calls        = 0;

// Buffer copy an toàn ngoài ngắt để xử lý
static uint32_t g_commit_dur[RAW_FRAME_MAX];
static uint8_t  g_commit_lvl[RAW_FRAME_MAX];

// Session buffer (toàn bộ các frame hợp lệ bắt được trong phiên)
static uint32_t* g_session_dur      = nullptr;
static uint8_t*  g_session_lvl      = nullptr;
static uint16_t  g_session_capacity = 0;
static uint16_t  g_session_count    = 0;

struct FrameMark { uint16_t start; uint16_t count; int rssi; };
static FrameMark g_marks[RAW_MAX_FRAMES];
static uint16_t  g_mark_count = 0;

static Result    g_result = {};
static uint32_t  g_begin_ms = 0;
static uint32_t  g_last_valid_ms = 0;

static SPISettings g_spi(2000000, MSBFIRST, SPI_MODE0);

static inline void cs_low()  { digitalWrite(PIN_CC_CSN, LOW);  delayMicroseconds(2); }
static inline void cs_high() { digitalWrite(PIN_CC_CSN, HIGH); delayMicroseconds(2); }

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

static void cc_set_frequency(float mhz) {
    if (mhz <= 0) return;
    uint32_t freq_reg = (uint32_t)(((double)mhz * 1000000.0 * 65536.0) / CC_OSC_FREQ + 0.5);
    cc_write(CC_FREQ2, (freq_reg >> 16) & 0xFF);
    cc_write(CC_FREQ1, (freq_reg >> 8)  & 0xFF);
    cc_write(CC_FREQ0, freq_reg & 0xFF);
}

static int cc_read_rssi_dbm() {
    uint8_t raw = cc_read_status(CC_RSSI);
    int rssi_dbm;
    if (raw >= 128) {
        rssi_dbm = ((int8_t)(raw - 256) / 2) - 74;
    } else {
        rssi_dbm = (raw / 2) - 74;
    }
    return rssi_dbm;
}

// -----------------------------------------------------------------------------
// Khởi tạo CC1101 — cấu hình GIỐNG HỆT rf_listen.cpp (đã verify chạy được)
// ★ BW = 812 kHz (MDMCFG4 = 0x07) — bao trùm dung sai tần số của remote
// -----------------------------------------------------------------------------
static bool cc_init_rx(float target_freq_mhz) {
    pinMode(PIN_CC_CSN, OUTPUT);
    digitalWrite(PIN_CC_CSN, HIGH);
    pinMode(PIN_CC_GDO0, INPUT);

    cs_high(); delayMicroseconds(100);
    cs_low();  delayMicroseconds(50);
    cs_high(); delayMicroseconds(100);

    cc_strobe(CC_SRES);
    delay(20);

    uint8_t version = cc_read_status(CC_VERSION);
    if (version == 0x00 || version == 0xFF) {
        Serial.printf("[RAW] Lỗi CC1101! Version=0x%02X\n", version);
        return false;
    }

    // GDO0 = Asynchronous Serial Data out (0x0D) — GIỐNG rf_listen
    cc_write(CC_IOCFG0,   0x0D);
    cc_write(CC_IOCFG2,   0x0D);   // ★ 0x2E → 0x0D (giống rf_listen)
    cc_write(CC_IOCFG1,   0x2E);

    cc_write(CC_FIFOTHR,  0x47);
    cc_write(CC_SYNC1,    0xD3);
    cc_write(CC_SYNC0,    0x91);
    cc_write(CC_PKTLEN,   0xFF);
    cc_write(CC_PKTCTRL1, 0x04);
    cc_write(CC_PKTCTRL0, 0x32);

    cc_write(CC_CHANNR,   0x00);
    cc_write(CC_FSCTRL1,  0x06);
    cc_write(CC_FSCTRL0,  0x00);

    cc_set_frequency(target_freq_mhz);

    // ★★★ FIX CHÍNH: BW = 812 kHz (MDMCFG4 = 0x07) — GIỐNG rf_listen
    //     Bản cũ dùng 0x57 (BW 325 kHz) → remote 314.x lọt ngoài filter
    cc_write(CC_MDMCFG4,  0x07);   // CHANBW=00 (812kHz), DRATE_E=7
    cc_write(CC_MDMCFG3,  0x83);   // DRATE_M=131 → ~4.8 kBaud
    cc_write(CC_MDMCFG2,  0x30);   // OOK, no Manchester, no sync check
    cc_write(CC_MDMCFG1,  0x00);
    cc_write(CC_MDMCFG0,  0x00);
    cc_write(CC_DEVIATN,  0x00);

    cc_write(CC_MCSM2,    0x07);
    cc_write(CC_MCSM1,    0x30);
    cc_write(CC_MCSM0,    0x18);
    cc_write(CC_FOCCFG,   0x18);
    cc_write(CC_BSCFG,    0x6C);

    // AGC sensitive — giống rf_listen
    cc_write(CC_AGCCTRL2, 0x03);
    cc_write(CC_AGCCTRL1, 0x00);
    cc_write(CC_AGCCTRL0, 0x91);

    cc_write(CC_FREND1,   0x56);
    cc_write(CC_FREND0,   0x11);
    cc_write(CC_FSCAL3,   0xE9);
    cc_write(CC_FSCAL2,   0x2A);
    cc_write(CC_FSCAL1,   0x00);
    cc_write(CC_FSCAL0,   0x1F);
    cc_write(CC_RCCTRL1,  0x41);
    cc_write(CC_RCCTRL0,  0x00);
    cc_write(CC_TEST2,    0x81);
    cc_write(CC_TEST1,    0x35);
    cc_write(CC_TEST0,    0x09);

    uint8_t pa_table[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc_write_burst(CC_PATABLE, pa_table, 8);

    cc_strobe(CC_SIDLE);
    delayMicroseconds(500);
    cc_strobe(CC_SFRX);
    delayMicroseconds(200);
    cc_strobe(CC_SRX);
    delay(10);

    return true;
}

// -----------------------------------------------------------------------------
// ISR NGẮT CẠNH XUNG TRÊN GDO0
// -----------------------------------------------------------------------------
static void __not_in_flash_func(on_edge)() {
    if (!g_capturing) return;
    uint32_t now_us = micros();
    uint32_t dt = now_us - g_isr_last_us;
    g_isr_last_us = now_us;
    g_isr_calls++;

    // Glitch filter
    if (dt < g_glitch_filter_us) return;

    bool current_level = digitalRead(PIN_CC_GDO0);
    uint8_t finished_level = g_isr_last_level ? 1 : 0;
    g_isr_last_level = current_level;

    if (!g_frame_active) {
        g_frame_active = true;
        g_isr_count = 0;
    }

    if (g_isr_count < RAW_FRAME_MAX) {
        g_frame_dur[g_isr_count] = dt;
        g_frame_lvl[g_isr_count] = finished_level;
        g_isr_count++;
    }
}

// -----------------------------------------------------------------------------
// ★ Estimate T từ histogram pulse 200-2000µs
//   Trả về 0 nếu không đủ pulse để estimate
// -----------------------------------------------------------------------------
static uint16_t estimate_T(const uint32_t* dur, uint16_t n) {
    uint16_t buckets[36] = {0};   // 200..2000µs, step 50
    for (uint16_t i = 0; i < n; i++) {
        uint32_t d = dur[i];
        if (d >= 200 && d <= 2000) {
            uint16_t idx = (d - 200) / 50;
            if (idx < 36) buckets[idx]++;
        }
    }
    uint16_t best_idx = 0, best_cnt = 0;
    for (uint16_t i = 0; i < 36; i++) {
        if (buckets[i] > best_cnt) { best_cnt = buckets[i]; best_idx = i; }
    }
    if (best_cnt < 6) return 0;
    return (uint16_t)(200 + best_idx * 50 + 25);
}

// -----------------------------------------------------------------------------
// IN PHÂN TÍCH FRAME
// -----------------------------------------------------------------------------
static void print_frame_analysis(const uint32_t* dur, const uint8_t* lvl, uint16_t count, int rssi) {
    if (count < 2) return;

    uint64_t total_duration_us = 0;
    uint32_t min_dur = 0xFFFFFFFF;
    uint32_t max_dur = 0;
    uint16_t high_pulses = 0, low_pulses = 0;

    for (uint16_t i = 0; i < count; i++) {
        uint32_t d = dur[i];
        total_duration_us += d;
        if (d < min_dur) min_dur = d;
        if (d > max_dur) max_dur = d;
        if (lvl[i]) high_pulses++; else low_pulses++;
    }

    uint16_t T_est = estimate_T(dur, count);

    Serial.println();
    Serial.println(F("================================================================================"));
    Serial.printf(" [RAW FRAME #%u] @ %.3f MHz\n",
                  g_mark_count, g_cfg.frequency_mhz);
    Serial.println(F("================================================================================"));
    Serial.printf("  - RSSI: %d dBm (Ngưỡng: %d dBm)\n", rssi, g_rssi_threshold);
    Serial.printf("  - Tổng xung: %u (HIGH:%u LOW:%u)\n", count, high_pulses, low_pulses);
    Serial.printf("  - Thời lượng: %.2f ms\n", total_duration_us / 1000.0f);
    Serial.printf("  - Min/Max: %lu / %lu µs\n",
                  (unsigned long)min_dur, (unsigned long)max_dur);
    Serial.printf("  - T ước tính: %u µs\n", T_est);
    Serial.println(F("--------------------------------------------------------------------------------"));

    Serial.print(F(" [1] RAW: "));
    for (uint16_t i = 0; i < count && i < 80; i++) {
        if (lvl[i]) Serial.printf("+%lu", (unsigned long)dur[i]);
        else        Serial.printf("-%lu", (unsigned long)dur[i]);
        if (i < count - 1 && i < 79) Serial.print(F(", "));
        if ((i + 1) % 12 == 0 && i < count - 1 && i < 79) Serial.print(F("\n     "));
    }
    if (count > 80) Serial.print(F(" ..."));
    Serial.println();

    Serial.print(F(" [2] uint32_t raw_timings[] = { "));
    for (uint16_t i = 0; i < count; i++) {
        Serial.printf("%lu", (unsigned long)dur[i]);
        if (i < count - 1) Serial.print(F(", "));
    }
    Serial.println(F(" };"));
    Serial.println(F("================================================================================\n"));
}

// -----------------------------------------------------------------------------
// LƯU 1 FRAME — có reject ambient dựa vào T estimate
// -----------------------------------------------------------------------------
static void commit_valid_frame(const uint32_t* dur, const uint8_t* lvl, uint16_t n, int rssi) {
    if (g_mark_count >= RAW_MAX_FRAMES) return;

    // Bỏ xung đầu nếu là gap mở frame (>20ms)
    uint16_t start_idx = (n > 2 && dur[0] > 20000) ? 1 : 0;
    uint16_t effective_n = n - start_idx;

    if (effective_n < g_min_frame_pulses) return;

    // ★ REJECT AMBIENT: T < 250 hoặc > 900 → gần như chắc chắn không phải remote
    uint16_t T_est = estimate_T(&dur[start_idx], effective_n);
    if (T_est > 0 && (T_est < 250 || T_est > 900)) {
        // Silent reject — weather station T=125µs sẽ bị chặn ở đây
        return;
    }

    // Cắt giảm nếu session buffer sắp đầy
    if (g_session_count + effective_n > g_session_capacity) {
        effective_n = g_session_capacity - g_session_count;
        if (effective_n < g_min_frame_pulses) return;
    }

    g_marks[g_mark_count].start = g_session_count;
    g_marks[g_mark_count].count = effective_n;
    g_marks[g_mark_count].rssi  = rssi;
    g_mark_count++;

    for (uint16_t i = 0; i < effective_n; i++) {
        g_session_dur[g_session_count] = dur[start_idx + i];
        g_session_lvl[g_session_count] = lvl[start_idx + i];
        g_session_count++;
    }
    g_last_valid_ms = millis();

    print_frame_analysis(&dur[start_idx], &lvl[start_idx], effective_n, rssi);
}

// -----------------------------------------------------------------------------
// PUBLIC API
// -----------------------------------------------------------------------------
bool begin(const Config& cfg) {
    if (g_state != RC_IDLE) end();
    g_cfg = cfg;

    g_glitch_filter_us = (g_cfg.noise_filter_us >= 20 && g_cfg.noise_filter_us <= 500)
                         ? g_cfg.noise_filter_us : 150;

    uint32_t gap_ms = (g_cfg.frame_gap_ms >= 4 && g_cfg.frame_gap_ms <= 100)
                      ? g_cfg.frame_gap_ms : 12;
    g_gap_timeout_us = gap_ms * 1000;

    g_rssi_threshold = -95;
    g_min_frame_pulses = 16;

    if (!g_session_dur) {
        uint32_t free_kb = rp2040.getFreeHeap() / 1024;
        uint16_t cap;
        if      (free_kb > 150) cap = RAW_MAX_EDGES;
        else if (free_kb > 100) cap = 8192;
        else if (free_kb > 60)  cap = 4096;
        else if (free_kb > 30)  cap = 2048;
        else                    cap = 0;

        if (cap == 0) {
            Serial.println(F("[RAW] Không đủ bộ nhớ heap!"));
            return false;
        }

        g_session_dur = (uint32_t*)malloc(cap * sizeof(uint32_t));
        g_session_lvl = (uint8_t*)malloc(cap);
        if (!g_session_dur || !g_session_lvl) {
            if (g_session_dur) free(g_session_dur);
            if (g_session_lvl) free(g_session_lvl);
            g_session_dur = nullptr; g_session_lvl = nullptr;
            Serial.println(F("[RAW] malloc thất bại!"));
            return false;
        }
        g_session_capacity = cap;
    }

    g_session_count = 0;
    g_mark_count = 0;
    g_isr_calls = 0;
    g_begin_ms = millis();
    g_last_valid_ms = 0;
    memset(&g_result, 0, sizeof(g_result));

    noInterrupts();
    g_isr_count      = 0;
    g_frame_active   = false;
    g_isr_last_level = digitalRead(PIN_CC_GDO0);
    g_isr_last_us    = micros();
    interrupts();

    SPI.setSCK(PIN_CC_SCK);
    SPI.setTX(PIN_CC_MOSI);
    SPI.setRX(PIN_CC_MISO);
    SPI.begin();

    if (!cc_init_rx(g_cfg.frequency_mhz)) {
        Serial.println(F("[RAW] CC1101 init RX FAIL!"));
        return false;
    }

    attachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0), on_edge, CHANGE);
    g_capturing = true;
    g_state = RC_CAPTURING;

    Serial.printf("[RAW] Sẵn sàng @ %.3f MHz (Glitch:%luµs, Gap:%luµs, RSSI≥%ddBm)\n",
                  g_cfg.frequency_mhz,
                  (unsigned long)g_glitch_filter_us,
                  (unsigned long)g_gap_timeout_us,
                  g_rssi_threshold);
    return true;
}

void end() {
    if (g_capturing) {
        g_capturing = false;
        detachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0));
    }
    cc_strobe(CC_SIDLE);
    g_state = RC_IDLE;
}

void stop_early() {
    if (g_state != RC_CAPTURING) return;
    g_capturing = false;
    detachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0));

    noInterrupts();
    uint16_t n = g_isr_count;
    if (n > RAW_FRAME_MAX) n = RAW_FRAME_MAX;
    memcpy(g_commit_dur, g_frame_dur, n * sizeof(uint32_t));
    memcpy(g_commit_lvl, g_frame_lvl, n);
    g_isr_count = 0;
    g_frame_active = false;
    interrupts();

    if (n >= g_min_frame_pulses) {
        int rssi = cc_read_rssi_dbm();
        commit_valid_frame(g_commit_dur, g_commit_lvl, n, rssi);
    }

    g_state = RC_ANALYZING;
}

void cancel() { end(); }

void update() {
    if (g_state != RC_CAPTURING) return;

    uint32_t now_us = micros();

    // Chốt frame khi gap > timeout
    if (g_frame_active && (now_us - g_isr_last_us > g_gap_timeout_us)) {
        noInterrupts();
        uint16_t count = g_isr_count;
        g_frame_active = false;
        g_isr_count = 0;
        uint16_t commit_n = 0;
        if (count >= g_min_frame_pulses && count <= RAW_FRAME_MAX) {
            memcpy(g_commit_dur, g_frame_dur, count * sizeof(uint32_t));
            memcpy(g_commit_lvl, g_frame_lvl, count);
            commit_n = count;
        }
        interrupts();

        if (commit_n >= g_min_frame_pulses) {
            int measured_rssi = cc_read_rssi_dbm();
            if (g_rssi_threshold == 0 || measured_rssi >= g_rssi_threshold) {
                commit_valid_frame(g_commit_dur, g_commit_lvl, commit_n, measured_rssi);
            }
        }
    }

    if (g_session_count >= g_session_capacity - 100 || g_mark_count >= RAW_MAX_FRAMES) {
        Serial.println(F("[RAW] Session đầy → stop."));
        stop_early();
        return;
    }

    if (g_mark_count > 0 && (millis() - g_last_valid_ms) > 15000) {
        Serial.println(F("[RAW] Hết 15s chờ → stop."));
        stop_early();
        return;
    }
}

void analyze() {
    if (g_state != RC_ANALYZING) return;

    g_result.valid = (g_mark_count > 0 && g_session_count > 0);
    g_result.total_edges = g_session_count;
    g_result.frame_count = g_mark_count;
    g_result.freq_mhz_x1000 = (uint32_t)(g_cfg.frequency_mhz * 1000.0f + 0.5f);
    g_result.rssi_dbm = (int8_t)cc_read_rssi_dbm();

    if (g_mark_count > 0) {
        // ★ Chọn frame có T trong dải EV1527 và nhiều xung nhất
        uint16_t best = 0;
        uint16_t best_score = 0;
        for (uint16_t i = 0; i < g_mark_count; i++) {
            uint16_t T = estimate_T(&g_session_dur[g_marks[i].start], g_marks[i].count);
            bool t_ok = (T >= 250 && T <= 900);
            uint16_t score = g_marks[i].count + (t_ok ? 500 : 0);
            if (score > best_score) { best_score = score; best = i; }
        }
        g_result.best_frame_start = g_marks[best].start;
        g_result.best_frame_count = g_marks[best].count;
        g_result.rssi_dbm         = (int8_t)g_marks[best].rssi;
    }

    Serial.println(F("\n[RAW] ============ KẾT QUẢ ============"));
    Serial.printf("[RAW] Tần số     : %.3f MHz\n", g_cfg.frequency_mhz);
    Serial.printf("[RAW] RSSI       : %d dBm\n", g_result.rssi_dbm);
    Serial.printf("[RAW] Số frame   : %u\n", g_mark_count);
    Serial.printf("[RAW] Tổng xung  : %u\n", g_session_count);
    Serial.printf("[RAW] Best frame : %u xung\n", g_result.best_frame_count);
    Serial.println(F("[RAW] ==================================\n"));

    g_state = RC_DONE;
}

State    get_state()          { return g_state; }
bool     is_running()        { return g_state == RC_CAPTURING; }
bool     is_done()           { return g_state == RC_DONE; }
uint32_t get_elapsed_ms()    { return g_begin_ms ? millis() - g_begin_ms : 0; }
uint16_t get_edge_count()    { return g_session_count; }
uint16_t get_edge_capacity() { return g_session_capacity; }
uint16_t get_frame_count()   { return g_mark_count; }
Result   get_result()        { return g_result; }
int      get_live_rssi_dbm() { return cc_read_rssi_dbm(); }

uint16_t get_best_frame_edges(uint32_t* dur_out, uint8_t* lvl_out, uint16_t max_n) {
    if (!dur_out || !lvl_out || max_n == 0) return 0;
    if (g_mark_count == 0) return 0;

    // Chọn frame tốt nhất (ưu tiên T đúng dải)
    uint16_t best = 0;
    uint16_t best_score = 0;
    for (uint16_t i = 0; i < g_mark_count; i++) {
        uint16_t T = estimate_T(&g_session_dur[g_marks[i].start], g_marks[i].count);
        bool t_ok = (T >= 250 && T <= 900);
        uint16_t score = g_marks[i].count + (t_ok ? 500 : 0);
        if (score > best_score) { best_score = score; best = i; }
    }

    uint16_t n = g_marks[best].count;
    if (n > max_n) n = max_n;
    uint16_t start = g_marks[best].start;
    for (uint16_t i = 0; i < n; i++) {
        dur_out[i] = g_session_dur[start + i];
        lvl_out[i] = g_session_lvl[start + i];
    }
    return n;
}

uint16_t get_session_edges(uint32_t* dur_out, uint8_t* lvl_out, uint16_t max_n) {
    if (!dur_out || !lvl_out || max_n == 0) return 0;
    uint16_t n = g_session_count;
    if (n > max_n) n = max_n;
    for (uint16_t i = 0; i < n; i++) {
        dur_out[i] = g_session_dur[i];
        lvl_out[i] = g_session_lvl[i];
    }
    return n;
}

} // namespace RawCapture