#include "jammer.h"
#include "rf_replay.h"
#include "config.h"
#include <math.h>

#define GDO0_PIN PIN_CC_GDO0

// =====================================================================
// ============ TINH CHỈNH (bật bằng -D... khi compile) ================
// =====================================================================
// #define JAMMER_SWEEP        // sweep tần số ±JAM_SWEEP_RADIUS_KHZ mỗi JAM_SWEEP_STEP_US
// #define JAMMER_MAX_PA       // đặt PATABLE = 0xC0 (~+10dBm) — cần rf_replay_write_burst()
// #define JAMMER_AGGRESSIVE   // burst dài hơn, gap ngắn hơn (jam dữ hơn, tốn pin hơn)

// =====================================================================
// ============ Fast GPIO (nhanh gấp ~10x digitalWrite) ================
// =====================================================================
#if defined(ESP32)
  #define GDO0_HIGH() (GPIO.out_w1ts = (1UL << GDO0_PIN))
  #define GDO0_LOW()  (GPIO.out_w1tc = (1UL << GDO0_PIN))
#elif defined(ESP8266)
  #define GDO0_HIGH() (GPOS = (1U << GDO0_PIN))
  #define GDO0_LOW()  (GPOC = (1U << GDO0_PIN))
#else
  #define GDO0_HIGH() digitalWrite(GDO0_PIN, HIGH)
  #define GDO0_LOW()  digitalWrite(GDO0_PIN, LOW)
#endif

// =====================================================================
// ============ xorshift32 PRNG (nhanh, chất lượng tốt) ================
// =====================================================================
static uint32_t g_rng = 0x12345678u;
static inline uint32_t rnd32() {
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return g_rng = x;
}

// =====================================================================
// ============ Tham số mặc định cho noise burst =======================
// =====================================================================
#define JAM_DEFAULT_TGL_MAX_US   32     // half-period tối đa (µs); 1..32 → BW hiệu dụng ~500 kHz
#define JAM_BURST_MIN_US        800
#define JAM_BURST_MAX_US       9000
#define JAM_GAP_MIN_US           10
#define JAM_GAP_MAX_US          500
#define JAM_SWEEP_STEP_US       500     // đổi tần số mỗi 500 µs
#define JAM_SWEEP_RADIUS_KHZ    500     // ±500 kHz quanh tâm

#ifdef JAMMER_AGGRESSIVE
  #undef  JAM_BURST_MIN_US
  #undef  JAM_BURST_MAX_US
  #undef  JAM_GAP_MIN_US
  #undef  JAM_GAP_MAX_US
  #define JAM_BURST_MIN_US     2000
  #define JAM_BURST_MAX_US    15000
  #define JAM_GAP_MIN_US          0
  #define JAM_GAP_MAX_US         80
#endif

namespace Jammer {

static Config   g_cfg;
static bool     g_running      = false;
static uint32_t g_start_ms     = 0;
static uint32_t g_last_hop_ms  = 0;
static uint32_t g_last_sweep_us= 0;
static int      g_hop_idx      = 0;
static uint32_t g_packets      = 0;
static float    g_current_freq = 0;
static float    g_center_freq  = 0;
static uint32_t g_last_spam_ms = 0;
static uint32_t g_last_wd_ms   = 0;

// ---------------------------------------------------------------------
static bool tx_is_active() {
    uint8_t ms = rf_replay_get_marcstate() & 0x1F;
    return (ms == 0x13 || ms == 0x14 || ms == 0x15);
}

// ---------------------------------------------------------------------
// OOK burst ultra-wideband: half-period random 1..tgl_max (µs)
// → phổ trải rộng, không có vạch, khó bị notch filter
// ---------------------------------------------------------------------
static void ook_burst_wide(uint32_t duration_us, uint16_t tgl_max) {
    if (tgl_max < 2) tgl_max = 2;
    uint32_t end_us = micros() + duration_us;
    bool level = false;
    while ((int32_t)(end_us - micros()) > 0 && g_running) {
        level = !level;
        if (level) GDO0_HIGH(); else GDO0_LOW();
        delayMicroseconds(1 + (rnd32() % tgl_max));
    }
}

// ---------------------------------------------------------------------
// Sweep tần số (chỉ hoạt động nếu bật JAMMER_SWEEP)
// ---------------------------------------------------------------------
static void sweep_update() {
#ifdef JAMMER_SWEEP
    if (g_center_freq < 280.0f) return;
    uint32_t now = micros();
    if ((uint32_t)(now - g_last_sweep_us) < JAM_SWEEP_STEP_US) return;
    g_last_sweep_us = now;
    int32_t off_khz = (int32_t)(rnd32() % (2 * JAM_SWEEP_RADIUS_KHZ + 1)) - JAM_SWEEP_RADIUS_KHZ;
    g_current_freq = g_center_freq + (off_khz / 1000.0f);
    rf_replay_set_frequency(g_current_freq);
#else
    (void)g_last_sweep_us;
#endif
}

// ---------------------------------------------------------------------
// Một "khối" noise: chia burst thành nhiều chunk để có thể sweep giữa
// ---------------------------------------------------------------------
static void noise_chunk(uint32_t total_us, uint16_t tgl_max) {
    uint32_t done = 0;
    while (done < total_us && g_running) {
        uint32_t chunk = (total_us - done) > JAM_SWEEP_STEP_US
                       ? JAM_SWEEP_STEP_US : (total_us - done);
        ook_burst_wide(chunk, tgl_max);
        done += chunk;
        sweep_update();
    }
}

// ---------------------------------------------------------------------
#ifdef JAMMER_MAX_PA
// Kỳ vọng rf_replay module cung cấp hàm này (adapter). Nếu chưa có,
// thêm wrapper tương ứng vào rf_replay.cpp rồi bật -DJAMMER_MAX_PA.
extern void rf_replay_write_burst(uint8_t addr, const uint8_t* buf, uint8_t len);
static void set_max_pa() {
    uint8_t pa[8] = { 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    rf_replay_write_burst(0x3E /*PATABLE*/, pa, 8); // ~+10dBm @433MHz
}
#endif

// =====================================================================
// ============================ PUBLIC API =============================
// =====================================================================
bool begin(const Config& cfg) {
    if (g_running) stop();

    g_cfg          = cfg;
    g_start_ms     = millis();
    g_last_hop_ms  = g_start_ms;
    g_last_sweep_us= micros();
    g_hop_idx      = 0;
    g_packets      = 0;
    g_last_spam_ms = 0;
    g_last_wd_ms   = 0;

    // Seed PRNG từ micros + địa chỉ stack để mỗi lần chạy khác nhau
    g_rng = (uint32_t)micros() ^ ((uint32_t)(uintptr_t)&cfg * 2654435761u);
    if (g_rng == 0) g_rng = 0xDEADBEEF;
    randomSeed(g_rng);

    float target = cfg.freq_mhz;
    if ((cfg.mode == MODE_HOP_CW || cfg.mode == MODE_HOP_OOK) &&
        cfg.hop_list && cfg.hop_count > 0)
        target = cfg.hop_list[0];

    if (target < 280.0f || target > 928.0f) {
        Serial.printf("[JAM] invalid freq %.3f\n", target);
        return false;
    }

    g_center_freq = target;

    rf_replay_set_frequency(target);
    g_current_freq = target;

    if (!tx_is_active()) {
        delay(5);
        rf_replay_set_frequency(target);
        if (!tx_is_active()) {
            Serial.printf("[JAM] TX FAIL MARC=0x%02X\n", rf_replay_get_marcstate());
            return false;
        }
    }

    pinMode(GDO0_PIN, OUTPUT);
    GDO0_LOW();

#ifdef JAMMER_MAX_PA
    set_max_pa();
    Serial.println("[JAM] PA = MAX (~+10dBm)");
#endif

    if (cfg.mode == MODE_CW_SPOT || cfg.mode == MODE_HOP_CW) {
        GDO0_HIGH();
    }

    g_running = true;
    Serial.printf("[JAM] start mode=%u freq=%.3f dwell=%u hop=%d tglMax=%u"
#ifdef JAMMER_SWEEP
                  " sweep=±%dkHz"
#endif
                  "\n",
                  (unsigned)cfg.mode, target, cfg.dwell_ms, cfg.hop_count,
                  (unsigned)(cfg.ook_toggle_us ? cfg.ook_toggle_us : JAM_DEFAULT_TGL_MAX_US)
#ifdef JAMMER_SWEEP
                  , JAM_SWEEP_RADIUS_KHZ
#endif
                  );
    return true;
}

void stop() {
    if (!g_running) return;
    g_running = false;
    GDO0_LOW();
    Serial.printf("[JAM] stop pkts=%lu elapsed=%lu\n",
                  (unsigned long)g_packets,
                  (unsigned long)(millis() - g_start_ms));
}

void end() { stop(); }
bool is_running() { return g_running; }

// =====================================================================
void tick() {
    if (!g_running) return;

    uint32_t now     = millis();
    uint32_t elapsed = now - g_start_ms;

    // CW/OOK/SPAM: có giới hạn thời gian
    if ((g_cfg.mode == MODE_CW_SPOT ||
         g_cfg.mode == MODE_OOK_NOISE ||
         g_cfg.mode == MODE_SPAM) &&
        g_cfg.dwell_ms > 0 && elapsed >= g_cfg.dwell_ms) {
        stop();
        return;
    }

    switch (g_cfg.mode) {

        case MODE_CW_SPOT: {
            if (digitalRead(GDO0_PIN) != HIGH) GDO0_HIGH();
            if (now - g_last_wd_ms > 100) {
                g_last_wd_ms = now;
                if (!tx_is_active()) {
                    Serial.printf("[JAM] CW lost TX, MARC=0x%02X -> recover\n",
                                  rf_replay_get_marcstate());
                    rf_replay_set_frequency(g_current_freq);
                    GDO0_HIGH();
                }
            }
            g_packets = elapsed / 100;
            break;
        }

        case MODE_HOP_CW: {
            if (now - g_last_hop_ms >= g_cfg.dwell_ms) {
                GDO0_LOW();
                g_hop_idx = (g_hop_idx + 1) % g_cfg.hop_count;
                g_current_freq = g_cfg.hop_list[g_hop_idx];
                g_center_freq  = g_current_freq;
                rf_replay_set_frequency(g_current_freq);
                GDO0_HIGH();
                g_last_hop_ms = millis();
                g_packets++;
            }
            break;
        }

        case MODE_OOK_NOISE: {
            uint16_t tgl_max = g_cfg.ook_toggle_us ? g_cfg.ook_toggle_us
                                                   : JAM_DEFAULT_TGL_MAX_US;
            // Một chu kỳ burst: độ dài random + gap random
            uint32_t burst = JAM_BURST_MIN_US
                           + (rnd32() % (JAM_BURST_MAX_US - JAM_BURST_MIN_US + 1));
            noise_chunk(burst, tgl_max);
            uint32_t gap = JAM_GAP_MIN_US
                         + (rnd32() % (JAM_GAP_MAX_US - JAM_GAP_MIN_US + 1));
            delayMicroseconds(gap);
            g_packets = elapsed / 10;
            break;
        }

        case MODE_HOP_OOK: {
            uint16_t tgl_max = g_cfg.ook_toggle_us ? g_cfg.ook_toggle_us
                                                   : JAM_DEFAULT_TGL_MAX_US;
            uint32_t burst = JAM_BURST_MIN_US
                           + (rnd32() % (JAM_BURST_MAX_US - JAM_BURST_MIN_US + 1));
            noise_chunk(burst, tgl_max);

            if (now - g_last_hop_ms >= g_cfg.dwell_ms) {
                GDO0_LOW();
                g_hop_idx = (g_hop_idx + 1) % g_cfg.hop_count;
                g_current_freq = g_cfg.hop_list[g_hop_idx];
                g_center_freq  = g_current_freq;
                rf_replay_set_frequency(g_current_freq);
                g_last_hop_ms = millis();
                g_packets++;
            }
            break;
        }

        case MODE_SPAM: {
            uint16_t interval = g_cfg.spam_interval_ms ? g_cfg.spam_interval_ms : 45;
            if (g_last_spam_ms == 0 || (now - g_last_spam_ms) >= interval) {
                uint64_t payload = g_cfg.spam_random
                                 ? (uint64_t)random(0x1000000)
                                 : g_cfg.spam_payload;
                uint8_t  bits    = g_cfg.spam_bits ? g_cfg.spam_bits : 24;
                uint16_t T       = g_cfg.T_us      ? g_cfg.T_us      : 350;
                rf_replay_send_raw_code(payload, bits, T,
                                        g_cfg.spam_inverted, 1);
                g_last_spam_ms = millis();
                g_packets++;
            }
            break;
        }

        default: break;
    }
}

// =====================================================================
uint32_t get_packets_sent() { return g_packets; }
uint32_t get_elapsed_ms()   { return g_running ? (millis() - g_start_ms) : 0; }
float    get_current_freq() { return g_current_freq; }
Mode     get_mode()         { return g_cfg.mode; }
int      get_hop_idx()      { return g_hop_idx; }
int      get_hop_count()    { return g_cfg.hop_count; }

} // namespace Jammer