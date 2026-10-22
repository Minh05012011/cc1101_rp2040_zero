#include "debruijn.h"
#include "rf_replay.h"
#include "hardware.h"
#include <math.h>
#include <string.h>

namespace DeBruijn {

static uint8_t*  g_seq      = nullptr;
static uint32_t  g_seq_len  = 0;
static uint8_t   g_bits     = 0;
static bool      g_running  = false;

// ---------------------------------------------------------------------------
// Thuật toán tạo De Bruijn sequence (FKM algorithm — lexicographically smallest)
// Tham khảo: https://en.wikipedia.org/wiki/De_Bruijn_sequence
// ---------------------------------------------------------------------------
static void db_recursive(uint8_t* a, uint32_t n, uint32_t k,
                         uint32_t* seq, uint32_t* seq_len) {
    auto db = [&](uint32_t t, uint32_t p, auto&& self) -> void {
        if (t > n) {
            if (n % p == 0) {
                for (uint32_t j = 1; j <= p; j++) {
                    seq[(*seq_len)++] = a[j];
                }
            }
        } else {
            a[t] = a[t - p];
            self(t + 1, p, self);
            for (uint8_t j = a[t - p] + 1; j < k; j++) {
                a[t] = j;
                self(t + 1, t, self);
            }
        }
    };
    db(1, 1, db);
}

const uint8_t* generate(uint8_t bits) {
    if (bits < 1 || bits > 16) return nullptr;  // Giới hạn an toàn

    uint32_t n = 1u << bits;  // 2^bits
    if (g_seq && g_bits == bits) {
        return g_seq;  // Cache
    }

    if (g_seq) { free(g_seq); g_seq = nullptr; }

    g_seq = (uint8_t*)malloc(n + bits);  // + bits cho wrap-around
    if (!g_seq) return nullptr;

    uint8_t  a[17] = {0};
    uint32_t seq_len = 0;

    db_recursive(a, bits, 2, (uint32_t*)g_seq, &seq_len);

    // De Bruijn sequence có độ dài 2^bits, nhưng FKM tạo ra 2^bits - bits + ...
    // Thực tế seq_len sẽ xấp xỉ 2^bits. Đảm bảo đủ.
    if (seq_len < n) {
        // Nối thêm phần đầu để đủ wrap-around
        for (uint32_t i = 0; i < n - seq_len; i++) {
            g_seq[seq_len + i] = g_seq[i];
        }
        seq_len = n;
    }

    g_seq_len = seq_len;
    g_bits = bits;
    return g_seq;
}

uint32_t length() { return g_seq_len; }

// ---------------------------------------------------------------------------
// Phát De Bruijn sequence qua CC1101
// ---------------------------------------------------------------------------
bool transmit(const Config& cfg) {
    if (cfg.bits < 1 || cfg.bits > 16) return false;

    const uint8_t* seq = generate(cfg.bits);
    if (!seq || g_seq_len == 0) return false;

    // Khởi tạo RF Replay ở TX mode
    RFReplayConfig rcfg = RF_REPLAY_DEFAULT_CONFIG;
    rcfg.frequency_mhz = cfg.frequency_mhz;
    rcfg.verbose = false;
    if (!rf_replay_begin(rcfg)) return false;

    // Đảm bảo đúng tần số
    rf_replay_set_frequency(cfg.frequency_mhz);

    g_running = true;

    Serial.printf("[DeBruijn] TX %u-bit, len=%lu, T=%uus @ %.3fMHz\n",
                  cfg.bits, (unsigned long)g_seq_len,
                  cfg.T_us, cfg.frequency_mhz);

    uint32_t t_start = millis();

    for (uint16_t r = 0; r < cfg.repeats && g_running; r++) {
        for (uint32_t i = 0; i < g_seq_len && g_running; i++) {
            // Phát một bit: nếu là 1 → HIGH, nếu là 0 → LOW
            // Mỗi bit kéo dài T_us
            if (seq[i]) {
                rf_replay_transmit_bit_high(cfg.T_us);
            } else {
                rf_replay_transmit_bit_low(cfg.T_us);
            }

            // Yield để không block watchdog
            if ((i & 0x3FF) == 0) {
                delay(1);
            }
        }

        // Gap giữa các lần lặp
        if (r + 1 < cfg.repeats) {
            delay(10);
        }
    }

    uint32_t elapsed = millis() - t_start;
    Serial.printf("[DeBruijn] Done in %lu ms\n", (unsigned long)elapsed);

    g_running = false;
    return true;
}

void stop() {
    g_running = false;
}

bool is_running() {
    return g_running;
}

} // namespace DeBruijn