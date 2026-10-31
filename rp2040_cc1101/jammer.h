#pragma once
#include <Arduino.h>

namespace Jammer {

enum Mode : uint8_t {
    MODE_NONE = 0,
    MODE_CW_SPOT,
    MODE_HOP_CW,
    MODE_OOK_NOISE,
    MODE_HOP_OOK,       // ★ NEW: hop qua list + OOK noise mỗi bước
    MODE_SPAM,
};

struct Config {
    Mode     mode;
    float    freq_mhz;
    const float* hop_list;
    int      hop_count;
    uint16_t dwell_ms;
    uint16_t T_us;
    uint64_t spam_payload;
    uint8_t  spam_bits;
    bool     spam_inverted;
    bool     spam_random;
    uint16_t spam_interval_ms;
    uint16_t ook_toggle_us;
    bool     ook_random;
};

bool begin(const Config& cfg);
void end();
void stop();
void tick();
bool is_running();
uint32_t get_packets_sent();
uint32_t get_elapsed_ms();
float get_current_freq();
Mode get_mode();
int get_hop_idx();
int get_hop_count();

} // namespace Jammer   