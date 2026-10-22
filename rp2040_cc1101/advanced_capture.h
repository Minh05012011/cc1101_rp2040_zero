#pragma once
#include <Arduino.h>

namespace AdvCapture {

// ---------------------------------------------------------------------------
// Config — user configurable
// ---------------------------------------------------------------------------
struct AdvConfig {
    float    frequency_mhz;
    uint32_t noise_filter_us;   // glitch filter — khớp normal mode (khuyến nghị 80)
    uint32_t frame_gap_ms;      // reserved, không dùng (auto-stop fixed 2000ms)
    uint8_t  max_bits;          // ★ v3: giới hạn bits tối đa — sẽ bị clamp [12,64]
};

// ---------------------------------------------------------------------------
// Result — kết quả phân tích
// ---------------------------------------------------------------------------
struct AdvResult {
    uint64_t payload;
    uint8_t  bits;
    uint32_t T_us;
    bool     inverted;
    char     protocol[20];
    uint32_t freq_mhz_x1000;
    int8_t   rssi_dbm;
    uint16_t repeats;           // số lần lặp của winner
    uint16_t total_bursts;      // tổng số burst đã decode được
    bool     valid;             // true nếu repeats >= MIN_REPEATS
};

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum AdvState {
    ADV_IDLE = 0,
    ADV_CAPTURING,
    ADV_ANALYZING,
    ADV_DONE
};

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
bool begin(const AdvConfig& cfg);   // false nếu không đủ RAM hoặc CC1101 fail
void end();
void update();                       // gọi mỗi loop — auto-stop
void analyze();                      // blocking ~50–500ms
void stop_early();                   // user bấm MID
void cancel();                       // user bấm LEFT

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------
AdvState get_state();
bool     is_running();
bool     is_done();
uint32_t get_elapsed_ms();
uint16_t get_edge_count();
uint16_t get_edge_capacity();
AdvResult get_result();
uint8_t  get_free_heap_kb();

} // namespace AdvCapture