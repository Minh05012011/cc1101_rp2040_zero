#pragma once
#include <Arduino.h>

namespace RawCapture {

#define RAW_MAX_EDGES      16384
#define RAW_MAX_FRAMES     64     
#define RAW_FRAME_MAX      2048

struct Config {
    float    frequency_mhz        = 315.0f;
    uint32_t noise_filter_us      = 150;     // ★ FIX: 70 → 150 (lọc noise nền)
    uint32_t frame_gap_ms         = 10;      // ★ FIX: 20 → 10 (tách frame nhanh hơn)
    int      rssi_threshold_dbm   = -95;
    uint16_t max_frame_edges      = 200;     // ★ FIX: chốt khi đủ 200 edges
    uint32_t max_frame_duration_us= 300000;  // ★ FIX: chốt nếu frame > 300ms
};

struct Result {
    bool     valid;
    uint16_t total_edges;
    uint16_t frame_count;
    uint32_t freq_mhz_x1000;
    int8_t   rssi_dbm;
    uint16_t best_frame_start;
    uint16_t best_frame_count;
};

enum State {
    RC_IDLE = 0,
    RC_CAPTURING,
    RC_ANALYZING,
    RC_DONE
};

bool begin(const Config& cfg);
void end();
void stop_early();
void cancel();
void update();
void analyze();

State    get_state();
bool     is_running();
bool     is_done();
uint32_t get_elapsed_ms();
uint16_t get_edge_count();
uint16_t get_edge_capacity();
uint16_t get_frame_count();

Result get_result();
int    get_live_rssi_dbm();

uint16_t get_best_frame_edges(uint32_t* dur_out, uint8_t* lvl_out,
                              uint16_t max_n);
                              

uint16_t get_session_edges(uint32_t* dur_out, uint8_t* lvl_out,
                           uint16_t max_n);

} // namespace RawCapture