/*
 * ============================================================================
 *  RF Listener Library — RP2040 + CC1101  v3.0
 *  v3: Fast scan (peak-hold, refine, hold hiển thị 5.5s), fix hiển thị dBm
 * ============================================================================
 */

#ifndef RF_LISTEN_H
#define RF_LISTEN_H

#include <Arduino.h>
#include <SPI.h>

#define RF_LISTEN_VERSION "3.0.0"

// ---------------------------------------------------------------------------
// Giới hạn
// ---------------------------------------------------------------------------
#define RF_LISTEN_MAX_EDGES    1024
#define RF_LISTEN_MAX_PACKETS  48
#define RF_LISTEN_CACHE_SIZE   16

// ---------------------------------------------------------------------------
// Result struct
// ---------------------------------------------------------------------------
struct RFResult {
    uint64_t payload;
    uint8_t  bits;
    uint32_t T_us;
    bool     inverted;
    char     protocol[20];
    uint32_t frequency_mhz_x1000;
    uint32_t timestamp_ms;
    int8_t   rssi_dbm;     // ★ v2: RSSI lúc capture
    uint8_t  confidence;   // ★ v2: 0-100, độ tin cậy
};

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
struct RFListenerConfig {
    uint8_t  pin_sck;            // 1
    uint8_t  pin_mosi;           // 2
    uint8_t  pin_miso;           // 3
    uint8_t  pin_csn;            // 4
    uint8_t  pin_gdo0;           // 5
    float    frequency_mhz;      // 6
    int      scan_threshold_dbm; // 7  ← -70 phải rơi vào đây (int, giữ dấu âm)
    uint32_t frame_gap_ms;       // 8
    uint32_t noise_filter_us;    // 9
    uint32_t settle_time_ms;     // 10
    bool     verbose;            // 11
    bool     agc_sensitive;      // 12
    uint8_t  min_confidence;     // 13
    uint8_t  max_bits;           // 14 ← max_bits PHẢI Ở CUỐI
};

#define RF_LISTENER_DEFAULT_CONFIG {        \
    /* sck */  2, /* mosi */ 3,             \
    /* miso */ 4, /* csn */  5,             \
    /* gdo0 */ 6,                           \
    /* freq */ 315.000f,                    \
    /* threshold */ -70,                    \
    /* frame gap */ 12,                     \
    /* noise filter */ 80,                  \
    /* settle */ 4,                         \
    /* verbose */ true,                     \
    /* agc_sensitive */ true,               \
    /* min_confidence */ 0,                 \
    /* max_bits */ 32                       \
}

// ---------------------------------------------------------------------------
// Mode
// ---------------------------------------------------------------------------
enum RFListenMode {
    RF_LISTEN_SCAN = 0,
    RF_LISTEN_CAPTURE,
    RF_LISTEN_IDLE
};

// ---------------------------------------------------------------------------
// ★ v3.1: Scan band — toàn dải (như trước) hoặc dải nhanh 433+315 MHz
// (phổ biến nhất cho remote), quét trọn 1 lượt mỗi update() để bắt được
// 1 lần bấm duy nhất thay vì phải giữ nút.
// ---------------------------------------------------------------------------
enum RFScanBand {
    RF_SCAN_BAND_ALL  = 0,   // toàn dải tần hỗ trợ (chậm hơn, bao phủ rộng)
    RF_SCAN_BAND_FAST = 1    // chỉ 433 MHz + 315 MHz (nhanh nhất, 1 lần bấm)
};

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
typedef void (*RFListenCallback)(const RFResult &result, void *user_data);

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
bool rf_listen_begin(const RFListenerConfig &cfg);
bool rf_listen_available();
void rf_listen_end();
void rf_listen_set_nonblocking(bool nb);
bool rf_listen_is_nonblocking();
void rf_listen_update();
bool rf_listen_handle_serial(Stream &in = Serial);
void rf_listen_set_callback(RFListenCallback cb, void *user_data = nullptr);

uint16_t rf_listen_get_edge_count();
uint16_t rf_listen_snapshot_recent_edges(uint32_t* dur, uint8_t* lvl, uint16_t max_n);

void rf_listen_set_mode(RFListenMode mode);
RFListenMode rf_listen_get_mode();

// ★ v3.1: chọn dải quét cho SCAN mode (gọi trước hoặc trong lúc SCAN)
void  rf_listen_set_scan_band(RFScanBand band);
RFScanBand rf_listen_get_scan_band();
bool rf_listen_is_confirmed();
float rf_listen_get_last_peak_freq();
int   rf_listen_get_last_peak_rssi();

// ★ v3: true chỉ khi peak hiện tại là 1 tín hiệu thật (>= threshold),
//        false khi đang "LISTENING..." / chỉ là nhiễu nền.
bool  rf_listen_last_peak_is_hit();
// ★ v3: trả về true đúng 1 lần ngay khi scan vừa phát hiện 1 hit mới
//        (tần số đã được khóa sẵn vào cfg, sẵn sàng chuyển CAPTURE).
bool  rf_listen_scan_hit_pop();

bool rf_listen_has_result();
RFResult rf_listen_get_result();
void rf_listen_clear_result();

void  rf_listen_set_frequency(float mhz);
float rf_listen_get_frequency();
void  rf_listen_set_threshold(int dbm);
void  rf_listen_set_frame_gap(uint32_t ms);
void  rf_listen_set_noise_filter(uint32_t us);

void rf_listen_print_status(Stream &out = Serial);
void rf_listen_print_help(Stream &out = Serial);
void rf_listen_dump_registers(Stream &out = Serial);

#endif // RF_LISTEN_H