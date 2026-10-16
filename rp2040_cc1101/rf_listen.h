/*
 * ============================================================================
 *  RF Listener Library — RP2040 + CC1101  v2.0
 *  Cải tiến: scan sensitivity cao hơn, capture thông minh hơn,
 *             multi-protocol decoder, AGC tốt hơn.
 * ============================================================================
 */

#ifndef RF_LISTEN_H
#define RF_LISTEN_H

#include <Arduino.h>
#include <SPI.h>

#define RF_LISTEN_VERSION "2.0.0"

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
    uint8_t  pin_sck;
    uint8_t  pin_mosi;
    uint8_t  pin_miso;
    uint8_t  pin_csn;
    uint8_t  pin_gdo0;
    float    frequency_mhz;
    int      scan_threshold_dbm;
    uint32_t frame_gap_ms;
    uint32_t noise_filter_us;
    uint32_t settle_time_ms;
    bool     verbose;
    // ★ v2
    bool     agc_sensitive;     // true = dùng AGC nhạy hơn cho remote yếu
    uint8_t  min_confidence;    // ngưỡng confidence tối thiểu để publish (0=tắt)
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
    /* min_confidence */ 0                  \
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
bool rf_listen_is_confirmed();
float rf_listen_get_last_peak_freq();
int   rf_listen_get_last_peak_rssi();

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
