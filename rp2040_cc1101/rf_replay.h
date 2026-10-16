/*
 * ============================================================================
 *  RF Replay Library — RP2040 + CC1101
 *  ---------------------------------------------------------------------------
 *  Giao tiếp với CC1101 qua SPI thủ công (không dùng ELECHOUSE driver).
 *  Hỗ trợ phát lại các protocol OOK dạng PWM (EV1527, PT2262, Came, Nice...).
 *
 *  Cách dùng cơ bản:
 *      RFReplayConfig cfg = RF_REPLAY_DEFAULT_CONFIG;
 *      rf_replay_begin(cfg);
 *      rf_replay_bind_button(9, 0);     // nút GP9 -> lệnh #0
 *      rf_replay_update();              // gọi trong loop()
 *
 *  Thêm protocol / lệnh / nút mới: xem các hàm rf_replay_add_* bên dưới.
 * ============================================================================
 */

#ifndef RF_REPLAY_H
#define RF_REPLAY_H

#include <Arduino.h>
#include <SPI.h>
#include <vector>

// ---------------------------------------------------------------------------
// Version
// ---------------------------------------------------------------------------
#define RF_REPLAY_VERSION   "1.0.0"

// ---------------------------------------------------------------------------
// Giới hạn kích thước (có thể tăng, RP2040 dư RAM)
// ---------------------------------------------------------------------------
#define RF_REPLAY_MAX_PROTOCOLS   16
#define RF_REPLAY_MAX_COMMANDS    32
#define RF_REPLAY_MAX_BUTTONS     12
#define RF_REPLAY_NAME_LEN        20

// ---------------------------------------------------------------------------
// Error codes
// ---------------------------------------------------------------------------
enum RFReplayError : uint8_t {
    RF_OK = 0,
    RF_ERR_NOT_INIT,
    RF_ERR_NO_CHIP,
    RF_ERR_TABLE_FULL,
    RF_ERR_NOT_FOUND,
    RF_ERR_INVALID_PARAM,
    RF_ERR_BUSY,
    RF_ERR_LAST = RF_ERR_BUSY
};

// ---------------------------------------------------------------------------
// Protocol definition — data-driven
// ---------------------------------------------------------------------------
// Timing mỗi phase tính theo đơn vị T (T_us).
//   d > 0  → mức HIGH trong d*T_us
//   d < 0  → mức LOW  trong |d|*T_us
// Một symbol = 2 phase liên tiếp (d1 rồi d2).
// ---------------------------------------------------------------------------
struct RFProtocolDef {
    char     name[RF_REPLAY_NAME_LEN];
    uint8_t  bits;            // số bit payload
    uint16_t T_us;            // 1T = ? micro giây
    int16_t  sync_d1;         // sync phase 1 (vd +1 = 1T HIGH)
    int16_t  sync_d2;         // sync phase 2 (vd -31 = 31T LOW)
    int16_t  bit0_d1;         // bit '0' phase 1
    int16_t  bit0_d2;         // bit '0' phase 2
    int16_t  bit1_d1;         // bit '1' phase 1
    int16_t  bit1_d2;         // bit '1' phase 2
    bool     msb_first;       // true = phát bit cao trước
};

// ---------------------------------------------------------------------------
// Command definition
// ---------------------------------------------------------------------------
struct RFCommand {
    char     name[RF_REPLAY_NAME_LEN];
    uint64_t payload;         // mã (đã mask theo bits)
    uint8_t  protocol_idx;    // index vào bảng protocols
    uint8_t  repeats;         // số packet mỗi lần phát
};

// ---------------------------------------------------------------------------
// Button binding
// ---------------------------------------------------------------------------
struct RFButtonBinding {
    uint8_t pin;
    int8_t  cmd_idx;          // -1 = chưa gán
    // State cho debounce
    bool          stable;
    bool          last_reading;
    unsigned long last_change_ms;
    unsigned long press_start_ms;
};

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
struct RFReplayConfig {
    uint8_t  pin_sck;
    uint8_t  pin_mosi;
    uint8_t  pin_miso;
    uint8_t  pin_csn;
    uint8_t  pin_gdo0;
    float    frequency_mhz;
    uint16_t button_debounce_ms;
    bool     verbose;         // in log ra Serial
};

#define RF_REPLAY_DEFAULT_CONFIG {                    \
    /* sck */ 2, /* mosi */ 3, /* miso */ 4,          \
    /* csn */ 5, /* gdo0 */ 6,                        \
    /* freq */ 315.000f,                              \
    /* debounce */ 30,                                \
    /* verbose */ true                                \
}

// ---------------------------------------------------------------------------
// API chính
// ---------------------------------------------------------------------------

// Khởi tạo SPI + CC1101, nạp các protocol/lệnh mặc định.
// Trả về false nếu không tìm thấy CC1101.
bool rf_replay_begin(const RFReplayConfig& cfg);

// Chip có sẵn sàng không?
bool rf_replay_available();

// Lấy mã lỗi cuối
RFReplayError rf_replay_last_error();
const char*   rf_replay_error_str(RFReplayError err);

// ---------------------------------------------------------------------------
// Cấu hình tần số
// ---------------------------------------------------------------------------
void  rf_replay_set_frequency(float mhz);
float rf_replay_get_frequency();

// Ghi trực tiếp 3 thanh ghi FREQ (dùng khi cần tinh chỉnh thủ công)
void  rf_replay_set_frequency_raw(uint8_t f2, uint8_t f1, uint8_t f0);

// ---------------------------------------------------------------------------
// Quản lý protocols
// ---------------------------------------------------------------------------
int  rf_replay_add_protocol(const RFProtocolDef& proto);   // trả về index hoặc -1
int  rf_replay_find_protocol(const char* name);
const RFProtocolDef* rf_replay_get_protocol(int idx);
size_t rf_replay_protocol_count();

// ---------------------------------------------------------------------------
// Quản lý commands
// ---------------------------------------------------------------------------
int  rf_replay_add_command(const RFCommand& cmd);          // trả về index hoặc -1
int  rf_replay_find_command(const char* name);
bool rf_replay_remove_command(const char* name);
const RFCommand* rf_replay_get_command(int idx);
size_t rf_replay_command_count();

// ---------------------------------------------------------------------------
// Button binding
// ---------------------------------------------------------------------------
bool rf_replay_bind_button(uint8_t pin, int8_t cmd_idx);
bool rf_replay_unbind_button(uint8_t pin);
void rf_replay_clear_buttons();

// ---------------------------------------------------------------------------
// Phát
// ---------------------------------------------------------------------------
bool rf_replay_send(int cmd_idx);                    // phát 1 lệnh theo index
bool rf_replay_send_by_name(const char* name);       // theo tên
bool rf_replay_send_raw(const RFCommand& cmd);       // ad-hoc, không lưu bảng
uint8_t rf_replay_get_marcstate();
void    rf_replay_test_cw(uint16_t duration_ms);
bool rf_replay_send_continuous(int cmd_idx);         // phát liên tục
void rf_replay_stop_continuous();
bool rf_replay_is_continuous();
// Phát trực tiếp không cần đăng ký protocol/command
// ★ v2: send với full protocol definition (dùng cho Bruce custom timing)
bool rf_replay_send_proto(uint64_t payload, const RFProtocolDef& proto, uint16_t repeats);
bool rf_replay_send_raw_code(uint64_t payload, uint8_t bits, uint16_t T_us,
                             bool inverted, uint16_t repeats);
// ---------------------------------------------------------------------------
// Update — gọi trong loop()
// ---------------------------------------------------------------------------
// Quét tất cả nút đã bind, tự động phát khi có nút nhấn.
void rf_replay_update();

// ---------------------------------------------------------------------------
// Serial CLI (tùy chọn)
// ---------------------------------------------------------------------------
// Đọc và xử lý lệnh từ Stream (mặc định Serial).
// Trả về true nếu vừa xử lý xong 1 lệnh.
bool rf_replay_handle_serial(Stream& in = Serial);

// In trạng thái / trợ giúp
void rf_replay_print_status(Stream& out = Serial);
void rf_replay_print_help(Stream& out = Serial);
void rf_replay_print_protocols(Stream& out = Serial);
void rf_replay_print_commands(Stream& out = Serial);

// ---------------------------------------------------------------------------
// Tiện ích
// ---------------------------------------------------------------------------
// Nạp sẵn các protocol phổ biến (EV1527, PT2262, Came, Nice).
void rf_replay_load_builtin_protocols();

// Debug: đọc thanh ghi CC1101
uint8_t rf_replay_read_register(uint8_t addr);
uint8_t rf_replay_read_status(uint8_t addr);
void    rf_replay_dump_registers(Stream& out = Serial);

#endif // RF_REPLAY_H