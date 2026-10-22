#pragma once
#include <Arduino.h>

namespace DeBruijn {

// Cấu hình cho một lần brute-force De Bruijn
struct Config {
    float    frequency_mhz;   // 433.920 hoặc 315.000
    uint8_t  bits;            // Số bit của mã (thường 8-12 cho cổng fixed-code)
    uint16_t T_us;            // 1T = ? micro giây (thời gian của một bit)
    uint16_t repeats;         // Số lần phát toàn bộ chuỗi (mặc định 1)
    bool     inverted;        // Đảo pha (Came/Nice style)
    uint8_t  sync_d1;         // Sync phase 1 (x T)
    uint8_t  sync_d2;         // Sync phase 2 (x T)
};

// Tạo và phát De Bruijn sequence
// Trả về true nếu phát thành công
bool transmit(const Config& cfg);

// Chỉ tạo sequence, không phát (dùng để debug)
// Trả về con trỏ tới buffer nội bộ, độ dài = 2^bits
const uint8_t* generate(uint8_t bits);

// Lấy độ dài của sequence vừa tạo
uint32_t length();

// Dừng khẩn cấp (nếu đang phát)
void stop();

// Trạng thái
bool is_running();

} // namespace DeBruijn