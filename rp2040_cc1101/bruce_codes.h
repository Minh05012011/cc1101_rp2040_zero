/*
 * ============================================================================
 *  bruce_codes.h — Brute-force codes cho menu Bruce
 *
 *  5 loại cổng:
 *   0 = CAME-fast  433.92MHz  T=185µs  12bit  inv  ~224s
 *   1 = CAME       433.92MHz  T=320µs  12bit  inv  ~287s
 *   2 = Ansonic    434.075MHz T=450µs  12bit  norm ~276s
 *   3 = Holtek     433.92MHz  T=350µs  12bit  norm ~387s
 *   4 = NICE       433.92MHz  T=700µs  12bit  inv  ~628s
 *
 *  Mỗi code phát BRUCE_REPEATS lần, nghỉ BRUCE_GAP_MS ms giữa các mã.
 * ============================================================================
 */

#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Thông số phát
// ---------------------------------------------------------------------------
#define BRUCE_REPEATS       3      // số lần phát mỗi mã
#define BRUCE_GAP_MS        8      // nghỉ giữa các mã (ms)
#define BRUCE_TOTAL_CODES   4096   // 2^12

// ---------------------------------------------------------------------------
// Định nghĩa loại cổng
// ---------------------------------------------------------------------------
struct BruceGateType {
    const char* name;
    float       freq_mhz;
    uint8_t     bits;
    uint16_t    T_us;
    bool        inverted;   // true = Came/Nice style (sync LOW); false = EV style
    // Timing
    int16_t     sync_d1;
    int16_t     sync_d2;
    int16_t     bit0_d1;
    int16_t     bit0_d2;
    int16_t     bit1_d1;
    int16_t     bit1_d2;
    uint32_t    est_seconds; // thời gian ước tính
};

// ---------------------------------------------------------------------------
// 5 loại cổng (dữ liệu tĩnh)
// ---------------------------------------------------------------------------
static const BruceGateType BRUCE_GATES[] = {
    {
        /* name */      "CAME-fast",
        /* freq */      433.920f,
        /* bits */      12,
        /* T_us */      185,
        /* inv  */      true,
        /* sync */      -36, +1,
        /* bit0 */      -1,  +2,
        /* bit1 */      -2,  +1,
        /* est  */      224
    },
    {
        /* name */      "CAME",
        /* freq */      433.920f,
        /* bits */      12,
        /* T_us */      320,
        /* inv  */      true,
        /* sync */      -36, +1,
        /* bit0 */      -1,  +2,
        /* bit1 */      -2,  +1,
        /* est  */      287
    },
    {
        /* name */      "Ansonic",
        /* freq */      434.075f,
        /* bits */      12,
        /* T_us */      450,
        /* inv  */      false,
        /* sync */      +1,  -31,
        /* bit0 */      +1,  -3,
        /* bit1 */      +3,  -1,
        /* est  */      276
    },
    {
        /* name */      "Holtek",
        /* freq */      433.920f,
        /* bits */      12,
        /* T_us */      350,
        /* inv  */      false,
        /* sync */      +1,  -31,
        /* bit0 */      +1,  -3,
        /* bit1 */      +3,  -1,
        /* est  */      387
    },
    {
        /* name */      "NICE",
        /* freq */      433.920f,
        /* bits */      12,
        /* T_us */      700,
        /* inv  */      true,
        /* sync */      -36, +1,
        /* bit0 */      -1,  +2,
        /* bit1 */      -2,  +1,
        /* est  */      628
    },
};
static const int BRUCE_GATE_COUNT = 5;
