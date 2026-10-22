#include "menu.h"
#include "config.h"
#include "hardware.h"
#include "storage.h"
#include "keyboard.h"
#include "rf_listen.h"
#include "rf_replay.h"
#include "bruteforce_codes.h"
#include <Adafruit_GFX.h>
#include <math.h>
#include "debruijn.h"
#include "advanced_capture.h"
namespace Menu {

// ===========================================================================
// States
// ===========================================================================
enum State {
    ST_BOOT = 0, ST_MAIN, ST_SCAN, ST_CAPTURE, ST_CAPTURE_DONE,
    ST_SAVED_LIST, ST_SAVED_DETAIL, ST_REPLAY, ST_JAMMER,ST_BRUCE_DEBRUIJN_MENU,
ST_BRUCE_DEBRUIJN_RUNNING,
    ST_JAMMER_ALL,
    ST_BRUCE_MENU, ST_BRUCE_RUNNING,
    ST_SETTINGS, ST_ABOUT,
    // ★ Advanced Capture
    ST_CAPTURE_MENU, ST_ADV_CAPTURE, ST_ADV_RESULT,
    // ★ Multi-Button 4-Bit Remote Analysis
    ST_SAVED_MODE_SELECT, ST_SAVED_BTN_COUNT, ST_SAVED_BTN_TX
};
// ★ De Bruijn state
static int      g_db_bits       = DEBRUIJN_DEFAULT_BITS;
static int      g_db_T_us       = DEBRUIJN_DEFAULT_T_US;
static int      g_db_repeats    = 1;
static int      g_db_freq_idx   = 0;  // 0=433, 1=315
static bool     g_db_running    = false;
static uint32_t g_db_start_ms   = 0;

static const float DB_FREQS[]   = {433.920f, 315.000f};
static const char* DB_FREQ_NAMES[] = {"433.920 MHz", "315.000 MHz"};
static State    g_state = ST_BOOT;
static uint32_t g_enter_ms = 0;
static uint32_t g_clear_confirm_ms = 0;
// ★ Advanced capture
static int                       g_capture_menu_idx = 0;
static AdvCapture::AdvResult     g_adv_result = {};
// ===========================================================================
// Main menu — "Bruce" đổi tên thành "Brute Force"
// ===========================================================================
static int g_main_idx = 0;
static const char* MAIN_ITEMS[] = {
    "Live Scan", "Signal Capture", "Code Library", "RF Jammer",
    "Brute Force", "Settings", "About"
};
static const int MAIN_COUNT = 7;

static int g_settings_idx = 0;
static const char* SET_ITEMS[] = {"Frequency", "LED Bright", "Max bits","Clear Flash"};
static const int   SET_COUNT = 4;

static int g_saved_idx = 0;
static int g_saved_detail_action = 0; // 0=TX, 1=JAM, 2=REN, 3=DEL
static bool g_delete_confirm = false;
static float g_main_scroll = 0.0f;
static float g_saved_scroll = 0.0f;

// ★ Multi-Button 4-Bit Remote Analysis
static int      g_saved_mode_idx   = 0; // 0 = 4-Bit Remote, 1 = Code Detail
static uint8_t  g_remote_root_cmd  = 0; // 4-bit command (0..15)
static int      g_remote_root_btn  = 1; // 1..4
static int      g_btn_count        = 3; // 2..4

struct RemoteBtn {
    char     label[16];
    uint64_t payload;
    uint8_t  cmd;
    bool     is_root;
};
static RemoteBtn g_remote_btns[4];
static int       g_remote_btn_count = 0;
static int       g_remote_btn_idx   = 0;
static uint32_t  g_btn_tx_ms        = 0;
static bool      g_btn_tx_ok        = false;

// ---------------------------------------------------------------------------
// Nav repeat helper
// ---------------------------------------------------------------------------
struct NavRepeat {
    bool held = false;
    bool repeated = false;
    uint32_t started_ms = 0;
    uint32_t last_step_ms = 0;
};
static NavRepeat g_main_left_nav, g_main_right_nav;
static NavRepeat g_saved_left_nav, g_saved_right_nav;
static NavRepeat g_settings_left_nav, g_settings_right_nav;
static NavRepeat g_bf_left_nav, g_bf_right_nav;

static bool nav_step(HW::Btn button, HW::Ev event, int direction,
                     int& index, int count, NavRepeat& nav) {
    const uint32_t now = millis();
    const bool down = HW::is_pressed(button);
    bool changed = false;
    if (down) {
        if (!nav.held) {
            nav.held = true;
            nav.repeated = false;
            nav.started_ms = now;
            nav.last_step_ms = now;
        } else if ((now - nav.started_ms) >= 320 &&
                   (now - nav.last_step_ms) >= 88) {
            index = (index + direction + count) % count;
            nav.last_step_ms = now;
            nav.repeated = true;
            changed = true;
        }
    } else {
        if (event == HW::EV_CLICK && !nav.repeated) {
            index = (index + direction + count) % count;
            changed = true;
        }
        nav.held = false;
        nav.repeated = false;
    }
    return changed;
}

// ---------------------------------------------------------------------------
// Captured + play state
// ---------------------------------------------------------------------------
static RFResult  g_cap_result = {};
static bool      g_cap_has    = false;
static SavedCode g_play       = {};
static bool      g_play_ok    = false;

// Jammer state
static uint32_t g_jammer_enter_ms = 0;
static int      g_jam_all_idx = 0;
static uint32_t g_jam_all_last_freq = 0;  
// ★ Brute force state
static int       g_bf_proto_idx   = 0;    // 0..BF_PROTOCOL_COUNT-1
static uint32_t  g_bf_code        = 0;    // mã đang phát (0..4095)
static bool      g_bf_running     = false;

// ---------------------------------------------------------------------------
// Render throttle
// ---------------------------------------------------------------------------
static uint32_t g_last_render_ms = 0;
static bool     g_force_render   = true;
static void request_render() { g_force_render = true; }
static bool should_render() {
    uint32_t now = millis();
    if (g_force_render || (now - g_last_render_ms) >= 16) {
        g_last_render_ms = now; g_force_render = false; return true;
    }
    return false;
}

static void enter(State s) { g_state = s; g_enter_ms = millis(); request_render(); }

// ===========================================================================
// Brute-force helpers
// ===========================================================================
// Convert BFProtocol (unsigned hi/lo) → RFProtocolDef (signed HIGH/LOW)
static RFProtocolDef bf_to_rfproto(const BFProtocol& bf) {
    RFProtocolDef p{};
    strncpy(p.name, bf.name, RF_REPLAY_NAME_LEN - 1);
    p.name[RF_REPLAY_NAME_LEN - 1] = 0;
    p.bits      = bf.bits;
    p.T_us      = bf.T_us;
    p.msb_first = true;
    // dương = HIGH, âm = LOW
    p.sync_d1 = +(int16_t)bf.sync_hi;
    p.sync_d2 = -(int16_t)bf.sync_lo;
    p.bit0_d1 = +(int16_t)bf.b0_hi;
    p.bit0_d2 = -(int16_t)bf.b0_lo;
    p.bit1_d1 = +(int16_t)bf.b1_hi;
    p.bit1_d2 = -(int16_t)bf.b1_lo;
    return p;
}

// ===========================================================================
// Draw header
// ===========================================================================
static void draw_header(const char* title) {
    auto& d = HW::oled;
    // Xóa sạch vùng dải vàng (y = 0..15) để đè lên mọi nội dung trượt
    d.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    d.fillRect(0, 0, 128, 15, SSD1306_WHITE);
    d.setTextSize(1);
    d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(3, 4);
    for (int i = 0; title[i] && i < 14; ++i) d.write((uint8_t)title[i]);
    char freq[12];
    snprintf(freq, sizeof(freq), "%.0fM", Storage::get_frequency());
    int fx = 125 - (int)strlen(freq) * 6;
    d.setCursor(fx, 4);
    d.print(freq);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.drawFastHLine(0, 15, 128, SSD1306_BLACK);
}


// ===========================================================================
// Emergency exit (hold MID 3s)
// ===========================================================================
static bool emergency_check() {
    if (g_state == ST_BOOT || g_state == ST_JAMMER || g_state == ST_JAMMER_ALL
        || g_state == ST_BRUCE_RUNNING || g_state == ST_ADV_CAPTURE) return false;
    if (HW::is_long_pressed(HW::BTN_MID, 3000)) {
        Serial.println(F("[EXIT] M-hold 3s -> MAIN"));
        if (Keyboard::is_active()) Keyboard::close();
        rf_listen_end(); rf_replay_clear_buttons();
        HW::led_idle(); g_main_idx = 0; enter(ST_MAIN);
        while (HW::is_pressed(HW::BTN_MID)) { HW::buttons_tick(); delay(5); }
        HW::get_event(HW::BTN_LEFT); HW::get_event(HW::BTN_MID); HW::get_event(HW::BTN_RIGHT);
        return true;
    }
    return false;
}

// ===========================================================================
// RENDER
// ===========================================================================
static void render_boot() {
    const uint32_t el = millis() - g_enter_ms;
    auto& d = HW::oled;
    d.clearDisplay();
    const int cx = 64, cy = 38;
    int r = 5 + (int)((el / 28) % 13);
    d.drawCircle(cx, cy, 19, SSD1306_WHITE);
    d.drawCircle(cx, cy, r, SSD1306_WHITE);
    const int a = (int)((el / 18) % 24);
    const float ang = a * 6.2831853f / 24.0f;
    d.drawLine(cx, cy, cx + (int)(15.0f * cosf(ang)),
               cy + (int)(15.0f * sinf(ang)), SSD1306_WHITE);
    d.setTextSize(1);
    d.setCursor(28, 59); d.print("STARTING SYSTEM");
    draw_header("RF GATEWAY");
}

static void render_main() {
    auto& d = HW::oled;
    d.clearDisplay();
    const int visible = 4;
    float target = (float)(g_main_idx - 1);
    if (target < 0.0f) target = 0.0f;
    const float maxOffset = (float)(MAIN_COUNT - visible);
    if (target > maxOffset) target = maxOffset;
    g_main_scroll += (target - g_main_scroll) * 0.30f;
    if (fabsf(target - g_main_scroll) < 0.035f) g_main_scroll = target;
    d.setTextSize(1);
    for (int i = 0; i < MAIN_COUNT; ++i) {
        const float yf = 18.0f + ((float)i - g_main_scroll) * 11.0f;
        const int y = (int)lroundf(yf);
        if (y < 16 || y > 61) continue; // ★ Không bao giờ vẽ đè lên vùng header vàng (y=0..15)
        const bool sel = (i == g_main_idx);
        if (sel) d.fillRoundRect(1, y, 126, 9, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(5, y + 1); d.print(sel ? ">" : " ");
        d.setCursor(17, y + 1); d.print(MAIN_ITEMS[i]);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    // ★ Header vàng luôn vẽ sau cùng để đè lên tất cả
    draw_header("RF GATEWAY");
}

static void render_scan() {
    auto& d = HW::oled;
    d.clearDisplay();
    const float pk   = rf_listen_get_last_peak_freq();
    const int   rssi = rf_listen_get_last_peak_rssi();
    const bool  valid = (pk > 0.0f && rssi > -200);
    char b[24];
    d.setTextSize(1);
    if (valid) snprintf(b, sizeof(b), "%.3f MHz", pk);
    else       snprintf(b, sizeof(b), "LISTENING...");
    d.setCursor(3, 18); d.print(b);
    if (valid) snprintf(b, sizeof(b), "%d dBm", rssi);
    else       snprintf(b, sizeof(b), "-- dBm");
    d.setCursor(125 - (int)strlen(b) * 6, 18); d.print(b);

    static int8_t trace[120];
    static bool trace_ready = false;
    static uint32_t last_sample_ms = 0;
    static float shown_amp = 2.0f;
    if (!trace_ready) {
        for (int i = 0; i < 120; ++i) trace[i] = 0;
        trace_ready = true; last_sample_ms = millis();
    }
    float strength = 0.0f;
    if (valid) {
        strength = ((float)rssi + 105.0f) / 65.0f;
        if (strength < 0.0f) strength = 0.0f;
        if (strength > 1.0f) strength = 1.0f;
    }
    const float targetAmp = valid ? (2.5f + strength * 9.5f) : 1.4f;
    shown_amp += (targetAmp - shown_amp) * 0.22f;
    const uint32_t now = millis();
    if (now - last_sample_ms >= 26) {
        memmove(trace, trace + 1, 119 * sizeof(trace[0]));
        static const int8_t ecg[32] = {
            0,0,0,0,1,1,2,1,0,0,-2,-4,10,-6,3,1,0,0,0,0,0,
            1,2,3,2,1,0,0,0,0,0,0
        };
        const uint8_t phase = (uint8_t)((now % 960UL) * 32UL / 960UL);
        const float scale = shown_amp / 10.0f;
        trace[119] = (int8_t)lroundf((float)ecg[phase] * scale);
        last_sample_ms = now;
    }
    for (int x = 4; x < 128; x += 16)
        for (int y = 30; y < 55; y += 4) d.drawPixel(x, y, SSD1306_WHITE);
    for (int x = 4; x < 128; x += 4) d.drawPixel(x, 42, SSD1306_WHITE);
    const int base = 42;
    for (int i = 1; i < 120; ++i) {
        d.drawLine(3 + (i-1), base - trace[i-1], 3 + i, base - trace[i], SSD1306_WHITE);
    }
    d.fillRect(0, 55, 128, 9, SSD1306_BLACK);
    d.setCursor(3, 56);
    if (!valid) d.print("NO PEAK  |  SWEEPING");
    else if (strength > 0.66f) d.print("SIGNAL: STRONG");
    else if (strength > 0.30f) d.print("SIGNAL: MEDIUM");
    else d.print("SIGNAL: WEAK");

    // Header vàng đè lên trên cùng
    draw_header("LIVE SCAN");
}

// ---------------------------------------------------------------------------
// ★ CAPTURE sub-menu render — Giao diện thiết kế mới hiện đại, dạng 2 Card
// ---------------------------------------------------------------------------
static void render_capture_menu() {
    auto& d = HW::oled;
    d.clearDisplay();

    // Card 0: Standard Capture (y = 18..35)
    const bool sel0 = (g_capture_menu_idx == 0);
    if (sel0) {
        d.fillRoundRect(2, 18, 124, 18, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        d.drawRoundRect(2, 18, 124, 18, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    d.setCursor(6, 21);
    d.print(sel0 ? "> STANDARD CAP" : "  STANDARD CAP");
    d.setCursor(6, 29);
    d.print("  Fast / 1-Frame");

    // Card 1: Advanced Capture (y = 38..55)
    const bool sel1 = (g_capture_menu_idx == 1);
    if (sel1) {
        d.fillRoundRect(2, 38, 124, 18, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        d.drawRoundRect(2, 38, 124, 18, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    d.setCursor(6, 41);
    d.print(sel1 ? "> ADVANCED CAP" : "  ADVANCED CAP");
    d.setCursor(6, 49);
    d.print("  Hold Remote / Deep");

    // Footer hint (y = 57)
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.setCursor(4, 57);
    d.print("R:TOGGLE MID:OK L:BACK");


    // Header vàng đè lên trên cùng
    draw_header("CAPTURE MODE");
}


// ---------------------------------------------------------------------------
// ★ ADV CAPTURE render — đang thu
// ---------------------------------------------------------------------------
static void render_adv_capture() {
    auto& d = HW::oled;
    d.clearDisplay();

    const uint32_t el = AdvCapture::get_elapsed_ms();
    const uint16_t cnt = AdvCapture::get_edge_count();
    const uint16_t cap = AdvCapture::get_edge_capacity();

    // Timer lớn ở giữa
    char timer[16];
    snprintf(timer, sizeof(timer), "%.1fs", el / 1000.0f);
    d.setTextSize(3);
    int tw = strlen(timer) * 18;
    d.setCursor((128 - tw) / 2, 22);
    d.print(timer);

    // Edges bar
    d.setTextSize(1);
    char buf[24];
    snprintf(buf, sizeof(buf), "EDGES %u/%u", cnt, cap);
    d.setCursor(3, 46); d.print(buf);

    int barW = cap ? (int)((uint32_t)cnt * 120 / cap) : 0;
    d.drawRect(3, 54, 122, 6, SSD1306_WHITE);
    if (barW > 0) d.fillRect(4, 55, barW, 4, SSD1306_WHITE);

    // Bottom hint
    d.setCursor(3, 62 - 6);
    if (cnt == 0)       d.print("HOLD REMOTE...");
    else if (cnt < 100) d.print("KEEP HOLDING...");
    else                d.print("MID=STOP  L=CANCEL");

    draw_header("ADV CAPTURE");
}

static void render_adv_analyzing() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(2);
    d.setCursor(20, 28); d.print("ANALYZING");
    d.setTextSize(1);
    const uint32_t el = millis() / 100;
    for (int i = 0; i < (int)(el % 4); i++) d.print(".");
    d.setCursor(20, 52); d.print("Please wait...");

    draw_header("ADV CAPTURE");
}

// ---------------------------------------------------------------------------
// ★ ADV RESULT render
// ---------------------------------------------------------------------------
static void render_adv_result() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);
    char b[32];

    if (g_adv_result.repeats == 0) {
        d.setCursor(15, 26); d.print("NO RELIABLE CODE");
        d.setCursor(10, 40); d.print("Try holding longer");
        d.setCursor(20, 56); d.print("LEFT = RETRY");
        draw_header("ADV RESULT");
        return;
    }

    d.setCursor(3, 19);
    snprintf(b, sizeof(b), "PROTO  %.12s", g_adv_result.protocol);
    d.print(b);

    d.drawRoundRect(1, 28, 126, 12, 2, SSD1306_WHITE);
    d.setCursor(4, 31);
    snprintf(b, sizeof(b), "0x%llX", (unsigned long long)g_adv_result.payload);
    d.print(b);

    d.setCursor(3, 44);
    snprintf(b, sizeof(b), "BITS %u  T %luus",
             g_adv_result.bits, (unsigned long)g_adv_result.T_us);
    d.print(b);

    // Repeats
    d.setCursor(3, 54);
    if (g_adv_result.valid) {
        snprintf(b, sizeof(b), "REPEATS %u/%u  OK",
                 g_adv_result.repeats, g_adv_result.total_bursts);
    } else {
        snprintf(b, sizeof(b), "LOW REP %u/%u (<5)",
                 g_adv_result.repeats, g_adv_result.total_bursts);
    }
    d.print(b);

    // Hint
    d.setCursor(3, 62 - 6);
    d.print("MID=SAVE R=TX L=BACK");

    draw_header("ADV RESULT");
}

static void render_capture_listening() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    char b[20];
    snprintf(b, sizeof(b), "F %.1fM", Storage::get_frequency());
    d.setCursor(3, 19); d.print(b);
    d.setCursor(91, 19); d.print("SCOPE");

    const int xLeft = 3, xRight = 124;
    const int graphW = xRight - xLeft + 1;
    const int yTop = 27, yBottom = 53;
    const int yHigh = 32, yLow = 48, yMid = 40;
    d.drawRoundRect(1, yTop, 126, yBottom - yTop + 1, 2, SSD1306_WHITE);
    for (int x = xLeft + 7; x < xRight; x += 12)
        for (int y = yTop + 3; y < yBottom - 1; y += 4)
            d.drawPixel(x, y, SSD1306_WHITE);
    for (int x = xLeft + 2; x < xRight - 1; x += 6)
        d.drawPixel(x, yMid, SSD1306_WHITE);

    static uint32_t durations[128];
    static uint8_t  levels[128];
    const uint16_t got = rf_listen_snapshot_recent_edges(durations, levels, 128);

    static uint8_t acceptedLevels[128];
    uint16_t acceptedCount = 0;
    float bestBase = 0.0f;
    int bestShortCount = 0, bestLongCount = 0, bestScore = 0;
    for (uint16_t i = 0; i < got; ++i) {
        const uint32_t base2 = durations[i];
        if (base2 < 100 || base2 > 1400) continue;
        int sc = 0, lc = 0;
        for (uint16_t j = 0; j < got; ++j) {
            const uint32_t t = durations[j];
            if (t < 80 || t > 5000) continue;
            const float rel = fabsf((float)t - (float)base2) / (float)base2;
            if (rel <= 0.28f) { ++sc; continue; }
            const float ratio = (float)t / (float)base2;
            const int mul = (int)lroundf(ratio);
            if (mul >= 2 && mul <= 4 && fabsf(ratio - (float)mul) <= 0.28f) ++lc;
        }
        const int score = sc + lc;
        if (sc >= 4 && lc >= 3 && score > bestScore) {
            bestScore = score; bestBase = (float)base2;
            bestShortCount = sc; bestLongCount = lc;
        }
    }
    if (bestBase > 0.0f && bestShortCount >= 4 && bestLongCount >= 3 && bestScore >= 12) {
        for (uint16_t i = 0; i < got && acceptedCount < 128; ++i) {
            const uint32_t t = durations[i];
            if (t < 80 || t > 5000) continue;
            const float rel = fabsf((float)t - bestBase) / bestBase;
            if (rel <= 0.28f) { acceptedLevels[acceptedCount++] = levels[i] ? 1 : 0; continue; }
            const float ratio = (float)t / bestBase;
            const int mul = (int)lroundf(ratio);
            if (mul >= 2 && mul <= 4 && fabsf(ratio - (float)mul) <= 0.28f)
                acceptedLevels[acceptedCount++] = levels[i] ? 1 : 0;
        }
    }
    static float shownY[122], targetY[122];
    static bool traceInit = false, haveStableTrace = false;
    static uint32_t lastTraceUpdateMs = 0;
    const uint32_t now = millis();
    if (!traceInit) {
        for (int i = 0; i < graphW; ++i) shownY[i] = targetY[i] = (float)yMid;
        traceInit = true;
    }
    if (acceptedCount >= 12 && (now - lastTraceUpdateMs) >= 180) {
        for (int x = 0; x < graphW; ++x) {
            const uint16_t sample = (uint16_t)(((uint32_t)x * (acceptedCount - 1)) / (uint32_t)(graphW - 1));
            targetY[x] = acceptedLevels[sample] ? (float)yHigh : (float)yLow;
        }
        haveStableTrace = true; lastTraceUpdateMs = now;
    }
    if (haveStableTrace) {
        for (int x = 0; x < graphW; ++x) {
            shownY[x] += (targetY[x] - shownY[x]) * 0.28f;
            if (fabsf(targetY[x] - shownY[x]) < 0.20f) shownY[x] = targetY[x];
            if (x > 0) d.drawLine(xLeft + x - 1, (int)lroundf(shownY[x-1]),
                                  xLeft + x, (int)lroundf(shownY[x]), SSD1306_WHITE);
        }
    } else {
        d.drawFastHLine(xLeft + 2, yMid, graphW - 4, SSD1306_WHITE);
        const int cursorX = xLeft + 2 + (int)((now / 45UL) % (uint32_t)(graphW - 4));
        d.drawFastVLine(cursorX, yMid - 2, 5, SSD1306_WHITE);
    }
    d.drawFastHLine(0, 55, 128, SSD1306_WHITE);
    d.setCursor(3, 57);
    d.print(haveStableTrace ? "TRACE STABLE" : "SIGNAL MONITOR");

    draw_header("SIGNAL CAPTURE");
}

static void render_capture_done() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);
    char b[32];
    d.setCursor(3, 19); snprintf(b, sizeof(b), "PROTO  %s", g_cap_result.protocol); d.print(b);
    d.drawRoundRect(1, 29, 126, 12, 2, SSD1306_WHITE);
    d.setCursor(4, 32); snprintf(b, sizeof(b), "0x%llX", (unsigned long long)g_cap_result.payload); d.print(b);
    d.setCursor(3, 44); snprintf(b, sizeof(b), "BITS %u  T %luus", g_cap_result.bits, (unsigned long)g_cap_result.T_us); d.print(b);
    d.setCursor(3, 55);
    snprintf(b, sizeof(b), "RSSI%ddBm CONF%u%%", g_cap_result.rssi_dbm, g_cap_result.confidence);
    d.print(b);

    draw_header("CAPTURED");
}

// ---------------------------------------------------------------------------
// ★ CODE LIBRARY — render danh sách (item 0 là [+ ADD NEW CODE])
// ---------------------------------------------------------------------------
static void render_saved_list() {
    auto& d = HW::oled;
    d.clearDisplay();
    const int n = Storage::count();
    const int total = n + 1; // 0 = [+ ADD NEW CODE], 1..n = saved codes
    const int visible = 5;

    float target = (float)(g_saved_idx - (visible - 1));
    if (target < 0.0f) target = 0.0f;
    const float maxOffset = (float)max(0, total - visible);
    if (target > maxOffset) target = maxOffset;
    g_saved_scroll += (target - g_saved_scroll) * 0.32f;
    if (fabsf(target - g_saved_scroll) < 0.035f) g_saved_scroll = target;

    d.setTextSize(1);
    for (int i = 0; i < total; ++i) {
        const int y = (int)lroundf(18.0f + ((float)i - g_saved_scroll) * 9.0f);
        if (y < 16 || y > 62) continue; // ★ Clip không cho tràn vào header vàng

        const bool sel = (i == g_saved_idx);
        if (sel) d.fillRoundRect(1, y, 126, 8, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(4, y); d.print(sel ? ">" : " ");

        if (i == 0) {
            d.setCursor(13, y);
            d.print("[+] ADD NEW CODE");
        } else {
            SavedCode c;
            if (Storage::get(i - 1, c)) {
                char line[24];
                snprintf(line, sizeof(line), "%02d %-7.7s %6.2fM",
                         i, c.name, c.freq_khz / 1000.0f);
                d.setCursor(13, y); d.print(line);
            }
        }
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }

    draw_header("CODE LIBRARY");
}

static void render_saved_delete_confirm() {
    auto& d = HW::oled;
    d.clearDisplay();
    SavedCode c;
    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    if (!Storage::get(idx, c)) { draw_header("DELETE CODE"); return; }

    d.drawRoundRect(3, 18, 122, 44, 2, SSD1306_WHITE);
    d.setTextSize(1);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.setCursor(14, 22); d.print("DELETE THIS CODE?");

    char name_line[24];
    snprintf(name_line, sizeof(name_line), "\"%.12s\"", c.name);
    d.setCursor(14, 34); d.print(name_line);

    d.setCursor(8, 48); d.print("MID = YES (DELETE)");
    d.setCursor(8, 56); d.print("L = CANCEL");

    draw_header("DELETE CODE");
}

static void render_saved_detail() {
    if (g_delete_confirm) {
        render_saved_delete_confirm();
        return;
    }
    auto& d = HW::oled;
    d.clearDisplay();
    SavedCode c;
    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    if (!Storage::get(idx, c)) { draw_header("CODE DETAIL"); return; }

    d.setTextSize(1);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    char buf[32];
    snprintf(buf, sizeof(buf), "NAME: %.12s", c.name);
    d.setCursor(3, 18); d.print(buf);

    snprintf(buf, sizeof(buf), "0x%llX", (unsigned long long)c.payload);
    d.setCursor(3, 27); d.print(buf);

    snprintf(buf, sizeof(buf), "%ub  %uus  %s",
             c.bits, c.T_us, c.inverted ? "INV" : "NORM");
    d.setCursor(3, 36); d.print(buf);

    snprintf(buf, sizeof(buf), "FREQ: %.3fM  R%u",
             c.freq_khz / 1000.0f, c.repeats);
    d.setCursor(3, 45); d.print(buf);

    // 4 Action Buttons ở đáy màn hình (y = 54..62):
    // TX, JAM, REN, DEL
    const char* actions[4] = {"TX", "JAM", "REN", "DEL"};
    const int bx[4] = {3, 33, 63, 95};
    const int bw[4] = {27, 27, 29, 30};

    for (int i = 0; i < 4; ++i) {
        bool sel = (g_saved_detail_action == i);
        if (sel) {
            d.fillRoundRect(bx[i], 54, bw[i], 9, 2, SSD1306_WHITE);
            d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        } else {
            d.drawRoundRect(bx[i], 54, bw[i], 9, 2, SSD1306_WHITE);
            d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        }
        int lx = bx[i] + (bw[i] - (int)strlen(actions[i]) * 6) / 2;
        d.setCursor(lx, 55);
        d.print(actions[i]);
    }
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    draw_header("CODE DETAIL");
}

// ===========================================================================
// ★ MULTI-BUTTON 4-BIT REMOTE HELPERS & RENDER
// ===========================================================================
static int detect_root_button(uint8_t cmd) {
    if (cmd == 0x01) return 1;
    if (cmd == 0x02) return 2;
    if (cmd == 0x04) return 3;
    if (cmd == 0x08) return 4;
    // Fallback nếu có nhiều bit hoặc không chuẩn
    if (cmd & 0x01) return 1;
    if (cmd & 0x02) return 2;
    if (cmd & 0x04) return 3;
    if (cmd & 0x08) return 4;
    return 1;
}

static void generate_remote_buttons(const SavedCode& c, int count) {
    g_remote_btn_count = (count < 1) ? 1 : ((count > 4) ? 4 : count);
    uint64_t base_payload = c.payload & ~((uint64_t)0x0F);
    static const uint8_t BTN_CMDS[4] = { 0x01, 0x02, 0x04, 0x08 };

    for (int i = 0; i < g_remote_btn_count; ++i) {
        int btn_num = i + 1;
        bool is_root = (btn_num == g_remote_root_btn);
        g_remote_btns[i].is_root = is_root;
        g_remote_btns[i].cmd     = BTN_CMDS[i];

        if (is_root) {
            g_remote_btns[i].payload = c.payload;
            snprintf(g_remote_btns[i].label, sizeof(g_remote_btns[i].label),
                     "(*) button %d", btn_num);
        } else {
            g_remote_btns[i].payload = base_payload | BTN_CMDS[i];
            snprintf(g_remote_btns[i].label, sizeof(g_remote_btns[i].label),
                     "button %d", btn_num);
        }
    }
}

// ---------------------------------------------------------------------------
// 1. Menu chọn chế độ (1: Phân tích 4-bit Remote, 2: Code Detail gốc)
// ---------------------------------------------------------------------------
static void render_saved_mode_select() {
    auto& d = HW::oled;
    d.clearDisplay();
    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) { draw_header("CODE OPTION"); return; }

    d.setTextSize(1);
    char buf[32];

    // Dòng tóm tắt thông tin mã (y = 17..33)
    snprintf(buf, sizeof(buf), "[%02d] %.8s  %.2fM", g_saved_idx, c.name, c.freq_khz / 1000.0f);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.setCursor(3, 17); d.print(buf);

    snprintf(buf, sizeof(buf), "CODE: 0x%llX", (unsigned long long)c.payload);
    d.setCursor(3, 26); d.print(buf);

    d.drawFastHLine(2, 35, 124, SSD1306_WHITE);

    // Card 0: Mục 1 - 4-BIT REMOTE (y = 37..48)
    bool sel0 = (g_saved_mode_idx == 0);
    if (sel0) {
        d.fillRoundRect(2, 37, 124, 12, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        d.drawRoundRect(2, 37, 124, 12, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    d.setCursor(6, 39);
    d.print(sel0 ? "> 1. REMOTE 4-BIT" : "  1. REMOTE 4-BIT");

    // Card 1: Mục 2 - CODE DETAIL (y = 51..62)
    bool sel1 = (g_saved_mode_idx == 1);
    if (sel1) {
        d.fillRoundRect(2, 51, 124, 12, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        d.drawRoundRect(2, 51, 124, 12, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    d.setCursor(6, 53);
    d.print(sel1 ? "> 2. CODE DETAIL" : "  2. CODE DETAIL");

    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    char hdr[16];
    snprintf(hdr, sizeof(hdr), "%.12s", c.name);
    draw_header(hdr);
}

// ---------------------------------------------------------------------------
// 2. Màn hình chọn số nút (e.g. 2, 3, 4 nút)
// ---------------------------------------------------------------------------
static void render_saved_btn_count() {
    auto& d = HW::oled;
    d.clearDisplay();
    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) { draw_header("BUTTON COUNT"); return; }

    d.setTextSize(1);
    char buf[32];

    // Hộp hiển thị kết quả phân tích 4-bit (y = 17..35)
    d.drawRoundRect(2, 17, 124, 19, 2, SSD1306_WHITE);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // Chuỗi binary 4 bit
    char bin[5];
    for (int b = 3; b >= 0; --b) bin[3 - b] = (g_remote_root_cmd & (1 << b)) ? '1' : '0';
    bin[4] = 0;

    snprintf(buf, sizeof(buf), "CMD: 0x%X (%sb)", g_remote_root_cmd, bin);
    d.setCursor(6, 20); d.print(buf);

    snprintf(buf, sizeof(buf), "BASE: (*) BUTTON %d", g_remote_root_btn);
    d.setCursor(6, 27); d.print(buf);

    // Hộp chọn số nút (y = 38..50)
    d.fillRoundRect(14, 38, 100, 13, 2, SSD1306_WHITE);
    d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(18, 41); d.print("<");
    d.setCursor(104, 41); d.print(">");

    snprintf(buf, sizeof(buf), "%d BUTTONS", g_btn_count);
    int text_len = (int)strlen(buf) * 6;
    int tx = 14 + (100 - text_len) / 2;
    d.setCursor(tx, 41); d.print(buf);

    // Hướng dẫn chân trang (y = 54)
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.setCursor(8, 54); d.print("MID=OK   HOLD M=BACK");

    draw_header("BUTTON COUNT");
}

// ---------------------------------------------------------------------------
// 3. Màn hình danh sách button sẵn sàng phát (bấm MID phát 15 lần, giữ MID back)
// ---------------------------------------------------------------------------
static void render_saved_btn_tx() {
    auto& d = HW::oled;
    d.clearDisplay();
    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) { draw_header("REMOTE TX"); return; }

    d.setTextSize(1);

    // Vẽ danh sách các nút
    int count = g_remote_btn_count;
    int row_h = (count <= 3) ? 10 : 9;
    int start_y = (count == 2) ? 20 : ((count == 3) ? 18 : 17);
    int step_y  = (count == 2) ? 14 : ((count == 3) ? 11 : 9);

    for (int i = 0; i < count; ++i) {
        int y = start_y + i * step_y;
        bool sel = (i == g_remote_btn_idx);

        if (sel) {
            d.fillRoundRect(2, y, 124, row_h, 2, SSD1306_WHITE);
            d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            d.setCursor(4, y + 1); d.print(">");
        } else {
            d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            d.setCursor(4, y + 1); d.print(" ");
        }

        d.setCursor(12, y + 1);
        d.print(g_remote_btns[i].label);

        // Hiển thị mã command [0x1], [0x2], [0x4], [0x8] bên phải
        char cmd_str[8];
        snprintf(cmd_str, sizeof(cmd_str), "[0x%X]", g_remote_btns[i].cmd);
        d.setCursor(94, y + 1);
        d.print(cmd_str);
    }

    // Vùng chân trang: Hiển thị trạng thái phát và Payload
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    d.drawFastHLine(2, 53, 124, SSD1306_WHITE);

    if (millis() - g_btn_tx_ms < 900) {
        // Vừa mới phát xong: hiển thị banner nổi bật
        d.fillRoundRect(2, 54, 124, 9, 2, SSD1306_WHITE);
        d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        if (g_btn_tx_ok) {
            d.setCursor(11, 55); d.print(">>> TX OK (15x) <<<");
        } else {
            d.setCursor(14, 55); d.print("! TX ERROR (15x) !");
        }
    } else {
        // Trạng thái sẵn sàng
        char pbuf[24];
        snprintf(pbuf, sizeof(pbuf), "0x%llX",
                 (unsigned long long)g_remote_btns[g_remote_btn_idx].payload);
        d.setCursor(3, 55);
        d.print(pbuf);

        d.drawRoundRect(74, 54, 51, 9, 2, SSD1306_WHITE);
        d.setCursor(77, 55);
        d.print("READY 15x");
    }

    char hdr[16];
    snprintf(hdr, sizeof(hdr), "%.8s REMOTE", c.name);
    draw_header(hdr);
}

static void render_replay() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1); char buf[32];
    d.setCursor(3, 19); snprintf(buf, sizeof(buf), "CODE  %.12s", g_play.name); d.print(buf);
    d.setCursor(3, 30); snprintf(buf, sizeof(buf), "0x%llX", (unsigned long long)g_play.payload); d.print(buf);
    d.setCursor(3, 40); snprintf(buf, sizeof(buf), "BITS %u  T %u us", g_play.bits, g_play.T_us); d.print(buf);
    d.setCursor(3, 50);
    snprintf(buf, sizeof(buf), "FREQ %.3fM", g_play.freq_khz / 1000.0f);
    d.print(buf);
    d.drawRoundRect(74, 47, 51, 14, 3, SSD1306_WHITE);
    d.setCursor(87, 51); d.print("READY");

    draw_header("RELAY READY");
}

static void render_jammer() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.drawRoundRect(7, 22, 114, 28, 5, SSD1306_WHITE);
    d.setTextSize(2); d.setCursor(20, 29); d.print("ACTIVE");
    d.setTextSize(1); char buf[32];
    snprintf(buf, sizeof(buf), "CODE 0x%llX", (unsigned long long)g_play.payload);
    d.setCursor(3, 54); d.print(buf);

    draw_header("JAMMER MODE");
}

static void render_jammer_all() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.drawRoundRect(6, 21, 116, 30, 4, SSD1306_WHITE);
    d.setTextSize(2); d.setCursor(25, 27); d.print("RUNNING");
    d.setTextSize(1);
    const int n = Storage::count();
    char buf[32];
    d.setCursor(3, 55); snprintf(buf, sizeof(buf), "ITEM %d / %d", min(g_jam_all_idx + 1, n), n); d.print(buf);

    draw_header("JAM ALL");
}

// ---------------------------------------------------------------------------
// ★ BRUTE FORCE — render
// ---------------------------------------------------------------------------
static void render_bruce_menu() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);

    // 5 protocol + 1 De Bruijn = 6 items
    // 6 × 8px = 48px, fits exactly in y=16..63
    const int total_items = BF_PROTOCOL_COUNT + 1;
    const int start_y = 16;
    const int step_y  = 8;

    for (int i = 0; i < total_items; i++) {
        const int y = start_y + i * step_y;
        if (y < 16 || y > 57) continue;   // clamp an toàn

        const bool sel = (i == g_bf_proto_idx);
        if (sel) d.fillRoundRect(1, y, 126, 7, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);

        d.setCursor(5, y);  d.print(sel ? ">" : " ");

        if (i < BF_PROTOCOL_COUNT) {
            // ── Protocol brute-force thường ──
            char line[32];
            snprintf(line, sizeof(line), "%-8s %2ub %4lus",
                     BF_PROTOCOLS[i].name,
                     BF_PROTOCOLS[i].bits,
                     (unsigned long)BF_PROTOCOLS[i].est_seconds);
            d.setCursor(14, y); d.print(line);
        } else {
            // ── Mục De Bruijn (item cuối) ──
            d.setCursor(14, y);
            d.print("DE BRUIJN (FAST)");
        }

        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }

    draw_header("BRUTE FORCE");
}

static void render_bruce_running() {
    auto& d = HW::oled;
    d.clearDisplay();
    const BFProtocol& bf = BF_PROTOCOLS[g_bf_proto_idx];
    d.setTextSize(1);
    char buf[32];

    d.setCursor(3, 18);
    snprintf(buf, sizeof(buf), "PROTO: %s", bf.name);
    d.print(buf);

    uint32_t total   = bf.key_count;
    uint32_t current = g_bf_code;
    int barW = (int)((uint64_t)current * 120ULL / total);
    d.drawRect(3, 28, 122, 10, SSD1306_WHITE);
    if (barW > 0) d.fillRect(4, 29, barW, 8, SSD1306_WHITE);

    snprintf(buf, sizeof(buf), "CODE %4lu / %lu",
             (unsigned long)current, (unsigned long)total);
    d.setCursor(3, 40); d.print(buf);

    snprintf(buf, sizeof(buf), "FREQ %.3f MHz", bf.freq_mhz);
    d.setCursor(3, 50); d.print(buf);

    d.setCursor(3, 59); d.print("MID=STOP");

    draw_header("BF SENDING");
}

static void render_settings_confirm_clear() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1); d.setCursor(17, 27); d.print("DELETE ALL SAVED?");
    d.drawRoundRect(23, 40, 82, 15, 3, SSD1306_WHITE);
    d.setCursor(41, 44); d.print("CONFIRM");

    draw_header("CLEAR DATA");
}

static void render_settings() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);
    for (int i = 0; i < SET_COUNT; ++i) {
        const int y = 18 + i * 11;
        if (y < 16 || y > 62) continue;
        const bool sel = (i == g_settings_idx);
        if (sel) d.fillRoundRect(1, y, 126, 10, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(5, y + 1); d.print(SET_ITEMS[i]);
        char val[16] = {0};
        switch (i) {
            case 0: snprintf(val, sizeof(val), "%.3fM", Storage::get_frequency()); break;
            case 1: snprintf(val, sizeof(val), "%d", Storage::get_led_brightness()); break;
            case 2: snprintf(val, sizeof(val), "%d", Storage::get_max_bits()); break;
            case 3: snprintf(val, sizeof(val), "RESET"); break;
        }
        d.setCursor(123 - (int)strlen(val) * 6, y + 1); d.print(val);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }

    draw_header("PREFERENCES");
}

static void render_about() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);
    d.setCursor(3, 20); d.print("RF GATEWAY  /  V2.0");
    d.setCursor(3, 32); d.print("RP2040 + CC1101");
    char buf[32];
    snprintf(buf, sizeof(buf), "CODES %d / %d", Storage::count(), MAX_SAVED_CODES);
    d.setCursor(3, 44); d.print(buf);
    snprintf(buf, sizeof(buf), "FREQ %.3f MHz", Storage::get_frequency());
    d.setCursor(3, 56); d.print(buf);

    draw_header("ABOUT");
}


// ===========================================================================
// SETUP TX + BIND
// ===========================================================================
static bool setup_tx_and_bind() {
    g_play.repeats = DEFAULT_REPEATS;
     float play_freq = g_play.freq_khz / 1000.0f;  
    Serial.printf("[REPLAY] setup @ %.3f MHz  bits=%u T=%u inv=%d\n",
                  play_freq, g_play.bits, g_play.T_us, g_play.inverted);

    rf_listen_end();

    // ★ Full init: selalu reinit agar CC1101 dalam keadaan bersih
    RFReplayConfig cfg = RF_REPLAY_DEFAULT_CONFIG;
    cfg.verbose       = false;
    cfg.frequency_mhz = play_freq;   // set freq BENAR sejak awal
    if (!rf_replay_begin(cfg)) {
        Serial.println(F("[REPLAY] cc1101 init FAIL"));
        HW::led_error();
        g_play_ok = false;
        return false;
    }

    // ★ Setelah begin(), paksa freq + calibrate lagi untuk memastikan PATABLE benar
    rf_replay_set_frequency(play_freq);

    rf_replay_clear_buttons();
    g_play_ok = true;
    return true;
}

// ★ Setup TX cho brute-force (không cần SavedCode)
static bool bruce_setup_tx(int proto_idx) {
    const BFProtocol& g = BF_PROTOCOLS[proto_idx];
    rf_listen_end();
    RFReplayConfig cfg = RF_REPLAY_DEFAULT_CONFIG;
    cfg.verbose = false;
    cfg.frequency_mhz = g.freq_mhz;
    if (!rf_replay_begin(cfg)) { HW::led_error(); return false; }
    rf_replay_clear_buttons();
    Serial.printf("[BF] proto=%s  freq=%.3f  T=%u  bits=%u\n",
                  g.name, g.freq_mhz, g.T_us, g.bits);
    return true;
}

// ===========================================================================
// UPDATE
// ===========================================================================
static void update_boot() {
    uint32_t el = millis() - g_enter_ms;
    if (el < 800) {
        uint8_t b = (el * 255) / 800;
        HW::led_set_color(b/4, b/4, b); HW::led_set_mode(HW::LED_SOLID);
    } else if (el < 1500) {
        static bool fd = false;
        if (!fd) { HW::led_flash(255,255,255,300); fd = true; }
    }
    if (el > 2000) { HW::led_idle(); enter(ST_MAIN); }
}

static void update_main() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);
    bool moved = nav_step(HW::BTN_LEFT, eL, -1, g_main_idx, MAIN_COUNT, g_main_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_main_idx, MAIN_COUNT, g_main_right_nav) || moved;
    if (moved) request_render();
    if (eM == HW::EV_CLICK) {
        switch (g_main_idx) {
            case 0: {   // Scan
                rf_listen_end();
                RFListenerConfig cfg = RF_LISTENER_DEFAULT_CONFIG;
                cfg.max_bits = Storage::get_max_bits();    // ★ v2
                cfg.verbose = false; cfg.frequency_mhz = Storage::get_frequency();
                cfg.agc_sensitive = true;
                if (!rf_listen_begin(cfg)) { HW::led_error(); break; }
                rf_listen_set_nonblocking(true);
                rf_listen_set_mode(RF_LISTEN_SCAN);
                enter(ST_SCAN); break;
            }
case 1: {   // Capture → sub-menu
    rf_listen_end();
    g_capture_menu_idx = 0;
    enter(ST_CAPTURE_MENU); break;
}
            case 2:     // Code Library
                g_saved_idx = (Storage::count() > 0) ? 1 : 0;
                enter(ST_SAVED_LIST); break;

            case 3: {   // Jam All
                if (Storage::count() == 0) { HW::led_error(); break; }
                rf_listen_end();
                RFReplayConfig cfg = RF_REPLAY_DEFAULT_CONFIG;
                cfg.verbose = false;
                cfg.frequency_mhz = Storage::get_frequency();
                if (!rf_replay_begin(cfg)) { HW::led_error(); break; }
                rf_replay_clear_buttons();
                g_jammer_enter_ms = millis();
                g_jam_all_idx = 0; g_jam_all_last_freq = 0;
                Serial.println(F("[JAM-ALL] start"));
                enter(ST_JAMMER_ALL); break;
            }
            case 4: {   // ★ Brute Force
                g_bf_proto_idx = 0;
                enter(ST_BRUCE_MENU); break;
            }
            case 5:     // Settings
                g_settings_idx = 0;
                enter(ST_SETTINGS); break;
            case 6:     // About
                enter(ST_ABOUT); break;
        }
    }
}

static void update_scan() {
    HW::led_scan();
    rf_listen_update();
    HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    HW::Ev eM = HW::get_event(HW::BTN_MID);
    if (eL == HW::EV_CLICK) { rf_listen_end(); HW::led_idle(); enter(ST_MAIN); return; }
    if (eM == HW::EV_CLICK) {
        float pk = rf_listen_get_last_peak_freq();
        int pr = rf_listen_get_last_peak_rssi();
        if (pk > 280.0f && pk < 928.0f) {
            Storage::set_frequency(pk);
            Serial.printf("[SCAN] set %.3f MHz (RSSI %d)\n", pk, pr);
            rf_listen_end(); HW::led_flash(0,255,0,400);
            RFListenerConfig cfg = RF_LISTENER_DEFAULT_CONFIG;
            cfg.max_bits = Storage::get_max_bits();    // ★ v2
            cfg.verbose = false; cfg.frequency_mhz = pk;
            cfg.agc_sensitive = true;
            if (rf_listen_begin(cfg)) {
                rf_listen_set_nonblocking(true);
                rf_listen_set_mode(RF_LISTEN_CAPTURE);
                enter(ST_CAPTURE);
            } else enter(ST_MAIN);
        } else { Serial.println(F("[SCAN] no peak")); HW::led_error(); }
        return;
    }
}

static void update_capture() {
    HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    HW::Ev eM = HW::get_event(HW::BTN_MID);
    HW::Ev eR = HW::get_event(HW::BTN_RIGHT);
    if (eL == HW::EV_CLICK) { rf_listen_end(); HW::led_idle(); enter(ST_MAIN); return; }
    if (eR == HW::EV_CLICK) {
        rf_listen_end();
        RFListenerConfig cfg = RF_LISTENER_DEFAULT_CONFIG;
        cfg.max_bits = Storage::get_max_bits();    // ★ v2
        cfg.verbose = false; cfg.frequency_mhz = Storage::get_frequency();
        cfg.agc_sensitive = true;
        if (rf_listen_begin(cfg)) {
            rf_listen_set_nonblocking(true);
            rf_listen_set_mode(RF_LISTEN_SCAN);
            enter(ST_SCAN);
        } else enter(ST_MAIN);
        return;
    }
    rf_listen_update();
    if (rf_listen_is_confirmed()) {
        g_cap_result = rf_listen_get_result();
        g_cap_has = true;
        rf_listen_clear_result();
        HW::led_flash(0, 255, 0, 300);
        Serial.printf("[CAPTURE] OK: %s 0x%llX conf=%u rssi=%d\n",
                      g_cap_result.protocol,
                      (unsigned long long)g_cap_result.payload,
                      g_cap_result.confidence,
                      g_cap_result.rssi_dbm);
        enter(ST_CAPTURE_DONE); return;
    }
    if (eM == HW::EV_CLICK && g_cap_has) {
        enter(ST_CAPTURE_DONE); return;
    }
}

// ---------------------------------------------------------------------------
// ★ CAPTURE_DONE — MID mở bàn phím để đặt tên + lưu, RIGHT vào replay luôn
// ---------------------------------------------------------------------------
static void update_capture_done() {
    HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    HW::Ev eM = HW::get_event(HW::BTN_MID);
    HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    // LEFT: quay lại capture
    if (eL == HW::EV_CLICK) {
        rf_listen_end();
        RFListenerConfig cfg = RF_LISTENER_DEFAULT_CONFIG;
        cfg.max_bits = Storage::get_max_bits();    // ★ v2
        cfg.verbose = false; cfg.frequency_mhz = Storage::get_frequency();
        cfg.agc_sensitive = true;
        if (rf_listen_begin(cfg)) {
            rf_listen_set_nonblocking(true);
            rf_listen_set_mode(RF_LISTEN_CAPTURE);
            enter(ST_CAPTURE);
        } else enter(ST_MAIN);
        return;
    }

    // MID: mở keyboard nhập tên → lưu
    if (eM == HW::EV_CLICK) {
        if (!g_cap_has) { enter(ST_MAIN); return; }

        char initial[MAX_NAME_LEN];
        snprintf(initial, sizeof(initial), "Code%d", Storage::count() + 1);
        char out[MAX_NAME_LEN];

        if (Keyboard::input_text("Save as", initial, out, MAX_NAME_LEN)) {
            SavedCode code = {};
            code.payload  = g_cap_result.payload;
            code.bits     = g_cap_result.bits;
            code.T_us     = (uint16_t)g_cap_result.T_us;
            code.inverted = g_cap_result.inverted;
            code.repeats  = DEFAULT_REPEATS;
            code.freq_khz = g_cap_result.frequency_mhz_x1000;   // ★ v2: kHz chính xác lúc capture
            strncpy(code.name, out, MAX_NAME_LEN - 1);
            code.name[MAX_NAME_LEN - 1] = 0;

            int idx = Storage::add(code);
            if (idx >= 0) {
                Storage::get(idx, g_play);
                HW::led_flash(0, 255, 255, 400);
                Serial.printf("[SAVE] OK: %s 0x%llX\n",
                              code.name, (unsigned long long)code.payload);
            } else {
                HW::led_error();
                Serial.println(F("[SAVE] FAIL"));
            }
        }
        rf_listen_end();
        HW::led_idle();
        enter(ST_MAIN);
        return;
    }

    // RIGHT: replay trực tiếp không cần lưu
    if (eR == HW::EV_CLICK) {
        if (!g_cap_has) { enter(ST_MAIN); return; }
        g_play.payload   = g_cap_result.payload;
        g_play.bits      = g_cap_result.bits;
        g_play.T_us      = g_cap_result.T_us;
        g_play.inverted  = g_cap_result.inverted;
        g_play.freq_khz  = g_cap_result.frequency_mhz_x1000;
        g_play.repeats   = DEFAULT_REPEATS;
        strncpy(g_play.name, g_cap_result.protocol, MAX_NAME_LEN - 1);
        g_play.name[MAX_NAME_LEN - 1] = 0;
        if (setup_tx_and_bind()) enter(ST_REPLAY);
        else enter(ST_MAIN);
        return;
    }
}

// ---------------------------------------------------------------------------
// ★ ADD CODE WIZARD — Nhập Payload, Bits, Tần số, T_us, Tên
// ---------------------------------------------------------------------------
static void add_code_wizard() {
    HW::led_idle();

    // 1. Nhập Payload (tự động thêm 0x nếu chưa có)
    char pbuf[32] = "";
    if (!Keyboard::input_text("PAYLOAD (HEX)", "0x", pbuf, sizeof(pbuf))) {
        return;
    }

    // Xử lý chuỗi hex: bỏ khoảng trắng và tiền tố 0x/0X nếu có
    const char* p = pbuf;
    while (*p == ' ') p++;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        p += 2;
    }
    while (*p == ' ') p++;
    if (*p == '\0') {
        HW::led_error();
        return;
    }
    char* endp = nullptr;
    uint64_t payload = strtoull(p, &endp, 16);
    if (payload == 0 && *p != '0') {
        HW::led_error();
        return;
    }

    // Tự động gợi ý số bits dựa trên độ dài payload nhập vào (ví dụ 6 ký tự hex -> 24 bits)
    size_t hex_len = strlen(p);
    size_t def_bits = hex_len * 4;
    if (def_bits < 8)  def_bits = 24;
    if (def_bits > 64) def_bits = 64;

    // 2. Nhập số bits (8-64)
    char bbuf[16];
    snprintf(bbuf, sizeof(bbuf), "%u", (unsigned)def_bits);
    if (!Keyboard::input_text("BIT COUNT", bbuf, bbuf, sizeof(bbuf))) {
        return;
    }
    int bits = atoi(bbuf);
    if (bits < 8)  bits = 8;
    if (bits > 64) bits = 64;

    // 3. Nhập tần số (MHz)
    char fbuf[16];
    snprintf(fbuf, sizeof(fbuf), "%.3f", Storage::get_frequency());
    if (!Keyboard::input_text("FREQ (MHz)", fbuf, fbuf, sizeof(fbuf))) {
        return;
    }
    float freq_mhz = atof(fbuf);
    if (freq_mhz < 280.0f || freq_mhz > 928.0f) {
        freq_mhz = Storage::get_frequency();
    }

    // 4. Nhập chu kỳ xung us (T_us)
    char tbuf[16] = "350";
    if (!Keyboard::input_text("PULSE T (us)", tbuf, tbuf, sizeof(tbuf))) {
        return;
    }
    int t_us = atoi(tbuf);
    if (t_us < 50)   t_us = 350;
    if (t_us > 5000) t_us = 5000;

    // 5. Nhập tên mã
    char nbuf[MAX_NAME_LEN];
    snprintf(nbuf, sizeof(nbuf), "Code%d", Storage::count() + 1);
    if (!Keyboard::input_text("CODE NAME", nbuf, nbuf, sizeof(nbuf))) {
        return;
    }

    // Tạo SavedCode và lưu vào EEPROM
    SavedCode sc = {};
    sc.payload  = payload;
    sc.bits     = (uint8_t)bits;
    sc.T_us     = (uint16_t)t_us;
    sc.inverted = false;
    sc.repeats  = DEFAULT_REPEATS;
    sc.freq_khz = (uint32_t)(freq_mhz * 1000.0f + 0.5f);
    strncpy(sc.name, nbuf, MAX_NAME_LEN - 1);
    sc.name[MAX_NAME_LEN - 1] = 0;

    int new_idx = Storage::add(sc);
    if (new_idx >= 0) {
        HW::led_flash(0, 255, 0, 400);
        g_saved_idx = new_idx + 1; // Chọn code vừa add trong danh sách
        Serial.printf("[ADD CODE] OK: %s 0x%llX %ub %.3fM %uus\n",
                      sc.name, (unsigned long long)sc.payload, sc.bits,
                      freq_mhz, sc.T_us);
    } else {
        HW::led_error();
        Serial.println(F("[ADD CODE] Storage FULL or Error"));
    }
}

static void update_saved_list() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    if (eM == HW::EV_LONG || eL == HW::EV_LONG) { enter(ST_MAIN); return; }

    const int total = Storage::count() + 1;
    bool moved = nav_step(HW::BTN_LEFT,  eL, -1, g_saved_idx, total, g_saved_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_saved_idx, total, g_saved_right_nav) || moved;
    if (moved) request_render();

    if (eM == HW::EV_CLICK) {
        if (g_saved_idx == 0) {
            // Mục [+ ADD NEW CODE]
            add_code_wizard();
            HW::led_idle();
            request_render();
            return;
        } else {
            // Mở menu 2 mục (1: Phân tích Remote 4-bit, 2: Code Detail)
            g_saved_mode_idx = 0;
            enter(ST_SAVED_MODE_SELECT);
            return;
        }
    }
}

static void update_saved_detail() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) {
        enter(ST_SAVED_LIST);
        return;
    }

    // Nếu đang ở màn hình xác nhận Delete
    if (g_delete_confirm) {
        if (eM == HW::EV_CLICK) {
            // Xác nhận xóa
            Storage::remove(idx);
            g_delete_confirm = false;
            HW::led_flash(255, 0, 0, 400);
            if (g_saved_idx > Storage::count()) {
                g_saved_idx = Storage::count();
            }
            enter(ST_SAVED_LIST);
            return;
        }
        if (eL == HW::EV_CLICK || eM == HW::EV_LONG || eL == HW::EV_LONG) {
            // Hủy xóa
            g_delete_confirm = false;
            request_render();
            return;
        }
        return;
    }

    // Khi không ở màn hình xác nhận xóa:
    if (eL == HW::EV_LONG || eM == HW::EV_LONG) {
        enter(ST_SAVED_MODE_SELECT);
        return;
    }

    if (eL == HW::EV_CLICK) {
        g_saved_detail_action = (g_saved_detail_action + 3) % 4;
        request_render();
        return;
    }

    if (eR == HW::EV_CLICK) {
        g_saved_detail_action = (g_saved_detail_action + 1) % 4;
        request_render();
        return;
    }

    if (eM == HW::EV_CLICK) {
        switch (g_saved_detail_action) {
            case 0: // TX (Send / Replay)
                g_play = c;
                if (setup_tx_and_bind()) enter(ST_REPLAY);
                else enter(ST_SAVED_LIST);
                break;

            case 1: // JAMMER
                g_play = c;
                if (setup_tx_and_bind()) {
                    Serial.println(F("[JAMMER] ON"));
                    g_jammer_enter_ms = millis();
                    enter(ST_JAMMER);
                } else {
                    enter(ST_SAVED_LIST);
                }
                break;

            case 2: { // RENAME
                char new_name[MAX_NAME_LEN];
                if (Keyboard::input_text("RENAME CODE", c.name, new_name, MAX_NAME_LEN)) {
                    strncpy(c.name, new_name, MAX_NAME_LEN - 1);
                    c.name[MAX_NAME_LEN - 1] = 0;
                    Storage::update(idx, c);
                    HW::led_flash(0, 255, 0, 300);
                }
                HW::led_idle();
                request_render();
                break;
            }

            case 3: // DELETE
                g_delete_confirm = true;
                request_render();
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// 1. Update menu chọn chế độ (1: Remote 4-Bit, 2: Code Detail)
// ---------------------------------------------------------------------------
static void update_saved_mode_select() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) {
        enter(ST_SAVED_LIST);
        return;
    }

    // Long press để quay lại danh sách
    if (eL == HW::EV_LONG || eM == HW::EV_LONG) {
        enter(ST_SAVED_LIST);
        return;
    }

    if (eL == HW::EV_CLICK) {
        if (g_saved_mode_idx == 0) {
            enter(ST_SAVED_LIST);
            return;
        } else {
            g_saved_mode_idx = 0;
            request_render();
            return;
        }
    }

    if (eR == HW::EV_CLICK) {
        g_saved_mode_idx = 1 - g_saved_mode_idx;
        request_render();
        return;
    }

    if (eM == HW::EV_CLICK) {
        if (g_saved_mode_idx == 1) {
            // Mục 2: Logic như cũ, vào menu chi tiết trong (TX, JAM, REN, DEL)
            g_saved_detail_action = 0;
            g_delete_confirm = false;
            enter(ST_SAVED_DETAIL);
            return;
        } else {
            // Mục 1: Phân tích 4 bit command ở cuối
            uint8_t cmd = (uint8_t)(c.payload & 0x0F);
            g_remote_root_cmd = cmd;
            g_remote_root_btn = detect_root_button(cmd);
            // Mặc định: nếu gốc là nút 4 thì chọn 4, còn lại mặc định 3 (theo ví dụ người dùng)
            g_btn_count = (g_remote_root_btn > 3) ? g_remote_root_btn : 3;
            enter(ST_SAVED_BTN_COUNT);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Update chọn số nút
// ---------------------------------------------------------------------------
static void update_saved_btn_count() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) {
        enter(ST_SAVED_LIST);
        return;
    }

    if (eM == HW::EV_LONG || eL == HW::EV_LONG) {
        enter(ST_SAVED_MODE_SELECT);
        return;
    }

    int min_btn = 2;
    if (g_remote_root_btn > min_btn) min_btn = g_remote_root_btn;
    if (min_btn > 4) min_btn = 4;

    if (eL == HW::EV_CLICK) {
        if (g_btn_count > min_btn) {
            g_btn_count--;
            request_render();
        }
        return;
    }

    if (eR == HW::EV_CLICK) {
        if (g_btn_count < 4) {
            g_btn_count++;
            request_render();
        }
        return;
    }

    if (eM == HW::EV_CLICK) {
        generate_remote_buttons(c, g_btn_count);

        g_play = c;
        g_play.repeats = 15;
        if (setup_tx_and_bind()) {
            g_remote_btn_idx = (g_remote_root_btn >= 1 && g_remote_root_btn <= g_remote_btn_count)
                               ? (g_remote_root_btn - 1) : 0;
            g_btn_tx_ms = 0;
            enter(ST_SAVED_BTN_TX);
        } else {
            HW::led_error();
        }
        return;
    }
}

// ---------------------------------------------------------------------------
// 3. Update danh sách button phát tín hiệu
// ---------------------------------------------------------------------------
static void update_saved_btn_tx() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    int idx = (g_saved_idx > 0) ? (g_saved_idx - 1) : 0;
    SavedCode c;
    if (!Storage::get(idx, c)) {
        enter(ST_SAVED_LIST);
        return;
    }

    // Giữ middle để back về (theo yêu cầu)
    if (eM == HW::EV_LONG || eL == HW::EV_LONG) {
        rf_replay_clear_buttons();
        enter(ST_SAVED_BTN_COUNT);
        return;
    }

    if (eL == HW::EV_CLICK) {
        g_remote_btn_idx = (g_remote_btn_idx - 1 + g_remote_btn_count) % g_remote_btn_count;
        request_render();
        return;
    }

    if (eR == HW::EV_CLICK) {
        g_remote_btn_idx = (g_remote_btn_idx + 1) % g_remote_btn_count;
        request_render();
        return;
    }

    // Bấm middle: phát 15 lần
    if (eM == HW::EV_CLICK) {
        if (!g_play_ok) setup_tx_and_bind();
        if (g_play_ok) {
            float play_freq = c.freq_khz / 1000.0f;
            if (fabsf(rf_replay_get_frequency() - play_freq) > 0.001f) {
                rf_replay_set_frequency(play_freq);
            }

            uint64_t tx_payload = g_remote_btns[g_remote_btn_idx].payload;

            // Hiển thị trạng thái đang phát ngay lập tức
            auto& d = HW::oled;
            d.fillRoundRect(2, 53, 124, 10, 2, SSD1306_WHITE);
            d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            d.setTextSize(1);
            d.setCursor(12, 54);
            d.print(">> SENDING 15x <<");
            d.display();

            uint32_t t0 = micros();
            bool ok = rf_replay_send_raw_code(tx_payload, c.bits, c.T_us, c.inverted, 15);
            uint32_t dt = micros() - t0;

            g_btn_tx_ms = millis();
            g_btn_tx_ok = ok;

            Serial.printf("[BTN TX] %s 0x%llX 15x @ %.3fMHz dt=%lu us ok=%d\n",
                          g_remote_btns[g_remote_btn_idx].label,
                          (unsigned long long)tx_payload, play_freq,
                          (unsigned long)dt, ok);

            if (ok) HW::led_tx_ok();
            else    HW::led_error();
            request_render();
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// ★ DE BRUIJN MENU — render
// ---------------------------------------------------------------------------
static void render_bruce_debruijn_menu() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(1);
    char buf[32];

    // Header info
    d.setCursor(3, 17);
    snprintf(buf, sizeof(buf), "DE BRUIJN BRUTE FORCE");
    d.print(buf);

    // Frequency selector
    d.drawRoundRect(3, 25, 60, 11, 2, SSD1306_WHITE);
    d.setCursor(6, 27); d.print(DB_FREQ_NAMES[g_db_freq_idx]);

    // Bits selector
    d.drawRoundRect(66, 25, 58, 11, 2, SSD1306_WHITE);
    d.setCursor(69, 27);
    snprintf(buf, sizeof(buf), "BITS: %d", g_db_bits);
    d.print(buf);

    // T_us selector
    d.drawRoundRect(3, 39, 60, 11, 2, SSD1306_WHITE);
    d.setCursor(6, 41);
    snprintf(buf, sizeof(buf), "T: %d us", g_db_T_us);
    d.print(buf);

    // Repeats selector
    d.drawRoundRect(66, 39, 58, 11, 2, SSD1306_WHITE);
    d.setCursor(69, 41);
    snprintf(buf, sizeof(buf), "REP: %d", g_db_repeats);
    d.print(buf);

    // Estimated time
    uint32_t total_bits = (1UL << g_db_bits) * g_db_repeats;
    uint32_t est_ms = (total_bits * g_db_T_us) / 1000;
    d.setCursor(3, 54);
    snprintf(buf, sizeof(buf), "EST: ~%lu.%lu s",
             (unsigned long)(est_ms / 1000),
             (unsigned long)((est_ms % 1000) / 100));
    d.print(buf);

    d.setCursor(3, 62);
    d.print("L/R:Sel M:Start M-Long:Back");

    draw_header("DE BRUIJN");
}

static void render_bruce_debruijn_running() {
    auto& d = HW::oled;
    d.clearDisplay();
    d.setTextSize(2);
    d.setCursor(15, 22); d.print("SENDING...");

    d.setTextSize(1);
    char buf[32];
    uint32_t el = millis() - g_db_start_ms;
    snprintf(buf, sizeof(buf), "Elapsed: %lu.%lu s",
             (unsigned long)(el / 1000),
             (unsigned long)((el % 1000) / 100));
    d.setCursor(10, 42); d.print(buf);

    snprintf(buf, sizeof(buf), "%d-bit @ %s",
             g_db_bits, DB_FREQ_NAMES[g_db_freq_idx]);
    d.setCursor(10, 52); d.print(buf);

    d.setCursor(10, 62 - 6); d.print("MID: STOP");

    draw_header("DE BRUIJN TX");
}// ---------------------------------------------------------------------------
// ★ DE BRUIJN MENU — update
// ---------------------------------------------------------------------------
static void update_bruce_debruijn_menu() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    // Long MID = back
    if (eM == HW::EV_LONG) {
        enter(ST_BRUCE_MENU);
        return;
    }

    // LEFT: previous field / decrease value
    if (eL == HW::EV_CLICK) {
        // Cycle through 4 fields: freq, bits, T_us, repeats
        static int field = 0;
        field = (field + 3) % 4;
        // Handle field selection (simplified: we'll use R to cycle fields)
        // For now, let's just cycle values
        return;
    }

    // RIGHT: next field / increase value
    if (eR == HW::EV_CLICK) {
        // Increment current field
        // Simplified: just cycle through presets
        return;
    }

    // MID: start
    if (eM == HW::EV_CLICK) {
        DeBruijn::Config cfg;
        cfg.frequency_mhz = DB_FREQS[g_db_freq_idx];
        cfg.bits          = g_db_bits;
        cfg.T_us          = g_db_T_us;
        cfg.repeats       = g_db_repeats;
        cfg.inverted      = false;
        cfg.sync_d1       = 1;
        cfg.sync_d2       = 31;

        g_db_running = true;
        g_db_start_ms = millis();
        enter(ST_BRUCE_DEBRUIJN_RUNNING);

        // Chạy transmit trong một task riêng hoặc blocking
        // Vì đây là firmware đơn giản, ta sẽ chạy blocking trong update
    }
}
static void update_bruce_debruijn_running() {
    // Nếu đang chạy, kiểm tra nút MID để dừng
    if (DeBruijn::is_running()) {
        if (HW::is_pressed(HW::BTN_MID)) {
            DeBruijn::stop();
            while (HW::is_pressed(HW::BTN_MID)) { HW::buttons_tick(); delay(5); }
            HW::get_event(HW::BTN_LEFT); HW::get_event(HW::BTN_MID); HW::get_event(HW::BTN_RIGHT);
            HW::led_idle();
            enter(ST_BRUCE_DEBRUIJN_MENU);
            return;
        }
    } else {
        // Đã phát xong
        HW::led_flash(0, 255, 0, 500);
        enter(ST_BRUCE_DEBRUIJN_MENU);
        return;
    }
}
static void update_replay() {
    HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    HW::Ev eM = HW::get_event(HW::BTN_MID);
    HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    if (eL == HW::EV_CLICK) {
        rf_replay_clear_buttons();
        if (g_saved_idx > 0) enter(ST_SAVED_DETAIL);
        else enter(ST_SAVED_LIST);
        return;
    }


    if (eM == HW::EV_CLICK && g_play_ok) {
        // ★ Đảm bảo tần số đúng trước mỗi lần phát
        float play_freq = g_play.freq_khz / 1000.0f;          // ★ v2
        if (fabsf(rf_replay_get_frequency() - play_freq) > 0.001f) {
            rf_replay_set_frequency(play_freq);
            Serial.printf("[TX] freq set → %.3f MHz\n", play_freq);
        }

        uint32_t t0 = micros();
        bool ok = rf_replay_send_raw_code(g_play.payload, g_play.bits, g_play.T_us,
                                          g_play.inverted, g_play.repeats);
        uint32_t dt = micros() - t0;
        uint8_t ms1 = rf_replay_get_marcstate();
        Serial.printf("[TX] ok=%d MARCSTATE=0x%02X dt=%lu us payload=0x%llX bits=%u T=%u inv=%d\n",
                      ok, ms1, (unsigned long)dt,
                      (unsigned long long)g_play.payload,
                      g_play.bits, g_play.T_us, g_play.inverted);

        if (ok) HW::led_tx_ok();
        else    HW::led_error();
        request_render();
    }
}

static void update_jammer() {
    HW::led_jammer();
    if (millis() - g_jammer_enter_ms > 500 && HW::is_pressed(HW::BTN_MID)) {
        Serial.println(F("[JAMMER] OFF"));
        while (HW::is_pressed(HW::BTN_MID)) { HW::buttons_tick(); delay(5); }
        HW::get_event(HW::BTN_LEFT); HW::get_event(HW::BTN_MID); HW::get_event(HW::BTN_RIGHT);
        HW::led_idle(); setup_tx_and_bind(); enter(ST_REPLAY); return;
    }
    if (!g_play_ok) return;
    static uint32_t last_log = 0;
    static uint32_t pkt_count = 0;
    rf_replay_send_raw_code(g_play.payload, g_play.bits, g_play.T_us, g_play.inverted, 5);
    pkt_count++;
    if (millis() - last_log > 5000) {
        last_log = millis();
        Serial.printf("[JAMMER] sent %lu pkts\n", (unsigned long)pkt_count);
    }
}

static void update_jammer_all() {
    HW::led_jammer();
    if (millis() - g_jammer_enter_ms > 500 && HW::is_pressed(HW::BTN_MID)) {
        Serial.println(F("[JAM-ALL] stop"));
        while (HW::is_pressed(HW::BTN_MID)) { HW::buttons_tick(); delay(5); }
        HW::get_event(HW::BTN_LEFT); HW::get_event(HW::BTN_MID); HW::get_event(HW::BTN_RIGHT);
        HW::led_idle(); enter(ST_MAIN); return;
    }
    int n = Storage::count();
    if (n == 0) { enter(ST_MAIN); return; }
    if (g_jam_all_idx >= n) { g_jam_all_idx = 0; g_jam_all_last_freq = 0; }
    SavedCode c;
    if (!Storage::get(g_jam_all_idx, c)) { g_jam_all_idx++; return; }
    if (c.freq_khz != g_jam_all_last_freq) {                       // ★ v2
        rf_replay_set_frequency(c.freq_khz / 1000.0f);             // ★ v2
        g_jam_all_last_freq = c.freq_khz;                          // ★ v2
    }
    rf_replay_send_raw_code(c.payload, c.bits, c.T_us, c.inverted, 2);
    g_jam_all_idx++;
    request_render();
}

// ---------------------------------------------------------------------------
// ★ BRUTE FORCE — update menu
// ---------------------------------------------------------------------------
static void update_bruce_menu() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    // ★ Tổng số mục = 5 protocol + 1 mục De Bruijn = 6
    const int total_items = BF_PROTOCOL_COUNT + 1;

    // Navigation cho phép chọn từ 0..(BF_PROTOCOL_COUNT) — index cuối là De Bruijn
    bool moved = nav_step(HW::BTN_LEFT,  eL, -1, g_bf_proto_idx,
                          total_items, g_bf_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_bf_proto_idx,
                     total_items, g_bf_right_nav) || moved;
    if (moved) request_render();

    // LEFT long = thoát về Main menu
    if (eL == HW::EV_LONG) {
        enter(ST_MAIN);
        return;
    }

    // RIGHT long = vào De Bruijn menu nhanh (shortcut)
    if (eR == HW::EV_LONG) {
        g_db_bits     = DEBRUIJN_DEFAULT_BITS;
        g_db_T_us     = DEBRUIJN_DEFAULT_T_US;
        g_db_repeats  = 1;
        g_db_freq_idx = 0;
        enter(ST_BRUCE_DEBRUIJN_MENU);
        return;
    }

    // MID click = chọn mục đang highlight
    if (eM == HW::EV_CLICK) {

        // ── TRƯỜNG HỢP 1: Mục De Bruijn (index cuối cùng) ──
        if (g_bf_proto_idx == BF_PROTOCOL_COUNT) {
            g_db_bits     = DEBRUIJN_DEFAULT_BITS;
            g_db_T_us     = DEBRUIJN_DEFAULT_T_US;
            g_db_repeats  = 1;
            g_db_freq_idx = 0;

            Serial.println(F("[BF] → De Bruijn menu"));
            enter(ST_BRUCE_DEBRUIJN_MENU);
            return;   // ★ PHẢI return để không rơi xuống brute-force
        }

        // ── TRƯỜNG HỢP 2: Protocol brute-force thường (0..BF_PROTOCOL_COUNT-1) ──
        if (bruce_setup_tx(g_bf_proto_idx)) {
            g_bf_code    = 0;
            g_bf_running = true;
            Serial.printf("[BF] START: %s\n", BF_PROTOCOLS[g_bf_proto_idx].name);

            // Flush pending events + chờ user nhả nút MID trước khi vào running
            HW::get_event(HW::BTN_LEFT);
            HW::get_event(HW::BTN_MID);
            HW::get_event(HW::BTN_RIGHT);
            delay(200);

            enter(ST_BRUCE_RUNNING);
        }
    }
}

// ---------------------------------------------------------------------------
// ★ BRUTE FORCE — update running (phát 1 mã / tick, dùng bf_code_at)
// ---------------------------------------------------------------------------
static void update_bruce_running() {
    HW::led_jammer();

    // Nút MID = dừng
    if (HW::is_pressed(HW::BTN_MID)) {
        delay(30);
        if (HW::is_pressed(HW::BTN_MID)) {
            while (HW::is_pressed(HW::BTN_MID)) { HW::buttons_tick(); delay(5); }
            HW::get_event(HW::BTN_LEFT);
            HW::get_event(HW::BTN_MID);
            HW::get_event(HW::BTN_RIGHT);
            Serial.printf("[BF] STOP at code %lu\n", (unsigned long)g_bf_code);
            HW::led_idle();
            g_bf_running = false;
            enter(ST_BRUCE_MENU);
            return;
        }
    }

    if (!g_bf_running) return;

    const BFProtocol& bf = BF_PROTOCOLS[g_bf_proto_idx];

    // Build protocol từ BFProtocol (unsigned hi/lo → signed HIGH/LOW)
    RFProtocolDef proto = bf_to_rfproto(bf);

    // Lấy mã từ lookup table (bf_code_at xử lý offset nội bộ)
    uint64_t payload = (uint64_t)bf_code_at((uint8_t)g_bf_proto_idx, g_bf_code);

    // Gửi 3 lần (BRUCE_REPEATS cũ = 3)
    rf_replay_send_proto(payload, proto, 3);

    HW::buttons_tick();

    // Nghỉ ngắn giữa các mã
    delay(8);

    g_bf_code++;
    if (g_bf_code >= bf.key_count) {
        Serial.printf("[BF] CYCLE DONE (%s)\n", bf.name);
        g_bf_code = 0;   // loop liên tục cho đến khi bấm dừng
    }

    request_render();
}
// ---------------------------------------------------------------------------
// ★ CAPTURE sub-menu — chọn Default / Advanced
// ---------------------------------------------------------------------------
static void update_capture_menu() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    if (eL == HW::EV_CLICK || eL == HW::EV_LONG) {
        enter(ST_MAIN);
        return;
    }

    if (eR == HW::EV_CLICK) {
        g_capture_menu_idx = 1 - g_capture_menu_idx;
        request_render();
    }


    if (eM == HW::EV_CLICK) {
        if (g_capture_menu_idx == 0) {
            // Default capture
            rf_listen_end();
            RFListenerConfig cfg = RF_LISTENER_DEFAULT_CONFIG;
            cfg.max_bits = Storage::get_max_bits();
            cfg.verbose = false;
            cfg.frequency_mhz = Storage::get_frequency();
            cfg.agc_sensitive = true;
            if (!rf_listen_begin(cfg)) { HW::led_error(); return; }
            rf_listen_set_nonblocking(true);
            rf_listen_set_mode(RF_LISTEN_CAPTURE);
            enter(ST_CAPTURE);
        } else {
    // Advanced capture
    AdvCapture::AdvConfig acfg;
    acfg.frequency_mhz   = Storage::get_frequency();
    acfg.noise_filter_us = 80;
    acfg.frame_gap_ms    = 12;
    acfg.max_bits        = Storage::get_max_bits();   // ★ THÊM DÒNG NÀY
    if (!AdvCapture::begin(acfg)) {
        Serial.println(F("[ADV] init FAIL (RAM or CC1101)"));
        HW::led_error();
        return;
    }
    Serial.printf("[ADV] start @ %.3f MHz  cap=%u edges\n",
                  acfg.frequency_mhz, AdvCapture::get_edge_capacity());
    enter(ST_ADV_CAPTURE);
}
    }
}

// ---------------------------------------------------------------------------
// ★ ADV CAPTURE — đang thu
// ---------------------------------------------------------------------------
static void update_adv_capture() {
    // LEFT = cancel
    if (HW::get_event(HW::BTN_LEFT) == HW::EV_CLICK) {
        AdvCapture::cancel();
        enter(ST_CAPTURE_MENU);
        return;
    }
    // MID = stop early
    if (HW::get_event(HW::BTN_MID) == HW::EV_CLICK) {
        AdvCapture::stop_early();
    }

    AdvCapture::update();

    AdvCapture::AdvState s = AdvCapture::get_state();
    if (s == AdvCapture::ADV_ANALYZING) {
        // Render "analyzing" once then run analysis
        render_adv_analyzing();
        HW::oled.display();
        delay(80);
        AdvCapture::analyze();
        g_adv_result = AdvCapture::get_result();
        Serial.printf("[ADV] done  valid=%d  repeats=%u/%u  bits=%u  T=%lu\n",
                      g_adv_result.valid,
                      g_adv_result.repeats, g_adv_result.total_bursts,
                      g_adv_result.bits, (unsigned long)g_adv_result.T_us);
        enter(ST_ADV_RESULT);
        return;
    }
}

// ---------------------------------------------------------------------------
// ★ ADV RESULT — hiển thị winner
// ---------------------------------------------------------------------------
static void update_adv_result() {
    HW::led_idle();
    HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    HW::Ev eM = HW::get_event(HW::BTN_MID);
    HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    // LEFT = back (thu lại)
if (eL == HW::EV_CLICK) {
    AdvCapture::end();
    AdvCapture::AdvConfig acfg;
    acfg.frequency_mhz   = Storage::get_frequency();
    acfg.noise_filter_us = 80;
    acfg.frame_gap_ms    = 12;
    acfg.max_bits        = Storage::get_max_bits();   // ★ THÊM DÒNG NÀY
    if (AdvCapture::begin(acfg)) {
        enter(ST_ADV_CAPTURE);
    } else {
        enter(ST_CAPTURE_MENU);
    }
    return;
}

    // MID = save (giống capture done)
    if (eM == HW::EV_CLICK) {
        if (!g_adv_result.valid && g_adv_result.repeats == 0) {
            // Không có kết quả → không cho save
            return;
        }

        char initial[MAX_NAME_LEN];
        snprintf(initial, sizeof(initial), "Adv%d", Storage::count() + 1);
        char out[MAX_NAME_LEN];

        if (Keyboard::input_text("Save as", initial, out, MAX_NAME_LEN)) {
            SavedCode code = {};
            code.payload  = g_adv_result.payload;
            code.bits     = g_adv_result.bits;
            code.T_us     = (uint16_t)g_adv_result.T_us;
            code.inverted = g_adv_result.inverted;
            code.repeats  = DEFAULT_REPEATS;
            code.freq_khz = g_adv_result.freq_mhz_x1000;
            strncpy(code.name, out, MAX_NAME_LEN - 1);
            code.name[MAX_NAME_LEN - 1] = 0;

            int idx = Storage::add(code);
            if (idx >= 0) {
                HW::led_flash(0, 255, 255, 400);
                Serial.printf("[ADV] SAVED: %s 0x%llX (repeats=%u)\n",
                              code.name, (unsigned long long)code.payload,
                              g_adv_result.repeats);
            } else {
                HW::led_error();
            }
        }
        AdvCapture::end();
        HW::led_idle();
        enter(ST_MAIN);
        return;
    }

    // RIGHT = replay trực tiếp
    if (eR == HW::EV_CLICK) {
        if (g_adv_result.repeats == 0) return;
        g_play.payload   = g_adv_result.payload;
        g_play.bits      = g_adv_result.bits;
        g_play.T_us      = (uint16_t)g_adv_result.T_us;
        g_play.inverted  = g_adv_result.inverted;
        g_play.freq_khz  = g_adv_result.freq_mhz_x1000;
        g_play.repeats   = DEFAULT_REPEATS;
        strncpy(g_play.name, g_adv_result.protocol, MAX_NAME_LEN - 1);
        g_play.name[MAX_NAME_LEN - 1] = 0;
        AdvCapture::end();
        if (setup_tx_and_bind()) enter(ST_REPLAY);
        else enter(ST_MAIN);
        return;
    }
}
// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------
static void update_settings() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);

    if (g_clear_confirm_ms > 0) {
        if (millis() - g_clear_confirm_ms > 3000) {
            g_clear_confirm_ms = 0;
            request_render();
        } else {
            if (eM == HW::EV_CLICK) {
                Storage::clear();
                g_clear_confirm_ms = 0;
                HW::led_flash(255, 0, 0, 500);
                request_render();
            } else if (eL == HW::EV_CLICK || eM == HW::EV_LONG) {
                g_clear_confirm_ms = 0;
                request_render();
            }
            return;
        }
    }

    if (eM == HW::EV_LONG) { enter(ST_MAIN); return; }

    bool moved = nav_step(HW::BTN_LEFT,  eL, -1, g_settings_idx, SET_COUNT, g_settings_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_settings_idx, SET_COUNT, g_settings_right_nav) || moved;
    if (moved) request_render();

        if (eM == HW::EV_CLICK) {
        switch (g_settings_idx) {

            case 0: {   // Frequency
                char cur[16], out[16];
                snprintf(cur, sizeof(cur), "%.3f", Storage::get_frequency());
                if (Keyboard::input_text("Freq (MHz)", cur, out, sizeof(out))) {
                    float v = atof(out);
                    if (v >= 280.0f && v <= 928.0f) {
                        Storage::set_frequency(v);
                        Serial.printf("[SET] freq = %.3f MHz\n", v);
                    } else {
                        HW::led_error();
                        Serial.printf("[SET] invalid freq %.3f\n", v);
                    }
                }
                HW::led_idle();
                request_render();
                break;
            }

            case 1: {   // LED Brightness
                char cur[8], out[8];
                snprintf(cur, sizeof(cur), "%d", Storage::get_led_brightness());
                if (Keyboard::input_text("LED (0-255)", cur, out, sizeof(out))) {
                    int v = atoi(out);
                    if (v >= 0 && v <= 255) {
                        uint8_t b = (uint8_t)v;
                        Storage::set_led_brightness(b);
                        HW::led_set_brightness(b);
                        Serial.printf("[SET] LED = %d\n", v);
                    } else {
                        HW::led_error();
                    }
                }
                HW::led_idle();
                request_render();
                break;
            }

            case 2: {   // ★ v2: Max Bits
                char cur[8], out[8];
                snprintf(cur, sizeof(cur), "%d", Storage::get_max_bits());
                if (Keyboard::input_text("Max Bits", cur, out, sizeof(out))) {
                    int v = atoi(out);
                    if (v >= MIN_MAX_BITS && v <= MAX_MAX_BITS) {
                        Storage::set_max_bits((uint8_t)v);
                        Serial.printf("[SET] max_bits = %d\n", v);
                    } else {
                        HW::led_error();
                        Serial.printf("[SET] max_bits %d out of range (%d-%d)\n",
                                      v, MIN_MAX_BITS, MAX_MAX_BITS);
                    }
                }
                HW::led_idle();
                request_render();
                break;
            }

            case 3:     // Clear Flash (đổi từ 2 → 3)
                g_clear_confirm_ms = millis();
                request_render();
                break;
        }
    }
}

static void update_about() {
    HW::led_idle();
    if (HW::get_event(HW::BTN_LEFT) == HW::EV_CLICK ||
        HW::get_event(HW::BTN_MID)  == HW::EV_CLICK) enter(ST_MAIN);
}

// ===========================================================================
// Public
// ===========================================================================
void begin() { Keyboard::begin(); HW::led_idle(); enter(ST_BOOT); }

void update() {
    HW::buttons_tick();
    HW::led_tick();
    if (emergency_check()) return;

    switch (g_state) {
        case ST_BRUCE_DEBRUIJN_MENU:    update_bruce_debruijn_menu();    break;
case ST_BRUCE_DEBRUIJN_RUNNING: update_bruce_debruijn_running(); break;
    case ST_BOOT:           update_boot();           break;
    case ST_MAIN:           update_main();           break;
    case ST_SCAN:           update_scan();           break;
    case ST_CAPTURE:        update_capture();        break;
    case ST_CAPTURE_DONE:   update_capture_done();   break;
    case ST_SAVED_LIST:     update_saved_list();     break;
    case ST_SAVED_DETAIL:   update_saved_detail();   break;
    case ST_SAVED_MODE_SELECT: update_saved_mode_select(); break;
    case ST_SAVED_BTN_COUNT:   update_saved_btn_count();   break;
    case ST_SAVED_BTN_TX:      update_saved_btn_tx();      break;
    case ST_REPLAY:         update_replay();         break;
    case ST_JAMMER:         update_jammer();         break;
    case ST_JAMMER_ALL:     update_jammer_all();     break;
    case ST_BRUCE_MENU:     update_bruce_menu();     break;
    case ST_BRUCE_RUNNING:  update_bruce_running();  break;
    case ST_SETTINGS:       update_settings();       break;
    case ST_ABOUT:          update_about();          break;
    // ★ v3
    case ST_CAPTURE_MENU:   update_capture_menu();   break;
    case ST_ADV_CAPTURE:    update_adv_capture();    break;
    case ST_ADV_RESULT:     update_adv_result();     break;
    }

    if (should_render()) {
        switch (g_state) {
            case ST_BRUCE_DEBRUIJN_MENU:    render_bruce_debruijn_menu();    break;
case ST_BRUCE_DEBRUIJN_RUNNING: render_bruce_debruijn_running(); break;
            case ST_BOOT:           render_boot();              break;
            case ST_MAIN:           render_main();              break;
            case ST_SCAN:           render_scan();              break;
            case ST_CAPTURE:        render_capture_listening(); break;
            case ST_CAPTURE_DONE:   render_capture_done();      break;
            case ST_SAVED_LIST:     render_saved_list();        break;
            case ST_SAVED_DETAIL:   render_saved_detail();      break;
            case ST_SAVED_MODE_SELECT: render_saved_mode_select(); break;
            case ST_SAVED_BTN_COUNT:   render_saved_btn_count();   break;
            case ST_SAVED_BTN_TX:      render_saved_btn_tx();      break;
            case ST_REPLAY:         render_replay();            break;
            case ST_JAMMER:         render_jammer();            break;
            case ST_JAMMER_ALL:     render_jammer_all();        break;
            case ST_BRUCE_MENU:     render_bruce_menu();        break;
            case ST_BRUCE_RUNNING:  render_bruce_running();     break;
            case ST_SETTINGS:
                if (g_clear_confirm_ms > 0) render_settings_confirm_clear();
                else                        render_settings();
                break;
            case ST_ABOUT:          render_about();             break;
            // ★ v3 — THÊM 3 DÒNG NÀY
            case ST_CAPTURE_MENU:   render_capture_menu();      break;
            case ST_ADV_CAPTURE:    render_adv_capture();       break;
            case ST_ADV_RESULT:     render_adv_result();        break;
        }
        HW::oled.display();
    }

    delay(1);
}

} // namespace Menu