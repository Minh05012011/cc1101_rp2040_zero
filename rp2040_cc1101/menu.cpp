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

namespace Menu {

// ===========================================================================
// States
// ===========================================================================
enum State {
    ST_BOOT = 0, ST_MAIN, ST_SCAN, ST_CAPTURE, ST_CAPTURE_DONE,
    ST_SAVED_LIST, ST_SAVED_DETAIL, ST_REPLAY, ST_JAMMER,
    ST_JAMMER_ALL,
    // ★ Brute force
    ST_BRUCE_MENU, ST_BRUCE_RUNNING,
    ST_SETTINGS, ST_ABOUT
};

static State    g_state = ST_BOOT;
static uint32_t g_enter_ms = 0;
static uint32_t g_clear_confirm_ms = 0;

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
static const char* SET_ITEMS[] = {"Frequency", "LED Bright", "Clear Flash"};
static const int   SET_COUNT = 3;

static int g_saved_idx = 0;
static float g_main_scroll = 0.0f;
static float g_saved_scroll = 0.0f;

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
static uint16_t g_jam_all_last_freq = 0;

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
    d.fillRect(0, 0, 128, 16, SSD1306_WHITE);
    d.setTextSize(1);
    d.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    d.setCursor(2, 4);
    for (int i = 0; title[i] && i < 15; ++i) d.write((uint8_t)title[i]);
    char freq[12];
    snprintf(freq, sizeof(freq), "%.0fM", Storage::get_frequency());
    int fx = 126 - (int)strlen(freq) * 6;
    d.setCursor(fx, 4);
    d.print(freq);
    d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
}

// ===========================================================================
// Emergency exit (hold MID 3s)
// ===========================================================================
static bool emergency_check() {
    if (g_state == ST_BOOT || g_state == ST_JAMMER || g_state == ST_JAMMER_ALL
        || g_state == ST_BRUCE_RUNNING) return false;
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
    draw_header("RF GATEWAY");
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
}

static void render_main() {
    auto& d = HW::oled;
    d.clearDisplay();
    draw_header("RF GATEWAY");
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
        if (y < 7 || y > 61) continue;
        const bool sel = (i == g_main_idx);
        if (sel) d.fillRoundRect(1, y, 126, 9, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(5, y + 1); d.print(sel ? ">" : " ");
        d.setCursor(17, y + 1); d.print(MAIN_ITEMS[i]);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
}

static void render_scan() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("LIVE SCAN");
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
}

static void render_capture_listening() {
    auto& d = HW::oled;
    d.clearDisplay();
    draw_header("SIGNAL CAPTURE");
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
}

static void render_capture_done() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("CAPTURED");
    d.setTextSize(1);
    char b[32];
    d.setCursor(3, 19); snprintf(b, sizeof(b), "PROTO  %s", g_cap_result.protocol); d.print(b);
    d.drawRoundRect(1, 29, 126, 12, 2, SSD1306_WHITE);
    d.setCursor(4, 32); snprintf(b, sizeof(b), "0x%llX", (unsigned long long)g_cap_result.payload); d.print(b);
    d.setCursor(3, 44); snprintf(b, sizeof(b), "BITS %u  T %luus", g_cap_result.bits, (unsigned long)g_cap_result.T_us); d.print(b);
    d.setCursor(3, 55);
    snprintf(b, sizeof(b), "RSSI%ddBm CONF%u%%", g_cap_result.rssi_dbm, g_cap_result.confidence);
    d.print(b);
}

static void render_saved_list() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("CODE LIBRARY");
    const int n = Storage::count();
    const int visible = 5;
    float target = (float)(g_saved_idx - (visible - 1));
    if (target < 0.0f) target = 0.0f;
    const float maxOffset = (float)max(0, n - visible);
    if (target > maxOffset) target = maxOffset;
    g_saved_scroll += (target - g_saved_scroll) * 0.32f;
    if (fabsf(target - g_saved_scroll) < 0.035f) g_saved_scroll = target;
    if (n == 0) { d.setCursor(30, 36); d.print("NO SAVED CODES"); return; }
    for (int i = 0; i < n; ++i) {
        const int y = (int)lroundf(18.0f + ((float)i - g_saved_scroll) * 9.0f);
        if (y < 8 || y > 62) continue;
        SavedCode c;
        if (!Storage::get(i, c)) continue;
        const bool sel = (i == g_saved_idx);
        if (sel) d.fillRoundRect(1, y, 126, 8, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(4, y); d.print(sel ? ">" : " ");
        char line[24];
        snprintf(line, sizeof(line), "%02d %-11.11s %3uM", i + 1, c.name, c.freq_mhz);
        d.setCursor(13, y); d.print(line);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
}

static void render_saved_detail() {
    auto& d = HW::oled;
    d.clearDisplay();
    SavedCode c;
    if (!Storage::get(g_saved_idx, c)) { draw_header("CODE DETAIL"); return; }
    draw_header("CODE DETAIL");
    d.setTextSize(1); char buf[32];
    d.setCursor(3, 19); snprintf(buf, sizeof(buf), "NAME  %.12s", c.name); d.print(buf);
    d.setCursor(3, 29); snprintf(buf, sizeof(buf), "0x%llX", (unsigned long long)c.payload); d.print(buf);
    d.setCursor(3, 39); snprintf(buf, sizeof(buf), "BITS %u  T %u us", c.bits, c.T_us); d.print(buf);
    d.setCursor(3, 49); snprintf(buf, sizeof(buf), "FREQ %uM  REP %u", c.freq_mhz, c.repeats); d.print(buf);
    d.setCursor(3, 59); d.print(c.inverted ? "POLARITY INVERTED" : "POLARITY NORMAL");
}

static void render_replay() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("RELAY READY");
    d.setTextSize(1); char buf[32];
    d.setCursor(3, 19); snprintf(buf, sizeof(buf), "CODE  %.12s", g_play.name); d.print(buf);
    d.setCursor(3, 30); snprintf(buf, sizeof(buf), "0x%llX", (unsigned long long)g_play.payload); d.print(buf);
    d.setCursor(3, 40); snprintf(buf, sizeof(buf), "BITS %u  T %u us", g_play.bits, g_play.T_us); d.print(buf);
    d.setCursor(3, 50); snprintf(buf, sizeof(buf), "FREQ %uM", g_play.freq_mhz); d.print(buf);
    d.drawRoundRect(74, 47, 51, 14, 3, SSD1306_WHITE);
    d.setCursor(87, 51); d.print("READY");
}

static void render_jammer() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("JAMMER MODE");
    d.drawRoundRect(7, 22, 114, 28, 5, SSD1306_WHITE);
    d.setTextSize(2); d.setCursor(20, 29); d.print("ACTIVE");
    d.setTextSize(1); char buf[32];
    snprintf(buf, sizeof(buf), "CODE 0x%llX", (unsigned long long)g_play.payload);
    d.setCursor(3, 54); d.print(buf);
}

static void render_jammer_all() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("JAM ALL");
    d.drawRoundRect(6, 21, 116, 30, 4, SSD1306_WHITE);
    d.setTextSize(2); d.setCursor(25, 27); d.print("RUNNING");
    d.setTextSize(1);
    const int n = Storage::count();
    char buf[32];
    d.setCursor(3, 55); snprintf(buf, sizeof(buf), "ITEM %d / %d", min(g_jam_all_idx + 1, n), n); d.print(buf);
}

// ---------------------------------------------------------------------------
// ★ BRUTE FORCE — render
// ---------------------------------------------------------------------------
static void render_bruce_menu() {
    auto& d = HW::oled;
    d.clearDisplay();
    draw_header("BRUTE FORCE");
    d.setTextSize(1);
    for (int i = 0; i < BF_PROTOCOL_COUNT; i++) {
        const int y = 18 + i * 9;
        if (y > 62) break;
        const bool sel = (i == g_bf_proto_idx);
        if (sel) d.fillRoundRect(1, y, 126, 8, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(5, y + 1); d.print(sel ? ">" : " ");
        char line[32];
        snprintf(line, sizeof(line), "%-8s %2ub %4lus",
                 BF_PROTOCOLS[i].name,
                 BF_PROTOCOLS[i].bits,
                 (unsigned long)BF_PROTOCOLS[i].est_seconds);
        d.setCursor(14, y + 1); d.print(line);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
}

static void render_bruce_running() {
    auto& d = HW::oled;
    d.clearDisplay();
    draw_header("BF SENDING");
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
}

static void render_settings_confirm_clear() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("CLEAR DATA");
    d.setTextSize(1); d.setCursor(17, 27); d.print("DELETE ALL SAVED?");
    d.drawRoundRect(23, 40, 82, 15, 3, SSD1306_WHITE);
    d.setCursor(41, 44); d.print("CONFIRM");
}

static void render_settings() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("PREFERENCES");
    d.setTextSize(1);
    for (int i = 0; i < SET_COUNT; ++i) {
        const int y = 20 + i * 13;
        const bool sel = (i == g_settings_idx);
        if (sel) d.fillRoundRect(1, y, 126, 11, 2, SSD1306_WHITE);
        d.setTextColor(sel ? SSD1306_BLACK : SSD1306_WHITE,
                       sel ? SSD1306_WHITE : SSD1306_BLACK);
        d.setCursor(5, y + 2); d.print(SET_ITEMS[i]);
        char val[16] = {0};
        switch (i) {
            case 0: snprintf(val, sizeof(val), "%.1fM", Storage::get_frequency()); break;
            case 1: snprintf(val, sizeof(val), "%d", Storage::get_led_brightness()); break;
            case 2: snprintf(val, sizeof(val), "RESET"); break;
        }
        d.setCursor(123 - (int)strlen(val) * 6, y + 2); d.print(val);
        d.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
}

static void render_about() {
    auto& d = HW::oled;
    d.clearDisplay(); draw_header("ABOUT");
    d.setTextSize(1);
    d.setCursor(3, 20); d.print("RF GATEWAY  /  V2.0");
    d.setCursor(3, 32); d.print("RP2040 + CC1101");
    char buf[32];
    snprintf(buf, sizeof(buf), "CODES %d / %d", Storage::count(), MAX_SAVED_CODES);
    d.setCursor(3, 44); d.print(buf);
    snprintf(buf, sizeof(buf), "FREQ %.3f MHz", Storage::get_frequency());
    d.setCursor(3, 56); d.print(buf);
}

// ===========================================================================
// SETUP TX + BIND
// ===========================================================================
static bool setup_tx_and_bind() {
    Serial.printf("[REPLAY] init @ %u MHz\n", g_play.freq_mhz);
    rf_listen_end();
    RFReplayConfig cfg = RF_REPLAY_DEFAULT_CONFIG;
    cfg.verbose = false;
    cfg.frequency_mhz = (float)g_play.freq_mhz;
    if (!rf_replay_begin(cfg)) {
        Serial.println(F("[REPLAY] init FAIL"));
        HW::led_error(); return false;
    }
    rf_replay_clear_buttons();
    RFProtocolDef proto{};
    strncpy(proto.name, "Saved", RF_REPLAY_NAME_LEN - 1);
    proto.bits = g_play.bits;
    proto.T_us = g_play.T_us;
    proto.msb_first = true;
    if (!g_play.inverted) {
        proto.sync_d1 = +1; proto.sync_d2 = -31;
        proto.bit0_d1 = +1; proto.bit0_d2 = -3;
        proto.bit1_d1 = +3; proto.bit1_d2 = -1;
    } else {
        proto.sync_d1 = -36; proto.sync_d2 = +1;
        proto.bit0_d1 = -1;  proto.bit0_d2 = +2;
        proto.bit1_d1 = -2;  proto.bit1_d2 = +1;
    }
    int pidx = rf_replay_find_protocol("Saved");
    if (pidx < 0) pidx = rf_replay_add_protocol(proto);
    if (pidx < 0) { HW::led_error(); return false; }
    rf_replay_remove_command("Remote");
    RFCommand cmd{};
    strncpy(cmd.name, "Remote", RF_REPLAY_NAME_LEN - 1);
    cmd.payload = g_play.payload;
    cmd.protocol_idx = pidx;
    cmd.repeats = 1;
    int cidx = rf_replay_add_command(cmd);
    if (cidx < 0) { HW::led_error(); return false; }
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
                cfg.verbose = false; cfg.frequency_mhz = Storage::get_frequency();
                cfg.agc_sensitive = true;
                if (!rf_listen_begin(cfg)) { HW::led_error(); break; }
                rf_listen_set_nonblocking(true);
                rf_listen_set_mode(RF_LISTEN_SCAN);
                enter(ST_SCAN); break;
            }
            case 1: {   // Capture
                rf_listen_end();
                RFListenerConfig cfg = RF_LISTENER_DEFAULT_CONFIG;
                cfg.verbose = false; cfg.frequency_mhz = Storage::get_frequency();
                cfg.agc_sensitive = true;
                if (!rf_listen_begin(cfg)) { HW::led_error(); break; }
                rf_listen_set_nonblocking(true);
                rf_listen_set_mode(RF_LISTEN_CAPTURE);
                enter(ST_CAPTURE); break;
            }
            case 2:     // Code Library
                g_saved_idx = 0;
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
            code.freq_mhz = (uint16_t)Storage::get_frequency();
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
        g_play.freq_mhz  = (uint16_t)(g_cap_result.frequency_mhz_x1000 / 1000);
        g_play.repeats   = DEFAULT_REPEATS;
        strncpy(g_play.name, g_cap_result.protocol, MAX_NAME_LEN - 1);
        g_play.name[MAX_NAME_LEN - 1] = 0;
        if (setup_tx_and_bind()) enter(ST_REPLAY);
        else enter(ST_MAIN);
        return;
    }
}

static void update_saved_list() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);
    if (eM == HW::EV_LONG) { enter(ST_MAIN); return; }
    bool moved = nav_step(HW::BTN_LEFT, eL, -1, g_saved_idx, max(1, Storage::count()), g_saved_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_saved_idx, max(1, Storage::count()), g_saved_right_nav) || moved;
    if (moved) request_render();
    if (eM == HW::EV_CLICK) {
        int n = Storage::count();
        if (n == 0) { enter(ST_MAIN); return; }
        enter(ST_SAVED_DETAIL); return;
    }
}

static void update_saved_detail() {
    HW::led_idle();
    const HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    const HW::Ev eM = HW::get_event(HW::BTN_MID);
    const HW::Ev eR = HW::get_event(HW::BTN_RIGHT);
    if (eL == HW::EV_CLICK) { enter(ST_SAVED_LIST); return; }
    if (eM == HW::EV_LONG)  { enter(ST_MAIN); return; }
    if (eR == HW::EV_CLICK) {
        SavedCode c;
        if (!Storage::get(g_saved_idx, c)) { enter(ST_SAVED_LIST); return; }
        g_play = c;
        if (setup_tx_and_bind()) enter(ST_REPLAY);
        else enter(ST_SAVED_LIST);
        return;
    }
    if (eM == HW::EV_CLICK) {
        SavedCode c;
        if (!Storage::get(g_saved_idx, c)) { enter(ST_SAVED_LIST); return; }
        g_play = c;
        if (setup_tx_and_bind()) {
            Serial.println(F("[JAMMER] ON"));
            g_jammer_enter_ms = millis();
            enter(ST_JAMMER);
        } else {
            enter(ST_SAVED_LIST);
        }
    }
}

static void update_replay() {
    HW::Ev eL = HW::get_event(HW::BTN_LEFT);
    HW::Ev eM = HW::get_event(HW::BTN_MID);
    HW::Ev eR = HW::get_event(HW::BTN_RIGHT);
    if (eL == HW::EV_CLICK) {
        rf_replay_clear_buttons(); enter(ST_SAVED_LIST); return;
    }
    if (eM == HW::EV_CLICK && g_play_ok) {
        uint32_t t0 = micros();
        rf_replay_send_raw_code(g_play.payload, g_play.bits, g_play.T_us,
                                g_play.inverted, g_play.repeats);
        uint32_t dt = micros() - t0;
        uint8_t ms1 = rf_replay_get_marcstate();
        Serial.printf("[TX] MARCSTATE after=0x%02X dt=%lu us\n", ms1, (unsigned long)dt);
        HW::led_tx_ok();
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
    if (c.freq_mhz != g_jam_all_last_freq) {
        rf_replay_set_frequency((float)c.freq_mhz);
        g_jam_all_last_freq = c.freq_mhz;
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

    bool moved = nav_step(HW::BTN_LEFT,  eL, -1, g_bf_proto_idx,
                          BF_PROTOCOL_COUNT, g_bf_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_bf_proto_idx,
                     BF_PROTOCOL_COUNT, g_bf_right_nav) || moved;
    if (moved) request_render();

    if (eL == HW::EV_LONG) {
        enter(ST_MAIN); return;
    }

    if (eM == HW::EV_CLICK) {
        if (bruce_setup_tx(g_bf_proto_idx)) {
            g_bf_code = 0;
            g_bf_running = true;
            Serial.printf("[BF] START: %s\n", BF_PROTOCOLS[g_bf_proto_idx].name);

            // Flush pending events + chờ user nhả nút MID
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
        } else {
            if (eM == HW::EV_CLICK) {
                Storage::clear(); g_clear_confirm_ms = 0; HW::led_flash(255,0,0,500);
            } else if (eL == HW::EV_CLICK || eM == HW::EV_LONG) {
                g_clear_confirm_ms = 0;
            }
            return;
        }
    }
    if (eM == HW::EV_LONG) { enter(ST_MAIN); return; }
    bool moved = nav_step(HW::BTN_LEFT, eL, -1, g_settings_idx, SET_COUNT, g_settings_left_nav);
    moved = nav_step(HW::BTN_RIGHT, eR, +1, g_settings_idx, SET_COUNT, g_settings_right_nav) || moved;
    if (moved) request_render();
    if (eM == HW::EV_CLICK) {
        switch (g_settings_idx) {
            case 0: {
                float v;
                if (Keyboard::input_number("Frequency", Storage::get_frequency(), &v, 0.1f, 280.0f, 928.0f, 3))
                    Storage::set_frequency(v);
                request_render(); break;
            }
            case 1: {
                float v;
                if (Keyboard::input_number("LED Brightness", (float)Storage::get_led_brightness(), &v, 5.0f, 0.0f, 255.0f, 0)) {
                    uint8_t b = (uint8_t)v; Storage::set_led_brightness(b); HW::led_set_brightness(b);
                }
                request_render(); break;
            }
            case 2: g_clear_confirm_ms = millis(); request_render(); break;
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
        case ST_BOOT:           update_boot();           break;
        case ST_MAIN:           update_main();           break;
        case ST_SCAN:           update_scan();           break;
        case ST_CAPTURE:        update_capture();        break;
        case ST_CAPTURE_DONE:   update_capture_done();   break;
        case ST_SAVED_LIST:     update_saved_list();     break;
        case ST_SAVED_DETAIL:   update_saved_detail();   break;
        case ST_REPLAY:         update_replay();         break;
        case ST_JAMMER:         update_jammer();         break;
        case ST_JAMMER_ALL:     update_jammer_all();     break;
        case ST_BRUCE_MENU:     update_bruce_menu();     break;
        case ST_BRUCE_RUNNING:  update_bruce_running();  break;
        case ST_SETTINGS:       update_settings();       break;
        case ST_ABOUT:          update_about();          break;
    }

    if (should_render()) {
        switch (g_state) {
            case ST_BOOT:           render_boot();              break;
            case ST_MAIN:           render_main();              break;
            case ST_SCAN:           render_scan();              break;
            case ST_CAPTURE:        render_capture_listening(); break;
            case ST_CAPTURE_DONE:   render_capture_done();      break;
            case ST_SAVED_LIST:     render_saved_list();        break;
            case ST_SAVED_DETAIL:   render_saved_detail();      break;
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
        }
        HW::oled.display();
    }

    delay(1);
}

} // namespace Menu