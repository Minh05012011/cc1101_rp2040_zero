/*
 * ============================================================================
 *  RF Replay Library — implementation
 * ============================================================================
 */

#include "rf_replay.h"

// ---------------------------------------------------------------------------
// CC1101 register addresses
// ---------------------------------------------------------------------------
#define CC_IOCFG2     0x00
#define CC_IOCFG0     0x02
#define CC_FIFOTHR    0x03
#define CC_PKTLEN     0x06
#define CC_PKTCTRL1   0x07
#define CC_PKTCTRL0   0x08
#define CC_FSCTRL1    0x0B
#define CC_FSCTRL0    0x0C
#define CC_FREQ2      0x0D
#define CC_FREQ1      0x0E
#define CC_FREQ0      0x0F
#define CC_MDMCFG4    0x10
#define CC_MDMCFG3    0x11
#define CC_MDMCFG2    0x12
#define CC_MDMCFG1    0x13
#define CC_MDMCFG0    0x14
#define CC_DEVIATN    0x15
#define CC_MCSM2      0x16
#define CC_MCSM1      0x17
#define CC_MCSM0      0x18
#define CC_FOCCFG     0x19
#define CC_BSCFG      0x1A
#define CC_AGCCTRL2   0x1B
#define CC_AGCCTRL1   0x1C
#define CC_AGCCTRL0   0x1D
#define CC_FREND1     0x21
#define CC_FREND0     0x22
#define CC_FSCAL3     0x23
#define CC_FSCAL2     0x24
#define CC_FSCAL1     0x25
#define CC_FSCAL0     0x26
#define CC_FSTEST     0x29
#define CC_TEST2      0x2C
#define CC_TEST1      0x2D
#define CC_TEST0      0x2E

// Status registers
#define CC_PARTNUM    0x30
#define CC_VERSION    0x31
#define CC_RSSI       0x34
#define CC_MARCSTATE  0x35

// Strobes
#define CC_SRES       0x30
#define CC_SCAL       0x33
#define CC_SRX        0x34
#define CC_STX        0x35
#define CC_SIDLE      0x36
#define CC_SFRX       0x3A
#define CC_SFTX       0x3B

#define CC_PATABLE    0x3E

#define WRITE_BURST   0x40
#define READ_SINGLE   0x80
#define READ_BURST    0xC0

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------
static RFReplayConfig     g_cfg;
static bool               g_initialized   = false;
static bool               g_chip_present  = false;
static RFReplayError      g_last_error    = RF_OK;

static std::vector<RFProtocolDef>    g_protocols;
static std::vector<RFCommand>        g_commands;
static std::vector<RFButtonBinding>  g_buttons;

static volatile bool      g_continuous    = false;
static int8_t             g_cont_cmd_idx  = -1;

static SPISettings        g_spi(1000000, MSBFIRST, SPI_MODE0);

// ---------------------------------------------------------------------------
// SPI primitives
// ---------------------------------------------------------------------------
static inline void cs_low()  { digitalWrite(g_cfg.pin_csn, LOW);  delayMicroseconds(5); }
static inline void cs_high() { digitalWrite(g_cfg.pin_csn, HIGH); delayMicroseconds(5); }

static void cc_strobe(uint8_t cmd) {
    SPI.beginTransaction(g_spi);
    cs_low();
    SPI.transfer(cmd);
    cs_high();
    SPI.endTransaction();
}

static void cc_write(uint8_t addr, uint8_t val) {
    SPI.beginTransaction(g_spi);
    cs_low();
    SPI.transfer(addr);
    SPI.transfer(val);
    cs_high();
    SPI.endTransaction();
}

static uint8_t cc_read(uint8_t addr) {
    SPI.beginTransaction(g_spi);
    cs_low();
    SPI.transfer(addr | READ_SINGLE);
    uint8_t v = SPI.transfer(0x00);
    cs_high();
    SPI.endTransaction();
    return v;
}

static uint8_t cc_read_status(uint8_t addr) {
    SPI.beginTransaction(g_spi);
    cs_low();
    SPI.transfer(addr | READ_BURST);
    uint8_t v = SPI.transfer(0x00);
    cs_high();
    SPI.endTransaction();
    return v;
}

static void cc_write_burst(uint8_t addr, const uint8_t* buf, size_t len) {
    SPI.beginTransaction(g_spi);
    cs_low();
    SPI.transfer(addr | WRITE_BURST);
    for (size_t i = 0; i < len; i++) SPI.transfer(buf[i]);
    cs_high();
    SPI.endTransaction();
}

// ---------------------------------------------------------------------------
// Frequency computation
// ---------------------------------------------------------------------------
static void cc_set_frequency_regs(float mhz) {
    if (mhz <= 0) return;
    uint32_t val = (uint32_t)(((double)mhz * 1e6 * 65536.0) / 26.0e6 + 0.5);
    uint8_t f2 = (val >> 16) & 0xFF;
    uint8_t f1 = (val >> 8)  & 0xFF;
    uint8_t f0 = val & 0xFF;
    cc_write(CC_FREQ2, f2);
    cc_write(CC_FREQ1, f1);
    cc_write(CC_FREQ0, f0);
}

// ---------------------------------------------------------------------------
// ★ Vào TX mode và CHỜ cho đến khi MARCSTATE thực sự = TX
// ---------------------------------------------------------------------------
static bool cc_enter_tx_sync(uint32_t timeout_ms = 50) {
    cc_strobe(CC_SIDLE);
    delayMicroseconds(200);
    cc_strobe(CC_STX);

    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) {
        uint8_t ms = cc_read_status(CC_MARCSTATE) & 0x1F;
        if (ms == 0x13 || ms == 0x14 || ms == 0x15) return true;
        delayMicroseconds(50);
    }
    uint8_t ms = cc_read_status(CC_MARCSTATE) & 0x1F;
    if (g_cfg.verbose) Serial.printf("[RFReplay] TX timeout, MARCSTATE=0x%02X\n", ms);
    return false;
}

// ---------------------------------------------------------------------------
// CC1101 hardware init (OOK async TX)
// ---------------------------------------------------------------------------
static bool cc_hardware_init() {
    pinMode(g_cfg.pin_csn, OUTPUT);
    digitalWrite(g_cfg.pin_csn, HIGH);

    pinMode(g_cfg.pin_gdo0, OUTPUT);
    digitalWrite(g_cfg.pin_gdo0, LOW);

    // Power-on reset toggle
    cs_high(); delayMicroseconds(50);
    cs_low();  delayMicroseconds(50);
    cs_high(); delayMicroseconds(50);
    cc_strobe(CC_SRES);
    delay(10);

    uint8_t part = cc_read_status(CC_PARTNUM);
    uint8_t ver  = cc_read_status(CC_VERSION);

    if (g_cfg.verbose) {
        Serial.printf("[RFReplay] PARTNUM=0x%02X VERSION=0x%02X\n", part, ver);
    }

    if (ver == 0x00 || ver == 0xFF) {
        g_last_error = RF_ERR_NO_CHIP;
        return false;
    }

    // ---- Registers cho OOK async TX ----
    cc_write(CC_IOCFG0,   0x02);   // GDO0 = TX data input (async)
    cc_write(CC_IOCFG2,   0x0D);   // GDO2 = RX data output
    cc_write(CC_FIFOTHR,  0x47);
    cc_write(CC_PKTCTRL0, 0x32);   // async serial, infinite packet
    cc_write(CC_PKTCTRL1, 0x00);
    cc_write(CC_PKTLEN,   0xFF);

    cc_write(CC_FSCTRL1,  0x06);
    cc_write(CC_FSCTRL0,  0x00);

    cc_set_frequency_regs(g_cfg.frequency_mhz);

    cc_write(CC_MDMCFG4,  0x67);   // RX BW ~ 102 kHz
    cc_write(CC_MDMCFG3,  0x32);   // DRATE ~ 2.5 kBaud
    cc_write(CC_MDMCFG2,  0x30);   // OOK/ASK, no preamble, no sync
    cc_write(CC_MDMCFG1,  0x00);
    cc_write(CC_MDMCFG0,  0x00);
    cc_write(CC_DEVIATN,  0x00);

    cc_write(CC_MCSM2,    0x07);
    cc_write(CC_MCSM1,    0x30);
    cc_write(CC_MCSM0,    0x18);

    cc_write(CC_FOCCFG,   0x18);
    cc_write(CC_BSCFG,    0x6C);

    cc_write(CC_AGCCTRL2, 0x07);
    cc_write(CC_AGCCTRL1, 0x00);
    cc_write(CC_AGCCTRL0, 0x92);

    cc_write(CC_FREND1,   0x56);
    cc_write(CC_FREND0,   0x11);

    cc_write(CC_FSCAL3,   0xE9);
    cc_write(CC_FSCAL2,   0x2A);
    cc_write(CC_FSCAL1,   0x00);
    cc_write(CC_FSCAL0,   0x1F);

    cc_write(CC_FSTEST,   0x59);
    cc_write(CC_TEST2,    0x81);
    cc_write(CC_TEST1,    0x35);
    cc_write(CC_TEST0,    0x09);

    uint8_t pa[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc_write_burst(CC_PATABLE, pa, 8);

    cc_strobe(CC_STX);
    delay(5);
    return true;
}

// ---------------------------------------------------------------------------
// Low-level symbol transmit
// ---------------------------------------------------------------------------
static inline void hold_level(int16_t d_T, uint16_t T_us) {
    if (d_T == 0) return;
    if (d_T > 0) {
        digitalWrite(g_cfg.pin_gdo0, HIGH);
        delayMicroseconds((uint32_t)d_T * T_us);
    } else {
        digitalWrite(g_cfg.pin_gdo0, LOW);
        delayMicroseconds((uint32_t)(-d_T) * T_us);
    }
}

static void transmit_packet(const RFProtocolDef& p, uint64_t payload) {
    // Sync
    hold_level(p.sync_d1, p.T_us);
    hold_level(p.sync_d2, p.T_us);

    // Data bits
    for (int i = 0; i < p.bits; i++) {
        int bit_pos = p.msb_first ? (p.bits - 1 - i) : i;
        int bit = (payload >> bit_pos) & 1;

        if (bit == 0) {
            hold_level(p.bit0_d1, p.T_us);
            hold_level(p.bit0_d2, p.T_us);
        } else {
            hold_level(p.bit1_d1, p.T_us);
            hold_level(p.bit1_d2, p.T_us);
        }
    }

    digitalWrite(g_cfg.pin_gdo0, LOW);
}

// ---------------------------------------------------------------------------
// Public API — init
// ---------------------------------------------------------------------------
bool rf_replay_begin(const RFReplayConfig& cfg) {
    g_cfg = cfg;
    g_initialized = false;
    g_chip_present = false;
    g_last_error = RF_OK;

    SPI.setSCK(cfg.pin_sck);
    SPI.setTX (cfg.pin_mosi);
    SPI.setRX (cfg.pin_miso);
    SPI.begin();

    if (!cc_hardware_init()) {
        if (cfg.verbose) Serial.println(F("[RFReplay] CC1101 init FAIL"));
        return false;
    }

    g_chip_present = true;
    g_initialized  = true;

    if (g_protocols.empty()) {
        rf_replay_load_builtin_protocols();
    }

    if (cfg.verbose) {
        Serial.printf("[RFReplay] Ready v%s — %.3f MHz\n",
                      RF_REPLAY_VERSION, g_cfg.frequency_mhz);
    }
    return true;
}

bool rf_replay_available() {
    return g_initialized && g_chip_present;
}

RFReplayError rf_replay_last_error() { return g_last_error; }

const char* rf_replay_error_str(RFReplayError err) {
    switch (err) {
        case RF_OK:               return "OK";
        case RF_ERR_NOT_INIT:     return "not initialized";
        case RF_ERR_NO_CHIP:      return "CC1101 not found";
        case RF_ERR_TABLE_FULL:   return "table full";
        case RF_ERR_NOT_FOUND:    return "not found";
        case RF_ERR_INVALID_PARAM:return "invalid parameter";
        case RF_ERR_BUSY:         return "busy";
        default:                  return "unknown";
    }
}

// ---------------------------------------------------------------------------
// Frequency
// ---------------------------------------------------------------------------
void rf_replay_set_frequency(float mhz) {
    if (mhz < 280.0f || mhz > 928.0f) {
        g_last_error = RF_ERR_INVALID_PARAM;
        return;
    }
    g_cfg.frequency_mhz = mhz;
    if (!g_chip_present) return;

    cc_strobe(CC_SIDLE);
    cc_set_frequency_regs(mhz);
    cc_strobe(CC_STX);
    delay(2);
}

float rf_replay_get_frequency() {
    return g_cfg.frequency_mhz;
}

void rf_replay_set_frequency_raw(uint8_t f2, uint8_t f1, uint8_t f0) {
    if (!g_chip_present) return;
    cc_strobe(CC_SIDLE);
    cc_write(CC_FREQ2, f2);
    cc_write(CC_FREQ1, f1);
    cc_write(CC_FREQ0, f0);
    cc_strobe(CC_STX);
    delay(2);
}

// ---------------------------------------------------------------------------
// Protocols
// ---------------------------------------------------------------------------
int rf_replay_add_protocol(const RFProtocolDef& proto) {
    if (g_protocols.size() >= RF_REPLAY_MAX_PROTOCOLS) {
        g_last_error = RF_ERR_TABLE_FULL;
        return -1;
    }
    g_protocols.push_back(proto);
    return (int)g_protocols.size() - 1;
}

int rf_replay_find_protocol(const char* name) {
    for (size_t i = 0; i < g_protocols.size(); i++) {
        if (strcmp(g_protocols[i].name, name) == 0) return (int)i;
    }
    return -1;
}

const RFProtocolDef* rf_replay_get_protocol(int idx) {
    if (idx < 0 || idx >= (int)g_protocols.size()) return nullptr;
    return &g_protocols[idx];
}

size_t rf_replay_protocol_count() { return g_protocols.size(); }

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
int rf_replay_add_command(const RFCommand& cmd) {
    if (g_commands.size() >= RF_REPLAY_MAX_COMMANDS) {
        g_last_error = RF_ERR_TABLE_FULL;
        return -1;
    }
    if (cmd.protocol_idx >= g_protocols.size()) {
        g_last_error = RF_ERR_INVALID_PARAM;
        return -1;
    }
    g_commands.push_back(cmd);
    return (int)g_commands.size() - 1;
}

int rf_replay_find_command(const char* name) {
    for (size_t i = 0; i < g_commands.size(); i++) {
        if (strcmp(g_commands[i].name, name) == 0) return (int)i;
    }
    return -1;
}

bool rf_replay_remove_command(const char* name) {
    for (size_t i = 0; i < g_commands.size(); i++) {
        if (strcmp(g_commands[i].name, name) == 0) {
            g_commands.erase(g_commands.begin() + i);
            for (auto& b : g_buttons) {
                if (b.cmd_idx == (int8_t)i) b.cmd_idx = -1;
                else if (b.cmd_idx > (int8_t)i) b.cmd_idx--;
            }
            return true;
        }
    }
    g_last_error = RF_ERR_NOT_FOUND;
    return false;
}

const RFCommand* rf_replay_get_command(int idx) {
    if (idx < 0 || idx >= (int)g_commands.size()) return nullptr;
    return &g_commands[idx];
}

size_t rf_replay_command_count() { return g_commands.size(); }

// ---------------------------------------------------------------------------
// Button binding
// ---------------------------------------------------------------------------
bool rf_replay_bind_button(uint8_t pin, int8_t cmd_idx) {
    if (cmd_idx >= (int8_t)g_commands.size()) {
        g_last_error = RF_ERR_INVALID_PARAM;
        return false;
    }

    for (auto& b : g_buttons) {
        if (b.pin == pin) { b.cmd_idx = cmd_idx; return true; }
    }

    if (g_buttons.size() >= RF_REPLAY_MAX_BUTTONS) {
        g_last_error = RF_ERR_TABLE_FULL;
        return false;
    }

    RFButtonBinding b{};
    b.pin = pin;
    b.cmd_idx = cmd_idx;
    b.stable = true;
    b.last_reading = true;
    b.last_change_ms = 0;
    b.press_start_ms = 0;
    g_buttons.push_back(b);
    pinMode(pin, INPUT_PULLUP);
    return true;
}

bool rf_replay_unbind_button(uint8_t pin) {
    for (size_t i = 0; i < g_buttons.size(); i++) {
        if (g_buttons[i].pin == pin) {
            g_buttons.erase(g_buttons.begin() + i);
            return true;
        }
    }
    return false;
}

void rf_replay_clear_buttons() { g_buttons.clear(); }

// ---------------------------------------------------------------------------
// ★ Transmit — dùng cc_enter_tx_sync để đảm bảo chip vào TX
// ---------------------------------------------------------------------------
static bool transmit_command(const RFCommand& cmd, uint8_t repeats) {
    if (!g_chip_present) {
        g_last_error = RF_ERR_NOT_INIT;
        return false;
    }
    if (cmd.protocol_idx >= g_protocols.size()) {
        g_last_error = RF_ERR_INVALID_PARAM;
        return false;
    }

    const RFProtocolDef& p = g_protocols[cmd.protocol_idx];

    if (!cc_enter_tx_sync(50)) {
        g_last_error = RF_ERR_BUSY;
        return false;
    }

    pinMode(g_cfg.pin_gdo0, OUTPUT);
    digitalWrite(g_cfg.pin_gdo0, LOW);
    delayMicroseconds(50);   // PA ramp

    for (uint8_t r = 0; r < repeats; r++) {
        transmit_packet(p, cmd.payload);
    }

    digitalWrite(g_cfg.pin_gdo0, LOW);
    return true;
}

bool rf_replay_send(int cmd_idx) {
    if (cmd_idx < 0 || cmd_idx >= (int)g_commands.size()) {
        g_last_error = RF_ERR_NOT_FOUND;
        return false;
    }
    const RFCommand& c = g_commands[cmd_idx];
    if (g_cfg.verbose) {
        Serial.printf("[RFReplay] TX %s: 0x%llX x%u @ %.3f MHz\n",
                      c.name, (unsigned long long)c.payload,
                      c.repeats, g_cfg.frequency_mhz);
    }
    return transmit_command(c, c.repeats);
}

bool rf_replay_send_by_name(const char* name) {
    int idx = rf_replay_find_command(name);
    if (idx < 0) { g_last_error = RF_ERR_NOT_FOUND; return false; }
    return rf_replay_send(idx);
}

bool rf_replay_send_raw(const RFCommand& cmd) {
    return transmit_command(cmd, cmd.repeats);
}

bool rf_replay_send_continuous(int cmd_idx) {
    if (cmd_idx < 0 || cmd_idx >= (int)g_commands.size()) {
        g_last_error = RF_ERR_NOT_FOUND;
        return false;
    }
    g_continuous    = true;
    g_cont_cmd_idx  = cmd_idx;
    return true;
}

void rf_replay_stop_continuous() {
    g_continuous   = false;
    g_cont_cmd_idx = -1;
}

bool rf_replay_is_continuous() { return g_continuous; }

// ---------------------------------------------------------------------------
// ★ Send raw code với cc_enter_tx_sync
// ---------------------------------------------------------------------------
bool rf_replay_send_raw_code(uint64_t payload, uint8_t bits, uint16_t T_us,
                             bool inverted, uint16_t repeats) {
    if (!g_chip_present) {
        g_last_error = RF_ERR_NOT_INIT;
        return false;
    }
    if (repeats == 0) repeats = 1;

    RFProtocolDef proto{};
    strcpy(proto.name, "RAW");
    proto.bits      = bits;
    proto.T_us      = T_us;
    proto.msb_first = true;

    if (!inverted) {
        proto.sync_d1 = +1;  proto.sync_d2 = -31;
        proto.bit0_d1 = +1;  proto.bit0_d2 = -3;
        proto.bit1_d1 = +3;  proto.bit1_d2 = -1;
    } else {
        proto.sync_d1 = -36; proto.sync_d2 = +1;
        proto.bit0_d1 = -1;  proto.bit0_d2 = +2;
        proto.bit1_d1 = -2;  proto.bit1_d2 = +1;
    }

    if (!cc_enter_tx_sync(50)) {
        g_last_error = RF_ERR_BUSY;
        return false;
    }

    pinMode(g_cfg.pin_gdo0, OUTPUT);
    digitalWrite(g_cfg.pin_gdo0, LOW);
    delayMicroseconds(50);   // PA ramp

    for (uint16_t r = 0; r < repeats; r++) {
        transmit_packet(proto, payload);
    }

    digitalWrite(g_cfg.pin_gdo0, LOW);
    return true;
}


// ---------------------------------------------------------------------------
// ★ v2: send_proto — phát với full RFProtocolDef tùy chỉnh (dùng cho Bruce)
// ---------------------------------------------------------------------------
bool rf_replay_send_proto(uint64_t payload, const RFProtocolDef& proto, uint16_t repeats) {
    if (!g_chip_present) { g_last_error = RF_ERR_NOT_INIT; return false; }
    if (repeats == 0) repeats = 1;
    if (!cc_enter_tx_sync(50)) { g_last_error = RF_ERR_BUSY; return false; }
    pinMode(g_cfg.pin_gdo0, OUTPUT);
    digitalWrite(g_cfg.pin_gdo0, LOW);
    delayMicroseconds(50);
    for (uint16_t r = 0; r < repeats; r++) transmit_packet(proto, payload);
    digitalWrite(g_cfg.pin_gdo0, LOW);
    return true;
}

// ---------------------------------------------------------------------------
// ★ MARCSTATE + CW test (cho menu debug)
// ---------------------------------------------------------------------------
uint8_t rf_replay_get_marcstate() {
    if (!g_chip_present) return 0xFF;
    return cc_read_status(CC_MARCSTATE) & 0x1F;
}

void rf_replay_test_cw(uint16_t duration_ms) {
    if (!g_chip_present) {
        Serial.println(F("[CW] chip not present"));
        return;
    }

    Serial.println(F("[CW] start"));
    bool ok = cc_enter_tx_sync(100);
    uint8_t ms = cc_read_status(CC_MARCSTATE) & 0x1F;
    Serial.printf("[CW] enter_tx=%d MARCSTATE=0x%02X\n", ok, ms);

    if (!ok) {
        Serial.println(F("[CW] FAILED to enter TX"));
        return;
    }

    pinMode(g_cfg.pin_gdo0, OUTPUT);
    digitalWrite(g_cfg.pin_gdo0, HIGH);
    delay(duration_ms);
    digitalWrite(g_cfg.pin_gdo0, LOW);

    ms = cc_read_status(CC_MARCSTATE) & 0x1F;
    Serial.printf("[CW] stop, MARCSTATE=0x%02X\n", ms);
}

// ---------------------------------------------------------------------------
// Update — quét nút
// ---------------------------------------------------------------------------
void rf_replay_update() {
    unsigned long now = millis();

    if (g_continuous && g_cont_cmd_idx >= 0) {
        const RFCommand& c = g_commands[g_cont_cmd_idx];
        transmit_command(c, 1);

        if (Serial.available()) {
            int peek = Serial.peek();
            if (peek == 'k' || peek == 'K') {
                while (Serial.available()) Serial.read();
                g_continuous = false;
                g_cont_cmd_idx = -1;
                if (g_cfg.verbose) Serial.println(F("[RFReplay] Continuous STOP"));
            }
        }
        return;
    }

    for (auto& b : g_buttons) {
        bool reading = digitalRead(b.pin);

        if (reading != b.last_reading) {
            b.last_change_ms = now;
            b.last_reading = reading;
        }

        if ((now - b.last_change_ms) > g_cfg.button_debounce_ms
            && reading != b.stable) {
            b.stable = reading;

            if (b.stable == LOW) {
                b.press_start_ms = now;
                if (g_cfg.verbose) {
                    Serial.printf("[RFReplay] BTN GP%u PRESS -> cmd %d\n",
                                  b.pin, b.cmd_idx);
                }
                if (b.cmd_idx >= 0) rf_replay_send(b.cmd_idx);
            } else {
                if (g_cfg.verbose) {
                    Serial.printf("[RFReplay] BTN GP%u RELEASE (%lu ms)\n",
                                  b.pin, now - b.press_start_ms);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Serial CLI
// ---------------------------------------------------------------------------
bool rf_replay_handle_serial(Stream& in) {
    if (!in.available()) return false;

    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return false;

    char cmd = line.charAt(0);
    String arg = line.substring(1); arg.trim();

    switch (cmd) {
        case 'r': {
            if (arg.length() == 0) rf_replay_send(0);
            else rf_replay_send(arg.toInt());
            break;
        }
        case 'R': {
            int idx = arg.length() ? arg.toInt() : 0;
            if (rf_replay_send_continuous(idx)) {
                Serial.printf(">> continuous cmd %d\n", idx);
            }
            break;
        }
        case 'k':
            rf_replay_stop_continuous();
            Serial.println(F(">> continuous stopped"));
            break;
        case 'f': {
            float f = arg.toFloat();
            rf_replay_set_frequency(f);
            Serial.printf(">> freq = %.3f MHz\n", rf_replay_get_frequency());
            break;
        }
        case 's': rf_replay_print_status();    break;
        case 'p': rf_replay_print_protocols(); break;
        case 'c': rf_replay_print_commands();  break;
        case 'd': rf_replay_dump_registers();  break;
        case '?': rf_replay_print_help();      break;
        default:
            Serial.println(F("?? unknown — type '?'"));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Print helpers
// ---------------------------------------------------------------------------
void rf_replay_print_status(Stream& out) {
    out.println(F("--- RFReplay status ---"));
    out.printf("Version   : %s\n", RF_REPLAY_VERSION);
    out.printf("Init      : %s\n", g_initialized ? "yes" : "no");
    out.printf("Chip      : %s\n", g_chip_present ? "CC1101" : "not detected");
    out.printf("Frequency : %.3f MHz\n", g_cfg.frequency_mhz);
    out.printf("Protocols : %u\n", (unsigned)g_protocols.size());
    out.printf("Commands  : %u\n", (unsigned)g_commands.size());
    out.printf("Buttons   : %u\n", (unsigned)g_buttons.size());
    for (auto& b : g_buttons) {
        out.printf("   GP%u -> cmd %d\n", b.pin, b.cmd_idx);
    }
    out.println(F("-----------------------"));
}

void rf_replay_print_help(Stream& out) {
    out.println();
    out.println(F("=== RFReplay CLI ==="));
    out.println(F("  r [idx]  - send command (default 0)"));
    out.println(F("  R [idx]  - continuous send"));
    out.println(F("  k        - stop continuous"));
    out.println(F("  f <mhz>  - change frequency"));
    out.println(F("  s        - status"));
    out.println(F("  p        - list protocols"));
    out.println(F("  c        - list commands"));
    out.println(F("  d        - dump registers"));
    out.println(F("  ?        - this help"));
    out.println();
}

void rf_replay_print_protocols(Stream& out) {
    out.printf("Protocols (%u):\n", (unsigned)g_protocols.size());
    for (size_t i = 0; i < g_protocols.size(); i++) {
        const auto& p = g_protocols[i];
        out.printf("  [%u] %-12s bits=%u T=%uus sync=%+d/%+d b0=%+d/%+d b1=%+d/%+d\n",
                   (unsigned)i, p.name, p.bits, p.T_us,
                   p.sync_d1, p.sync_d2,
                   p.bit0_d1, p.bit0_d2,
                   p.bit1_d1, p.bit1_d2);
    }
}

void rf_replay_print_commands(Stream& out) {
    out.printf("Commands (%u):\n", (unsigned)g_commands.size());
    for (size_t i = 0; i < g_commands.size(); i++) {
        const auto& c = g_commands[i];
        out.printf("  [%u] %-12s payload=0x%llX proto=%u repeats=%u\n",
                   (unsigned)i, c.name,
                   (unsigned long long)c.payload,
                   c.protocol_idx, c.repeats);
    }
}

// ---------------------------------------------------------------------------
// Register debug
// ---------------------------------------------------------------------------
uint8_t rf_replay_read_register(uint8_t addr) { return cc_read(addr); }
uint8_t rf_replay_read_status(uint8_t addr)   { return cc_read_status(addr); }

void rf_replay_dump_registers(Stream& out) {
    out.println(F("--- CC1101 config registers ---"));
    for (uint8_t a = 0x00; a <= 0x2E; a++) {
        uint8_t v = cc_read(a);
        out.printf("  %02X : %02X\n", a, v);
    }
    out.println(F("--- status ---"));
    out.printf("  PARTNUM  : %02X\n", cc_read_status(CC_PARTNUM));
    out.printf("  VERSION  : %02X\n", cc_read_status(CC_VERSION));
    out.printf("  RSSI     : %02X\n", cc_read_status(CC_RSSI));
    out.printf("  MARCSTATE: %02X\n", cc_read_status(CC_MARCSTATE));
}

// ---------------------------------------------------------------------------
// Built-in protocols
// ---------------------------------------------------------------------------
void rf_replay_load_builtin_protocols() {
    RFProtocolDef ev1527{};
    strncpy(ev1527.name, "EV1527", RF_REPLAY_NAME_LEN);
    ev1527.bits      = 24;
    ev1527.T_us      = 415;
    ev1527.sync_d1   = +1;    ev1527.sync_d2   = -31;
    ev1527.bit0_d1   = +1;    ev1527.bit0_d2   = -3;
    ev1527.bit1_d1   = +3;    ev1527.bit1_d2   = -1;
    ev1527.msb_first = true;
    rf_replay_add_protocol(ev1527);

    RFProtocolDef pt2262 = ev1527;
    strncpy(pt2262.name, "PT2262", RF_REPLAY_NAME_LEN);
    pt2262.T_us = 350;
    rf_replay_add_protocol(pt2262);

    RFProtocolDef came{};
    strncpy(came.name, "Came", RF_REPLAY_NAME_LEN);
    came.bits      = 12;
    came.T_us      = 320;
    came.sync_d1   = -36;    came.sync_d2   = +1;
    came.bit0_d1   = -1;     came.bit0_d2   = +2;
    came.bit1_d1   = -2;     came.bit1_d2   = +1;
    came.msb_first = true;
    rf_replay_add_protocol(came);

    RFProtocolDef nice{};
    strncpy(nice.name, "Nice", RF_REPLAY_NAME_LEN);
    nice.bits      = 12;
    nice.T_us      = 700;
    nice.sync_d1   = -36;    nice.sync_d2   = +1;
    nice.bit0_d1   = -1;     nice.bit0_d2   = +2;
    nice.bit1_d1   = -2;     nice.bit1_d2   = +1;
    nice.msb_first = true;
    rf_replay_add_protocol(nice);
}