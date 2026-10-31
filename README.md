# RF Gateway — RP2040 Zero + CC1101 + OLED + WS2812B

Thiết bị RF đa năng cầm tay: nghe/quét sóng 300–928MHz, bắt mã (capture — chuẩn, nâng cao, hoặc raw waveform), lưu thư viện mã, phát lại (replay), jammer đa chế độ, và một module brute-force **chỉ mang tính demo/minh họa** (xem phần cảnh báo bên dưới).

Firmware viết cho RP2040 (Arduino core), giao tiếp CC1101 qua SPI thủ công (không dùng thư viện ELECHOUSE).

---

## ⚠️ CẢNH BÁO QUAN TRỌNG 

### Brute Force (5 protocol) và De Bruijn: KHÔNG hoạt động thực tế — chỉ DEMO

Dự án có 2 tính năng "brute force" trong menu:

1. **Brute Force (CAME / NICE / Holtek / EV1527 / Ansonic)** — dùng bảng mã sinh sẵn `bruteforce_codes.h` (20480 mã, ~4MB flash) để phát tuần tự từng mã 12/20-bit.
2. **De Bruijn (FAST)** — thuật toán nén chuỗi brute-force bằng De Bruijn sequence (`debruijn.cpp`) để giảm số bit cần phát.

**Thực tế:**
- Cả hai chỉ chạy được ở mức *trình diễn giao diện* (quét UI, thanh tiến trình, đếm mã...). Việc phát sóng có chạy (gọi `rf_replay_send_proto` / `rf_replay_transmit_raw_bits`), nhưng **không có gì đảm bảo thứ tự timing, encoding, hoặc lựa chọn mã là đúng với cổng thật ngoài đời** — đây không phải brute-force "được kiểm chứng hoạt động trên cổng thật", mà là khung sườn thuật toán minh họa cách một brute-force tool *có thể* được viết.
- Riêng De Bruijn: màn hình cấu hình (`update_bruce_debruijn_menu`) có code điều hướng LEFT/RIGHT nhưng **chưa cài logic đổi field/giá trị** — nút trái/phải hiện không làm gì, bạn chỉ có thể bấm MID để chạy với giá trị mặc định cứng (12-bit, 350µs, 433.92MHz, 1 lần lặp).
- Thời gian ước tính trên màn hình (vd CAME ~224s cho 4096 mã) là lý thuyết, **chưa test với cổng thật nào** trong phạm vi dự án này.
- Nếu bạn cần brute-force thực sự dùng được, nên tham khảo Flipper Zero hoặc ESP32 Bruce Firmware (dự án này có mượn ý tưởng/toàn bộ bảng mã từ `flipperzero-bruteforce`) và tự kiểm chứng timing bằng máy phân tích sóng.

**Tất cả các chức năng còn lại (Scan, Capture thường/nâng cao/raw, Code Library, Replay, Jammer, Settings) đã được code đầy đủ logic và nhằm mục đích hoạt động thật.**

### RF Jammer: chỉ dùng hợp pháp, trên thiết bị của chính bạn

Phiên bản này mở rộng module Jammer với nhiều chế độ hơn (phát sóng liên tục, nhảy tần, noise băng rộng...). Việc phát sóng gây nhiễu trên băng tần được cấp phép là **bất hợp pháp tại hầu hết các quốc gia** kể cả khi chỉ thử nghiệm ngắn. Phần [4.5](#45-rf-jammer--phát-lại-liên-tục) chỉ mô tả các chế độ đã có trong code ở mức tổng quan, không phải hướng dẫn tối ưu công suất/hiệu quả gây nhiễu. **Chỉ bật tính năng này trong môi trường được che chắn RF (RF-shielded) hoặc trên hệ thống/tần số bạn được phép thử nghiệm hợp pháp.**

---

## 1. Tính năng

| Tính năng | Trạng thái | Mô tả |
|---|---|---|
| Live Scan | Đầy đủ | Quét dải tần cố định (300–928MHz), hiển thị RSSI + sóng ECG-style, có **lịch sử tần số đã bắt được tín hiệu thật** (Scan History) để chọn lại nhanh |
| Signal Capture (Standard) | Đầy đủ | Bắt 1 frame tín hiệu OOK, decode EV1527/PT2262/Came/Nice/Holtek/Ansonic tự động |
| Advanced Capture | Đầy đủ | Gom nhiều frame, lọc nhiễu 3-pass, tìm payload lặp lại nhiều lần để tăng độ tin cậy |
| **Raw Capture (LISTEN RAW)** | **Mới** | Bắt tín hiệu RF theo đúng dạng sóng thô (danh sách edge + độ dài xung), dùng cho remote/protocol mà bộ decode chuẩn không nhận ra được; lưu và replay lại nguyên waveform đã bắt, không cần decode ra payload/bits |
| Code Library | Đầy đủ | Lưu mã **PROTO** (đã decode) và mã **RAW** (waveform thô) vào flash, đổi tên, xóa, replay, jam |
| 4-Bit Remote Analysis | Đầy đủ | Suy ra 2–4 nút bấm còn lại từ 1 mã PROTO gốc (cho remote dùng 4-bit command fixed-code) |
| Replay (TX) | Đầy đủ | Phát lại mã PROTO (đúng timing gốc) hoặc mã RAW (đúng waveform đã ghi) |
| RF Jammer — Jamming (Continuous) | Đầy đủ | Phát lặp liên tục 1 mã đã chọn trong Code Library |
| RF Jammer — Jam Delay | Đầy đủ | Phát lặp 1 mã đã chọn theo chu kỳ cấu hình được (LOW/MEDIUM/HIGH) thay vì liên tục |
| Jam All | Đầy đủ | Phát lần lượt toàn bộ thư viện mã đã lưu |
| **Jammer module nâng cao** (`jammer.cpp`) | Đầy đủ (code), cảnh báo pháp lý | Engine jammer độc lập hỗ trợ nhiều chế độ: CW Spot (1 tần số cố định), Hop CW (nhảy qua danh sách tần số), OOK Noise (nhiễu băng hẹp), Hop OOK, Spam (phát 1 payload tuỳ chỉnh lặp lại) |
| Settings | Đầy đủ | Đổi tần số mặc định, độ sáng LED, max bits decode, xóa toàn bộ flash |
| Brute Force 5 protocol | **DEMO** | Xem cảnh báo ở trên |
| De Bruijn | **DEMO, UI chưa hoàn thiện** | Xem cảnh báo ở trên |

---

## 2. Phần cứng cần có

- 1x board RP2040-Zero (hoặc tương thích RP2040, cần đủ chân GPIO như bảng dưới)
- 1x module CC1101 (433MHz hoặc universal 300–928MHz, có anten phù hợp)
- 1x màn hình OLED SSD1306 128x64, giao tiếp I2C, địa chỉ `0x3C`
- 3x nút bấm (LEFT, MID, RIGHT) — momentary switch, nối GND
- 1x WS2812B (NeoPixel) — nếu dùng board có LED onboard thì khỏi cần thêm (RP2040-Zero có sẵn ở GP16)
- Dây breadboard, nguồn 5V hoặc pin (board RP2040-Zero dùng USB-C cấp nguồn + nạp code)

### 2.1 Sơ đồ nối dây

```
RP2040-Zero          OLED SSD1306 (I2C)
  GP0  ────────────── SDA
  GP1  ────────────── SCL
  3V3  ────────────── VCC
  GND  ────────────── GND

RP2040-Zero          CC1101 (SPI)
  GP2  ────────────── SCK
  GP3  ────────────── MOSI (SI)
  GP4  ────────────── MISO (SO)
  GP5  ────────────── CSN
  GP6  ────────────── GDO0
  3V3  ────────────── VCC   (⚠ CC1101 dùng 3.3V, KHÔNG cấp 5V)
  GND  ────────────── GND

RP2040-Zero          Nút bấm (momentary, chân kia nối GND)
  GP9  ────────────── LEFT
  GP11 ────────────── MID
  GP13 ────────────── RIGHT

RP2040-Zero          WS2812B
  GP16 ────────────── DIN  (board RP2040-Zero có sẵn LED tại GP16, không cần nối thêm nếu dùng LED onboard)
```

Giá trị pin đầy đủ nằm trong `config.h` (`PIN_I2C_SDA`, `PIN_CC_SCK`, `PIN_BTN_LEFT`, `PIN_LED`, …) — đổi ở đó nếu board bạn khác layout.

> ⚠️ CC1101 chạy 3.3V, RP2040-Zero GPIO cũng là 3.3V logic nên tương thích trực tiếp, không cần level shifter. Tuyệt đối không cấp 5V vào VCC của module CC1101.

---

## 3. Cài đặt firmware (Arduino IDE)

### 3.1 Thêm board RP2040

1. Arduino IDE → **File → Preferences** → Additional Boards Manager URLs, thêm:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
2. **Tools → Board → Boards Manager**, tìm "Raspberry Pi Pico/RP2040" (by Earle F. Philhower, III), cài đặt.
3. **Tools → Board**, chọn board tương ứng (ví dụ "Waveshare RP2040 Zero" hoặc "Generic RP2040" tùy board bạn mua).

### 3.2 Cài thư viện (Library Manager)

- `Adafruit SSD1306`
- `Adafruit GFX Library`
- `Adafruit NeoPixel`

(SPI, Wire, EEPROM đã có sẵn trong core RP2040.)

### 3.3 Cấu trúc file project

Đặt tất cả các file sau **cùng 1 thư mục**, tên thư mục phải trùng tên file `.ino` chính:

```
rp2040_cc1101/
├── rp2040_cc1101.ino        (file chính, setup()/loop())
├── config.h                 (pin map, hằng số cấu hình)
├── hardware.h / .cpp        (OLED, LED — gồm theme màu + hiệu ứng rainbow/jammer, nút bấm)
├── storage.h   / .cpp       (lưu flash — mã PROTO + mã RAW, 64KB)
├── keyboard.h  / .cpp       (bàn phím ảo trên OLED để nhập tên/số)
├── menu.h      / .cpp       (toàn bộ state machine UI)
├── rf_listen.h / .cpp       (nghe/scan/capture chuẩn)
├── rf_raw_capture.h / .cpp  (★ capture raw waveform — không qua decoder protocol)
├── rf_replay.h / .cpp       (phát/TX)
├── jammer.h    / .cpp       (★ engine jammer đa chế độ: CW/Hop/OOK Noise/Spam)
├── advanced_capture.h / .cpp (advanced capture 3-pass)
├── debruijn.h  / .cpp       (De Bruijn — DEMO)
├── bruce_codes.h            (5 loại cổng cho De Bruijn — DEMO)
└── bruteforce_codes.h       (bảng mã sinh sẵn 20480 mã — DEMO)
```

### 3.4 Nạp code

1. Nối board qua USB-C, giữ nút BOOTSEL khi cắm nếu lần đầu nạp (tùy board).
2. Chọn đúng **Port** trong Arduino IDE.
3. Bấm **Upload**.
4. Mở **Serial Monitor** ở baud `115200` để xem log debug (`[STORAGE]`, `[REPLAY]`, `[CAPTURE]`, `[RAW]`, …) — rất hữu ích khi gỡ lỗi.

Khi boot thành công, OLED hiện màn hình "RF GATEWAY — STARTING SYSTEM" khoảng 2 giây rồi vào menu chính.

---

## 4. Hướng dẫn dùng (điều khiển 3 nút)

Nguyên tắc chung xuyên suốt toàn bộ menu:

- **LEFT / RIGHT**: di chuyển lựa chọn hoặc lùi/tiến giá trị. Giữ lâu (>320ms) sẽ tự lặp (nhanh dần) để cuộn danh sách dài.
- **MID (click)**: xác nhận/chọn mục.
- **MID (giữ 3 giây)** ở hầu hết màn hình: thoát khẩn cấp về Main Menu (trừ lúc đang Jammer/Brute Force đang chạy — khi đó phải dừng theo cách riêng của màn hình đó).

### 4.1 Menu chính

7 mục, cuộn bằng LEFT/RIGHT, chọn bằng MID:

```
Live Scan | Signal Capture | Code Library | RF Jammer | Brute Force | Settings | About
```

### 4.2 Live Scan — dò tần số

Mục đích: tìm tần số remote của bạn đang phát khi bạn chưa biết chính xác (CC1101 chỉ nghe tốt ở 1 tần số tại 1 thời điểm, nên máy quét lần lượt qua danh sách tần số phổ biến: 300–928MHz, dày nhất quanh dải 433MHz).

Vào **Live Scan** có 3 lựa chọn (LEFT/RIGHT để chuyển): **Full Range** (quét toàn dải), **433+315 MHz Fast** (quét nhanh 2 băng phổ biến nhất), **History** (xem lại các tần số đã từng bắt được tín hiệu thật, chọn lại nhanh bằng MID).

Cách dùng:
1. Chọn chế độ quét, bấm **MID**.
2. Đứng gần remote, bấm nút remote liên tục vài lần trong lúc máy đang quét.
3. Khi đồ thị hiện "SIGNAL: STRONG/MEDIUM" và tần số đỉnh (vd `433.920 MHz`) kèm dBm, bấm **MID** để lưu tần số này làm mặc định và **tự động chuyển sang Signal Capture** ở đúng tần số đó. Các lần bắt tín hiệu thật (đúng băng remote + RSSI đủ mạnh) sẽ tự được ghi vào **History**.
4. **LEFT** để thoát về Main.

### 4.3 Signal Capture — bắt mã (3 chế độ)

Vào **Signal Capture** → chọn 1 trong 3 card bằng LEFT/RIGHT để chuyển, MID để chọn:

**a) Standard Capture** (nhanh, 1-frame)
- Dao động ký đơn giản hiển thị trên màn hình khi có tín hiệu.
- Bấm remote 1 lần. Nếu decode được (đủ edge, sync hợp lệ, bits trong khoảng 12–max_bits), máy tự nhảy sang màn hình kết quả.
- Ở màn hình kết quả (**Captured**): `MID` = mở bàn phím ảo đặt tên rồi lưu vào Code Library; `RIGHT` = phát lại ngay không cần lưu; `LEFT` = bắt lại.

**b) Advanced Capture** (chậm hơn, độ tin cậy cao hơn)
- Giữ nút remote **ấn liên tục nhiều lần trong vài giây** (khuyến nghị giữ 2–3 giây, bấm nhả nhiều lần) để máy gom đủ số lần lặp giống nhau.
- Máy tự dừng sau khi: buffer đầy, bạn bấm MID, hoặc 10 giây không có frame hợp lệ mới.
- Kết quả hiện `REPEATS x/y` — nếu `REPEATS ≥ 5` thì đánh dấu **OK** (đáng tin cậy để lưu); nếu thấp hơn, màn hình cảnh báo **LOW REP**, bạn nên bắt lại giữ lâu hơn.
- **MID** = lưu; **RIGHT** = replay ngay; **LEFT** = bắt lại từ đầu.

**c) Raw Capture — LISTEN RAW** (dự phòng khi 2 chế độ trên không decode được)
- Dùng khi remote của bạn **không thuộc 6 protocol chuẩn** (EV1527/PT2262/Came/Nice/Holtek/Ansonic) nên Standard/Advanced Capture không ra kết quả, hoặc bạn chỉ cần "ghi âm lại y nguyên" waveform để phát lại chứ không cần biết payload là gì.
- Máy ghi trực tiếp danh sách cạnh tín hiệu (edge) + độ dài mỗi mức cao/thấp (tối đa 16384 edge, gom theo frame, tối đa 64 frame/phiên), lọc nhiễu nền theo `noise_filter_us` và `rssi_threshold_dbm` cấu hình sẵn trong `rf_raw_capture.h`.
- Sau khi bắt xong, máy tự phân tích (`RC_ANALYZING`) để tìm frame "đẹp" nhất (đủ edge, lặp lại) làm kết quả hiển thị.
- Lưu vào Code Library như một mã **RAW** (khác với mã PROTO) — khi xem lại trong Code Library sẽ có nhãn `[R]` (raw) thay vì `[P]` (proto). Replay mã RAW sẽ phát lại **đúng từng xung** đã ghi, không qua bước encode protocol nào.

> Lưu ý: Standard/Advanced Capture decode tốt nhất với remote fixed-code (EV1527, PT2262, Came, Nice, Holtek, Ansonic — loại không đổi mã sau mỗi lần bấm). Remote rolling-code (hầu hết xe hơi, cổng hiện đại) sẽ **không** replay được dù dùng cả Raw Capture — mã đổi mỗi lần bấm là giới hạn vật lý của hệ thống rolling-code, không phải giới hạn phần mềm.

### 4.4 Code Library — quản lý mã đã lưu

- Mục đầu tiên `[+] ADD NEW CODE` cho phép **tự nhập tay** payload (hex), số bit, tần số, T (µs/bit), tên — hữu ích khi bạn đã biết thông số mã từ nguồn khác (chỉ áp dụng cho mã PROTO).
- Danh sách hiển thị gộp cả mã **PROTO** (`[P]`) và mã **RAW** (`[R]`), tối đa 20 mã PROTO + 7 mã RAW (giới hạn bởi `MAX_SAVED_CODES` / `MAX_RAW_CODES` trong `config.h`).
- Chọn 1 mã PROTO đã lưu → **MID** mở menu phụ 2 lựa chọn:
  - **1. REMOTE 4-BIT**: nếu mã của bạn có 4 bit cuối là command (kiểu remote nhà để xe 2–4 nút dùng chung 1 mã gốc), máy sẽ tự suy ra các nút còn lại (XOR bit theo vị trí 0x01/0x02/0x04/0x08) để bạn phát thử từng nút — chọn số nút (2–4) bằng LEFT/RIGHT, MID xác nhận, sau đó LEFT/RIGHT chọn nút muốn phát, MID để phát (15 lần/lượt).
  - **2. CODE DETAIL**: xem chi tiết mã (payload, bits, T_us, tần số) + 4 action: **TX** (phát), **JAM** (jam liên tục), **REN** (đổi tên), **DEL** (xóa, có xác nhận).
- Chọn 1 mã RAW đã lưu → vào thẳng **CODE DETAIL** dạng raw (hiển thị số edge, tần số, số lần lặp) với cùng 4 action **TX / JAM / REN / DEL**.
- **MID giữ lâu** hoặc **LEFT giữ lâu**: lùi lại 1 cấp menu.

### 4.5 RF Jammer — phát lại liên tục

> ⚠️ Đọc cảnh báo pháp lý ở đầu README trước khi dùng mục này.

Từ Main Menu chọn **RF Jammer** → chọn 1 mã trong Code Library → chọn chế độ:

- **1. JAMMING (CONT)**: phát lặp lại mã đã chọn **liên tục không ngừng** cho tới khi bạn dừng.
- **2. JAM DELAY**: phát lặp theo chu kỳ, chọn tốc độ **LOW** (1.0s/lần, 15x), **MEDIUM** (0.5s/lần, 15x), hoặc **HIGH** (250ms/lần, 15x) thay vì phát liên tục — tiết kiệm pin hơn, phù hợp khi chỉ cần giữ 1 kênh "bận" định kỳ.

Jam toàn bộ thư viện cùng lúc: chọn **Jam All** ở Code Library — phát lần lượt toàn bộ mã đã lưu liên tục (2 lần/mã/vòng). Dừng bằng cách giữ **MID** >0.5 giây.

Jam 1 mã riêng lẻ (không qua màn hình chọn mode): vào qua **Code Library → chọn mã → CODE DETAIL → JAM**.

**Engine jammer độc lập (`jammer.cpp`)**: ngoài kiểu "phát lại mã đã lưu" ở trên, firmware còn có một engine jammer tổng quát hơn với các chế độ (dùng nội bộ, không phải lúc nào cũng lộ hết ra UI):

| Mode | Mô tả ngắn |
|---|---|
| `MODE_CW_SPOT` | Phát sóng liên tục ở 1 tần số cố định |
| `MODE_HOP_CW` | Nhảy tuần tự qua một danh sách tần số (`hop_list`), dừng ở mỗi tần số `dwell_ms` |
| `MODE_OOK_NOISE` | Phát nhiễu dạng xung OOK ngẫu nhiên quanh 1 tần số |
| `MODE_HOP_OOK` | Kết hợp nhảy tần + nhiễu OOK ở mỗi bước |
| `MODE_SPAM` | Phát lặp lại 1 payload tùy chỉnh (`spam_payload`/`spam_bits`) theo chu kỳ `spam_interval_ms`, có thể đảo bit hoặc random hoá |

Module này có vài cờ biên dịch tùy chỉnh độ "mạnh tay" (sweep tần số, công suất PATABLE, burst/gap timing) khai báo ở đầu `jammer.cpp` — mặc định **tắt**, và tài liệu này **cố ý không hướng dẫn chi tiết cách chỉnh** vì đây là phần nhạy cảm về mặt pháp lý/tần số được cấp phép. Nếu bạn cần dùng, hãy tự đọc code và chỉ thử nghiệm trong môi trường được phép (buồng đo RF, băng tần ISM không cấp phép ở công suất hợp lệ, hoặc hệ thống của chính bạn).

### 4.6 Brute Force (DEMO — xem cảnh báo mục 0)

- Danh sách 5 protocol (CAME, NICE, Holtek, EV1527, Ansonic) + 1 mục **DE BRUIJN (FAST)**.
- Chọn 1 protocol, **MID** để bắt đầu phát tuần tự từ mã 0 → hết bảng, lặp vô hạn cho tới khi bấm **MID** để dừng.
- Thanh tiến trình hiển thị số mã đã gửi / tổng số mã.
- **LEFT giữ lâu** ở menu chọn protocol = thoát về Main. **RIGHT giữ lâu** = mở nhanh menu De Bruijn.
- De Bruijn: chỉ bấm được **MID** để chạy với tham số mặc định cứng (12-bit/350µs/433.92MHz/1 lần lặp) — **nút trái/phải hiện chưa có tác dụng đổi tham số** (xem cảnh báo mục 0). Giữ **MID** trong lúc đang chạy để dừng.

### 4.7 Settings

| Mục | Ý nghĩa |
|---|---|
| Frequency | Tần số mặc định dùng cho Scan/Capture/Replay (280–928MHz) |
| LED Bright | Độ sáng LED trạng thái (0–255) |
| Max bits | Giới hạn số bit tối đa khi decode (lọc nhiễu/rác dài bất thường), mặc định 32, cho phép 8–64 |
| Clear Flash | Xóa toàn bộ mã đã lưu (cả PROTO lẫn RAW) + về cấu hình mặc định — cần xác nhận bằng MID trong 3 giây, LEFT để hủy |

Khi nhập số/chuỗi (tần số, LED, max bits, tên mã, payload…), máy mở **bàn phím ảo** trên OLED:
- Bàn phím chữ: LEFT/RIGHT di chuyển con trỏ ký tự, MID để gõ ký tự đang highlight; hàng dưới cùng có DEL/SPACE/CLR/OK.
- Bàn phím số: LEFT/RIGHT di chuyển ô số 0-9/./<, MID để nhập; giữ LEFT = Hủy, giữ RIGHT = Lưu (OK).

### 4.8 About

Hiển thị phiên bản firmware, số mã đã lưu (PROTO + RAW) / tối đa, tần số hiện tại.

---

## 5. Giải thích kỹ thuật ngắn gọn

- **RF Listen (`rf_listen.cpp`)**: CC1101 cấu hình async serial OOK, ISR bắt cạnh tín hiệu trên GDO0 (lọc glitch <20µs), ước lượng chu kỳ T bằng clustering, tìm sync bằng khoảng lặng dài, decode PWM 2-phase cho mỗi bit, gắn nhãn protocol theo heuristics (bits + T + đảo pha).
- **Raw Capture (`rf_raw_capture.cpp`)**: không decode ra bits/protocol, chỉ ghi lại chuỗi `(độ dài xung, mức cao/thấp)` thô theo từng frame, tách frame bằng khoảng lặng (`frame_gap_ms`), tự chốt khi đủ `max_frame_edges` hoặc quá `max_frame_duration_us`; sau khi bắt xong chạy bước `analyze()` để chọn frame đáng tin nhất trong phiên.
- **RF Replay (`rf_replay.cpp`)**: bit-bang trực tiếp GDO0 làm chân TX (OOK async), tự tính PATABLE theo băng tần (300–348 / 387–464 / 779–928 MHz) để công suất phát đúng, tự calibrate VCO (`SCAL`) mỗi lần đổi tần số. Hỗ trợ phát cả mã PROTO (encode theo T_us/bits) lẫn mã RAW (phát lại nguyên chuỗi edge đã ghi).
- **Jammer (`jammer.cpp`)**: engine riêng, dùng chung tầng vật lý với `rf_replay` nhưng điều khiển GDO0 trực tiếp bằng macro fast-GPIO thay vì `digitalWrite` thường để đạt timing chính xác hơn ở các chế độ noise/spam; dùng PRNG xorshift32 nội bộ cho các chế độ ngẫu nhiên hoá.
- **Advanced Capture**: tách biệt frame buffer (1 lần bấm) và session buffer (nhiều lần bấm dồn lại), chỉ validate frame đủ điều kiện (≥8 edge hợp lệ, T trong [150,2500]µs, decode thử được ≥1 segment) mới gom vào session, sau đó chạy 3 pass: tìm T phổ biến → decode tất cả candidate → gom cụm giống nhau, lấy cụm có `count` (số lần lặp) cao nhất làm kết quả.
- **Storage**: flash giả lập EEPROM, **64KB** (`EEPROM_SIZE`), magic number đổi thành `"RFDY"` để tự phát hiện layout cũ/mới và reset nếu không khớp. Mã PROTO lưu như trước; mã RAW lưu theo slot riêng 8KB/mã (`RAW_SLOT_SIZE`, tối đa 7 mã) để đủ chỗ cho chuỗi edge dài.

---

## 6. Khắc phục sự cố thường gặp

| Hiện tượng | Nguyên nhân khả dĩ |
|---|---|
| Boot vào vòng lặp đèn LED nháy đỏ liên tục | OLED không init được — kiểm tra lại dây SDA/SCL, địa chỉ I2C (mặc định `0x3C`, đổi trong `config.h` nếu màn hình bạn là `0x3D`) |
| Sau khi cập nhật firmware, toàn bộ mã đã lưu bị mất | Bình thường — `STORAGE_MAGIC` đã đổi từ `"RFDU"` sang `"RFDY"` do layout flash thay đổi (thêm slot RAW), máy tự reset về mặc định khi phát hiện magic cũ không khớp |
| Scan không bắt được tín hiệu gì | Anten CC1101 chưa gắn/gắn sai, hoặc remote phát ở tần số ngoài danh sách `FREQ_LIST` trong `rf_listen.cpp` — tự thêm tần số nếu cần |
| Standard/Advanced Capture không decode được, dùng Raw Capture cũng không thấy edge nào | Kiểm tra `rssi_threshold_dbm` trong `rf_raw_capture.h` (mặc định -95dBm) có đang quá khắt khe so với tín hiệu thực tế không; thử đứng gần remote hơn |
| Capture bắt được nhưng Replay không mở được cổng | Remote có thể dùng rolling-code (không replay được), hoặc T_us/polarity bị lệch — thử Advanced Capture hoặc Raw Capture để lấy kết quả ổn định hơn |
| TX yếu/không ra sóng | Kiểm tra anten CC1101, để ý `cc_set_patable()` trong `rf_replay.cpp` — band ngoài 300–928MHz không được map đúng PATABLE |
| Nút bấm không phản hồi hoặc phản hồi 2 lần | Debounce mặc định `BTN_DEBOUNCE_MS = 25` trong `config.h`, tăng lên nếu nút cơ học của bạn rung nhiều |

---

## 7. Giấy phép & nguồn tham khảo

- Bảng mã brute-force (`bruteforce_codes.h`) sinh từ tham khảo định nghĩa protocol của dự án [`flipperzero-bruteforce`](https://github.com/tobiabocchi/flipperzero-bruteforce) (xem header file để biết chi tiết nguồn/ngày sinh).
- Thuật toán De Bruijn sequence (FKM algorithm) tham khảo Wikipedia: De Bruijn sequence.
- Phần còn lại là firmware tự viết, không đi kèm giấy phép cụ thể.

**Chỉ sử dụng thiết bị này trên hệ thống bạn sở hữu hoặc được phép thử nghiệm. Việc bắt/phát lại tín hiệu điều khiển không thuộc sở hữu của bạn, hoặc phát sóng gây nhiễu (jamming) trên băng tần được cấp phép, có thể vi phạm pháp luật tại nơi bạn sinh sống.**
