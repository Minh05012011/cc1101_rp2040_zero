# RF Gateway — RP2040 Zero + CC1101 + OLED + WS2812B

Thiết bị RF đa năng cầm tay: nghe/quét sóng 300–928MHz, bắt mã (capture) điều khiển cổng/garage, lưu thư viện mã, phát lại (replay), jammer, và một module brute-force **chỉ mang tính demo/minh họa** (xem phần cảnh báo bên dưới).

Firmware viết cho RP2040 (Arduino core), giao tiếp CC1101 qua SPI thủ công (không dùng thư viện ELECHOUSE).

---

## ⚠️ CẢNH BÁO QUAN TRỌNG — ĐỌC TRƯỚC KHI DÙNG

### Brute Force (5 protocol) và De Bruijn: KHÔNG hoạt động thực tế — chỉ DEMO

Dự án có 2 tính năng "brute force" trong menu:

1. **Brute Force (CAME / NICE / Holtek / EV1527 / Ansonic)** — dùng bảng mã sinh sẵn `bruteforce_codes.h` (20480 mã, ~4MB flash) để phát tuần tự từng mã 12/20-bit.
2. **De Bruijn (FAST)** — thuật toán nén chuỗi brute-force bằng De Bruijn sequence (`debruijn.cpp`) để giảm số bit cần phát.

**Thực tế:**
- Cả hai chỉ chạy được ở mức *trình diễn giao diện* (quét UI, thanh tiến trình, đếm mã...). Việc phát sóng có chạy (gọi `rf_replay_send_proto` / `rf_replay_transmit_raw_bits`), nhưng **không có gì đảm bảo thứ tự timing, encoding, hoặc lựa chọn mã là đúng với cổng thật ngoài đời** — đây không phải brute-force "được kiểm chứng hoạt động trên cổng thật", mà là khung sườn thuật toán minh họa cách một brute-force tool *có thể* được viết.
- Riêng De Bruijn: màn hình cấu hình (`update_bruce_debruijn_menu`) có code điều hướng LEFT/RIGHT nhưng **chưa cài logic đổi field/giá trị** (xem comment `// Simplified: ...` trong `menu.cpp`) — nút trái/phải hiện không làm gì, bạn chỉ có thể bấm MID để chạy với giá trị mặc định cứng (12-bit, 350µs, 433.92MHz, 1 lần lặp).
- Thời gian ước tính trên màn hình (vd CAME ~224s cho 4096 mã) là lý thuyết, **chưa test với cổng thật nào** trong phạm vi dự án này.
- Nếu bạn cần brute-force thực sự dùng được, nên tham khảo Flipper Zero hoặc ESP32 Bruce Firmware (dự án này có mượn ý tưởng/toàn bộ bảng mã từ `flipperzero-bruteforce`) và tự kiểm chứng timing bằng máy phân tích sóng.

**Tất cả các chức năng còn lại (Scan, Capture thường, Advanced Capture, Code Library, Replay, Jammer, Settings) đã được code đầy đủ logic và nhằm mục đích hoạt động thật**

---

## 1. Tính năng

| Tính năng | Trạng thái | Mô tả |
|---|---|---|
| Live Scan | Đầy đủ | Quét dải tần cố định (300–928MHz, ~40 điểm), hiển thị RSSI + sóng ECG-style |
| Signal Capture (Standard) | Đầy đủ | Bắt 1 frame tín hiệu OOK, decode EV1527/PT2262/Came/Nice/Holtek/Ansonic tự động |
| Advanced Capture | Đầy đủ | Gom nhiều frame, lọc nhiễu 3-pass, tìm payload lặp lại nhiều lần để tăng độ tin cậy |
| Code Library | Đầy đủ | Lưu tối đa 20 mã vào EEPROM giả lập (flash), đổi tên, xóa, replay, jam |
| 4-Bit Remote Analysis | Đầy đủ | Suy ra 2–4 nút bấm còn lại từ 1 mã gốc (cho remote dùng 4-bit command fixed-code) |
| Replay (TX) | Đầy đủ | Phát lại mã đã lưu/bắt được theo đúng timing gốc |
| Jammer (1 mã) | Đầy đủ | Phát liên tục 1 mã đã chọn để chèn sóng |
| Jam All | Đầy đủ | Phát lần lượt toàn bộ thư viện mã đã lưu |
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
├── hardware.h / .cpp        (OLED, LED, nút bấm)
├── storage.h   / .cpp       (lưu EEPROM/flash)
├── keyboard.h  / .cpp       (bàn phím ảo trên OLED để nhập tên/số)
├── menu.h      / .cpp       (toàn bộ state machine UI)
├── rf_listen.h / .cpp       (nghe/scan/capture)
├── rf_replay.h / .cpp       (phát/TX/jammer)
├── advanced_capture.h / .cpp (advanced capture 3-pass)
├── debruijn.h  / .cpp       (De Bruijn — DEMO)
├── bruce_codes.h            (5 loại cổng cho De Bruijn — DEMO)
└── bruteforce_codes.h       (bảng mã sinh sẵn 20480 mã — DEMO)
```

### 3.4 Nạp code

1. Nối board qua USB-C, giữ nút BOOTSEL khi cắm nếu lần đầu nạp (tùy board).
2. Chọn đúng **Port** trong Arduino IDE.
3. Bấm **Upload**.
4. Mở **Serial Monitor** ở baud `115200` để xem log debug (`[STORAGE]`, `[REPLAY]`, `[CAPTURE]`, …) — rất hữu ích khi gỡ lỗi.

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

Cách dùng:
1. Vào **Live Scan**.
2. Đứng gần remote, bấm nút remote liên tục vài lần trong lúc máy đang quét.
3. Khi đồ thị hiện "SIGNAL: STRONG/MEDIUM" và tần số đỉnh (vd `433.920 MHz`) kèm dBm, bấm **MID** để lưu tần số này làm mặc định và **tự động chuyển sang Signal Capture** ở đúng tần số đó.
4. **LEFT** để thoát về Main.

### 4.3 Signal Capture — bắt mã (2 chế độ)

Vào **Signal Capture** → chọn 1 trong 2 card bằng RIGHT để toggle, MID để chọn:

**a) Standard Capture** (nhanh, 1-frame)
- Dao động ký đơn giản hiển thị trên màn hình khi có tín hiệu.
- Bấm remote 1 lần. Nếu decode được (đủ edge, sync hợp lệ, bits trong khoảng 12–max_bits), máy tự nhảy sang màn hình kết quả.
- **LEFT**: quay lại Scan. **RIGHT**: quay lại chế độ Scan tần số khác. **MID**: xem lại kết quả đã có.
- Ở màn hình kết quả (**Captured**): `MID` = mở bàn phím ảo đặt tên rồi lưu vào Code Library; `RIGHT` = phát lại ngay không cần lưu; `LEFT` = bắt lại.

**b) Advanced Capture** (chậm hơn, độ tin cậy cao hơn)
- Giữ nút remote **ấn liên tục nhiều lần trong vài giây** (khuyến nghị giữ 2–3 giây, bấm nhả nhiều lần) để máy gom đủ số lần lặp giống nhau.
- Máy tự dừng sau khi: buffer đầy, bạn bấm MID, hoặc 10 giây không có frame hợp lệ mới.
- Kết quả hiện `REPEATS x/y` — nếu `REPEATS ≥ 5` thì đánh dấu **OK** (đáng tin cậy để lưu); nếu thấp hơn, màn hình cảnh báo **LOW REP**, bạn nên bắt lại giữ lâu hơn.
- **MID** = lưu; **RIGHT** = replay ngay; **LEFT** = bắt lại từ đầu.

> Lưu ý: cả 2 chế độ decode tốt nhất với remote fixed-code (EV1527, PT2262, Came, Nice, Holtek, Ansonic — loại không đổi mã sau mỗi lần bấm). Remote rolling-code (hầu hết xe hơi, cổng hiện đại) sẽ **không** replay được vì mã đổi mỗi lần — đây là giới hạn vật lý, không phải lỗi phần mềm.

### 4.4 Code Library — quản lý mã đã lưu

- Mục đầu tiên `[+] ADD NEW CODE` cho phép **tự nhập tay** payload (hex), số bit, tần số, T (µs/bit), tên — hữu ích khi bạn đã biết thông số mã từ nguồn khác.
- Chọn 1 mã đã lưu → **MID** mở menu phụ 2 lựa chọn:
  - **1. REMOTE 4-BIT**: nếu mã của bạn có 4 bit cuối là command (kiểu remote nhà để xe 2–4 nút dùng chung 1 mã gốc), máy sẽ tự suy ra các nút còn lại (XOR bit theo vị trí 0x01/0x02/0x04/0x08) để bạn phát thử từng nút — chọn số nút (2–4) bằng LEFT/RIGHT, MID xác nhận, sau đó LEFT/RIGHT chọn nút muốn phát, MID để phát (15 lần/lượt).
  - **2. CODE DETAIL**: xem chi tiết mã (payload, bits, T_us, tần số) + 4 action: **TX** (phát), **JAM** (jam liên tục), **REN** (đổi tên), **DEL** (xóa, có xác nhận).
- **MID giữ lâu** hoặc **LEFT giữ lâu**: lùi lại 1 cấp menu.

### 4.5 RF Jammer

Từ Main Menu chọn **RF Jammer** = **Jam All**: phát lần lượt toàn bộ thư viện đã lưu liên tục (2 lần/mã/vòng). Dừng bằng cách giữ **MID** >0.5 giây.

Jam 1 mã riêng lẻ: vào qua **Code Library → chọn mã → CODE DETAIL → JAM**.

> Phát sóng liên tục trên băng tần được cấp phép có thể vi phạm pháp luật tại nhiều quốc gia. Chỉ dùng trên thiết bị/hệ thống bạn sở hữu hoặc có quyền thử nghiệm hợp pháp.

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
| Clear Flash | Xóa toàn bộ mã đã lưu + về cấu hình mặc định — cần xác nhận bằng MID trong 3 giây, LEFT để hủy |

Khi nhập số/chuỗi (tần số, LED, max bits, tên mã, payload…), máy mở **bàn phím ảo** trên OLED:
- Bàn phím chữ: LEFT/RIGHT di chuyển con trỏ ký tự, MID để gõ ký tự đang highlight; hàng dưới cùng có DEL/SPACE/CLR/OK.
- Bàn phím số: LEFT/RIGHT di chuyển ô số 0-9/./<, MID để nhập; giữ LEFT = Hủy, giữ RIGHT = Lưu (OK).

### 4.8 About

Hiển thị phiên bản firmware, số mã đã lưu / tối đa (20), tần số hiện tại.

---

## 5. Giải thích kỹ thuật ngắn gọn

- **RF Listen (`rf_listen.cpp`)**: CC1101 cấu hình async serial OOK, ISR bắt cạnh tín hiệu trên GDO0 (lọc glitch <20µs), ước lượng chu kỳ T bằng clustering, tìm sync bằng khoảng lặng dài, decode PWM 2-phase cho mỗi bit, gắn nhãn protocol theo heuristics (bits + T + đảo pha).
- **RF Replay (`rf_replay.cpp`)**: bit-bang trực tiếp GDO0 làm chân TX (OOK async), tự tính PATABLE theo băng tần (300–348 / 387–464 / 779–928 MHz) để công suất phát đúng, tự calibrate VCO (`SCAL`) mỗi lần đổi tần số.
- **Advanced Capture**: tách biệt frame buffer (1 lần bấm) và session buffer (nhiều lần bấm dồn lại), chỉ validate frame đủ điều kiện (≥8 edge hợp lệ, T trong [150,2500]µs, decode thử được ≥1 segment) mới gom vào session, sau đó chạy 3 pass: tìm T phổ biến → decode tất cả candidate → gom cụm giống nhau, lấy cụm có `count` (số lần lặp) cao nhất làm kết quả.
- **Storage**: EEPROM giả lập (flash) 1024 byte, magic number `"RFDU"` để tự phát hiện layout cũ/mới và reset nếu không khớp.

---

## 6. Khắc phục sự cố thường gặp

| Hiện tượng | Nguyên nhân khả dĩ |
|---|---|
| Boot vào vòng lặp đèn LED nháy đỏ liên tục | OLED không init được — kiểm tra lại dây SDA/SCL, địa chỉ I2C (mặc định `0x3C`, đổi trong `config.h` nếu màn hình bạn là `0x3D`) |
| `[STORAGE] ... max_bits=0xFF` hoặc dữ liệu cũ bị reset | Bình thường lần đầu flash hoặc sau khi đổi `STORAGE_MAGIC` trong code — máy tự ghi lại giá trị mặc định |
| Scan không bắt được tín hiệu gì | Anten CC1101 chưa gắn/gắn sai, hoặc remote phát ở tần số ngoài danh sách `FREQ_LIST` trong `rf_listen.cpp` — tự thêm tần số nếu cần |
| Capture bắt được nhưng Replay không mở được cổng | Remote có thể dùng rolling-code (không replay được), hoặc T_us/polarity bị lệch — thử Advanced Capture để lấy kết quả ổn định hơn |
| TX yếu/không ra sóng | Kiểm tra anten CC1101, để ý `cc_set_patable()` trong `rf_replay.cpp` — band ngoài 300–928MHz không được map đúng PATABLE |
| Nút bấm không phản hồi hoặc phản hồi 2 lần | Debounce mặc định `BTN_DEBOUNCE_MS = 25` trong `config.h`, tăng lên nếu nút cơ học của bạn rung nhiều |

---

## 7. Giấy phép & nguồn tham khảo

- Bảng mã brute-force (`bruteforce_codes.h`) sinh từ tham khảo định nghĩa protocol của dự án [`flipperzero-bruteforce`](https://github.com/tobiabocchi/flipperzero-bruteforce) (xem header file để biết chi tiết nguồn/ngày sinh).
- Thuật toán De Bruijn sequence (FKM algorithm) tham khảo Wikipedia: De Bruijn sequence.
- Phần còn lại là firmware tự viết, không đi kèm giấy phép cụ thể

**Chỉ sử dụng thiết bị này trên hệ thống bạn sở hữu hoặc được phép thử nghiệm. Việc bắt/phát lại tín hiệu điều khiển không thuộc sở hữu của bạn có thể vi phạm pháp luật.**
