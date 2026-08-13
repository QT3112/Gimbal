# sbus_protocol — Thư viện SBUS cho STM32

Thư viện giải mã giao thức **SBUS** (Futaba / FrSky / TBS) sử dụng HAL UART DMA cho STM32.

---

## Giao thức SBUS

| Thông số        | Giá trị                          |
|-----------------|----------------------------------|
| Baud Rate       | **100000 bps**                   |
| Cấu hình UART   | **8E2** (8 data, Even, 2 Stop)   |
| Kích thước frame| **25 bytes**                     |
| Chu kỳ frame    | ~14ms (FrSky) / ~7ms (High-Speed)|
| Kênh analog     | **16 kênh × 11 bit** (0..2047)  |
| Kênh digital    | **CH17, CH18**                   |
| Flags           | Frame Lost, Failsafe             |

### Cấu trúc frame 25 bytes

```
[0x0F] [CH1..CH16 packed 22 bytes] [FLAGS] [0x00]
  ^                                    ^       ^
Start                               Flags    End
```

---

## Cấu trúc thư mục

```
sbus_protocol/
├── Inc/
│   └── sbus_protocol.h   — Header, API public
├── Src/
│   └── sbus_protocol.c   — Implementaiton
└── README.md
```

---

## Cách tích hợp vào dự án STM32CubeIDE

### Bước 1: Cấu hình UART trong CubeMX

Chọn USART (ví dụ USART1) và cấu hình:
- **Baud Rate**: `100000`
- **Word Length**: `8 Bits`
- **Stop Bits**: `2`
- **Parity**: `Even`
- **Mode**: `Receive Only` hoặc `Receive and Transmit`
- **DMA**: Thêm DMA RX (Direction: Peripheral to Memory, Mode: **Normal**)

> ⚠️ **Quan trọng**: SBUS dùng **8E2**, không phải 8N1 thông thường!

### Bước 2: Thêm đường dẫn include

Trong CubeIDE: **Project → Properties → C/C++ Build → Settings → Include Paths**
Thêm: `../UserLibs/sbus_protocol/Inc`

Thêm source file: `../UserLibs/sbus_protocol/Src/sbus_protocol.c`

### Bước 3: Sửa header MCU (nếu cần)

Trong `sbus_protocol.h`, thay đổi dòng include phù hợp với MCU của bạn:
```c
#include "stm32g4xx_hal.h"   // STM32G4
// #include "stm32f4xx_hal.h" // STM32F4
// #include "stm32h7xx_hal.h" // STM32H7
```

---

## Cách sử dụng (Quick Start)

### main.c

```c
#include "sbus_protocol.h"

SBUS_Handle_t sbus_rx;

int main(void) {
    // ... HAL_Init, SystemClock, Peripheral Init ...

    // Khởi tạo SBUS (UART đã được CubeMX init sẵn)
    if (SBUS_Init(&sbus_rx, &huart1) != SBUS_OK) {
        // Lỗi khởi tạo DMA
        Error_Handler();
    }

    while (1) {
        // Xử lý frame SBUS trong vòng lặp chính
        SBUS_Status_t st = SBUS_Process(&sbus_rx);

        if (st == SBUS_OK) {
            // Đọc kênh raw (0..2047)
            uint16_t throttle_raw;
            SBUS_GetChannel(&sbus_rx, 3, &throttle_raw);

            // Đọc kênh normalized cho aileron [-1.0 .. +1.0]
            float aileron;
            SBUS_GetChannelNorm(&sbus_rx, 1, &aileron);

            // Đọc throttle [0.0 .. 1.0]
            float throttle;
            SBUS_GetChannelNorm01(&sbus_rx, 3, &throttle);

            // Đọc kênh digital
            uint8_t sw_gear = SBUS_GetCH17(&sbus_rx);
        }

        if (SBUS_IsSignalLost(&sbus_rx)) {
            // Thực hiện hành động an toàn khi mất tín hiệu
        }
    }
}
```

### Callback ISR (stm32g4xx_it.c hoặc main.c)

```c
extern SBUS_Handle_t sbus_rx;

// Dùng Idle Line DMA (khuyến nghị)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        SBUS_RxEventCallback(&sbus_rx, Size);
    }
}
```

---

## API Reference

| Hàm | Mô tả |
|-----|-------|
| `SBUS_Init(hsbus, huart)` | Khởi tạo, bắt đầu DMA |
| `SBUS_Process(hsbus)` | Xử lý frame — gọi trong `while(1)` |
| `SBUS_GetChannel(hsbus, ch, &val)` | Giá trị raw kênh 1..16 (0..2047) |
| `SBUS_GetChannelNorm(hsbus, ch, &f)` | Normalized [-1.0 .. +1.0] |
| `SBUS_GetChannelNorm01(hsbus, ch, &f)` | Normalized [0.0 .. 1.0] |
| `SBUS_GetCH17(hsbus)` | Digital channel 17 |
| `SBUS_GetCH18(hsbus)` | Digital channel 18 |
| `SBUS_GetStatus(hsbus)` | Trạng thái hiện tại |
| `SBUS_IsFailsafe(hsbus)` | Kiểm tra failsafe |
| `SBUS_IsSignalLost(hsbus)` | Kiểm tra mất tín hiệu (timeout) |
| `SBUS_RxEventCallback(hsbus, size)` | Gọi từ HAL ISR callback |
| `SBUS_RxCpltCallback(hsbus)` | Dùng khi không có Idle Line DMA |

### Mapping kênh điển hình (FrSky/Futaba)

| Kênh | Chức năng RC |
|------|-------------|
| CH1  | Aileron (Roll)   |
| CH2  | Elevator (Pitch) |
| CH3  | Throttle         |
| CH4  | Rudder (Yaw)     |
| CH5  | Mode Switch      |
| CH6  | Gimbal Tilt      |

---

## Lưu ý

- Tốc độ nhận: Thư viện dùng `HAL_UARTEx_ReceiveToIdle_DMA()` — DMA tự động restart sau mỗi frame.
- Timeout mất tín hiệu mặc định: **500ms** (thay đổi qua `SBUS_TIMEOUT_MS`).
- Nếu MCU không hỗ trợ Idle Line DMA, dùng `HAL_UART_Receive_DMA()` với `SBUS_FRAME_SIZE` và callback `SBUS_RxCpltCallback()`.

