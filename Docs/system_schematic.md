# Sơ đồ Kiến trúc Hệ thống Nhúng (Gimbal 3 Trục)

Dưới đây là sơ đồ (schematic/block diagram) biểu diễn sự tương tác giữa các phần cứng ngoại vi và luồng xử lý phần mềm bên trong STM32G4 của chương trình.

```mermaid
graph TD
    %% Hardware External Components
    subgraph External Hardware
        PC[PC / GUI Terminal]
        IMU[ICM42688 Cảm biến IMU]
        EncP[AS5048A Encoder Pitch]
        EncR[AS5048A Encoder Roll]
        EncY[AS5048A Encoder Yaw]
        
        MotP[Động cơ BLDC Pitch]
        MotR[Động cơ BLDC Roll]
        MotY[Động cơ BLDC Yaw]
        
        Shunt[Điện trở Shunt & Khuyếch đại Dòng]
    end

    %% STM32G4 MCU
    subgraph STM32G4 Microcontroller
        
        %% Comm Interfaces
        USB[USB FS CDC]
        SPI3[SPI3 + Rx DMA]
        SPI1[SPI1 Polling]
        
        %% Timers
        TIM6[TIM6 Timer 2kHz \n Lấy mẫu Dữ liệu]
        TIM7[TIM7 Timer 2kHz \n Vòng lặp FOC Outer]
        TIM1[TIM1 / TIM2 / TIM3 \n Phát PWM 20kHz]
        
        %% ADC
        ADC[ADC1 & ADC2 \n Injected Mode]
        
        %% Software Modules
        subgraph Ngắt & Xử lý (ISRs & Loops)
            MainLoop[while 1 \n Main Loop]
            
            subgraph Priority 0: Hard Real-Time
                FOC_Curr[FOC Current Loop \n Tính SVPWM]
            end
            
            subgraph Priority 1: Outer Control
                FOC_Outer[FOC Position/Velocity \n PID Controllers]
            end
            
            subgraph Priority 2: Data Acq
                SensorAcq[Đọc Encoder \n & Kick IMU DMA]
            end
            
            subgraph Priority 3: Sensor Fusion
                Mahony[Mahony AHRS \n Quaternion Filter]
            end
        end
    end

    %% Connections - Communication
    PC <-->|USB Cable| USB
    USB <-->|Lệnh & Telemetry| MainLoop
    
    IMU -->|MISO| SPI3
    SPI3 -->|Kích hoạt Ngắt Rx Cplt| Mahony
    
    EncP -->|MISO| SPI1
    EncR -->|MISO| SPI1
    EncY -->|MISO| SPI1
    
    %% Timers Flow
    TIM6 -->|Kích hoạt 500µs| SensorAcq
    SensorAcq -->|Đọc Blocking| SPI1
    SensorAcq -->|Kích hoạt DMA| SPI3
    
    TIM7 -->|Kích hoạt 500µs| FOC_Outer
    FOC_Outer -->|Gửi lệnh Vq, Vd| FOC_Curr
    
    TIM1 -->|Trigger đồng bộ PWM| ADC
    ADC -->|Kích hoạt Ngắt Injected| FOC_Curr
    FOC_Curr -->|Cập nhật Duty Cycle| TIM1
    
    %% Power / Control Output
    TIM1 -->|Tín hiệu PWM 3 Pha| MotP
    TIM1 -->|Tín hiệu PWM 3 Pha| MotR
    TIM1 -->|Tín hiệu PWM 3 Pha| MotY
    
    %% Current Sense Feedback
    MotP -->|Dòng pha A, B| Shunt
    MotR -->|Dòng pha A, B| Shunt
    MotY -->|Dòng pha A, B| Shunt
    Shunt -->|Tín hiệu Analog| ADC
    
    %% Software Data Flow
    SensorAcq -.->|Góc Encoder Cập nhật| FOC_Outer
    Mahony -.->|Góc IMU Cập nhật| FOC_Outer
```

## Diễn giải Sơ đồ

1. **Thu thập dữ liệu (Trái qua Phải):**
   - **TIM6 (2kHz)** đảm nhiệm việc định thời gian cho việc lấy mẫu. Nó kích hoạt ngắt Priority 2.
   - Trong ngắt này, CPU trực tiếp đọc **SPI1** (Encoder của 3 trục) và đồng thời đánh thức **SPI3 DMA** để đọc 15 byte từ IMU một cách tự động.
   - Khi DMA của SPI3 nhận xong 15 byte, nó sinh ra ngắt Priority 3, gọi bộ lọc **Mahony AHRS** để biến đổi dữ liệu gia tốc và góc xoay thô thành Quaternion (góc nghiêng không gian 3D).

2. **Điều khiển Vòng ngoài (TIM7 - Priority 1):**
   - Chạy ở 2kHz, sau khi dữ liệu cảm biến đã được TIM6 lấy xong.
   - Lấy góc mục tiêu và góc hiện tại (từ Encoder hoặc Quaternion IMU tuỳ mode).
   - Đưa qua bộ PID (Position -> Velocity) sinh ra lệnh điện áp $V_q$ cho từng động cơ. Lệnh này được truyền xuống vòng lặp dòng điện (Inner Loop).

3. **Điều khiển Vòng trong (TIM1/2/3 & ADC - Priority 0):**
   - Các Timer phát PWM ở tần số cao (~20kHz).
   - Khi PWM đạt đỉnh (hoặc đáy tuỳ cấu hình Center-aligned), Timer tự động kích hoạt **ADC Injected Mode** đọc điện áp rơi trên điện trở Shunt.
   - ADC đọc xong lập tức gọi ngắt Priority 0 (Cao nhất). Tại đây, hàm `FOC_CurrentLoop()` chạy toán học biến đổi hệ toạ độ (Clarke/Park), tính toán vòng lặp dòng điện PI và sinh ra **SVPWM (Space Vector PWM)** để cập nhật lại thanh ghi Duty Cycle của Timer PWM.

4. **Vòng lặp Main (`while(1)`):**
   - Không chứa bất kỳ logic điều khiển real-time nào. 
   - Nó chỉ xử lý thao tác với **USB CDC** (nhận lệnh từ máy tính để chuyển mode) và đẩy log `printf` ra ngoài. Điều này đảm bảo việc in log chậm trễ không làm sập (crash) thuật toán cân bằng động cơ.
