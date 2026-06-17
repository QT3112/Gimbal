/**
 ******************************************************************************
 * @file    manage_code.c
 * @brief   Triển khai vòng điều khiển FOC qua ngắt TIM6
 ******************************************************************************
 */

#include "manage_code.h"
#include "mpu6050.h"
#include "tim.h"
#include "spi.h"
#include "gpio.h"
#include "stdio.h"

/* ===========================================================================
 * Biến toàn cục — được export qua extern trong manage_code.h
 * =========================================================================== */
MPU6050_Handle_t    imu;
AS5048A_Handle_t    encoder;
FOC_Handle_t        foc;

volatile uint8_t    encoder_ready = 0;
static   uint8_t    imu_ready     = 0;

/* ===========================================================================
 * Log buffer: FOC_Loop() ghi → while(1) trong main() đọc và printf
 * =========================================================================== */
volatile uint8_t       foc_log_ready = 0;
volatile FOC_LogData_t foc_log       = {0};

/* ===========================================================================
 * Tốc độ mục tiêu (static, chỉ App_Init đặt giá trị)
 * Muốn thay đổi tốc độ lúc runtime: đổi thành non-static hoặc thêm setter
 * =========================================================================== */
static volatile float s_target_vel = 0.0f;

/* ===========================================================================
 * App_Init — khởi tạo phần cứng và thuật toán (gọi 1 lần trong main)
 * =========================================================================== */
void App_Init(void)
{
    /* --- TIM6: kích hoạt ngắt để FOC_Loop() chạy định kỳ ---
     * Period của TIM6 phải được cấu hình = 10ms trong CubeMX
     * (ví dụ: PSC=169, ARR=999 → 10ms với clock 170MHz) */
    HAL_TIM_Base_Start_IT(&htim6);

    /* --- TIM2: khởi động 3 kênh PWM --- */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    /* --- PC6: Enable gate driver --- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

    /* --- Khởi tạo FOC ---
     * voltage_limit = 0.05V (thấp để test an toàn, tăng dần sau)
     * Ts = 0.01s = 10ms phải khớp với period của TIM6 */
    FOC_Init(&foc, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
             4249.0f,  /* PWM ARR */
             7,         /* Số cặp cực — chỉnh theo motor thực tế */
             0.05f,     /* voltage_limit [V] */
             0.0002f);    /* Ts [s] */

    /* --- Khởi tạo Encoder AS5048A --- */
    AS5048A_Status_t enc_ret = AS5048A_Init(&encoder, &hspi1, GPIOC, GPIO_PIN_4);
    printf("[AS5048A_Init] = %d  (0=OK, 1=SPI_ERR, 2=PARITY_ERR, 3=EF, 4=CORDIC)\r\n",
           enc_ret);

    if (enc_ret == AS5048A_OK) {
        encoder_ready = 1;
        AS5048A_ReadDiagnostics(&encoder);
        printf("[AS5048A] AGC=%u | CompH=%u CompL=%u | COF=%u | OCF=%u\r\n",
               encoder.agc_value, encoder.comp_high, encoder.comp_low,
               encoder.cordic_overflow, encoder.offset_comp_finished);
    }

    /* --- Cấu hình LPF và PID vòng tốc độ ---
     * alpha=0.9: lọc mạnh cho Ts=10ms (giảm xuống 0.7 nếu muốn nhanh hơn)
     * Kp/Ki nhỏ để khởi đầu an toàn — tăng dần khi đã căn chỉnh xong */
    FOC_SetLPF_Vel(&foc, 0.9f);
    FOC_SetPID_Vel(&foc,
                   0.008f,             /* Kp */
                   0.004f,             /* Ki */
                   0.0f,               /* Kd */
                   -foc.voltage_limit,
                    foc.voltage_limit);

    /* --- Bật FOC output --- */
    FOC_Start(&foc);

    /* --- Căn chỉnh góc encoder (Alignment) ---
     * Áp Vd tại theta_elec=0 trong 500ms → rotor kéo về D-axis
     * Đọc encoder → lưu angle_offset → FOC biết góc zero tuyệt đối
     *
     * LƯU Ý: Trong lúc này TIM6 ISR đã chạy nhưng FOC_Loop() return ngay
     * vì encoder_ready được set bởi AS5048A_Init ở trên.
     * FOC_AlignD() ghi trực tiếp vào PWM, không bị ISR ghi đè
     * vì HAL_Delay(10) ở đây chiếm CPU hoàn toàn giữa 2 lần ISR */
    if (encoder_ready) {
        printf("[ALIGN] Dang can chinh rotor (500ms)...\r\n");
        for (int i = 0; i < 50; i++) {     /* 50 × 10ms = 500ms */
            FOC_AlignD(&foc, foc.voltage_limit);
            HAL_Delay(10);
        }

        if (AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
            FOC_CalibrateAngle(&foc, encoder.angle_rad);
            foc.prev_angle_mech = encoder.angle_rad;
            printf("[ALIGN] Xong! Offset = %.3f rad | Enc = %.1f deg\r\n",
                   foc.angle_offset, encoder.angle_deg);
        }
    }

    /* --- Đặt tốc độ mục tiêu --- */
    s_target_vel = 0.1f * FOC_TWO_PI;   /* 0.1 vòng/s = ~0.628 rad/s */
    printf("[APP] San sang! TIM6 ISR se goi FOC_Loop() moi 10ms\r\n");
    printf("[APP] Target vel = %.3f rad/s\r\n", s_target_vel);
}

/* ===========================================================================
 * FOC_Loop — chạy trong ngắt TIM6 mỗi 10ms
 *
 * ⚠ KHÔNG printf, KHÔNG HAL_Delay, KHÔNG gọi hàm blocking trong đây!
 * =========================================================================== */
void FOC_Loop(void)
{
    if (!encoder_ready) return;

    /* Đọc góc encoder qua SPI (polling ~5–15µs) */
    if (AS5048A_ReadAngle(&encoder) != AS5048A_OK) return;

    /* Chạy 1 chu kỳ closed-loop velocity control
     * → tính velocity_mech từ encoder
     * → PID_vel → Vq
     * → InvPark + InvClarke → ghi PWM */
    FOC_RunVelocity(&foc, encoder.angle_rad, s_target_vel);

    /* Ghi log data vào buffer an toàn → main() đọc và printf */
    foc_log.target_vel = s_target_vel;
    foc_log.meas_vel   = foc.velocity_mech;
    foc_log.Vq         = foc.Vq_ref;
    foc_log.angle_deg  = encoder.angle_deg;
    foc_log_ready      = 1;   /* Báo hiệu cho while(1) có data mới */
}
