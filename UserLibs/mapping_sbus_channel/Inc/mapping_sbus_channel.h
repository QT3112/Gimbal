/**
 * ============================================================
 * @file    mapping_sbus_channel.h
 * @brief   Mapping tín hiệu SBUS raw → lệnh điều khiển gimbal
 *
 * Logic:
 *   - Vùng DEAD ZONE (600..1200): motor đứng yên (giữ vị trí hiện tại)
 *   - Vùng LOW (<600):  xoay với tốc độ cố định theo hướng âm
 *   - Vùng HIGH (>1200): xoay với tốc độ cố định theo hướng dương
 *
 * Hàm SBUS_Mapping_Update() được gọi mỗi vòng while(1),
 * tích phân tốc độ theo dt để cộng dần vào target angle.
 * Giới hạn góc được bảo vệ bên trong hàm, khớp với soft-limit
 * đã định nghĩa trong main.c.
 *
 * Kênh:
 *   CH1 → Roll  (raw < 600: trái | raw > 1200: phải)
 *   CH2 → Pitch (raw < 600: trước| raw > 1200: sau )
 *   CH4 → Yaw   (raw < 600: trái | raw > 1200: phải)
 *
 * ============================================================
 */

#ifndef MAPPING_SBUS_CHANNEL_H
#define MAPPING_SBUS_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sbus_protocol.h"

/* ============================================================
 * Cấu hình ngưỡng kênh SBUS (dead zone)
 * ============================================================ */
#define SBUS_MAP_LOW_THRESH     600U    /**< Dưới ngưỡng này: lệnh âm (trái/trước) */
#define SBUS_MAP_HIGH_THRESH    1200U   /**< Trên ngưỡng này: lệnh dương (phải/sau) */

/* ============================================================
 * Giới hạn góc mềm (deg) — phải khớp với main.c
 * ============================================================ */
#define SBUS_MAP_PITCH_MIN_DEG  15.0f
#define SBUS_MAP_PITCH_MAX_DEG  280.0f
#define SBUS_MAP_ROLL_MIN_DEG   90.0f
#define SBUS_MAP_ROLL_MAX_DEG   265.0f
#define SBUS_MAP_YAW_MIN_DEG    65.0f
#define SBUS_MAP_YAW_MAX_DEG    255.0f

/* ============================================================
 * Tốc độ xoay cố định khi sticks ra khỏi dead zone (deg/s)
 * Điều chỉnh giá trị này để thay đổi cảm giác điều khiển.
 * ============================================================ */
#define SBUS_MAP_ROLL_SPEED_DPS     30.0f
#define SBUS_MAP_PITCH_SPEED_DPS    30.0f
#define SBUS_MAP_YAW_SPEED_DPS      30.0f

/* ============================================================
 * Trạng thái lệnh cho 1 trục
 * ============================================================ */
typedef enum {
    SBUS_AXIS_HOLD =  0,    /**< Dead zone: giữ nguyên vị trí */
    SBUS_AXIS_NEG  = -1,    /**< Dưới LOW_THRESH: xoay âm     */
    SBUS_AXIS_POS  =  1,    /**< Trên HIGH_THRESH: xoay dương  */
} SBUS_AxisCmd_t;

/* ============================================================
 * Handle lưu trạng thái mapping
 * ============================================================ */
typedef struct {
    /* Con trỏ tới biến target angle trong main.c (rad) */
    volatile float *target_roll_rad;
    volatile float *target_pitch_rad;
    volatile float *target_yaw_rad;

    /* Lệnh trục đọc được lần cuối */
    SBUS_AxisCmd_t cmd_roll;
    SBUS_AxisCmd_t cmd_pitch;
    SBUS_AxisCmd_t cmd_yaw;

    /* Tick lần gọi Update() trước — tính dt tích phân */
    uint32_t last_tick_ms;
    uint8_t  initialized;
} SBUS_Mapping_Handle_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief Khởi tạo handle mapping, liên kết với target angle của gimbal.
 *
 * @param hmap           Con trỏ handle mapping cần khởi tạo
 * @param target_roll    Con trỏ đến target_roll_angle  (rad, volatile, trong main.c)
 * @param target_pitch   Con trỏ đến target_pitch_angle (rad, volatile, trong main.c)
 * @param target_yaw     Con trỏ đến target_yaw_angle   (rad, volatile, trong main.c)
 * @param init_roll_rad  Góc ban đầu Roll  (rad) — nên dùng encoder angle hiện tại
 * @param init_pitch_rad Góc ban đầu Pitch (rad)
 * @param init_yaw_rad   Góc ban đầu Yaw   (rad)
 */
void SBUS_Mapping_Init(SBUS_Mapping_Handle_t *hmap,
                       volatile float *target_roll,
                       volatile float *target_pitch,
                       volatile float *target_yaw,
                       float init_roll_rad,
                       float init_pitch_rad,
                       float init_yaw_rad);

/**
 * @brief Đọc CH1/CH2/CH4 từ SBUS, tích phân góc và cập nhật target.
 *
 * Gọi trong mỗi vòng while(1) sau SBUS_Process().
 * - SBUS_OK: xử lý bình thường theo sticks.
 * - SBUS_FAILSAFE / SBUS_TIMEOUT: giữ nguyên vị trí (không tích phân).
 * - SBUS_FRAME_LOST: vẫn giữ nguyên (tín hiệu kém).
 *
 * @param hmap    Handle mapping
 * @param hsbus   Handle SBUS đã qua SBUS_Process()
 * @param sbus_st Trạng thái trả về từ SBUS_Process()
 */
void SBUS_Mapping_Update(SBUS_Mapping_Handle_t *hmap,
                         SBUS_Handle_t         *hsbus,
                         SBUS_Status_t          sbus_st);

/**
 * @brief Truy vấn lệnh trục Roll hiện tại (NEG/HOLD/POS)
 */
SBUS_AxisCmd_t SBUS_Mapping_GetRollCmd(const SBUS_Mapping_Handle_t *hmap);

/**
 * @brief Truy vấn lệnh trục Pitch hiện tại
 */
SBUS_AxisCmd_t SBUS_Mapping_GetPitchCmd(const SBUS_Mapping_Handle_t *hmap);

/**
 * @brief Truy vấn lệnh trục Yaw hiện tại
 */
SBUS_AxisCmd_t SBUS_Mapping_GetYawCmd(const SBUS_Mapping_Handle_t *hmap);

#ifdef __cplusplus
}
#endif

#endif /* MAPPING_SBUS_CHANNEL_H */
