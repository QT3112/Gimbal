/**
 * ============================================================
 * @file    mapping_sbus_channel.c
 * @brief   Triển khai mapping SBUS raw → lệnh điều khiển gimbal
 *
 * Nguyên lý hoạt động:
 *   1. Mỗi lần SBUS_Mapping_Update() được gọi, tính dt = thời gian
 *      kể từ lần gọi trước (ms → s).
 *   2. Đọc raw CH1, CH2, CH4 từ SBUS handle.
 *   3. Phân loại lệnh mỗi trục (HOLD / NEG / POS) theo ngưỡng.
 *   4. Tích phân: target += sign * speed_dps * dt_s * (PI/180)
 *      để chuyển đổi từ deg/s sang rad/s rồi tính góc mới.
 *   5. Clamp target trong giới hạn góc mềm trước khi ghi ra.
 *
 * ============================================================
 */

#include "mapping_sbus_channel.h"
#include "stm32g4xx_hal.h"  /* HAL_GetTick() */

/* ============================================================
 * Private helpers
 * ============================================================ */

#define DEG_TO_RAD_F    0.01745329252f   /* PI / 180 */

/**
 * @brief Clamp giá trị float trong [lo, hi].
 */
static inline float fclampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief Chuyển giá trị raw SBUS sang lệnh trục.
 */
static inline SBUS_AxisCmd_t raw_to_cmd(uint16_t raw) {
    if (raw < SBUS_MAP_LOW_THRESH)  return SBUS_AXIS_NEG;
    if (raw > SBUS_MAP_HIGH_THRESH) return SBUS_AXIS_POS;
    return SBUS_AXIS_HOLD;
}

/* ============================================================
 * Public implementation
 * ============================================================ */

void SBUS_Mapping_Init(SBUS_Mapping_Handle_t *hmap,
                       volatile float *target_roll,
                       volatile float *target_pitch,
                       volatile float *target_yaw,
                       float init_roll_rad,
                       float init_pitch_rad,
                       float init_yaw_rad)
{
    if (hmap == NULL) return;

    hmap->target_roll_rad  = target_roll;
    hmap->target_pitch_rad = target_pitch;
    hmap->target_yaw_rad   = target_yaw;

    /* Đặt góc khởi đầu — clamp ngay vào giới hạn an toàn */
    if (target_roll != NULL) {
        *target_roll  = fclampf(init_roll_rad,
                                SBUS_MAP_ROLL_MIN_DEG  * DEG_TO_RAD_F,
                                SBUS_MAP_ROLL_MAX_DEG  * DEG_TO_RAD_F);
    }
    if (target_pitch != NULL) {
        *target_pitch = fclampf(init_pitch_rad,
                                SBUS_MAP_PITCH_MIN_DEG * DEG_TO_RAD_F,
                                SBUS_MAP_PITCH_MAX_DEG * DEG_TO_RAD_F);
    }
    if (target_yaw != NULL) {
        *target_yaw   = fclampf(init_yaw_rad,
                                SBUS_MAP_YAW_MIN_DEG   * DEG_TO_RAD_F,
                                SBUS_MAP_YAW_MAX_DEG   * DEG_TO_RAD_F);
    }

    hmap->cmd_roll  = SBUS_AXIS_HOLD;
    hmap->cmd_pitch = SBUS_AXIS_HOLD;
    hmap->cmd_yaw   = SBUS_AXIS_HOLD;

    hmap->last_tick_ms = HAL_GetTick();
    hmap->initialized  = 1U;
}

void SBUS_Mapping_Update(SBUS_Mapping_Handle_t *hmap,
                         SBUS_Handle_t         *hsbus,
                         SBUS_Status_t          sbus_st)
{
    if (!hmap->initialized || hsbus == NULL) return;

    /* --- Tính dt (giây) --- */
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - hmap->last_tick_ms;
    hmap->last_tick_ms = now;

    /* Giới hạn dt tối đa 100ms để tránh nhảy lớn khi debug/breakpoint */
    if (elapsed > 100U) elapsed = 100U;
    float dt_s = (float)elapsed * 0.001f;

    /* --- Chỉ tích phân khi tín hiệu SBUS hợp lệ --- */
    if (sbus_st != SBUS_OK) {
        /* Mất tín hiệu hoặc Failsafe: giữ nguyên vị trí, reset lệnh */
        hmap->cmd_roll  = SBUS_AXIS_HOLD;
        hmap->cmd_pitch = SBUS_AXIS_HOLD;
        hmap->cmd_yaw   = SBUS_AXIS_HOLD;
        return;
    }

    /* --- Đọc giá trị raw 3 kênh điều khiển --- */
    uint16_t raw_ch1, raw_ch2, raw_ch4;
    SBUS_GetChannel(hsbus, 1, &raw_ch1);  /* CH1: Roll  */
    SBUS_GetChannel(hsbus, 2, &raw_ch2);  /* CH2: Pitch */
    SBUS_GetChannel(hsbus, 4, &raw_ch4);  /* CH4: Yaw   */

    /* --- Phân loại lệnh --- */
    hmap->cmd_roll  = raw_to_cmd(raw_ch1);
    hmap->cmd_pitch = raw_to_cmd(raw_ch2);
    hmap->cmd_yaw   = raw_to_cmd(raw_ch4);

    /* --- Tích phân góc mục tiêu --- */

    /* Roll (CH1) */
    if (hmap->cmd_roll != SBUS_AXIS_HOLD && hmap->target_roll_rad != NULL) {
        float delta = (float)hmap->cmd_roll          /* +1 hoặc -1   */
                      * SBUS_MAP_ROLL_SPEED_DPS       /* deg/s        */
                      * dt_s                          /* delta (deg)  */
                      * DEG_TO_RAD_F;                 /* → rad        */
        *hmap->target_roll_rad = fclampf(
            *hmap->target_roll_rad + delta,
            SBUS_MAP_ROLL_MIN_DEG * DEG_TO_RAD_F,
            SBUS_MAP_ROLL_MAX_DEG * DEG_TO_RAD_F);
    }

    /* Pitch (CH2) */
    if (hmap->cmd_pitch != SBUS_AXIS_HOLD && hmap->target_pitch_rad != NULL) {
        float delta = (float)hmap->cmd_pitch
                      * SBUS_MAP_PITCH_SPEED_DPS
                      * dt_s
                      * DEG_TO_RAD_F;
        *hmap->target_pitch_rad = fclampf(
            *hmap->target_pitch_rad + delta,
            SBUS_MAP_PITCH_MIN_DEG * DEG_TO_RAD_F,
            SBUS_MAP_PITCH_MAX_DEG * DEG_TO_RAD_F);
    }

    /* Yaw (CH4) */
    if (hmap->cmd_yaw != SBUS_AXIS_HOLD && hmap->target_yaw_rad != NULL) {
        float delta = (float)hmap->cmd_yaw
                      * SBUS_MAP_YAW_SPEED_DPS
                      * dt_s
                      * DEG_TO_RAD_F;
        *hmap->target_yaw_rad = fclampf(
            *hmap->target_yaw_rad + delta,
            SBUS_MAP_YAW_MIN_DEG * DEG_TO_RAD_F,
            SBUS_MAP_YAW_MAX_DEG * DEG_TO_RAD_F);
    }
}

SBUS_AxisCmd_t SBUS_Mapping_GetRollCmd(const SBUS_Mapping_Handle_t *hmap) {
    return hmap->cmd_roll;
}

SBUS_AxisCmd_t SBUS_Mapping_GetPitchCmd(const SBUS_Mapping_Handle_t *hmap) {
    return hmap->cmd_pitch;
}

SBUS_AxisCmd_t SBUS_Mapping_GetYawCmd(const SBUS_Mapping_Handle_t *hmap) {
    return hmap->cmd_yaw;
}
