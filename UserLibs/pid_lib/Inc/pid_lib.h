#ifndef PID_LIB_H
#define PID_LIB_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID_Handle_t;

/**
 * @brief Initialize PID parameters.
 * @param pid Pointer to PID handle.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param min Minimum output limit.
 * @param max Maximum output limit.
 */
void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, float min, float max);

/**
 * @brief Update PID output based on error and elapsed time.
 * @param pid Pointer to PID handle.
 * @param error Current error value.
 * @param dt   Time step in seconds.
 * @return PID output constrained to [out_min, out_max].
 */
float PID_Update(PID_Handle_t *pid, float error, float dt);

/**
 * @brief Reset integral and previous error terms.
 */
void PID_Reset(PID_Handle_t *pid);

#endif // PID_LIB_H
