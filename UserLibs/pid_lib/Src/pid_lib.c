#include "pid_lib.h"

static float _clamp(float val, float min, float max) {
    return (val < min) ? min : ((val > max) ? max : val);
}

void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, float min, float max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = min;
    pid->out_max = max;
    PID_Reset(pid);
}

float PID_Update(PID_Handle_t *pid, float error, float dt) {
    // Proportional term
    float p_term = pid->Kp * error;

    // Derivative term — guard against dt == 0 to prevent NaN/Inf
    float d_term = 0.0f;
    if (dt > 1e-9f) {
        d_term = pid->Kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    // Calculate candidate unclamped output with potential new integral
    float new_integral = pid->integral + pid->Ki * error * dt;
    float out_unclamped = p_term + new_integral + d_term;
    float out = _clamp(out_unclamped, pid->out_min, pid->out_max);

    // Conditional Integration Anti-Windup:
    // Only accumulate integral if output is NOT saturated, OR if the error is pushing out back into linear region.
    if (out == out_unclamped) {
        pid->integral = _clamp(new_integral, pid->out_min, pid->out_max);
    } else {
        if ((out >= pid->out_max && error < 0.0f) || (out <= pid->out_min && error > 0.0f)) {
            pid->integral = _clamp(new_integral, pid->out_min, pid->out_max);
        }
    }

    return out;
}

void PID_Reset(PID_Handle_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
