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
    // Integral term with anti-windup clamping
    pid->integral += pid->Ki * error * dt;
    pid->integral = _clamp(pid->integral, pid->out_min, pid->out_max);

    // Derivative term — guard against dt == 0 to prevent NaN/Inf
    float d_term = 0.0f;
    if (dt > 1e-9f) {
        d_term = pid->Kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    // Compute total output and clamp
    float out = pid->Kp * error + pid->integral + d_term;
    return _clamp(out, pid->out_min, pid->out_max);
}

void PID_Reset(PID_Handle_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
