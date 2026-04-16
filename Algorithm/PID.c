#include "PID.h"

#include <string.h>

/* 对数值做对称限幅。 */
static float PID_Limit(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

void PID_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float dt_s,
              float integral_limit,
              float output_limit)
{
    if (pid == NULL)
    {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt_s = dt_s;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void PID_Reset(pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
}

float PID_Calculate(pid_t *pid, float reference, float feedback)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    return PID_CalculateByError(pid, reference - feedback);
}

float PID_CalculateByError(pid_t *pid, float error)
{
    float derivative;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->p_out = pid->kp * error;

    pid->integral += pid->ki * error * pid->dt_s;
    pid->integral = PID_Limit(pid->integral, pid->integral_limit);
    pid->i_out = pid->integral;

    if (pid->dt_s > 0.0f)
    {
        derivative = (error - pid->last_error) / pid->dt_s;
    }
    else
    {
        derivative = 0.0f;
    }

    pid->d_out = pid->kd * derivative;
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = PID_Limit(pid->output, pid->output_limit);
    pid->last_error = error;

    return pid->output;
}
