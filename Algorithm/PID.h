#ifndef PID_H
#define PID_H

#include "main.h"

/*
 * PID quick usage:
 * 1) Call PID_Init() once with gains, sample time, and limits.
 * 2) Optional: call PID_Reset() when mode changes or motor is disabled.
 * 3) In control loop call PID_Calculate(reference, feedback) every dt.
 * 4) If error is already computed externally, call PID_CalculateByError().
 */

/* 通用浮点 PID 控制器。 */
typedef struct
{
    float kp;                            /* 比例系数。 */
    float ki;                            /* 积分系数。 */
    float kd;                            /* 微分系数。 */
    float dt_s;                          /* 控制周期，单位 s。 */
    float integral;                      /* 当前积分累计值。 */
    float integral_limit;                /* 积分限幅绝对值。 */
    float output_limit;                  /* 输出限幅绝对值。 */
    float last_error;                    /* 上一次误差。 */
    float p_out;                         /* 当前比例项输出。 */
    float i_out;                         /* 当前积分项输出。 */
    float d_out;                         /* 当前微分项输出。 */
    float output;                        /* 当前总输出。 */
} pid_t;

/* Initialize PID gains, dt, integral limit, and output limit. */
void PID_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float dt_s,
              float integral_limit,
              float output_limit);

/* Clear integrator and history while keeping gains and limits. */
void PID_Reset(pid_t *pid);

/* Compute PID output from reference and feedback. */
float PID_Calculate(pid_t *pid, float reference, float feedback);

/* Compute PID output directly from a precomputed error value. */
float PID_CalculateByError(pid_t *pid, float error);

#endif
