#ifndef PID_H
#define PID_H

#include "main.h"

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

/* 初始化 PID 参数和限幅。 */
void PID_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float dt_s,
              float integral_limit,
              float output_limit);

/* 清空 PID 内部状态，但保留参数配置。 */
void PID_Reset(pid_t *pid);

/* 按参考值和反馈值计算 PID。 */
float PID_Calculate(pid_t *pid, float reference, float feedback);

/* 直接按误差计算 PID。 */
float PID_CalculateByError(pid_t *pid, float error);

#endif
