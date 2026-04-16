#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "main.h"

/* 摩擦轮任务的工作模式定义。 */
typedef enum
{
    SHOOT_FRICTION_MODE_STOP = 0U,              /* 停转模式：两路摩擦轮输出清零。 */
    SHOOT_FRICTION_MODE_RUN = 1U                /* 运行模式：两路摩擦轮速度环闭环。 */
} shoot_friction_mode_t;

/* 拨盘任务的工作模式定义。 */
typedef enum
{
    SHOOT_DIAL_MODE_STOP = 0U,                  /* 停止模式：拨盘停机，只做状态轮询。 */
    SHOOT_DIAL_MODE_CLOSED_LOOP = 1U            /* 闭环模式：拨盘执行角度环和速度环。 */
} shoot_dial_mode_t;

/* 提供给上层查看的发射任务状态。 */
typedef struct
{
    uint8_t remote_online;                      /* 当前图传遥控是否在线。 */
    uint8_t left_motor_online;                  /* 左摩擦轮电机是否在线。 */
    uint8_t right_motor_online;                 /* 右摩擦轮电机是否在线。 */
    uint8_t dial_motor_online;                  /* LK 拨盘电机是否在线。 */
    uint8_t software_enable_request;            /* 软件侧是否请求开启摩擦轮。 */
    uint8_t remote_enable_request;              /* 遥控侧是否请求开启摩擦轮。 */
    uint8_t friction_ready;                     /* 摩擦轮是否已经达到允许拨盘闭环的稳定状态。 */
    uint8_t dial_output_enabled;                /* 当前底层是否认为 LK 电机已进入输出使能状态。 */
    shoot_friction_mode_t friction_mode;        /* 当前摩擦轮任务实际模式。 */
    shoot_dial_mode_t dial_mode;                /* 当前拨盘任务实际模式。 */
    uint16_t dial_pending_step_count;           /* 当前尚未执行完成的拨盘步进请求数。 */
    uint32_t dial_total_step_count;             /* 自上电以来已经执行过的拨盘步进总数。 */
    float left_target_rpm;                      /* 左摩擦轮设定目标转速。 */
    float right_target_rpm;                     /* 右摩擦轮设定目标转速。 */
    float left_command_rpm;                     /* 左摩擦轮斜坡后的当前指令转速。 */
    float right_command_rpm;                    /* 右摩擦轮斜坡后的当前指令转速。 */
    float left_feedback_rpm;                    /* 左摩擦轮当前反馈转速。 */
    float right_feedback_rpm;                   /* 右摩擦轮当前反馈转速。 */
    float left_target_current;                  /* 左摩擦轮当前速度环输出的目标电流。 */
    float right_target_current;                 /* 右摩擦轮当前速度环输出的目标电流。 */
    float dial_step_angle_deg;                  /* 拨盘单次步进角度配置值。 */
    float dial_target_angle_deg;                /* 拨盘当前目标角度。 */
    float dial_feedback_angle_deg;              /* 拨盘当前反馈角度。 */
    float dial_feedback_speed_dps;              /* 拨盘当前反馈角速度。 */
    float dial_target_speed_dps;                /* 拨盘角度环输出的目标角速度。 */
    float dial_target_iq;                       /* 拨盘速度环输出的目标 iq/力矩命令。 */
} shoot_task_state_t;

/* 发射任务初始化。 */
void Shoot_Task_Init(void);

/* 主循环中的发射任务执行入口。 */
void Shoot_Task_Run(void);

/* TIM6 的 1 kHz 节拍入口。 */
void Shoot_Task_Timer1kHzCallback(void);

/* 由上层软件直接控制摩擦轮启停。 */
void Shoot_Task_SetFrictionEnable(uint8_t enable);

/* 由上层软件直接修改摩擦轮目标转速。 */
void Shoot_Task_SetFrictionTargetRpm(float left_target_rpm, float right_target_rpm);

/* 在拨盘允许闭环时，追加若干个步进请求。 */
void Shoot_Task_RequestDialStep(uint16_t step_count);

/* 获取当前发射任务状态。 */
const shoot_task_state_t *Shoot_Task_GetState(void);

#endif
