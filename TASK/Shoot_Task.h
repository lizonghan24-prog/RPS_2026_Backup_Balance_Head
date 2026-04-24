#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

/*
 * @file Shoot_Task.h
 * @brief 发射任务对外接口。
 *
 * 发射任务包含摩擦轮和拨盘两部分：
 * - 摩擦轮负责建立稳定弹速；
 * - 拨盘负责在摩擦轮 ready 后按步进角推弹。
 *
 * 本头文件集中放置发射任务的宏配置、模式枚举、任务结构体和对外函数。
 * 其他模块正常只需要调用下面的接口，不建议直接改任务内部状态。
 */

#include "main.h"
#include "PID.h"
#include "Remote.h"
#include "motor.h"

/*
 * Shoot task quick usage:
 * - Call Shoot_Task_Init() once after BSP_Init().
 * - Call Shoot_Task_Run() at fixed 1 kHz.
 * - Control friction start/stop with Shoot_Task_SetFrictionEnable().
 * - Optional: adjust wheel speed by Shoot_Task_SetFrictionTargetRpm().
 * - Queue dial step by Shoot_Task_RequestDialStep() in closed-loop mode.
 */

extern CAN_HandleTypeDef hcan2;

/* 发射任务按 1 kHz 运行，节拍由 TIM6 提供。 */
#define SHOOT_TASK_DT_S                               0.001f

/* ========================= 摩擦轮电机挂载配置区 ========================= */
/* 左摩擦轮使用 CAN2 上的 ID1 M3508。 */
#define SHOOT_FRICTION_LEFT_CAN                       (&hcan2)
#define SHOOT_FRICTION_LEFT_ID                        1U
/* 右摩擦轮使用 CAN2 上的 ID2 M3508。 */
#define SHOOT_FRICTION_RIGHT_CAN                      (&hcan2)
#define SHOOT_FRICTION_RIGHT_ID                       2U

/* ========================= LK dial motor mapping ========================= */
/* LK dial motor now uses CAN2 logical motor ID1 (StdId = 0x141). */
#define SHOOT_DIAL_CAN                                (&hcan2)
#define SHOOT_DIAL_ID                                 1U

/* ========================= 摩擦轮目标转速配置区 ========================= */
/* 默认左正右负；如果实物方向反了，直接改这里的符号。 */
#define SHOOT_FRICTION_LEFT_TARGET_RPM                4500.0f
#define SHOOT_FRICTION_RIGHT_TARGET_RPM              -4500.0f
/* 启停时做速度斜坡，避免电流突变太大。 */
#define SHOOT_FRICTION_RAMP_RPM_PER_S                 12000.0f

/* ========================= 摩擦轮稳定判定配置区 ========================= */
/* 两路摩擦轮都稳定一段时间后，才允许拨盘进入闭环。 */
#define SHOOT_FRICTION_READY_RPM_RATIO                0.85f
#define SHOOT_FRICTION_READY_ERROR_RPM                350.0f
#define SHOOT_FRICTION_READY_HOLD_TICKS               200U

/* ========================= 遥控映射配置区 ========================= */
/* 默认按住左侧自定义键开启摩擦轮。 */
#define SHOOT_ENABLE_BY_CUSTOM_LEFT                   1U
/* 如果想让扳机也能启摩擦轮，把这里改成 1U。 */
#define SHOOT_ENABLE_BY_TRIGGER                       0U
/* 默认扳机上升沿记一次拨盘步进请求。 */
#define SHOOT_DIAL_STEP_BY_TRIGGER                    1U

/* ========================= 摩擦轮速度环 PID 配置区 ========================= */
#define SHOOT_FRICTION_SPEED_KP                       5.0f
#define SHOOT_FRICTION_SPEED_KI                       0.12f
#define SHOOT_FRICTION_SPEED_KD                       0.0f
#define SHOOT_FRICTION_SPEED_I_LIMIT                  3000.0f
#define SHOOT_FRICTION_SPEED_OUT_LIMIT                10000.0f

/* ========================= 拨盘步进与状态轮询配置区 ========================= */
/* 默认一步转 45 度；如果方向反了，直接把这个值改成负数。 */
#define SHOOT_DIAL_STEP_DEG                           45.0f
/* 当前步进误差收敛到这个窗口内后，才继续吃下一次步进请求。 */
#define SHOOT_DIAL_STEP_ACCEPT_ERROR_DEG              2.0f
/* 停机状态下，按固定周期查询一次 LK 状态。 */
#define SHOOT_DIAL_STATE_POLL_TICKS                   10U

/* ========================= 拨盘角度环 PID 配置区 ========================= */
/* 外环输入是角度误差，输出是目标角速度。 */
#define SHOOT_DIAL_ANGLE_KP                           10.0f
#define SHOOT_DIAL_ANGLE_KI                           0.0f
#define SHOOT_DIAL_ANGLE_KD                           0.0f
#define SHOOT_DIAL_ANGLE_I_LIMIT                      200.0f
#define SHOOT_DIAL_ANGLE_OUT_LIMIT                    720.0f

/* ========================= 拨盘速度环 PID 配置区 ========================= */
/* 内环输入是角速度误差，输出是 LK 的 iq/力矩命令。 */
#define SHOOT_DIAL_SPEED_KP                           1.2f
#define SHOOT_DIAL_SPEED_KI                           0.08f
#define SHOOT_DIAL_SPEED_KD                           0.0f
#define SHOOT_DIAL_SPEED_I_LIMIT                      300.0f
#define SHOOT_DIAL_SPEED_OUT_LIMIT                    800.0f

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

/*
 * 提供给上层查看的发射任务状态。
 *
 * 这个结构体是只读快照，用来观察当前模式、在线状态、目标值和反馈值。
 * 上层不要通过它反向修改任务状态，控制入口请使用下面的 Set/Request 函数。
 */
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

/* 单个摩擦轮的控制对象。 */
typedef struct
{
    m3508_service_t motor;                           /* 当前摩擦轮绑定的 M3508 服务对象。 */
    pid_t speed_pid;                                 /* 当前摩擦轮的速度环 PID。 */
    float target_rpm;                                /* 目标转速，单位 rpm。 */
    float command_rpm;                               /* 经过斜坡后的实际指令转速，单位 rpm。 */
    float feedback_rpm;                              /* 当前反馈转速，单位 rpm。 */
    float target_current;                            /* 速度环输出的目标电流。 */
} shoot_friction_wheel_t;

/* LK 拨盘控制对象。 */
typedef struct
{
    lk_motor_service_t motor;                        /* 当前拨盘绑定的 LK 电机服务对象。 */
    pid_t angle_pid;                                 /* 拨盘外环角度 PID。 */
    pid_t speed_pid;                                 /* 拨盘内环速度 PID。 */
    float step_angle_deg;                            /* 单次步进角度，单位 deg。 */
    float target_angle_deg;                          /* 当前目标角度，单位 deg。 */
    float feedback_angle_deg;                        /* 当前反馈角度，单位 deg。 */
    float feedback_speed_dps;                        /* 当前反馈角速度，单位 deg/s。 */
    float target_speed_dps;                          /* 外环输出的目标角速度，单位 deg/s。 */
    float target_iq;                                 /* 内环输出的目标 iq/力矩命令。 */
    uint8_t last_trigger_pressed;                    /* 上一拍扳机状态，用来检测上升沿。 */
    uint16_t pending_step_count;                     /* 尚未执行的步进请求数量。 */
    uint32_t total_step_count;                       /* 上电以来累计执行的步进次数。 */
    uint16_t state_poll_tick;                        /* 停机轮询 LK 状态用的分频计数。 */
} shoot_dial_t;

/* 整个发射任务的运行状态。 */
typedef struct
{
    uint8_t friction_initialized;                    /* 是否已经完成过一次摩擦轮模式初始化。 */
    uint8_t dial_initialized;                        /* 是否已经完成过一次拨盘模式初始化。 */
    uint8_t remote_online;                           /* 图传遥控在线标志。 */
    uint8_t left_motor_online;                       /* 左摩擦轮在线标志。 */
    uint8_t right_motor_online;                      /* 右摩擦轮在线标志。 */
    uint8_t dial_motor_online;                       /* LK 拨盘在线标志。 */
    uint8_t software_enable_request;                 /* 软件侧是否请求开启摩擦轮。 */
    uint8_t remote_enable_request;                   /* 遥控侧是否请求开启摩擦轮。 */
    uint8_t friction_ready;                          /* 两路摩擦轮是否已经转稳。 */
    uint16_t friction_ready_ticks;                   /* 摩擦轮稳定累计计数。 */
    shoot_friction_mode_t friction_mode;             /* 当前摩擦轮实际模式。 */
    shoot_friction_mode_t last_friction_mode;        /* 上一拍摩擦轮实际模式。 */
    shoot_dial_mode_t dial_mode;                     /* 当前拨盘实际模式。 */
    shoot_dial_mode_t last_dial_mode;                /* 上一拍拨盘实际模式。 */
    shoot_friction_wheel_t left;                     /* 左摩擦轮控制对象。 */
    shoot_friction_wheel_t right;                    /* 右摩擦轮控制对象。 */
    shoot_dial_t dial;                               /* 拨盘控制对象。 */
} shoot_task_t;

/*
 * 发射任务初始化。
 *
 * 负责注册电机、初始化 PID 参数，并建立默认目标转速和拨盘步进角。
 */
void Shoot_Task_Init(void);

/*
 * 主循环或调度器中的发射任务执行入口。
 *
 * 内部会完成摩擦轮速度闭环、ready 判定、拨盘联锁和 CAN 输出。
 */
void Shoot_Task_Run(void);

/*
 * 由上层软件直接控制摩擦轮启停。
 *
 * enable 为 0 表示清除软件启动请求，非 0 表示置位软件启动请求。
 * 最终能否运行仍受遥控 pause、电机在线状态等保护条件约束。
 */
void Shoot_Task_SetFrictionEnable(uint8_t enable);

/*
 * 由上层软件直接修改摩擦轮目标转速。
 *
 * 目标值单位为 rpm；实际输出会在任务内部做速度斜坡，不会直接阶跃到目标。
 */
void Shoot_Task_SetFrictionTargetRpm(float left_target_rpm, float right_target_rpm);

/*
 * 在拨盘允许闭环时，追加若干个步进请求。
 *
 * step_count 表示追加多少个单步；每一步的角度由 Shoot_Task.c 内部配置。
 */
void Shoot_Task_RequestDialStep(uint16_t step_count);

/*
 * 获取当前发射任务状态。
 *
 * 返回值是内部状态快照指针，只用于读取。
 */
const shoot_task_state_t *Shoot_Task_GetState(void);

/* API notes:
 * - SHOOT_DIAL_ID is logical LK motor ID; StdId = 0x140 + SHOOT_DIAL_ID.
 * - Current mapping: dial ID1 -> StdId 0x141, left friction 0x201, right friction 0x202.
 */

#endif
