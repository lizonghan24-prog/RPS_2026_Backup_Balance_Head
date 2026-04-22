#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/*
 * @file Control_Task.h
 * @brief 云台控制任务对外接口。
 *
 * 本头文件集中放置控制任务的宏配置、模式枚举、任务结构体和对外函数。
 * Control_Task.c 只保留具体控制流程和函数实现，方便以后直接在这里改参数。
 */

#include "main.h"
#include "PID.h"
#include "Remote.h"
#include "motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* 控制任务按 1 kHz 运行，节拍由 TIM6 提供。 */
#define CONTROL_TASK_DT_S                           0.001f

/* ========================= 电机挂载配置区 ========================= */
/* Pitch 轴当前使用 CAN1 上的 ID1 GM6020，按电流环方式控制。 */
#define CONTROL_PITCH_MOTOR_CAN                     (&hcan1)
#define CONTROL_PITCH_MOTOR_ID                      1U
/* Yaw 轴当前使用 CAN2 上的 ID1 GM6020，按电流环方式控制。 */
#define CONTROL_YAW_MOTOR_CAN                       (&hcan2)
#define CONTROL_YAW_MOTOR_ID                        1U

/* ========================= 遥控映射配置区 ========================= */
/* 图传遥控通道中，默认 ch0 控制 yaw，ch1 控制 pitch。 */
#define CONTROL_REMOTE_YAW_CHANNEL_INDEX            0U
#define CONTROL_REMOTE_PITCH_CHANNEL_INDEX          1U
/* 遥控输入映射成目标角速度，单位是 deg/s。 */
#define CONTROL_YAW_REMOTE_SCALE_DEG_PER_S          0.18f
#define CONTROL_PITCH_REMOTE_SCALE_DEG_PER_S        0.12f
/* 小于死区的输入直接丢掉，避免摇杆回中附近抖动。 */
#define CONTROL_REMOTE_DEADBAND                     10

/* ========================= IMU 映射配置区 ========================= */
/* IMU 的欧拉角数组默认按 roll、pitch、yaw 排列。 */
#define CONTROL_IMU_PITCH_ANGLE_INDEX               1U
#define CONTROL_IMU_YAW_ANGLE_INDEX                 2U
/* IMU 的角速度数组默认按 x、y、z 排列，这里取 pitch 和 yaw 对应轴。 */
#define CONTROL_IMU_PITCH_RATE_INDEX                1U
#define CONTROL_IMU_YAW_RATE_INDEX                  2U

/* ========================= 机械限位与回中配置区 ========================= */
/* Pitch 轴按机械限位处理，Yaw 轴按回环角处理。 */
#define CONTROL_PITCH_MIN_DEG                      -25.0f
#define CONTROL_PITCH_MAX_DEG                       25.0f
#define CONTROL_PITCH_CENTER_DEG                    0.0f
#define CONTROL_PITCH_PROTECT_MARGIN_DEG            2.0f
#define CONTROL_PITCH_WRAP_ENABLE                   0U

#define CONTROL_YAW_MIN_DEG                      -180.0f
#define CONTROL_YAW_MAX_DEG                       180.0f
#define CONTROL_YAW_CENTER_DEG                      0.0f
#define CONTROL_YAW_PROTECT_MARGIN_DEG              0.0f
#define CONTROL_YAW_WRAP_ENABLE                     1U

/* ========================= 模式映射配置区 ========================= */
/* 当前约定：C 挡无力，N 挡手动，S 挡回中；暂停键始终强制无力。 */
#define CONTROL_MODE_GEAR_RELAX                     REMOTE_SWITCH_C
#define CONTROL_MODE_GEAR_MANUAL                    REMOTE_SWITCH_N
#define CONTROL_MODE_GEAR_RECENTER                  REMOTE_SWITCH_S

/* ========================= 调试模式配置区 ========================= */
/* 默认按住右侧自定义键进入调试模式，调试模式下左侧自定义键触发回中。 */
#define CONTROL_DEBUG_ENABLE_BY_CUSTOM_RIGHT        1U
#define CONTROL_DEBUG_RECENTER_BY_CUSTOM_LEFT       1U
#define CONTROL_DEBUG_PITCH_REMOTE_SCALE_RATIO      0.25f
#define CONTROL_DEBUG_YAW_REMOTE_SCALE_RATIO        0.25f
#define CONTROL_DEBUG_PITCH_SPEED_LIMIT_DPS         60.0f
#define CONTROL_DEBUG_YAW_SPEED_LIMIT_DPS           80.0f
#define CONTROL_DEBUG_PITCH_CURRENT_LIMIT           3000.0f
#define CONTROL_DEBUG_YAW_CURRENT_LIMIT             3500.0f

/* ========================= Pitch 轴 PID 配置区 ========================= */
#define CONTROL_PITCH_ANGLE_KP                      10.0f
#define CONTROL_PITCH_ANGLE_KI                      0.0f
#define CONTROL_PITCH_ANGLE_KD                      0.2f
#define CONTROL_PITCH_ANGLE_I_LIMIT                 100.0f
#define CONTROL_PITCH_ANGLE_OUT_LIMIT               250.0f

#define CONTROL_PITCH_SPEED_KP                      90.0f
#define CONTROL_PITCH_SPEED_KI                      8.0f
#define CONTROL_PITCH_SPEED_KD                      0.3f
#define CONTROL_PITCH_SPEED_I_LIMIT                 3000.0f
#define CONTROL_PITCH_SPEED_OUT_LIMIT               12000.0f

/* ========================= Yaw 轴 PID 配置区 ========================= */
#define CONTROL_YAW_ANGLE_KP                        8.0f
#define CONTROL_YAW_ANGLE_KI                        0.0f
#define CONTROL_YAW_ANGLE_KD                        0.1f
#define CONTROL_YAW_ANGLE_I_LIMIT                   100.0f
#define CONTROL_YAW_ANGLE_OUT_LIMIT                 300.0f

#define CONTROL_YAW_SPEED_KP                        80.0f
#define CONTROL_YAW_SPEED_KI                        6.0f
#define CONTROL_YAW_SPEED_KD                        0.25f
#define CONTROL_YAW_SPEED_I_LIMIT                   3000.0f
#define CONTROL_YAW_SPEED_OUT_LIMIT                 12000.0f

/* 云台控制模式。 */
typedef enum
{
    CONTROL_MODE_RELAX = 0U,                     /* 无力模式，直接下发 0 电流。 */
    CONTROL_MODE_HOLD = 1U,                      /* 保持模式，锁住当前目标姿态。 */
    CONTROL_MODE_MANUAL = 2U,                    /* 手动模式，遥控输入积分成目标角。 */
    CONTROL_MODE_RECENTER = 3U,                  /* 回中模式，把目标角拉回中心。 */
    CONTROL_MODE_DEBUG = 4U                      /* 调试模式，降低灵敏度和输出上限。 */
} control_mode_t;

/* 单个云台轴的控制对象。外环控角度，内环控角速度。 */
typedef struct
{
    gm6020_service_t motor;                     /* 该轴绑定的 GM6020 服务对象。 */
    pid_t angle_pid;                            /* 外环角度 PID。 */
    pid_t speed_pid;                            /* 内环角速度 PID。 */
    float target_angle_deg;                     /* 当前目标角度，单位 deg。 */
    float angle_feedback_deg;                   /* 当前角度反馈，单位 deg。 */
    float speed_feedback_dps;                   /* 当前角速度反馈，单位 deg/s。 */
    float target_speed_dps;                     /* 外环输出的目标角速度，单位 deg/s。 */
    float target_current;                       /* 内环输出的目标电流。 */
    float remote_scale_deg_per_s;               /* 遥控输入到目标角速度的缩放系数。 */
    float debug_remote_scale_ratio;             /* 调试模式下额外乘上的灵敏度比例。 */
    float debug_speed_limit_dps;                /* 调试模式下的角速度限幅。 */
    float debug_current_limit;                  /* 调试模式下的电流限幅。 */
    float min_angle_deg;                        /* 角度下限。 */
    float max_angle_deg;                        /* 角度上限。 */
    float center_angle_deg;                     /* 回中模式使用的目标中心角。 */
    float protect_margin_deg;                   /* 触发机械限位保护时额外预留的裕量。 */
    uint8_t wrap_enable;                        /* 1 表示按回环角计算误差，0 表示按普通角度处理。 */
    uint8_t limit_active;                       /* 当前是否处于限位保护状态。 */
    uint8_t imu_angle_index;                    /* IMU 欧拉角数组中的反馈索引。 */
    uint8_t imu_rate_index;                     /* IMU 角速度数组中的反馈索引。 */
    uint8_t remote_channel_index;               /* 遥控通道索引。 */
} gimbal_axis_control_t;

/* 整个双轴云台任务的运行状态。 */
typedef struct
{
    uint8_t initialized;                        /* 是否已经完成过一次模式切换初始化。 */
    uint8_t imu_online;                         /* IMU 在线标志。 */
    uint8_t remote_online;                      /* 图传遥控在线标志。 */
    uint8_t pitch_motor_online;                 /* Pitch 电机在线标志。 */
    uint8_t yaw_motor_online;                   /* Yaw 电机在线标志。 */
    control_mode_t mode;                        /* 本拍实际执行的模式。 */
    control_mode_t last_mode;                   /* 上一拍实际执行的模式。 */
    gimbal_axis_control_t yaw;                  /* Yaw 轴控制对象。 */
    gimbal_axis_control_t pitch;                /* Pitch 轴控制对象。 */
} gimbal_control_task_t;

/*
 * 控制任务初始化。
 *
 * 需要在 BSP 和底层电机服务初始化完成后调用一次。
 */
void Control_Task_Init(void);

/*
 * 主循环或调度器中的控制任务执行入口。
 *
 * 本函数会读取最新 IMU / 遥控状态，完成模式判断、双环 PID 和 CAN 输出。
 */
void Control_Task_Run(void);

#endif
