#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/*
 * @file Control_Task.h
 * @brief 云台控制函数库的对外接口和参数配置。
 *
 * Control_Task 只对外暴露可复用辅助函数和共享状态。
 * 真正的每拍控制顺序在 Up_Task.c，模式决策在 Mode_Switch_Task.c。
 */

#include "main.h"
#include "IMU.h"
#include "PID.h"
#include "Remote.h"
#include "motor.h"
#include "motor_dm.h"

/*
 * 快速使用说明：
 * - BSP_Init() 后调用一次 Control_Task_Init()。
 * - 运行时固定 1 kHz 调用 Up_Task_Run()。
 * - Control_Task_Run() 只保留为兼容入口。
 * - 调试时通过 Control_Task_GetState() 读取状态。
 *
 * 当前电机映射：
 * - Pitch：GM6020，CAN1，ID5。
 * - Yaw：倒装 DM4310，CAN2，ID1。
 * - Yaw 电机角度、速度和输出极性与 IMU 坐标相反。
 *
 * 遥控映射：
 * - ch2 -> pitch，ch3 -> yaw。
 * - C 档 -> RELAX，N 档 -> 遥控输入，S 档 -> 键鼠输入。
 */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* 固定 1 kHz 控制周期，运行时节拍由 TIM6 提供。 */
#define CONTROL_TASK_DT_S                           0.001f

/* 电机绑定配置。 */
/* Pitch 使用 CAN1 上 ID5 的 GM6020。 */
#define CONTROL_PITCH_MOTOR_CAN                     (&hcan1)
#define CONTROL_PITCH_MOTOR_ID                      5U
/* Pitch 电机方向与控制坐标一致。 */
#define CONTROL_PITCH_OUTPUT_SIGN                   1.0f

/* Yaw 使用 CAN2 上 ID1 的 DM4310，工作在 MIT 力矩模式。 */
#define CONTROL_YAW_MOTOR_CAN                       (&hcan2)
#define CONTROL_YAW_MOTOR_ID                        1U
/*
 * DM4310 反馈 StdId 由电机 MasterID 决定。
 * 当前约定是电机 ID1 -> 反馈 StdId 0x001。
 */
#define CONTROL_YAW_DM4310_FEEDBACK_STD_ID          0x001U
/*
 * Yaw 相对 IMU yaw / gyro 倒装，所以 PID 输出计算完成后，
 * 最终下发给 DM4310 的力矩命令需要取反。
 */
#define CONTROL_YAW_OUTPUT_SIGN                    -1.0f

/* 遥控和鼠标输入映射。 */
/* 遥控 ch2 控制 pitch，ch3 控制 yaw。 */
#define CONTROL_REMOTE_YAW_CHANNEL_INDEX            3U
#define CONTROL_REMOTE_PITCH_CHANNEL_INDEX          2U
/* 遥控输入映射为目标角速度，单位 deg/s。 */
#define CONTROL_YAW_REMOTE_SCALE_DEG_PER_S          0.18f
#define CONTROL_PITCH_REMOTE_SCALE_DEG_PER_S        0.12f
/* 忽略摇杆中位附近的小量抖动。 */
#define CONTROL_REMOTE_DEADBAND                     10
/* 鼠标输入映射为目标角速度，单位 deg/s/count。 */
#define CONTROL_MOUSE_YAW_SCALE_DEG_PER_S           0.05f
#define CONTROL_MOUSE_PITCH_SCALE_DEG_PER_S        -0.05f
/* 忽略鼠标微小抖动。 */
#define CONTROL_MOUSE_DEADBAND                      1

/* IMU 通道映射。 */
/* IMU 欧拉角数组默认按 roll、pitch、yaw 排列。 */
#define CONTROL_IMU_PITCH_ANGLE_INDEX               1U
#define CONTROL_IMU_YAW_ANGLE_INDEX                 2U
/* IMU 角速度数组默认按 x、y、z 排列。 */
#define CONTROL_IMU_PITCH_RATE_INDEX                1U
#define CONTROL_IMU_YAW_RATE_INDEX                  2U

/* 机械限位配置。 */
/* Pitch 按机械限位处理，yaw 按回环角处理。 */
#define CONTROL_PITCH_MIN_DEG                      -25.0f
#define CONTROL_PITCH_MAX_DEG                       25.0f
#define CONTROL_PITCH_PROTECT_MARGIN_DEG            2.0f
#define CONTROL_PITCH_WRAP_ENABLE                   0U

#define CONTROL_YAW_MIN_DEG                      -180.0f
#define CONTROL_YAW_MAX_DEG                       180.0f
#define CONTROL_YAW_PROTECT_MARGIN_DEG              0.0f
#define CONTROL_YAW_WRAP_ENABLE                     1U

/* 初始化目标和完成阈值，对齐原 gimbal_task。 */
#define CONTROL_INIT_PITCH_TARGET_DEG               0.0f
#define CONTROL_INIT_YAW_TARGET_DEG                 0.0f
#define CONTROL_INIT_PITCH_FINISH_ERROR_DEG         6.0f
#define CONTROL_INIT_YAW_FINISH_ERROR_DEG           5.0f

/* 模式档位映射；暂停键始终在 Mode_Switch_Task 中强制 RELAX。 */
#define CONTROL_MODE_GEAR_RELAX                     REMOTE_SWITCH_C
#define CONTROL_MODE_GEAR_REMOTE                    REMOTE_SWITCH_N
#define CONTROL_MODE_GEAR_KEY_MOUSE                 REMOTE_SWITCH_S

/* Pitch 角度环 PID。 */
#define CONTROL_PITCH_ANGLE_KP                      10.0f
#define CONTROL_PITCH_ANGLE_KI                      0.0f
#define CONTROL_PITCH_ANGLE_KD                      0.2f
#define CONTROL_PITCH_ANGLE_I_LIMIT                 100.0f
#define CONTROL_PITCH_ANGLE_OUT_LIMIT               250.0f

/* Pitch 速度环 PID。 */
#define CONTROL_PITCH_SPEED_KP                      90.0f
#define CONTROL_PITCH_SPEED_KI                      8.0f
#define CONTROL_PITCH_SPEED_KD                      0.3f
#define CONTROL_PITCH_SPEED_I_LIMIT                 3000.0f
#define CONTROL_PITCH_SPEED_OUT_LIMIT               12000.0f

/* Yaw 角度环 PID。 */
#define CONTROL_YAW_ANGLE_KP                        8.0f
#define CONTROL_YAW_ANGLE_KI                        0.0f
#define CONTROL_YAW_ANGLE_KD                        0.1f
#define CONTROL_YAW_ANGLE_I_LIMIT                   100.0f
#define CONTROL_YAW_ANGLE_OUT_LIMIT                 300.0f

/* Yaw 速度环 PID。 */
#define CONTROL_YAW_SPEED_KP                        0.08f
#define CONTROL_YAW_SPEED_KI                        0.0f
#define CONTROL_YAW_SPEED_KD                        0.0f
#define CONTROL_YAW_SPEED_I_LIMIT                   2.0f
#define CONTROL_YAW_SPEED_OUT_LIMIT                 8.0f

/* 云台控制模式。 */
typedef enum
{
    CONTROL_MODE_RELAX = 0U,                     /* 电机零输出。 */
    CONTROL_MODE_INIT = 1U,                      /* 云台转向初始化目标。 */
    CONTROL_MODE_FOLLOW_ZGYRO = 2U               /* 正常 IMU 跟随控制。 */
} control_mode_t;

/* 单个云台轴当前绑定的电机协议。 */
typedef enum
{
    CONTROL_AXIS_MOTOR_GM6020 = 0U,             /* DJI GM6020 电流环。 */
    CONTROL_AXIS_MOTOR_DM4310 = 1U              /* DM4310 MIT 力矩帧。 */
} control_axis_motor_type_t;

/* 单个云台轴控制对象：外环控制角度，内环控制角速度。 */
typedef struct
{
    control_axis_motor_type_t motor_type;       /* 当前轴使用的电机协议。 */
    gm6020_service_t gm6020_motor;              /* Pitch 使用的 GM6020 服务对象。 */
    dm4310_service_t dm4310_motor;              /* Yaw 使用的 DM4310 服务对象。 */
    pid_t angle_pid;                            /* 外环角度 PID。 */
    pid_t speed_pid;                            /* 内环角速度 PID。 */
    float target_angle_deg;                     /* 当前目标角度，单位 deg。 */
    float angle_feedback_deg;                   /* 当前角度反馈，单位 deg。 */
    float speed_feedback_dps;                   /* 当前角速度反馈，单位 deg/s。 */
    float target_speed_dps;                     /* 角度环输出的目标角速度，单位 deg/s。 */
    float target_output;                        /* 速度环输出：GM6020 电流或 DM4310 力矩。 */
    float output_sign;                          /* 电机安装方向补偿。 */
    float remote_scale_deg_per_s;               /* 遥控输入到目标角速度的缩放。 */
    float mouse_scale_deg_per_s;                /* 鼠标输入到目标角速度的缩放。 */
    float min_angle_deg;                        /* 角度下限。 */
    float max_angle_deg;                        /* 角度上限。 */
    float protect_margin_deg;                   /* 触发限位保护前的额外裕量。 */
    uint8_t wrap_enable;                        /* 1 表示回环角误差，0 表示机械限幅。 */
    uint8_t limit_active;                       /* 非零表示当前处于机械限位保护。 */
    uint8_t imu_angle_index;                    /* IMU 欧拉角反馈索引。 */
    uint8_t imu_rate_index;                     /* IMU 角速度反馈索引。 */
    uint8_t remote_channel_index;               /* 遥控通道索引。 */
} gimbal_axis_control_t;

/* 双轴云台控制函数库的运行时状态。 */
typedef struct
{
    uint8_t initialized;                        /* 已经完成过一次模式边沿处理。 */
    uint8_t imu_online;                         /* IMU 在线标志。 */
    uint8_t remote_online;                      /* 遥控在线标志。 */
    uint8_t pitch_motor_online;                 /* Pitch 电机在线标志。 */
    uint8_t yaw_motor_online;                   /* Yaw 电机在线标志。 */
    uint8_t init_finished;                      /* 初始化完成标志。 */
    control_mode_t mode;                        /* 本拍选中的模式。 */
    control_mode_t last_mode;                   /* 上一拍的模式。 */
    gimbal_axis_control_t yaw;                  /* Yaw 轴控制对象。 */
    gimbal_axis_control_t pitch;                /* Pitch 轴控制对象。 */
} gimbal_control_task_t;

/*
 * BSP 和底层电机服务就绪后，初始化一次控制函数库。
 */
void Control_Task_Init(void);

/*
 * 旧调度器兼容入口；真正运行顺序交给 Up_Task_Run()。
 */
void Control_Task_Run(void);

/* Up_Task 执行真实控制顺序时使用的函数库接口。 */
void Control_Task_UpdateOnlineFlags(const imu_hi91_t *imu, const remote_state_t *remote);
void Control_Task_SetMode(control_mode_t mode);
void Control_Task_UpdateFeedback(const imu_hi91_t *imu);
void Control_Task_UpdateInitTargets(void);
uint8_t Control_Task_IsInitFinished(void);
void Control_Task_ClearInitFinished(void);
void Control_Task_UpdateRemoteTargets(const remote_state_t *remote, float scale_ratio);
void Control_Task_UpdateMouseTargets(int16_t mouse_x, int16_t mouse_y);
void Control_Task_HandleModeTransition(void);
void Control_Task_StopOutput(void);
void Control_Task_SendOutput(void);
void Control_Task_RunClosedLoop(void);

/* 返回只读状态指针，用于调试和模式切换判断。 */
const gimbal_control_task_t *Control_Task_GetState(void);

/* API 说明：
 * - Control_Task_Init(): 初始化 PID 和电机注册。
 * - Control_Task_Run(): 兼容入口，内部转到 Up_Task_Run()。
 * - Control_Task_GetState(): 查看在线标志和目标值。
 */

#endif
