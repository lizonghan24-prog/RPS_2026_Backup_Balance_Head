#include "Control_Task.h"

#include "PID.h"
#include "IMU.h"
#include "Remote.h"
#include "motor.h"

#include <string.h>

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
#define CONTROL_MODE_GEAR_RELAX                REMOTE_SWITCH_C
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
    volatile uint32_t tick_pending;             /* 等待主循环处理的 1 kHz 节拍数。 */
    uint8_t imu_online;                         /* IMU 在线标志。 */
    uint8_t remote_online;                      /* 图传遥控在线标志。 */
    uint8_t pitch_motor_online;                 /* Pitch 电机在线标志。 */
    uint8_t yaw_motor_online;                   /* Yaw 电机在线标志。 */
    control_mode_t mode;                        /* 本拍实际执行的模式。 */
    control_mode_t last_mode;                   /* 上一拍实际执行的模式。 */
    gimbal_axis_control_t yaw;                  /* Yaw 轴控制对象。 */
    gimbal_axis_control_t pitch;                /* Pitch 轴控制对象。 */
} gimbal_control_task_t;

static gimbal_control_task_t gimbal_control;

/* 把数值限制在给定区间内。 */
static float Control_Limit(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/* 做对称限幅。limit 小于等于 0 时表示不额外限幅。 */
static float Control_LimitSymmetric(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }

    return Control_Limit(value, -limit, limit);
}

/* 把角度规范到 [-180, 180]。 */
static float Control_NormalizeAngle180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/* 计算角度误差。回环轴按最短路径取误差。 */
static float Control_CalcAngleError(float target_angle_deg, float feedback_angle_deg, uint8_t wrap_enable)
{
    float error;

    error = target_angle_deg - feedback_angle_deg;
    if (wrap_enable != 0U)
    {
        error = Control_NormalizeAngle180(error);
    }

    return error;
}

/* 统一约束目标角，避免目标值越界。 */
static void Control_ClampAxisTarget(gimbal_axis_control_t *axis)
{
    if (axis->wrap_enable != 0U)
    {
        axis->target_angle_deg = Control_NormalizeAngle180(axis->target_angle_deg);
    }
    else
    {
        axis->target_angle_deg = Control_Limit(axis->target_angle_deg,
                                               axis->min_angle_deg,
                                               axis->max_angle_deg);
    }
}

/* 把目标角对齐到当前反馈角，切模式时可以避免跳变。 */
static void Control_AlignAxisTargetToFeedback(gimbal_axis_control_t *axis)
{
    axis->target_angle_deg = axis->angle_feedback_deg;
    Control_ClampAxisTarget(axis);
}

/* 把目标角直接设置到回中角。 */
static void Control_SetAxisTargetToCenter(gimbal_axis_control_t *axis)
{
    axis->target_angle_deg = axis->center_angle_deg;
    Control_ClampAxisTarget(axis);
}

/* 给单路遥控输入做死区处理。 */
static int16_t Control_ApplyRemoteDeadband(int16_t input)
{
    if ((input < CONTROL_REMOTE_DEADBAND) && (input > -CONTROL_REMOTE_DEADBAND))
    {
        return 0;
    }

    return input;
}

/* 刷新单个轴的 IMU 反馈。 */
static void Control_UpdateAxisFeedback(gimbal_axis_control_t *axis, const imu_hi91_t *imu)
{
    axis->angle_feedback_deg = imu->euler_deg[axis->imu_angle_index];
    axis->speed_feedback_dps = imu->gyr_dps[axis->imu_rate_index];
}

/* 根据遥控输入更新单个轴的目标角。 */
static void Control_UpdateAxisTargetByRemote(gimbal_axis_control_t *axis,
                                             const remote_state_t *remote,
                                             float scale_ratio)
{
    float delta_angle_deg;
    int16_t input;

    if ((remote == NULL) || (remote->online == 0U))
    {
        return;
    }

    input = Control_ApplyRemoteDeadband(remote->channel[axis->remote_channel_index]);
    delta_angle_deg = (float)input
                    * axis->remote_scale_deg_per_s
                    * scale_ratio
                    * CONTROL_TASK_DT_S;
    axis->target_angle_deg += delta_angle_deg;
    Control_ClampAxisTarget(axis);
}

/*
 * 根据当前反馈角检查机械限位保护。
 * 这里只对非回环轴生效，也就是当前的 pitch 轴。
 *
 * 触发条件：
 * 1. 当前反馈角已经超过配置的最大角度 + 保护裕量
 * 2. 当前反馈角已经低于配置的最小角度 - 保护裕量
 *
 * 触发动作：
 * 1. 立刻把目标角钳回到允许范围边界
 * 2. 拉起 limit_active 标志，方便上层调试时观察
 *
 * 这样做的目的不是“撞到限位后继续顶住”，而是发现越界趋势后，
 * 直接把目标值拉回安全区，避免控制器继续往错误方向积分。
 */
static void Control_ApplyAxisLimitProtection(gimbal_axis_control_t *axis)
{
    axis->limit_active = 0U;

    if (axis->wrap_enable != 0U)
    {
        return;
    }

    if (axis->angle_feedback_deg > (axis->max_angle_deg + axis->protect_margin_deg))
    {
        axis->target_angle_deg = axis->max_angle_deg;
        axis->limit_active = 1U;
    }
    else if (axis->angle_feedback_deg < (axis->min_angle_deg - axis->protect_margin_deg))
    {
        axis->target_angle_deg = axis->min_angle_deg;
        axis->limit_active = 1U;
    }

    Control_ClampAxisTarget(axis);
}

/* 执行单个轴的双环 PID。 */
static void Control_RunAxisDualLoop(gimbal_axis_control_t *axis,
                                    float speed_limit_dps,
                                    float current_limit)
{
    float angle_error;

    angle_error = Control_CalcAngleError(axis->target_angle_deg,
                                         axis->angle_feedback_deg,
                                         axis->wrap_enable);

    axis->target_speed_dps = PID_CalculateByError(&axis->angle_pid, angle_error);
    axis->target_speed_dps = Control_LimitSymmetric(axis->target_speed_dps, speed_limit_dps);
    axis->target_current = PID_Calculate(&axis->speed_pid,
                                         axis->target_speed_dps,
                                         axis->speed_feedback_dps);
    axis->target_current = Control_LimitSymmetric(axis->target_current, current_limit);

    Motor_SetGm6020CurrentLoopOutput(&axis->motor, (int16_t)axis->target_current);
}

/* 只清 PID 内部状态，不改当前目标角。 */
static void Control_ClearAxisController(gimbal_axis_control_t *axis)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);
    axis->target_speed_dps = 0.0f;
    axis->target_current = 0.0f;
}

/* 清空单轴控制器状态，并把输出电流拉回 0。 */
static void Control_ResetAxisState(gimbal_axis_control_t *axis)
{
    Control_ClearAxisController(axis);
    Motor_SetGm6020CurrentLoopOutput(&axis->motor, 0);
}

/* 把两轴目标角都对齐到当前反馈角。 */
static void Control_AlignAllTargetsToFeedback(void)
{
    Control_AlignAxisTargetToFeedback(&gimbal_control.pitch);
    Control_AlignAxisTargetToFeedback(&gimbal_control.yaw);
}

/* 把两轴目标角都设置成回中角。 */
static void Control_SetAllTargetsToCenter(void)
{
    Control_SetAxisTargetToCenter(&gimbal_control.pitch);
    Control_SetAxisTargetToCenter(&gimbal_control.yaw);
}

/*
 * 模式切换时统一处理目标角和 PID 状态。
 *
 * 核心原则：
 * 1. 切模式时尽量避免目标角突变
 * 2. 切模式时把 PID 的积分和微分状态清掉
 * 3. 无力模式下直接清输出
 *
 * 这里单独集中处理，主要是为了避免各个模式分支自己改一套，
 * 最后出现“某个模式切进去会抽一下、另一个模式切回来会跳一下”的问题。
 */
static void Control_HandleModeTransition(void)
{
    if ((gimbal_control.initialized != 0U) && (gimbal_control.mode == gimbal_control.last_mode))
    {
        return;
    }

    switch (gimbal_control.mode)
    {
        case CONTROL_MODE_RELAX:
            Control_ResetAxisState(&gimbal_control.pitch);
            Control_ResetAxisState(&gimbal_control.yaw);
            break;

        case CONTROL_MODE_HOLD:
            Control_AlignAllTargetsToFeedback();
            Control_ClearAxisController(&gimbal_control.pitch);
            Control_ClearAxisController(&gimbal_control.yaw);
            break;

        case CONTROL_MODE_MANUAL:
            Control_AlignAllTargetsToFeedback();
            Control_ClearAxisController(&gimbal_control.pitch);
            Control_ClearAxisController(&gimbal_control.yaw);
            break;

        case CONTROL_MODE_RECENTER:
            Control_SetAllTargetsToCenter();
            Control_ClearAxisController(&gimbal_control.pitch);
            Control_ClearAxisController(&gimbal_control.yaw);
            break;

        case CONTROL_MODE_DEBUG:
            Control_AlignAllTargetsToFeedback();
            Control_ClearAxisController(&gimbal_control.pitch);
            Control_ClearAxisController(&gimbal_control.yaw);
            break;

        default:
            Control_ResetAxisState(&gimbal_control.pitch);
            Control_ResetAxisState(&gimbal_control.yaw);
            gimbal_control.mode = CONTROL_MODE_RELAX;
            break;
    }

    gimbal_control.initialized = 1U;
}

/*
 * 根据遥控器挡位和暂停键决定当前应该进入什么模式。
 *
 * 当前优先级从高到低是：
 * 1. pause -> RELAX
 * 2. 右侧自定义键 -> DEBUG
 * 3. 挡位映射
 * 4. 兜底回 RELAX
 *
 * 这样设计以后，急停和调试入口不会被普通挡位覆盖。
 */
static control_mode_t Control_DecodeRemoteMode(const remote_state_t *remote)
{
    if ((remote == NULL) || (remote->online == 0U))
    {
        return CONTROL_MODE_HOLD;
    }

    if (remote->pause_pressed != 0U)
    {
        return CONTROL_MODE_RELAX;
    }

    if ((CONTROL_DEBUG_ENABLE_BY_CUSTOM_RIGHT != 0U) && (remote->custom_right_pressed != 0U))
    {
        return CONTROL_MODE_DEBUG;
    }

    if (remote->gear == CONTROL_MODE_GEAR_MANUAL)
    {
        return CONTROL_MODE_MANUAL;
    }

    if (remote->gear == CONTROL_MODE_GEAR_RECENTER)
    {
        return CONTROL_MODE_RECENTER;
    }

    if (remote->gear == CONTROL_MODE_GEAR_RELAX)
    {
        return CONTROL_MODE_RELAX;
    }

    return CONTROL_MODE_RELAX;
}

/*
 * 刷新系统在线状态，并结合遥控状态决定当前模式。
 *
 * 保护优先级比遥控挡位更高：
 * 1. IMU 掉线 -> 强制 RELAX
 * 2. 任一云台电机掉线 -> 强制 RELAX
 * 3. 只有关键反馈都在线时，才允许按遥控挡位选模式
 *
 * 这样可以保证传感器或执行器掉线时，云台不会继续带着旧反馈盲跑。
 */
static void Control_UpdateSystemStatus(const imu_hi91_t *imu, const remote_state_t *remote)
{
    gimbal_control.last_mode = gimbal_control.mode;
    gimbal_control.imu_online = (uint8_t)((imu != NULL) ? imu->online : 0U);
    gimbal_control.remote_online = (uint8_t)((remote != NULL) ? remote->online : 0U);
    gimbal_control.pitch_motor_online = Motor_Gm6020IsOnline(&gimbal_control.pitch.motor);
    gimbal_control.yaw_motor_online = Motor_Gm6020IsOnline(&gimbal_control.yaw.motor);

    if ((gimbal_control.imu_online == 0U)
     || (gimbal_control.pitch_motor_online == 0U)
     || (gimbal_control.yaw_motor_online == 0U))
    {
        gimbal_control.mode = CONTROL_MODE_RELAX;
        return;
    }

    gimbal_control.mode = Control_DecodeRemoteMode(remote);
}

/*
 * 无力模式下把两轴输出都拉回 0。
 *
 * 注意这里不仅是“发送 0 电流”这么简单，
 * 还会把 PID 内部状态一起清掉，避免后面重新上电流时把旧积分一下子打出来。
 */
static void Control_StopGimbalOutput(void)
{
    Control_ResetAxisState(&gimbal_control.yaw);
    Control_ResetAxisState(&gimbal_control.pitch);
}

/* 发送两轴当前已经缓存好的电流指令。 */
static void Control_SendGimbalOutput(void)
{
    (void)Motor_SendGm6020CurrentLoopFrame(&gimbal_control.pitch.motor, NULL, NULL, NULL);
    (void)Motor_SendGm6020CurrentLoopFrame(&gimbal_control.yaw.motor, NULL, NULL, NULL);
}

/* 配置 Pitch 轴的电机挂载、输入映射、限位和 PID 参数。 */
static void Control_ConfigPitchAxis(void)
{
    gimbal_axis_control_t *axis;

    axis = &gimbal_control.pitch;

    if (Motor_RegisterGm6020CurrentLoop(&axis->motor,
                                        CONTROL_PITCH_MOTOR_CAN,
                                        CONTROL_PITCH_MOTOR_ID) != HAL_OK)
    {
        Error_Handler();
    }

    PID_Init(&axis->angle_pid,
             CONTROL_PITCH_ANGLE_KP,
             CONTROL_PITCH_ANGLE_KI,
             CONTROL_PITCH_ANGLE_KD,
             CONTROL_TASK_DT_S,
             CONTROL_PITCH_ANGLE_I_LIMIT,
             CONTROL_PITCH_ANGLE_OUT_LIMIT);

    PID_Init(&axis->speed_pid,
             CONTROL_PITCH_SPEED_KP,
             CONTROL_PITCH_SPEED_KI,
             CONTROL_PITCH_SPEED_KD,
             CONTROL_TASK_DT_S,
             CONTROL_PITCH_SPEED_I_LIMIT,
             CONTROL_PITCH_SPEED_OUT_LIMIT);

    axis->remote_scale_deg_per_s = CONTROL_PITCH_REMOTE_SCALE_DEG_PER_S;
    axis->debug_remote_scale_ratio = CONTROL_DEBUG_PITCH_REMOTE_SCALE_RATIO;
    axis->debug_speed_limit_dps = CONTROL_DEBUG_PITCH_SPEED_LIMIT_DPS;
    axis->debug_current_limit = CONTROL_DEBUG_PITCH_CURRENT_LIMIT;
    axis->min_angle_deg = CONTROL_PITCH_MIN_DEG;
    axis->max_angle_deg = CONTROL_PITCH_MAX_DEG;
    axis->center_angle_deg = CONTROL_PITCH_CENTER_DEG;
    axis->protect_margin_deg = CONTROL_PITCH_PROTECT_MARGIN_DEG;
    axis->wrap_enable = CONTROL_PITCH_WRAP_ENABLE;
    axis->imu_angle_index = CONTROL_IMU_PITCH_ANGLE_INDEX;
    axis->imu_rate_index = CONTROL_IMU_PITCH_RATE_INDEX;
    axis->remote_channel_index = CONTROL_REMOTE_PITCH_CHANNEL_INDEX;
}

/* 配置 Yaw 轴的电机挂载、输入映射、限位和 PID 参数。 */
static void Control_ConfigYawAxis(void)
{
    gimbal_axis_control_t *axis;

    axis = &gimbal_control.yaw;

    if (Motor_RegisterGm6020CurrentLoop(&axis->motor,
                                        CONTROL_YAW_MOTOR_CAN,
                                        CONTROL_YAW_MOTOR_ID) != HAL_OK)
    {
        Error_Handler();
    }

    PID_Init(&axis->angle_pid,
             CONTROL_YAW_ANGLE_KP,
             CONTROL_YAW_ANGLE_KI,
             CONTROL_YAW_ANGLE_KD,
             CONTROL_TASK_DT_S,
             CONTROL_YAW_ANGLE_I_LIMIT,
             CONTROL_YAW_ANGLE_OUT_LIMIT);

    PID_Init(&axis->speed_pid,
             CONTROL_YAW_SPEED_KP,
             CONTROL_YAW_SPEED_KI,
             CONTROL_YAW_SPEED_KD,
             CONTROL_TASK_DT_S,
             CONTROL_YAW_SPEED_I_LIMIT,
             CONTROL_YAW_SPEED_OUT_LIMIT);

    axis->remote_scale_deg_per_s = CONTROL_YAW_REMOTE_SCALE_DEG_PER_S;
    axis->debug_remote_scale_ratio = CONTROL_DEBUG_YAW_REMOTE_SCALE_RATIO;
    axis->debug_speed_limit_dps = CONTROL_DEBUG_YAW_SPEED_LIMIT_DPS;
    axis->debug_current_limit = CONTROL_DEBUG_YAW_CURRENT_LIMIT;
    axis->min_angle_deg = CONTROL_YAW_MIN_DEG;
    axis->max_angle_deg = CONTROL_YAW_MAX_DEG;
    axis->center_angle_deg = CONTROL_YAW_CENTER_DEG;
    axis->protect_margin_deg = CONTROL_YAW_PROTECT_MARGIN_DEG;
    axis->wrap_enable = CONTROL_YAW_WRAP_ENABLE;
    axis->imu_angle_index = CONTROL_IMU_YAW_ANGLE_INDEX;
    axis->imu_rate_index = CONTROL_IMU_YAW_RATE_INDEX;
    axis->remote_channel_index = CONTROL_REMOTE_YAW_CHANNEL_INDEX;
}

void Control_Task_Init(void)
{
    memset(&gimbal_control, 0, sizeof(gimbal_control));
    gimbal_control.mode = CONTROL_MODE_RELAX;
    gimbal_control.last_mode = CONTROL_MODE_RELAX;

    /* 当前工程约定：Pitch 用 CAN1 上的 ID1 GM6020 电流环。 */
    Control_ConfigPitchAxis();
    /* 当前工程约定：Yaw 用 CAN2 上的 ID1 GM6020 电流环。 */
    Control_ConfigYawAxis();
}

void Control_Task_Run(void)
{
    const imu_hi91_t *imu;
    const remote_state_t *remote;

    if (gimbal_control.tick_pending == 0U)
    {
        return;
    }

    __disable_irq();
    gimbal_control.tick_pending--;
    __enable_irq();

    imu = IMU_GetState();
    remote = Remote_GetState();

    Control_UpdateSystemStatus(imu, remote);

    /* IMU 在线时才刷新姿态反馈，防止用掉线数据继续控制。 */
    if (gimbal_control.imu_online != 0U)
    {
        Control_UpdateAxisFeedback(&gimbal_control.pitch, imu);
        Control_UpdateAxisFeedback(&gimbal_control.yaw, imu);
    }

    Control_HandleModeTransition();

    
    if (gimbal_control.mode == CONTROL_MODE_RELAX)
    {
        Control_StopGimbalOutput();
        Control_SendGimbalOutput();
        return;
    }

    /* 回中模式不接收遥控量，只把目标角往中心拉。 */
    if (gimbal_control.mode == CONTROL_MODE_RECENTER)
    {
        Control_SetAllTargetsToCenter();
    }
    else if (gimbal_control.mode == CONTROL_MODE_DEBUG)
    {
        if ((CONTROL_DEBUG_RECENTER_BY_CUSTOM_LEFT != 0U)
         && (remote != NULL)
         && (remote->online != 0U)
         && (remote->custom_left_pressed != 0U))
        {
            Control_SetAllTargetsToCenter();
        }
        else
        {
            Control_UpdateAxisTargetByRemote(&gimbal_control.pitch,
                                             remote,
                                             gimbal_control.pitch.debug_remote_scale_ratio);
            Control_UpdateAxisTargetByRemote(&gimbal_control.yaw,
                                             remote,
                                             gimbal_control.yaw.debug_remote_scale_ratio);
        }
    }
    else if (gimbal_control.mode == CONTROL_MODE_MANUAL)
    {
        Control_UpdateAxisTargetByRemote(&gimbal_control.pitch, remote, 1.0f);
        Control_UpdateAxisTargetByRemote(&gimbal_control.yaw, remote, 1.0f);
    }
    else
    {
        /* HOLD 模式保持现有目标角，不再积分新的遥控量。 */
    }

    /* 双环计算前先做一次目标约束和机械限位保护。 */
    Control_ApplyAxisLimitProtection(&gimbal_control.pitch);
    Control_ApplyAxisLimitProtection(&gimbal_control.yaw);

    if (gimbal_control.mode == CONTROL_MODE_DEBUG)
    {
        Control_RunAxisDualLoop(&gimbal_control.pitch,
                                gimbal_control.pitch.debug_speed_limit_dps,
                                gimbal_control.pitch.debug_current_limit);
        Control_RunAxisDualLoop(&gimbal_control.yaw,
                                gimbal_control.yaw.debug_speed_limit_dps,
                                gimbal_control.yaw.debug_current_limit);
    }
    else
    {
        Control_RunAxisDualLoop(&gimbal_control.pitch, 0.0f, 0.0f);
        Control_RunAxisDualLoop(&gimbal_control.yaw, 0.0f, 0.0f);
    }

    Control_SendGimbalOutput();
}

void Control_Task_Timer1kHzCallback(void)
{
    gimbal_control.tick_pending++;
}


