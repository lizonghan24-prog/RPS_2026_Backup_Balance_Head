#include "Control_Task.h"

/*
 * @file Control_Task.c
 * @brief 云台双轴控制任务。
 *
 * 本文件负责 Pitch / Yaw 两个 GM6020 云台电机的上层闭环：
 * 1. 从 IMU 读取姿态角和角速度反馈；
 * 2. 从图传遥控读取挡位、摇杆和自定义按键；
 * 3. 根据在线状态和遥控输入决定无力、保持、手动、回中、调试模式；
 * 4. 执行“角度外环 + 角速度内环”的双环 PID；
 * 5. 把计算出的电流给定缓存到 motor 层，再由 motor 层按 DJI 协议发 CAN。
 *
 * 注意：
 * - CAN 外设初始化、滤波器和中断接收不在本文件处理；
 * - GM6020 反馈解析和发送打包由 BSP/motor.c 负责；
 * - 本文件只关心任务逻辑和控制量计算。
 */

#include "PID.h"
#include "IMU.h"
#include "Remote.h"
#include "motor.h"

#include <string.h>

/* 云台任务的唯一全局上下文，所有运行时状态都集中放在这里。 */
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

/*
 * 初始化云台控制任务。
 *
 * 调用时机：
 * - main() 完成 HAL 外设初始化和 BSP_Init() 之后调用一次。
 *
 * 初始化内容：
 * - 清空任务上下文；
 * - 注册 Pitch / Yaw 两个 GM6020 到 motor 层；
 * - 初始化两轴角度环和速度环 PID 参数；
 * - 默认进入 RELAX，避免上电瞬间直接输出电流。
 */
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

/*
 * 云台控制任务主入口。
 *
 * 每次调用按下面顺序执行：
 * 1. 获取 IMU / 遥控状态；
 * 2. 刷新在线标志并决定当前模式；
 * 3. 在线时刷新姿态反馈；
 * 4. 处理模式切换带来的目标角同步和 PID 清零；
 * 5. 根据当前模式更新目标角；
 * 6. 执行限位保护和双环 PID；
 * 7. 下发 GM6020 电流控制帧。
 */
void Control_Task_Run(void)
{
    const imu_hi91_t *imu;
    const remote_state_t *remote;

    /* 读取各模块维护的最新状态指针；底层模块负责刷新这些数据。 */
    imu = IMU_GetState();
    remote = Remote_GetState();

    /* 在线状态决定是否允许闭环，安全保护优先于遥控挡位。 */
    Control_UpdateSystemStatus(imu, remote);

    /* IMU 在线时才刷新姿态反馈，防止用掉线数据继续控制。 */
    if (gimbal_control.imu_online != 0U)
    {
        Control_UpdateAxisFeedback(&gimbal_control.pitch, imu);
        Control_UpdateAxisFeedback(&gimbal_control.yaw, imu);
    }

    /*
     * 模式切换只在刚进入某个模式时执行一次，用来同步目标角和清 PID。
     * 如果模式没有变化，则不会反复清控制器状态。
     */
    Control_HandleModeTransition();

    /* RELAX 是最高优先级停机状态：清 PID、给 0 电流并立即返回。 */
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
        /*
         * 调试模式下：
         * - 左自定义键可临时触发回中；
         * - 否则继续响应遥控，但使用更小的比例和更低的输出限幅。
         */
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
        /* 调试模式额外限制目标角速度和目标电流，方便低风险试车。 */
        Control_RunAxisDualLoop(&gimbal_control.pitch,
                                gimbal_control.pitch.debug_speed_limit_dps,
                                gimbal_control.pitch.debug_current_limit);
        Control_RunAxisDualLoop(&gimbal_control.yaw,
                                gimbal_control.yaw.debug_speed_limit_dps,
                                gimbal_control.yaw.debug_current_limit);
    }
    else
    {
        /* 正常模式使用 PID 初始化时配置的输出限幅，不再叠加调试限幅。 */
        Control_RunAxisDualLoop(&gimbal_control.pitch, 0.0f, 0.0f);
        Control_RunAxisDualLoop(&gimbal_control.yaw, 0.0f, 0.0f);
    }

    /* 最后统一发 CAN，保证本次计算出的两轴输出都已经写入 motor 对象。 */
    Control_SendGimbalOutput();
}
