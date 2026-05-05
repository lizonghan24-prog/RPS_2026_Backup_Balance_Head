#include "Control_Task.h"
#include "Up_Task.h"

/*
 * 云台控制函数库。
 *
 * 的控制基础能力：目标限幅、反馈更新、PID 双环、
 * 电机注册和 CAN 输出辅助函数。真正的每拍控制顺序由 Up_Task.c 负责；
 * 如果逻辑有冲突，以 Mode_Switch_Task.c 和 Up_Task.c 为准。
 */

#include "PID.h"
#include "IMU.h"
#include "Remote.h"
#include "motor.h"
#include "motor_dm.h"

#include <math.h>
#include <string.h>

/* 云台控制共享状态，由 Up_Task 通过公开辅助函数驱动。 */
gimbal_control_task_t gimbal_control;

static void Control_StopGimbalOutput(void);
static void Control_SendGimbalOutput(void);

/* 将数值限制在给定范围内。 */
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

/* 将角度归一化到 [-180, 180]。 */
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

/* 计算角度误差；回环轴使用最短角度路径。 */
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

/* 根据当前反馈，计算离反馈最近的 360 度等效目标。 */
static float Control_CalcNearestWrappedReference(float target_angle_deg, float feedback_angle_deg)
{
    int32_t rotate_num;
    float reference;
    float error;

    /*
     * 对齐原 gimbal_init_handle() 的思路：
     * yaw 初始化时选择最近的等效目标，让电机走劣弧，而不是多转一整圈。
     */
    rotate_num = (int32_t)(feedback_angle_deg / 360.0f);
    reference = ((float)rotate_num * 360.0f) + target_angle_deg;
    error = reference - feedback_angle_deg;

    if (error >= 181.0f)
    {
        reference -= 360.0f;
    }
    else if (error < -179.0f)
    {
        reference += 360.0f;
    }

    return reference;
}

/* 将目标限制在机械范围内，或对回环轴做角度归一化。 */
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

/* 将目标角对齐到最新反馈角。 */
static void Control_AlignAxisTargetToFeedback(gimbal_axis_control_t *axis)
{
    axis->target_angle_deg = axis->angle_feedback_deg;
    Control_ClampAxisTarget(axis);
}

/* 遥控通道输入死区处理。 */
static int16_t Control_ApplyRemoteDeadband(int16_t input)
{
    if ((input < CONTROL_REMOTE_DEADBAND) && (input > -CONTROL_REMOTE_DEADBAND))
    {
        return 0;
    }

    return input;
}

/* 鼠标输入死区处理，用来抑制微小抖动。 */
static int16_t Control_ApplyMouseDeadband(int16_t input)
{
    if ((input < CONTROL_MOUSE_DEADBAND) && (input > -CONTROL_MOUSE_DEADBAND))
    {
        return 0;
    }

    return input;
}

/* 把选定的 IMU 反馈通道写入单轴控制状态。 */
static void Control_UpdateAxisFeedback(gimbal_axis_control_t *axis, const imu_hi91_t *imu)
{
    axis->angle_feedback_deg = imu->euler_deg[axis->imu_angle_index];
    axis->speed_feedback_dps = imu->gyr_dps[axis->imu_rate_index];
}

/* 将遥控摇杆输入积分到单轴目标角。 */
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

/* 将鼠标输入积分到单轴目标角。 */
static void Control_UpdateAxisTargetByMouse(gimbal_axis_control_t *axis, int16_t mouse_input)
{
    float delta_angle_deg;
    int16_t input;

    input = Control_ApplyMouseDeadband(mouse_input);
    delta_angle_deg = (float)input
                    * axis->mouse_scale_deg_per_s
                    * CONTROL_TASK_DT_S;
    axis->target_angle_deg += delta_angle_deg;
    Control_ClampAxisTarget(axis);
}

/*
 * 对非回环轴做机械限位保护。
 *
 * pitch 作为有机械限位的轴处理。当反馈超出配置范围和保护裕量后，
 * 目标会被拉回到最近的合法限位，同时置位 limit_active 便于调试观察。
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

/*
 * 按当前轴绑定的电机协议查询在线状态。
 * 当前 pitch 使用 GM6020，yaw 使用 DM4310。
 */
static uint8_t Control_IsAxisMotorOnline(const gimbal_axis_control_t *axis)
{
    if (axis == NULL)
    {
        return 0U;
    }

    if (axis->motor_type == CONTROL_AXIS_MOTOR_DM4310)
    {
        return Motor_Dm4310IsOnline(&axis->dm4310_motor);
    }

    return Motor_Gm6020IsOnline(&axis->gm6020_motor);
}

/*
 * 将控制器输出写入对应轴的电机服务对象。
 * output_sign 用于在底层发送前补偿电机安装方向。
 */
static void Control_SetAxisMotorOutput(gimbal_axis_control_t *axis, float output)
{
    float signed_output;

    if (axis == NULL)
    {
        return;
    }

    signed_output = output * axis->output_sign;
    if (axis->motor_type == CONTROL_AXIS_MOTOR_DM4310)
    {
        Motor_SetDm4310MitTorque(&axis->dm4310_motor, signed_output);
    }
    else
    {
        Motor_SetGm6020CurrentLoopOutput(&axis->gm6020_motor, (int16_t)signed_output);
    }
}

/*
 * 发送已经准备好的电机控制帧。
 * GM6020 使用 DJI 电流环帧，DM4310 使用 MIT 控制帧。
 */
static void Control_SendAxisMotorOutput(gimbal_axis_control_t *axis)
{
    if (axis == NULL)
    {
        return;
    }

    if (axis->motor_type == CONTROL_AXIS_MOTOR_DM4310)
    {
        (void)Motor_Dm4310SendMitControl(&axis->dm4310_motor);
    }
    else
    {
        (void)Motor_SendGm6020CurrentLoopFrame(&axis->gm6020_motor, NULL, NULL, NULL);
    }
}

/* 对单轴运行一次角度外环 + 速度内环 PID。 */
static void Control_RunAxisDualLoop(gimbal_axis_control_t *axis)
{
    float angle_error;

    angle_error = Control_CalcAngleError(axis->target_angle_deg,
                                         axis->angle_feedback_deg,
                                         axis->wrap_enable);

    axis->target_speed_dps = PID_CalculateByError(&axis->angle_pid, angle_error);
    axis->target_output = PID_Calculate(&axis->speed_pid,
                                        axis->target_speed_dps,
                                        axis->speed_feedback_dps);

    Control_SetAxisMotorOutput(axis, axis->target_output);
}

/* 清除单轴 PID 历史和缓存输出。 */
static void Control_ClearAxisController(gimbal_axis_control_t *axis)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);
    axis->target_speed_dps = 0.0f;
    axis->target_output = 0.0f;
}

/* 清除单轴控制状态，并将该轴输出置零。 */
static void Control_ResetAxisState(gimbal_axis_control_t *axis)
{
    Control_ClearAxisController(axis);
    Control_SetAxisMotorOutput(axis, 0.0f);
}

/* 将全部轴目标对齐到当前反馈。 */
static void Control_AlignAllTargetsToFeedback(void)
{
    Control_AlignAxisTargetToFeedback(&gimbal_control.pitch);
    Control_AlignAxisTargetToFeedback(&gimbal_control.yaw);
}

/*
 * 处理模式变化时只需要执行一次的动作。
 *
 * RELAX 会清控制器和输出；主动控制模式会先把目标对齐到最新反馈，
 * 再清 PID 历史，避免切模式瞬间冲击。
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
            gimbal_control.init_finished = 0U;
            break;

        case CONTROL_MODE_INIT:
            Control_ClearAxisController(&gimbal_control.pitch);
            Control_ClearAxisController(&gimbal_control.yaw);
            gimbal_control.init_finished = 0U;
            break;

        case CONTROL_MODE_FOLLOW_ZGYRO:
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

/* 刷新 Mode_Switch_Task 需要使用的在线状态标志。 */
void Control_Task_UpdateOnlineFlags(const imu_hi91_t *imu, const remote_state_t *remote)
{
    gimbal_control.imu_online = (uint8_t)((imu != NULL) ? imu->online : 0U);
    gimbal_control.remote_online = (uint8_t)((remote != NULL) ? remote->online : 0U);
    gimbal_control.pitch_motor_online = Control_IsAxisMotorOnline(&gimbal_control.pitch);
    gimbal_control.yaw_motor_online = Control_IsAxisMotorOnline(&gimbal_control.yaw);
}

/* 保存 Mode_Switch_Task 在这一拍选出的控制模式。 */
void Control_Task_SetMode(control_mode_t mode)
{
    gimbal_control.last_mode = gimbal_control.mode;
    gimbal_control.mode = mode;
}

void Control_Task_UpdateFeedback(const imu_hi91_t *imu)
{
    if (imu == NULL)
    {
        return;
    }

    Control_UpdateAxisFeedback(&gimbal_control.pitch, imu);
    Control_UpdateAxisFeedback(&gimbal_control.yaw, imu);
}

void Control_Task_UpdateInitTargets(void)
{
    float pitch_error;
    float yaw_error;

    /*
     * 让 pitch 回到 0，让 yaw 回到最近的 0 度等效位置。
     * Up_Task 在调用本函数前已经刷新过反馈。
     */
    gimbal_control.pitch.target_angle_deg = CONTROL_INIT_PITCH_TARGET_DEG;
    gimbal_control.yaw.target_angle_deg = Control_CalcNearestWrappedReference(CONTROL_INIT_YAW_TARGET_DEG,
                                                                              gimbal_control.yaw.angle_feedback_deg);

    Control_ClampAxisTarget(&gimbal_control.pitch);
    Control_ClampAxisTarget(&gimbal_control.yaw);

    pitch_error = Control_CalcAngleError(gimbal_control.pitch.target_angle_deg,
                                         gimbal_control.pitch.angle_feedback_deg,
                                         gimbal_control.pitch.wrap_enable);
    yaw_error = Control_CalcAngleError(gimbal_control.yaw.target_angle_deg,
                                       gimbal_control.yaw.angle_feedback_deg,
                                       gimbal_control.yaw.wrap_enable);

    if ((fabsf(pitch_error) <= CONTROL_INIT_PITCH_FINISH_ERROR_DEG)
     && (fabsf(yaw_error) <= CONTROL_INIT_YAW_FINISH_ERROR_DEG))
    {
        /* 这个标志对应原工程里的 if_finish_Init。 */
        gimbal_control.init_finished = 1U;
    }
}

uint8_t Control_Task_IsInitFinished(void)
{
    return gimbal_control.init_finished;
}

void Control_Task_ClearInitFinished(void)
{
    gimbal_control.init_finished = 0U;
}

void Control_Task_UpdateRemoteTargets(const remote_state_t *remote, float scale_ratio)
{
    Control_UpdateAxisTargetByRemote(&gimbal_control.pitch, remote, scale_ratio);
    Control_UpdateAxisTargetByRemote(&gimbal_control.yaw, remote, scale_ratio);
}

void Control_Task_UpdateMouseTargets(int16_t mouse_x, int16_t mouse_y)
{
    Control_UpdateAxisTargetByMouse(&gimbal_control.pitch, mouse_y);
    Control_UpdateAxisTargetByMouse(&gimbal_control.yaw, mouse_x);
}

void Control_Task_HandleModeTransition(void)
{
    Control_HandleModeTransition();
}

void Control_Task_StopOutput(void)
{
    Control_StopGimbalOutput();
}

void Control_Task_SendOutput(void)
{
    Control_SendGimbalOutput();
}

void Control_Task_RunClosedLoop(void)
{
    Control_ApplyAxisLimitProtection(&gimbal_control.pitch);
    Control_ApplyAxisLimitProtection(&gimbal_control.yaw);

    Control_RunAxisDualLoop(&gimbal_control.pitch);
    Control_RunAxisDualLoop(&gimbal_control.yaw);
}

/*
 * 强制两轴输出为 0，并清除 PID 状态。
 * Up_Task 在 RELAX 模式下调用。
 */
static void Control_StopGimbalOutput(void)
{
    Control_ResetAxisState(&gimbal_control.yaw);
    Control_ResetAxisState(&gimbal_control.pitch);
}

/* 发送两轴最新准备好的控制命令。 */
static void Control_SendGimbalOutput(void)
{
    Control_SendAxisMotorOutput(&gimbal_control.pitch);
    Control_SendAxisMotorOutput(&gimbal_control.yaw);
}

/* 配置 pitch 轴电机绑定、PID 参数和限位。 */
static void Control_ConfigPitchAxis(void)
{
    gimbal_axis_control_t *axis;

    axis = &gimbal_control.pitch;
    axis->motor_type = CONTROL_AXIS_MOTOR_GM6020;
    axis->output_sign = CONTROL_PITCH_OUTPUT_SIGN;

    if (Motor_RegisterGm6020CurrentLoop(&axis->gm6020_motor,
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
    axis->mouse_scale_deg_per_s = CONTROL_MOUSE_PITCH_SCALE_DEG_PER_S;
    axis->min_angle_deg = CONTROL_PITCH_MIN_DEG;
    axis->max_angle_deg = CONTROL_PITCH_MAX_DEG;
    axis->protect_margin_deg = CONTROL_PITCH_PROTECT_MARGIN_DEG;
    axis->wrap_enable = CONTROL_PITCH_WRAP_ENABLE;
    axis->imu_angle_index = CONTROL_IMU_PITCH_ANGLE_INDEX;
    axis->imu_rate_index = CONTROL_IMU_PITCH_RATE_INDEX;
    axis->remote_channel_index = CONTROL_REMOTE_PITCH_CHANNEL_INDEX;
}

/* 配置 yaw 轴电机绑定、PID 参数和回环行为。 */
static void Control_ConfigYawAxis(void)
{
    gimbal_axis_control_t *axis;

    axis = &gimbal_control.yaw;
    axis->motor_type = CONTROL_AXIS_MOTOR_DM4310;
    axis->output_sign = CONTROL_YAW_OUTPUT_SIGN;

    if (Motor_RegisterDm4310Mit(&axis->dm4310_motor,
                                CONTROL_YAW_MOTOR_CAN,
                                CONTROL_YAW_MOTOR_ID,
                                CONTROL_YAW_DM4310_FEEDBACK_STD_ID) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 当前安装下 yaw DM4310 的反馈极性与 IMU 坐标系相反。
     * 这里反向电机反馈，使控制环继续以 IMU yaw / gyro 为参考。
     */
    axis->dm4310_motor.feedback_angle_sign = -1.0f;
    axis->dm4310_motor.feedback_speed_sign = -1.0f;
    (void)Motor_Dm4310SendEnable(&axis->dm4310_motor);

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
    axis->mouse_scale_deg_per_s = CONTROL_MOUSE_YAW_SCALE_DEG_PER_S;
    axis->min_angle_deg = CONTROL_YAW_MIN_DEG;
    axis->max_angle_deg = CONTROL_YAW_MAX_DEG;
    axis->protect_margin_deg = CONTROL_YAW_PROTECT_MARGIN_DEG;
    axis->wrap_enable = CONTROL_YAW_WRAP_ENABLE;
    axis->imu_angle_index = CONTROL_IMU_YAW_ANGLE_INDEX;
    axis->imu_rate_index = CONTROL_IMU_YAW_RATE_INDEX;
    axis->remote_channel_index = CONTROL_REMOTE_YAW_CHANNEL_INDEX;
}

/*
 * 初始化可复用控制状态、电机服务和 PID 控制器。
 * 需要在 BSP 和底层电机支持就绪后调用一次。
 */
void Control_Task_Init(void)
{
    memset(&gimbal_control, 0, sizeof(gimbal_control));
    gimbal_control.mode = CONTROL_MODE_RELAX;
    gimbal_control.last_mode = CONTROL_MODE_RELAX;

    Control_ConfigPitchAxis();
    Control_ConfigYawAxis();
}

/*
 * 
 * 运行时顺序刻意交给 Up_Task_Run()，确保 Up_Task.c 仍然是真正控制逻辑入口。
 */
void Control_Task_Run(void)
{
    Up_Task_Run();
}

const gimbal_control_task_t *Control_Task_GetState(void)
{
    return &gimbal_control;
}
