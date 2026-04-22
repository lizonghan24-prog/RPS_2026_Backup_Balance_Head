#include "Shoot_Task.h"

/*
 * @file Shoot_Task.c
 * @brief 发射机构任务。
 *
 * 本文件负责两部分：
 * 1. 摩擦轮：两路 M3508 速度闭环，负责起转、停转、速度斜坡和 ready 判定；
 * 2. 拨盘：LK 电机角度步进闭环，只有摩擦轮转稳后才允许进入闭环。
 *
 * 任务安全策略：
 * - 遥控 pause 或关键电机掉线时，摩擦轮和拨盘都回到停止状态；
 * - 摩擦轮未达到稳定状态时，拨盘不会执行推弹；
 * - 模式切换时清 PID 和历史步进请求，避免旧积分或旧目标造成突然动作。
 *
 * 电机反馈解析、在线检测和 CAN 协议打包由 BSP/motor.c 完成。
 */

#include "PID.h"
#include "Remote.h"
#include "motor.h"

#include <string.h>

/* 发射任务内部上下文，保存摩擦轮、拨盘和模式状态。 */
static shoot_task_t shoot_task_ctx;
/* 对外只暴露这份快照，避免其他模块直接改内部控制状态。 */
static shoot_task_state_t shoot_task_state_view;

/* 取浮点数绝对值。 */
static float Shoot_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/* 把数值限制在给定区间内。 */
static float Shoot_Limit(float value, float min_value, float max_value)
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

/* 让当前值按最大步长逼近目标值，常用于做斜坡。 */
static float Shoot_StepTowards(float current_value, float target_value, float max_step)
{
    float delta;

    delta = target_value - current_value;
    delta = Shoot_Limit(delta, -max_step, max_step);
    return current_value + delta;
}

/* 清摩擦轮控制器内部状态，但不改目标转速。 */
static void Shoot_ClearWheelController(shoot_friction_wheel_t *wheel)
{
    PID_Reset(&wheel->speed_pid);
    wheel->command_rpm = 0.0f;
    wheel->feedback_rpm = 0.0f;
    wheel->target_current = 0.0f;
}

/* 清摩擦轮控制器内部状态，并把输出电流拉回 0。 */
static void Shoot_ResetWheelOutput(shoot_friction_wheel_t *wheel)
{
    Shoot_ClearWheelController(wheel);
    Motor_SetM3508CurrentLoopOutput(&wheel->motor, 0);
}

/* 把两路摩擦轮输出全部拉回 0。 */
static void Shoot_StopFrictionOutput(void)
{
    Shoot_ResetWheelOutput(&shoot_task_ctx.left);
    Shoot_ResetWheelOutput(&shoot_task_ctx.right);
}

/* 发送当前缓存好的两路 M3508 电流指令。 */
static void Shoot_SendFrictionOutput(void)
{
    (void)Motor_SendM3508CurrentLoopFrame(&shoot_task_ctx.left.motor,
                                          &shoot_task_ctx.right.motor,
                                          NULL,
                                          NULL);
}

/* 刷新单个摩擦轮的转速反馈。 */
static void Shoot_UpdateWheelFeedback(shoot_friction_wheel_t *wheel)
{
    wheel->feedback_rpm = (float)wheel->motor.speed_rpm;
}

/* 执行单个摩擦轮的速度环控制。 */
static void Shoot_RunWheelSpeedLoop(shoot_friction_wheel_t *wheel)
{
    wheel->command_rpm = Shoot_StepTowards(wheel->command_rpm,
                                           wheel->target_rpm,
                                           SHOOT_FRICTION_RAMP_RPM_PER_S * SHOOT_TASK_DT_S);

    wheel->target_current = PID_Calculate(&wheel->speed_pid,
                                          wheel->command_rpm,
                                          wheel->feedback_rpm);

    Motor_SetM3508CurrentLoopOutput(&wheel->motor, (int16_t)wheel->target_current);
}

/* 判断单个摩擦轮是否已经达到允许拨盘工作的稳定状态。 */
static uint8_t Shoot_IsWheelReady(const shoot_friction_wheel_t *wheel)
{
    float target_abs_rpm;
    float feedback_abs_rpm;
    float error_abs_rpm;

    target_abs_rpm = Shoot_AbsFloat(wheel->target_rpm);
    feedback_abs_rpm = Shoot_AbsFloat(wheel->feedback_rpm);
    error_abs_rpm = Shoot_AbsFloat(wheel->target_rpm - wheel->feedback_rpm);

    if (target_abs_rpm < 1.0f)
    {
        return 0U;
    }

    if (feedback_abs_rpm < (target_abs_rpm * SHOOT_FRICTION_READY_RPM_RATIO))
    {
        return 0U;
    }

    if (error_abs_rpm > SHOOT_FRICTION_READY_ERROR_RPM)
    {
        return 0U;
    }

    return 1U;
}

/*
 * 根据当前状态刷新摩擦轮 ready 标志。
 *
 * 这个标志不是“摩擦轮已经启动”那么简单，而是“摩擦轮已经稳定到允许发射”。
 * 只有同时满足下面几点，ready 才会被拉高：
 * 1. 摩擦轮模式当前是 RUN
 * 2. 左右摩擦轮电机都在线
 * 3. 左右摩擦轮都连续一段时间达到设定稳定条件
 *
 * 这里故意做了持续计数，不是一拍满足就算 ready，
 * 目的是把刚启动时的过冲、抖动和短时速度波动都过滤掉。
 */
static void Shoot_UpdateFrictionReady(void)
{
    if ((shoot_task_ctx.friction_mode != SHOOT_FRICTION_MODE_RUN)
     || (shoot_task_ctx.left_motor_online == 0U)
     || (shoot_task_ctx.right_motor_online == 0U))
    {
        shoot_task_ctx.friction_ready = 0U;
        shoot_task_ctx.friction_ready_ticks = 0U;
        return;
    }

    if ((Shoot_IsWheelReady(&shoot_task_ctx.left) != 0U)
     && (Shoot_IsWheelReady(&shoot_task_ctx.right) != 0U))
    {
        if (shoot_task_ctx.friction_ready_ticks < SHOOT_FRICTION_READY_HOLD_TICKS)
        {
            shoot_task_ctx.friction_ready_ticks++;
        }
    }
    else
    {
        shoot_task_ctx.friction_ready_ticks = 0U;
    }

    shoot_task_ctx.friction_ready = (uint8_t)(shoot_task_ctx.friction_ready_ticks >= SHOOT_FRICTION_READY_HOLD_TICKS);
}

/* 清拨盘 PID 内部状态，但不改当前目标角。 */
static void Shoot_ClearDialController(shoot_dial_t *dial)
{
    PID_Reset(&dial->angle_pid);
    PID_Reset(&dial->speed_pid);
    dial->target_speed_dps = 0.0f;
    dial->target_iq = 0.0f;
}

/* 刷新拨盘反馈角度和角速度。 */
static void Shoot_UpdateDialFeedback(shoot_dial_t *dial)
{
    dial->feedback_angle_deg = dial->motor.total_angle_deg;
    dial->feedback_speed_dps = dial->motor.speed_dps;
}

/* 把拨盘目标角同步到当前反馈角，切闭环时不会猛跳。 */
static void Shoot_SyncDialTargetToFeedback(shoot_dial_t *dial)
{
    Shoot_UpdateDialFeedback(dial);
    dial->target_angle_deg = dial->feedback_angle_deg;
}

/* 往步进队列里追加请求，并做上限保护。 */
static void Shoot_AddDialPendingStep(uint16_t step_count)
{
    uint32_t pending;

    if (step_count == 0U)
    {
        return;
    }

    pending = (uint32_t)shoot_task_ctx.dial.pending_step_count + (uint32_t)step_count;
    if (pending > 0xFFFFU)
    {
        pending = 0xFFFFU;
    }

    shoot_task_ctx.dial.pending_step_count = (uint16_t)pending;
}

/* 清空拨盘所有尚未执行的步进请求。 */
static void Shoot_ClearDialPendingStep(void)
{
    shoot_task_ctx.dial.pending_step_count = 0U;
}

/* 停机状态下按固定节拍查询一次 LK 状态，维持在线监测。 */
static void Shoot_PollDialState(void)
{
    shoot_task_ctx.dial.state_poll_tick++;
    if (shoot_task_ctx.dial.state_poll_tick >= SHOOT_DIAL_STATE_POLL_TICKS)
    {
        shoot_task_ctx.dial.state_poll_tick = 0U;
        (void)Motor_LkSendReadState2Request(&shoot_task_ctx.dial.motor);
    }
}

/* 根据遥控器当前状态刷新遥控侧的摩擦轮使能请求。 */
static void Shoot_UpdateRemoteEnableRequest(const remote_state_t *remote)
{
    shoot_task_ctx.remote_enable_request = 0U;

    if ((remote == NULL) || (remote->online == 0U))
    {
        return;
    }

    if (remote->pause_pressed != 0U)
    {
        return;
    }

    if ((SHOOT_ENABLE_BY_CUSTOM_LEFT != 0U) && (remote->custom_left_pressed != 0U))
    {
        shoot_task_ctx.remote_enable_request = 1U;
    }

    if ((SHOOT_ENABLE_BY_TRIGGER != 0U) && (remote->trigger_pressed != 0U))
    {
        shoot_task_ctx.remote_enable_request = 1U;
    }
}

/* 刷新当前在线标志。 */
static void Shoot_UpdateOnlineFlags(const remote_state_t *remote)
{
    shoot_task_ctx.remote_online = (uint8_t)((remote != NULL) ? remote->online : 0U);
    shoot_task_ctx.left_motor_online = Motor_M3508IsOnline(&shoot_task_ctx.left.motor);
    shoot_task_ctx.right_motor_online = Motor_M3508IsOnline(&shoot_task_ctx.right.motor);
    shoot_task_ctx.dial_motor_online = Motor_LkIsOnline(&shoot_task_ctx.dial.motor);
}

/*
 * 根据当前状态刷新摩擦轮模式。
 *
 * 保护逻辑优先级高于启动请求：
 * 1. pause 按下 -> 强制 STOP
 * 2. 任一摩擦轮掉线 -> 强制 STOP
 * 3. 软件请求或遥控请求存在 -> RUN
 * 4. 否则保持 STOP
 *
 * 这样可以保证急停和掉线保护永远覆盖普通开火请求。
 */
static void Shoot_UpdateFrictionMode(const remote_state_t *remote)
{
    shoot_task_ctx.last_friction_mode = shoot_task_ctx.friction_mode;
    Shoot_UpdateRemoteEnableRequest(remote);

    if ((remote != NULL) && (remote->online != 0U) && (remote->pause_pressed != 0U))
    {
        shoot_task_ctx.friction_mode = SHOOT_FRICTION_MODE_STOP;
        return;
    }

    if ((shoot_task_ctx.left_motor_online == 0U) || (shoot_task_ctx.right_motor_online == 0U))
    {
        shoot_task_ctx.friction_mode = SHOOT_FRICTION_MODE_STOP;
        return;
    }

    if ((shoot_task_ctx.software_enable_request != 0U) || (shoot_task_ctx.remote_enable_request != 0U))
    {
        shoot_task_ctx.friction_mode = SHOOT_FRICTION_MODE_RUN;
    }
    else
    {
        shoot_task_ctx.friction_mode = SHOOT_FRICTION_MODE_STOP;
    }
}

/*
 * 摩擦轮模式切换时统一处理 PID 状态和输出。
 *
 * STOP -> 直接清电流、清 PID、清 ready 计数
 * RUN  -> 保留目标转速，但把控制器内部状态清掉，从干净状态重新起转
 *
 * 这样做是为了避免：
 * 1. 停转后重新启动时带着旧积分
 * 2. ready 标志残留，导致拨盘过早进入闭环
 */
static void Shoot_HandleFrictionModeTransition(void)
{
    if ((shoot_task_ctx.friction_initialized != 0U)
     && (shoot_task_ctx.friction_mode == shoot_task_ctx.last_friction_mode))
    {
        return;
    }

    switch (shoot_task_ctx.friction_mode)
    {
        case SHOOT_FRICTION_MODE_STOP:
            Shoot_StopFrictionOutput();
            shoot_task_ctx.friction_ready = 0U;
            shoot_task_ctx.friction_ready_ticks = 0U;
            break;

        case SHOOT_FRICTION_MODE_RUN:
            Shoot_ClearWheelController(&shoot_task_ctx.left);
            Shoot_ClearWheelController(&shoot_task_ctx.right);
            shoot_task_ctx.friction_ready = 0U;
            shoot_task_ctx.friction_ready_ticks = 0U;
            break;

        default:
            Shoot_StopFrictionOutput();
            shoot_task_ctx.friction_mode = SHOOT_FRICTION_MODE_STOP;
            shoot_task_ctx.friction_ready = 0U;
            shoot_task_ctx.friction_ready_ticks = 0U;
            break;
    }

    shoot_task_ctx.friction_initialized = 1U;
}

/*
 * 判断当前是否满足“允许拨盘闭环”的条件。
 *
 * 当前三条缺一不可：
 * 1. 摩擦轮模式必须已经在 RUN
 * 2. friction_ready 必须已经成立
 * 3. LK 拨盘电机必须在线
 *
 * 这就是拨盘的总联锁。
 * 只有摩擦轮真正转稳以后，拨盘才允许开始闭环推弹。
 */
static uint8_t Shoot_CanRunDialClosedLoop(void)
{
    return (uint8_t)((shoot_task_ctx.friction_mode == SHOOT_FRICTION_MODE_RUN)
                  && (shoot_task_ctx.friction_ready != 0U)
                  && (shoot_task_ctx.dial_motor_online != 0U));
}

/*
 * 根据当前状态刷新拨盘模式。
 *
 * 当前拨盘只有两种状态：
 * 1. STOP
 * 2. CLOSED_LOOP
 *
 * 没有准备好就一律停机，不做“半开半关”的中间态，
 * 这样联锁关系更清楚，也更不容易留下边界问题。
 */
static void Shoot_UpdateDialMode(void)
{
    shoot_task_ctx.last_dial_mode = shoot_task_ctx.dial_mode;

    if (Shoot_CanRunDialClosedLoop() != 0U)
    {
        shoot_task_ctx.dial_mode = SHOOT_DIAL_MODE_CLOSED_LOOP;
    }
    else
    {
        shoot_task_ctx.dial_mode = SHOOT_DIAL_MODE_STOP;
    }
}

/*
 * 拨盘模式切换时统一处理命令、PID 和目标角同步。
 *
 * STOP 时：
 * 1. 清 PID
 * 2. 清尚未执行的步进请求
 * 3. 目标角同步到当前角度
 * 4. 输出 iq 置 0
 * 5. 给 LK 下 stop
 *
 * CLOSED_LOOP 时：
 * 1. 清 PID
 * 2. 清历史步进请求
 * 3. 目标角同步到当前角度
 * 4. 给 LK 上电
 * 5. 主动查询一次状态
 *
 * 这样可以保证拨盘每次重新进入闭环时，都是从“当前物理位置”平滑接管，
 * 不会拿着旧目标角突然追一大步。
 */
static void Shoot_HandleDialModeTransition(void)
{
    if ((shoot_task_ctx.dial_initialized != 0U)
     && (shoot_task_ctx.dial_mode == shoot_task_ctx.last_dial_mode))
    {
        return;
    }

    switch (shoot_task_ctx.dial_mode)
    {
        case SHOOT_DIAL_MODE_STOP:
            Shoot_ClearDialController(&shoot_task_ctx.dial);
            Shoot_ClearDialPendingStep();
            Shoot_SyncDialTargetToFeedback(&shoot_task_ctx.dial);
            Motor_SetLkIqOutput(&shoot_task_ctx.dial.motor, 0);
            (void)Motor_LkSendStop(&shoot_task_ctx.dial.motor);
            shoot_task_ctx.dial.state_poll_tick = 0U;
            break;

        case SHOOT_DIAL_MODE_CLOSED_LOOP:
            Shoot_ClearDialController(&shoot_task_ctx.dial);
            Shoot_ClearDialPendingStep();
            Shoot_SyncDialTargetToFeedback(&shoot_task_ctx.dial);
            (void)Motor_LkSendPowerOn(&shoot_task_ctx.dial.motor);
            (void)Motor_LkSendReadState2Request(&shoot_task_ctx.dial.motor);
            shoot_task_ctx.dial.state_poll_tick = 0U;
            break;

        default:
            shoot_task_ctx.dial_mode = SHOOT_DIAL_MODE_STOP;
            Shoot_ClearDialController(&shoot_task_ctx.dial);
            Shoot_ClearDialPendingStep();
            Shoot_SyncDialTargetToFeedback(&shoot_task_ctx.dial);
            Motor_SetLkIqOutput(&shoot_task_ctx.dial.motor, 0);
            (void)Motor_LkSendStop(&shoot_task_ctx.dial.motor);
            shoot_task_ctx.dial.state_poll_tick = 0U;
            break;
    }

    shoot_task_ctx.dial_initialized = 1U;
}

/*
 * 遥控拨盘步进请求默认看扳机上升沿，只在允许闭环时记入。
 *
 * 这里只认上升沿，不认按住不放，
 * 这样可以天然避免一帧按下、多帧重复加步进的问题。
 */
static void Shoot_UpdateRemoteDialStepRequest(const remote_state_t *remote)
{
    uint8_t trigger_pressed;

    trigger_pressed = 0U;
    if ((remote != NULL)
     && (remote->online != 0U)
     && (remote->pause_pressed == 0U)
     && (SHOOT_DIAL_STEP_BY_TRIGGER != 0U)
     && (remote->trigger_pressed != 0U))
    {
        trigger_pressed = 1U;
    }

    if ((trigger_pressed != 0U)
     && (shoot_task_ctx.dial.last_trigger_pressed == 0U)
     && (shoot_task_ctx.dial_mode == SHOOT_DIAL_MODE_CLOSED_LOOP))
    {
        Shoot_AddDialPendingStep(1U);
    }

    shoot_task_ctx.dial.last_trigger_pressed = trigger_pressed;
}

/* 停机状态下不跑闭环，只做在线轮询和状态同步。 */
static void Shoot_RunDialStopState(void)
{
    Shoot_UpdateDialFeedback(&shoot_task_ctx.dial);
    shoot_task_ctx.dial.target_speed_dps = 0.0f;
    shoot_task_ctx.dial.target_iq = 0.0f;
    Motor_SetLkIqOutput(&shoot_task_ctx.dial.motor, 0);
    Shoot_PollDialState();
}

/* 闭环状态下，角度环给目标角速度，速度环给 iq 命令。 */
static void Shoot_RunDialClosedLoop(void)
{
    Shoot_UpdateDialFeedback(&shoot_task_ctx.dial);

    if ((shoot_task_ctx.dial.pending_step_count > 0U)
     && (Shoot_AbsFloat(shoot_task_ctx.dial.target_angle_deg - shoot_task_ctx.dial.feedback_angle_deg)
         <= SHOOT_DIAL_STEP_ACCEPT_ERROR_DEG))
    {
        shoot_task_ctx.dial.target_angle_deg += shoot_task_ctx.dial.step_angle_deg;
        shoot_task_ctx.dial.pending_step_count--;
        shoot_task_ctx.dial.total_step_count++;
    }

    shoot_task_ctx.dial.target_speed_dps = PID_Calculate(&shoot_task_ctx.dial.angle_pid,
                                                       shoot_task_ctx.dial.target_angle_deg,
                                                       shoot_task_ctx.dial.feedback_angle_deg);
    shoot_task_ctx.dial.target_speed_dps = Shoot_Limit(shoot_task_ctx.dial.target_speed_dps,
                                                     -SHOOT_DIAL_ANGLE_OUT_LIMIT,
                                                     SHOOT_DIAL_ANGLE_OUT_LIMIT);

    shoot_task_ctx.dial.target_iq = PID_Calculate(&shoot_task_ctx.dial.speed_pid,
                                                shoot_task_ctx.dial.target_speed_dps,
                                                shoot_task_ctx.dial.feedback_speed_dps);
    shoot_task_ctx.dial.target_iq = Shoot_Limit(shoot_task_ctx.dial.target_iq,
                                              -SHOOT_DIAL_SPEED_OUT_LIMIT,
                                              SHOOT_DIAL_SPEED_OUT_LIMIT);

    Motor_SetLkIqOutput(&shoot_task_ctx.dial.motor, (int16_t)shoot_task_ctx.dial.target_iq);
    (void)Motor_LkSendIqControl(&shoot_task_ctx.dial.motor);
}

/* 把内部状态同步到对外提供的状态结构体。 */
static void Shoot_UpdatePublicState(void)
{
    shoot_task_state_view.remote_online = shoot_task_ctx.remote_online;
    shoot_task_state_view.left_motor_online = shoot_task_ctx.left_motor_online;
    shoot_task_state_view.right_motor_online = shoot_task_ctx.right_motor_online;
    shoot_task_state_view.dial_motor_online = shoot_task_ctx.dial_motor_online;
    shoot_task_state_view.software_enable_request = shoot_task_ctx.software_enable_request;
    shoot_task_state_view.remote_enable_request = shoot_task_ctx.remote_enable_request;
    shoot_task_state_view.friction_ready = shoot_task_ctx.friction_ready;
    shoot_task_state_view.dial_output_enabled = shoot_task_ctx.dial.motor.output_enabled;
    shoot_task_state_view.friction_mode = shoot_task_ctx.friction_mode;
    shoot_task_state_view.dial_mode = shoot_task_ctx.dial_mode;
    shoot_task_state_view.dial_pending_step_count = shoot_task_ctx.dial.pending_step_count;
    shoot_task_state_view.dial_total_step_count = shoot_task_ctx.dial.total_step_count;
    shoot_task_state_view.left_target_rpm = shoot_task_ctx.left.target_rpm;
    shoot_task_state_view.right_target_rpm = shoot_task_ctx.right.target_rpm;
    shoot_task_state_view.left_command_rpm = shoot_task_ctx.left.command_rpm;
    shoot_task_state_view.right_command_rpm = shoot_task_ctx.right.command_rpm;
    shoot_task_state_view.left_feedback_rpm = shoot_task_ctx.left.feedback_rpm;
    shoot_task_state_view.right_feedback_rpm = shoot_task_ctx.right.feedback_rpm;
    shoot_task_state_view.left_target_current = shoot_task_ctx.left.target_current;
    shoot_task_state_view.right_target_current = shoot_task_ctx.right.target_current;
    shoot_task_state_view.dial_step_angle_deg = shoot_task_ctx.dial.step_angle_deg;
    shoot_task_state_view.dial_target_angle_deg = shoot_task_ctx.dial.target_angle_deg;
    shoot_task_state_view.dial_feedback_angle_deg = shoot_task_ctx.dial.feedback_angle_deg;
    shoot_task_state_view.dial_feedback_speed_dps = shoot_task_ctx.dial.feedback_speed_dps;
    shoot_task_state_view.dial_target_speed_dps = shoot_task_ctx.dial.target_speed_dps;
    shoot_task_state_view.dial_target_iq = shoot_task_ctx.dial.target_iq;
}

/*
 * 初始化发射任务。
 *
 * 调用时机：
 * - main() 完成 BSP_Init() 和 motor 层初始化后调用一次。
 *
 * 初始化内容：
 * - 注册左右摩擦轮 M3508 和 LK 拨盘电机；
 * - 初始化摩擦轮速度 PID、拨盘角度 PID、拨盘速度 PID；
 * - 设置默认摩擦轮目标转速和拨盘单步角度；
 * - 主动查询一次 LK 状态，让在线检测更快建立。
 */
void Shoot_Task_Init(void)
{
    memset(&shoot_task_ctx, 0, sizeof(shoot_task_ctx));
    memset(&shoot_task_state_view, 0, sizeof(shoot_task_state_view));

    shoot_task_ctx.friction_mode = SHOOT_FRICTION_MODE_STOP;
    shoot_task_ctx.last_friction_mode = SHOOT_FRICTION_MODE_STOP;
    shoot_task_ctx.dial_mode = SHOOT_DIAL_MODE_STOP;
    shoot_task_ctx.last_dial_mode = SHOOT_DIAL_MODE_STOP;

    /* 左摩擦轮用 CAN2 上的 ID1 M3508。 */
    if (Motor_RegisterM3508CurrentLoop(&shoot_task_ctx.left.motor,
                                       SHOOT_FRICTION_LEFT_CAN,
                                       SHOOT_FRICTION_LEFT_ID) != HAL_OK)
    {
        Error_Handler();
    }

    /* 右摩擦轮用 CAN2 上的 ID2 M3508。 */
    if (Motor_RegisterM3508CurrentLoop(&shoot_task_ctx.right.motor,
                                       SHOOT_FRICTION_RIGHT_CAN,
                                       SHOOT_FRICTION_RIGHT_ID) != HAL_OK)
    {
        Error_Handler();
    }

    /* LK 拨盘用 CAN2 上的 ID5。 */
    if (Motor_RegisterLk(&shoot_task_ctx.dial.motor,
                         SHOOT_DIAL_CAN,
                         SHOOT_DIAL_ID) != HAL_OK)
    {
        Error_Handler();
    }

    PID_Init(&shoot_task_ctx.left.speed_pid,
             SHOOT_FRICTION_SPEED_KP,
             SHOOT_FRICTION_SPEED_KI,
             SHOOT_FRICTION_SPEED_KD,
             SHOOT_TASK_DT_S,
             SHOOT_FRICTION_SPEED_I_LIMIT,
             SHOOT_FRICTION_SPEED_OUT_LIMIT);

    PID_Init(&shoot_task_ctx.right.speed_pid,
             SHOOT_FRICTION_SPEED_KP,
             SHOOT_FRICTION_SPEED_KI,
             SHOOT_FRICTION_SPEED_KD,
             SHOOT_TASK_DT_S,
             SHOOT_FRICTION_SPEED_I_LIMIT,
             SHOOT_FRICTION_SPEED_OUT_LIMIT);

    PID_Init(&shoot_task_ctx.dial.angle_pid,
             SHOOT_DIAL_ANGLE_KP,
             SHOOT_DIAL_ANGLE_KI,
             SHOOT_DIAL_ANGLE_KD,
             SHOOT_TASK_DT_S,
             SHOOT_DIAL_ANGLE_I_LIMIT,
             SHOOT_DIAL_ANGLE_OUT_LIMIT);

    PID_Init(&shoot_task_ctx.dial.speed_pid,
             SHOOT_DIAL_SPEED_KP,
             SHOOT_DIAL_SPEED_KI,
             SHOOT_DIAL_SPEED_KD,
             SHOOT_TASK_DT_S,
             SHOOT_DIAL_SPEED_I_LIMIT,
             SHOOT_DIAL_SPEED_OUT_LIMIT);

    shoot_task_ctx.left.target_rpm = SHOOT_FRICTION_LEFT_TARGET_RPM;
    shoot_task_ctx.right.target_rpm = SHOOT_FRICTION_RIGHT_TARGET_RPM;
    shoot_task_ctx.dial.step_angle_deg = SHOOT_DIAL_STEP_DEG;
    shoot_task_ctx.dial.state_poll_tick = 0U;

    /* 初始化结束后先主动查一次 LK 状态，让在线检测尽快建立。 */
    (void)Motor_LkSendReadState2Request(&shoot_task_ctx.dial.motor);
    Shoot_UpdatePublicState();
}

/*
 * 发射任务主入口。
 *
 * 每次调用按下面顺序执行：
 * 1. 根据节拍门控决定是否处理本拍；
 * 2. 获取遥控状态并刷新电机在线标志；
 * 3. 更新摩擦轮模式，处理模式切换；
 * 4. 执行摩擦轮速度环或停机输出；
 * 5. 根据摩擦轮转稳状态决定拨盘是否允许闭环；
 * 6. 处理拨盘步进请求并执行拨盘闭环或停机轮询；
 * 7. 统一发送摩擦轮电流帧并刷新对外状态快照。
 */
void Shoot_Task_Run(void)
{
    const remote_state_t *remote;

    /* 遥控模块维护最新状态，本任务只读取快照。 */
    remote = Remote_GetState();

    /* 在线状态和遥控请求共同决定摩擦轮能否运行。 */
    Shoot_UpdateOnlineFlags(remote);
    Shoot_UpdateFrictionMode(remote);
    Shoot_HandleFrictionModeTransition();

    /* 先把反馈同步到控制对象，再执行速度环。 */
    Shoot_UpdateWheelFeedback(&shoot_task_ctx.left);
    Shoot_UpdateWheelFeedback(&shoot_task_ctx.right);

    if (shoot_task_ctx.friction_mode == SHOOT_FRICTION_MODE_RUN)
    {
        /* RUN 模式下，两路摩擦轮分别做速度环，输出先缓存到 motor 对象。 */
        Shoot_RunWheelSpeedLoop(&shoot_task_ctx.left);
        Shoot_RunWheelSpeedLoop(&shoot_task_ctx.right);
    }
    else
    {
        /* STOP 或保护状态下，持续把摩擦轮输出拉为 0。 */
        Shoot_StopFrictionOutput();
    }

    /* 摩擦轮 ready 是拨盘闭环的前置联锁。 */
    Shoot_UpdateFrictionReady();
    Shoot_UpdateDialMode();
    Shoot_HandleDialModeTransition();

    /* 遥控扳机只在拨盘允许闭环时转化为步进请求。 */
    Shoot_UpdateRemoteDialStepRequest(remote);

    if (shoot_task_ctx.dial_mode == SHOOT_DIAL_MODE_CLOSED_LOOP)
    {
        /* 闭环模式：处理步进队列，执行角度环和速度环，并立即发送 LK iq 控制。 */
        Shoot_RunDialClosedLoop();
    }
    else
    {
        /* 停机模式：不输出力矩，只定期查询状态来维持在线检测。 */
        Shoot_RunDialStopState();
    }

    /* M3508 两个摩擦轮同属 CAN2 ID1~ID4，一帧一起发。 */
    Shoot_SendFrictionOutput();
    /* 最后刷新给上层看的状态快照。 */
    Shoot_UpdatePublicState();
}

/*
 * 软件侧控制摩擦轮启停。
 *
 * enable != 0 表示软件请求启动；真正是否 RUN 还要经过在线状态、pause、
 * 遥控请求等联锁判断。
 */
void Shoot_Task_SetFrictionEnable(uint8_t enable)
{
    shoot_task_ctx.software_enable_request = (uint8_t)((enable != 0U) ? 1U : 0U);
    Shoot_UpdatePublicState();
}

/*
 * 软件侧修改摩擦轮目标转速。
 *
 * 这里只改目标值，不会立刻给电机一个阶跃；实际 command_rpm 会在任务运行时
 * 按 SHOOT_FRICTION_RAMP_RPM_PER_S 做斜坡逼近。
 */
void Shoot_Task_SetFrictionTargetRpm(float left_target_rpm, float right_target_rpm)
{
    shoot_task_ctx.left.target_rpm = left_target_rpm;
    shoot_task_ctx.right.target_rpm = right_target_rpm;
    Shoot_UpdatePublicState();
}

/*
 * 追加拨盘步进请求。
 *
 * 只有拨盘已经处于 CLOSED_LOOP 时才接受请求。
 * 请求会进入 pending_step_count 队列，由 Shoot_RunDialClosedLoop() 在目标角收敛后逐个执行。
 */
void Shoot_Task_RequestDialStep(uint16_t step_count)
{
    if ((step_count == 0U) || (shoot_task_ctx.dial_mode != SHOOT_DIAL_MODE_CLOSED_LOOP))
    {
        return;
    }

    Shoot_AddDialPendingStep(step_count);
    Shoot_UpdatePublicState();
}

/*
 * 获取发射任务状态快照。
 *
 * 返回的是只读指针，调用者可以用它做调试显示、上位机上传或其他状态判断。
 */
const shoot_task_state_t *Shoot_Task_GetState(void)
{
    Shoot_UpdatePublicState();
    return &shoot_task_state_view;
}

