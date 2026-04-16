#include "Shoot_Task.h"

#include "PID.h"
#include "Remote.h"
#include "motor.h"

#include <string.h>

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

/* ========================= LK 拨盘电机挂载配置区 ========================= */
/* 拨盘使用 CAN2 上的 ID5 LK 电机。 */
#define SHOOT_DIAL_CAN                                (&hcan2)
#define SHOOT_DIAL_ID                                 5U

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
    volatile uint32_t tick_pending;                  /* 等待主循环处理的 1 kHz 节拍数。 */
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

static shoot_task_t shoot_task_ctx;
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

void Shoot_Task_Run(void)
{
    const remote_state_t *remote;

    if (shoot_task_ctx.tick_pending == 0U)
    {
        return;
    }

    __disable_irq();
    shoot_task_ctx.tick_pending--;
    __enable_irq();

    remote = Remote_GetState();

    Shoot_UpdateOnlineFlags(remote);
    Shoot_UpdateFrictionMode(remote);
    Shoot_HandleFrictionModeTransition();
    Shoot_UpdateWheelFeedback(&shoot_task_ctx.left);
    Shoot_UpdateWheelFeedback(&shoot_task_ctx.right);

    if (shoot_task_ctx.friction_mode == SHOOT_FRICTION_MODE_RUN)
    {
        Shoot_RunWheelSpeedLoop(&shoot_task_ctx.left);
        Shoot_RunWheelSpeedLoop(&shoot_task_ctx.right);
    }
    else
    {
        Shoot_StopFrictionOutput();
    }

    Shoot_UpdateFrictionReady();
    Shoot_UpdateDialMode();
    Shoot_HandleDialModeTransition();
    Shoot_UpdateRemoteDialStepRequest(remote);

    if (shoot_task_ctx.dial_mode == SHOOT_DIAL_MODE_CLOSED_LOOP)
    {
        Shoot_RunDialClosedLoop();
    }
    else
    {
        Shoot_RunDialStopState();
    }

    Shoot_SendFrictionOutput();
    Shoot_UpdatePublicState();
}

void Shoot_Task_Timer1kHzCallback(void)
{
    shoot_task_ctx.tick_pending++;
}

void Shoot_Task_SetFrictionEnable(uint8_t enable)
{
    shoot_task_ctx.software_enable_request = (uint8_t)((enable != 0U) ? 1U : 0U);
    Shoot_UpdatePublicState();
}

void Shoot_Task_SetFrictionTargetRpm(float left_target_rpm, float right_target_rpm)
{
    shoot_task_ctx.left.target_rpm = left_target_rpm;
    shoot_task_ctx.right.target_rpm = right_target_rpm;
    Shoot_UpdatePublicState();
}

void Shoot_Task_RequestDialStep(uint16_t step_count)
{
    if ((step_count == 0U) || (shoot_task_ctx.dial_mode != SHOOT_DIAL_MODE_CLOSED_LOOP))
    {
        return;
    }

    Shoot_AddDialPendingStep(step_count);
    Shoot_UpdatePublicState();
}

const shoot_task_state_t *Shoot_Task_GetState(void)
{
    Shoot_UpdatePublicState();
    return &shoot_task_state_view;
}


