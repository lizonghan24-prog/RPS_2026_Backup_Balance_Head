#include "Mode_Switch_Task.h"

static control_mode_t mode_switch_control_mode = CONTROL_MODE_RELAX;
static control_mode_t mode_switch_last_control_mode = CONTROL_MODE_RELAX;
static control_input_mode_t mode_switch_input_mode = CONTROL_INPUT_STOP;
static control_input_mode_t mode_switch_last_input_mode = CONTROL_INPUT_STOP;

/*
 * 
 * - 停止 / 暂停 / 遥控离线 -> 不允许控制
 * - 遥控器 N 档 -> 遥控输入
 * - 遥控器 S 档 -> 键鼠输入
 */
static control_input_mode_t Mode_Switch_DecodeInputMode(const remote_state_t *remote)
{
    if ((remote == NULL) || (remote->online == 0U) || (remote->pause_pressed != 0U))
    {
        return CONTROL_INPUT_STOP;
    }

    switch (remote->gear)
    {
        case CONTROL_MODE_GEAR_REMOTE:
            return CONTROL_INPUT_REMOTE;

        case CONTROL_MODE_GEAR_KEY_MOUSE:
            return CONTROL_INPUT_KEY_MOUSE;

        case CONTROL_MODE_GEAR_RELAX:
        default:
            return CONTROL_INPUT_STOP;
    }
}

/*
 * 进入任何主动云台模式前的硬安全门。
 * 工程可用的条件是
 * IMU 在线、pitch 电机在线、yaw 电机在线。
 */
static uint8_t Mode_Switch_ControlReady(const imu_hi91_t *imu,
                                        const gimbal_control_task_t *control_state)
{
    if ((imu == NULL) || (imu->online == 0U))
    {
        return 0U;
    }

    if ((control_state == NULL)
     || (control_state->pitch_motor_online == 0U)
     || (control_state->yaw_motor_online == 0U))
    {
        return 0U;
    }

    return 1U;
}

void Mode_Switch_Task_Init(void)
{
    mode_switch_control_mode = CONTROL_MODE_RELAX;
    mode_switch_last_control_mode = CONTROL_MODE_RELAX;
    mode_switch_input_mode = CONTROL_INPUT_STOP;
    mode_switch_last_input_mode = CONTROL_INPUT_STOP;
    Control_Task_ClearInitFinished();
}

control_mode_t Mode_Switch_Task_Update(const imu_hi91_t *imu,
                                       const remote_state_t *remote,
                                       const gimbal_control_task_t *control_state)
{
    /* 先保存上一拍输入模式和控制模式，再计算这一拍的状态。 */
    mode_switch_last_control_mode = mode_switch_control_mode;
    mode_switch_last_input_mode = mode_switch_input_mode;
    mode_switch_input_mode = Mode_Switch_DecodeInputMode(remote);

    /*
     * 
     * 输入模式变化时先把云台复位到 RELAX，然后同一拍仍然可以继续进入
     * RELAX -> INIT -> FOLLOW_ZGYRO 的状态链。
     */
    if (mode_switch_input_mode != mode_switch_last_input_mode)
    {
        /*
         * 切换输入模式时，所有云台任务先回到初始状态，
         * 再允许新的主动模式启动。
         */
        mode_switch_control_mode = CONTROL_MODE_RELAX;
        mode_switch_last_control_mode = CONTROL_MODE_RELAX;
        Control_Task_ClearInitFinished();
    }

    if ((mode_switch_input_mode == CONTROL_INPUT_STOP)
     || (Mode_Switch_ControlReady(imu, control_state) == 0U))
    {
        /* 停止、暂停、遥控离线、IMU 离线或电机离线都会强制关控。 */
        mode_switch_control_mode = CONTROL_MODE_RELAX;
        Control_Task_ClearInitFinished();
        return mode_switch_control_mode;
    }

    if ((mode_switch_control_mode != CONTROL_MODE_INIT)
     && (mode_switch_last_control_mode == CONTROL_MODE_RELAX))
    {
        /*
         * 
         * 先标记目标主动模式，下面的判断再把 RELAX -> 主动模式的边沿转成 INIT。
         */
        mode_switch_control_mode = CONTROL_MODE_FOLLOW_ZGYRO;
    }

    if ((mode_switch_last_control_mode == CONTROL_MODE_RELAX)
     && (mode_switch_control_mode != CONTROL_MODE_RELAX))
    {
        /* 第一次进入主动控制时必须先执行云台初始化。 */
        mode_switch_control_mode = CONTROL_MODE_INIT;
        Control_Task_ClearInitFinished();
    }

    if ((control_state != NULL) && (control_state->init_finished != 0U))
    {
        /* 初始化完成后，切入正常云台跟随。 */
        mode_switch_control_mode = CONTROL_MODE_FOLLOW_ZGYRO;
    }

    return mode_switch_control_mode;
}

control_mode_t Mode_Switch_Task_GetControlMode(void)
{
    return mode_switch_control_mode;
}

control_input_mode_t Mode_Switch_Task_GetInputMode(void)
{
    return mode_switch_input_mode;
}

/* 兼容旧调度器使用的小写任务名。 */
void mode_switch_task(void)
{
    (void)Mode_Switch_Task_Update(IMU_GetState(),
                                  Remote_GetState(),
                                  Control_Task_GetState());
}
