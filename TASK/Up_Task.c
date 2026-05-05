#include "Up_Task.h"

#include "IMU.h"
#include "Mode_Switch_Task.h"
#include "Remote.h"
#include "USART_chassis_transmit.h"

/*
 * 上层云台控制任务。
 *
。
 */

static uint8_t Up_Task_GimbalInitAllowed(void)
{
   
    if ((USART_Chassis_IsOnline() != 0U) && (usart_gimbal_data.allow_gimbal_init == 0U))
    {
        return 0U;
    }

    return 1U;
}

void Up_Task_Init(void)
{
    Control_Task_Init();
    Mode_Switch_Task_Init();
}

void Up_Task_Run(void)
{
    const imu_hi91_t *imu;
    const remote_state_t *remote;
    control_mode_t mode;

    imu = IMU_GetState();
    remote = Remote_GetState();

    /* 先更新在线状态，Mode_Switch_Task 会依赖这些标志做安全判断。 */
    Control_Task_UpdateOnlineFlags(imu, remote);

    /*
     * Mode_Switch_Task 按原 mode_switch_task.c 的状态机选择模式。
     * Up_Task 不再重新判模式，只执行选中的模式。
     */
    mode = Mode_Switch_Task_Update(imu, remote, Control_Task_GetState());
    Control_Task_SetMode(mode);

    /* INIT 和 FOLLOW_ZGYRO 都要用反馈，所以在处理模式前先刷新。 */
    if ((imu != NULL) && (imu->online != 0U))
    {
        Control_Task_UpdateFeedback(imu);
    }

    /* 在模式边沿重置 PID / 历史量，对齐原工程任务状态行为。 */
    Control_Task_HandleModeTransition();

    switch (mode)
    {
        case CONTROL_MODE_RELAX:
            /* 清空控制输出，电机保持零输出。 */
            Control_Task_StopOutput();
            Control_Task_SendOutput();
            break;

        case CONTROL_MODE_INIT:
            /*
             *驱动 pitch / yaw 靠近初始化目标，
             * 两轴误差都进入阈值后置位 init_finished。
             */
            Control_Task_UpdateInitTargets();
            if (Up_Task_GimbalInitAllowed() != 0U)
            {
                Control_Task_RunClosedLoop();
            }
            else
            {
                Control_Task_StopOutput();
            }
            Control_Task_SendOutput();
            break;

        case CONTROL_MODE_FOLLOW_ZGYRO:
            /*
             * 先按当前输入来源更新动态目标，
             * 再运行通用双环控制器。
             */
            if (Mode_Switch_Task_GetInputMode() == CONTROL_INPUT_REMOTE)
            {
                /* 遥控模式使用摇杆通道作为增量角度命令。 */
                Control_Task_UpdateRemoteTargets(remote, 1.0f);
            }
            else if (Mode_Switch_Task_GetInputMode() == CONTROL_INPUT_KEY_MOUSE)
            {
                if ((remote != NULL) && (remote->online != 0U))
                {
                    /* 键鼠模式使用鼠标增量作为增量角度命令。 */
                    Control_Task_UpdateMouseTargets(remote->mouse_x, remote->mouse_y);
                }
            }
            else
            {
                /* 防御兜底：没有合法输入来源时不允许主动控制。 */
                Control_Task_SetMode(CONTROL_MODE_RELAX);
                Control_Task_StopOutput();
                Control_Task_SendOutput();
                break;
            }

            Control_Task_RunClosedLoop();
            Control_Task_SendOutput();
            break;

        default:
            Control_Task_SetMode(CONTROL_MODE_RELAX);
            Control_Task_StopOutput();
            Control_Task_SendOutput();
            break;
    }
}


void gimbal_parameter_Init(void)
{
    Up_Task_Init();
}

void gimbal_task(void)
{
    Up_Task_Run();
}
