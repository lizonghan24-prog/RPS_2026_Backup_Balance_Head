#ifndef MODE_SWITCH_TASK_H
#define MODE_SWITCH_TASK_H

#include "Control_Task.h"
#include "IMU.h"
#include "Remote.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CONTROL_INPUT_STOP = 0U,
    CONTROL_INPUT_REMOTE = 1U,
    CONTROL_INPUT_KEY_MOUSE = 2U
} control_input_mode_t;

/*
 * 模式切换层是云台模式选择的唯一入口。
 * 如果这里的模式规则和 Control_Task 的工具函数有冲突，以本层为准。
 */
void Mode_Switch_Task_Init(void);

control_mode_t Mode_Switch_Task_Update(const imu_hi91_t *imu,
                                       const remote_state_t *remote,
                                       const gimbal_control_task_t *control_state);

control_mode_t Mode_Switch_Task_GetControlMode(void);
control_input_mode_t Mode_Switch_Task_GetInputMode(void);

/* 旧代码兼容入口。 */
void mode_switch_task(void);

#ifdef __cplusplus
}
#endif

#endif /* MODE_SWITCH_TASK_H */
