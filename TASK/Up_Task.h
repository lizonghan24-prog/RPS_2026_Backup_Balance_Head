#ifndef UP_TASK_H
#define UP_TASK_H

#include "Control_Task.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Up_Task 是云台运行时逻辑的入口。
 * Control_Task 只提供背后的可复用控制函数库。
 */
void Up_Task_Init(void);
void Up_Task_Run(void);

/* 旧代码兼容入口。 */
void gimbal_parameter_Init(void);
void gimbal_task(void);

#ifdef __cplusplus
}
#endif

#endif /* UP_TASK_H */
