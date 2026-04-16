#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "main.h"

/* 控制任务初始化。 */
void Control_Task_Init(void);

/* 主循环中的控制任务执行入口。 */
void Control_Task_Run(void);

/* TIM6 的 1 kHz 节拍入口。 */
void Control_Task_Timer1kHzCallback(void);

#endif
