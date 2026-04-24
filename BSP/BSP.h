#ifndef BSP_H
#define BSP_H

#include "main.h"
#include "IMU.h"
#include "Remote.h"
#include "motor.h"

/*
 * BSP module quick usage:
 * - Call BSP_Init() once after HAL peripheral init in main().
 * - Call BSP_Poll() in while(1) loop to drain UART DMA/ring buffers.
 * - Forward UART idle IRQ to BSP_HandleUartIdle(huart).
 */

/* BSP 总初始化：启动串口 DMA 接收、CAN、1 kHz 定时器等底层资源。 */
void BSP_Init(void);

/* 主循环轮询入口：从环形缓冲中取出串口数据并分发给各模块解析。 */
void BSP_Poll(void);

/* 串口空闲中断处理：把 DMA 缓冲中新到的数据搬进软件环形缓冲。 */
void BSP_HandleUartIdle(UART_HandleTypeDef *huart);

#endif
