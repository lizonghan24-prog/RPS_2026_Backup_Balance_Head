#ifndef __CANBUS_TASK_H
#define __CANBUS_TASK_H

/*
 * @file CANBus_Task.h
 * @brief CANBus_Task 对外接口和旧工程 CAN ID 宏。
 *
 * 当前工程的电机收发主要由 motor 层承担，本头文件保留 CAN 分发入口、
 * 标准帧发送工具，以及旧工程中常用的 CAN 标准帧 ID 宏，方便迁移代码时引用。
 */

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 
 *
 * 当前工程的 HAL 接收链路是：
 * HAL_CAN_RxFifo0MsgPendingCallback()
 * -> HAL_CAN_GetRxMessage()
 * -> Motor_ProcessCanMessage()
 *
 * 如果后续把 CAN 分发重新集中到 CANBus_Task，在 HAL 回调里调用
 * 继续把电机反馈交给 motor 层解析
 */

/* ========================= 旧工程保留的标准帧 ID 宏 ========================= */
/* 舵轮底盘转向电机 / 麦轮底盘电机反馈 ID。未使用的 ID 先保留为 0。 */
#define GM1Encoder_MOTOR                         0x00U
#define GM2Encoder_MOTOR                         0x00U
#define GM3Encoder_MOTOR                         0x00U
#define GM4Encoder_MOTOR                         0x00U

#define CM1Encoder_MOTOR                         0x00U
#define CM2Encoder_MOTOR                         0x00U
#define CM3Encoder_MOTOR                         0x00U
#define CM4Encoder_MOTOR                         0x00U

/* 云台 GM6020 反馈 ID。 */
#define GIMBAL_YAW_MOTOR                         0x209U
#define GIMBAL_PITCH_MOTOR                       0x205U

/* 英雄小云台相关电机反馈 ID。 */
#define SMALL_GIMBAL_MOTOR                       0x00U
#define SCOPE_MOTOR                              0x00U

/* 摩擦轮反馈 ID。 */
#define LEFT_FRICTION                            0x201U
#define RIGHT_FRICTION                           0x202U
#define LEFT_UP_FRICTION                         0x00U
#define RIGHT_UP_FRICTION                        0x00U
#define LEFT_DOWN_FRIICTION                      0x00U
#define RIGHT_DOWN_FRICTION                      0x00U

/* 拨盘反馈 ID。 */
#define DOWN_POKE                                0x205U
#define UP_POKE                                  0x205U
#define LEFT_POKE                                0x00U
#define RIGHT_POKE                               0x00U
#define POKE                                     0x203U

/* 上下板通信标准帧 ID。 */
#define UP_CAN2_TO_DOWN_CAN1_1                   0x407U
#define UP_CAN2_TO_DOWN_CAN1_2                   0x408U
#define UP_CAN2_TO_DOWN_CAN1_3                   0x409U

/*
 * HAL CAN 接收分发入口。
 *
 * 参数：
 * - hcan：收到该帧的 CAN 句柄，例如 &hcan1 / &hcan2；
 * - rx_header：HAL 读出的接收帧头；
 * - rx_data：HAL 读出的 8 字节数据区。
 */
void CANBus_Task_ProcessRxMessage(CAN_HandleTypeDef *hcan,
                                  const CAN_RxHeaderTypeDef *rx_header,
                                  const uint8_t rx_data[8]);

/* CAN1 接收分发兼容入口，内部会使用 &hcan1。 */
void Can1ReceiveMsgProcess(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t rx_data[8]);

/* CAN2 接收分发兼容入口，内部会使用 &hcan2。 */
void Can2ReceiveMsgProcess(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t rx_data[8]);

/*
 * 标准数据帧发送工具函数。
 *
 * 当前控制任务和发射任务已经直接调用 motor 层发送电机控制帧。
 * 这个函数留给以后发送上下板通信帧或非电机模块帧使用。
 */
HAL_StatusTypeDef CANBus_Task_SendStdFrame(CAN_HandleTypeDef *hcan,
                                           uint32_t std_id,
                                           const uint8_t tx_data[8]);

/*

 *
 * 当前工程的云台/发射 CAN 输出已经分别在 Control_Task / Shoot_Task 中完成，
 * 所以这里默认不发送任何帧，只保留函数名保证旧调度代码可以链接。
 */
void dji_can_bus_send_task(void);

#ifdef __cplusplus
}
#endif

#endif /* __CANBUS_TASK_H */
