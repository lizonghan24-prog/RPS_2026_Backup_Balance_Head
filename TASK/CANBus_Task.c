#include "CANBus_Task.h"

/*
 * @file CANBus_Task.c
 * @brief CAN 总线任务兼容层。
 *
 * 旧标准库工程通常会把 CAN 接收解析和周期发送都放在 can_bus_task 里。
 * 当前 HAL 工程把电机协议细节集中到了 BSP/motor.c：
 * - CAN 中断读出 rx_header / rx_data；
 * - 本文件可以作为统一分发入口；
 * - motor 层根据 hcan + StdId 自动匹配 GM6020 / M3508 / LK 电机对象；
 * - 云台和发射任务各自负责计算输出，motor 层负责打包并发送。
 *
 * 因此本文件主要保留“任务入口”和“标准帧发送工具”，方便以后扩展上下板通信
 * 或把 CAN 分发重新集中到 CANBus_Task。
 */

#include "motor.h"

#include <string.h>

/* CubeMX 生成的 CAN 句柄。本文件只引用它们来兼容 CAN1/CAN2 旧入口。 */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*
 * 发送一帧 HAL 标准数据帧。
 *
 * 这里不用旧标准库的 CanTxMsg / CAN_Transmit，
 * 而是使用 STM32 HAL 的 CAN_TxHeaderTypeDef + HAL_CAN_AddTxMessage()。
 */
HAL_StatusTypeDef CANBus_Task_SendStdFrame(CAN_HandleTypeDef *hcan,
                                           uint32_t std_id,
                                           const uint8_t tx_data[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    /* 参数保护：没有 CAN 句柄或数据区时不能构造合法发送帧。 */
    if ((hcan == NULL) || (tx_data == NULL))
    {
        return HAL_ERROR;
    }

    /* HAL 标准数据帧配置：标准 ID、数据帧、8 字节负载。 */
    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

/*
 * HAL CAN 接收帧统一处理。
 *
 * 目前 CANBus_Task 不重复解析电机反馈，而是直接转交给 motor 层。
 * motor 层会按“CAN 句柄 + StdId”匹配已经注册的 GM6020 / M3508 / LK 电机。
 */
void CANBus_Task_ProcessRxMessage(CAN_HandleTypeDef *hcan,
                                  const CAN_RxHeaderTypeDef *rx_header,
                                  const uint8_t rx_data[8])
{
    if ((hcan == NULL) || (rx_header == NULL) || (rx_data == NULL))
    {
        return;
    }

    /* 当前只处理标准数据帧，扩展帧或远程帧直接忽略。 */
    if ((rx_header->IDE != CAN_ID_STD) || (rx_header->RTR != CAN_RTR_DATA))
    {
        return;
    }

    /* 电机反馈的具体协议解析统一交给 motor 层。 */
    Motor_ProcessCanMessage(hcan, rx_header, rx_data);
}

/*
 * CAN1 接收兼容入口。
 *
 * HAL 的接收数据由 header + data 两部分组成，所以旧工程里只传 msg 的接口
 * 已经不够用，这里改成同时传入 rx_header 和 rx_data。
 */
void Can1ReceiveMsgProcess(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t rx_data[8])
{
    CANBus_Task_ProcessRxMessage(&hcan1, rx_header, rx_data);
}

/*
 * CAN2 接收兼容入口。
 *
 * 使用方式与 Can1ReceiveMsgProcess() 相同，只是 CAN 句柄固定为 &hcan2。
 */
void Can2ReceiveMsgProcess(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t rx_data[8])
{
    CANBus_Task_ProcessRxMessage(&hcan2, rx_header, rx_data);
}

/*
 * 旧工程保留的 DJI CAN 总发送任务入口。
 *
 * 当前工程里：
 * - 云台 GM6020 输出由 Control_Task_Run() 计算并发送；
 * - 摩擦轮 M3508 和 LK 拨盘输出由 Shoot_Task_Run() 计算并发送；
 * - 这里暂时不主动发送任何帧，只保留函数名给旧调度代码或以后扩展使用。
 */
void dji_can_bus_send_task(void)
{
}
