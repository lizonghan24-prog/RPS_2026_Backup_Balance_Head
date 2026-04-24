#include "CANBus_Task.h"

/* Compatibility wrapper for legacy CAN task API in old project code. */

/*
 * @file CANBus_Task.c
 * @brief CAN 总线任务兼容层。

 */

#include "motor.h"

#include <string.h>


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*
 * 发送一帧 HAL 标准数据帧。
 *

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

 * 同时传入 rx_header 和 rx_data。
 */
void Can1ReceiveMsgProcess(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t rx_data[8])
{
    CANBus_Task_ProcessRxMessage(&hcan1, rx_header, rx_data);
}

/*
 * CAN2 接收兼容入口。

 */
void Can2ReceiveMsgProcess(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t rx_data[8])
{
    CANBus_Task_ProcessRxMessage(&hcan2, rx_header, rx_data);
}

/*
 * 旧工程保留的 DJI CAN 总发送任务入口。
 *

 */
void dji_can_bus_send_task(void)
{
}
