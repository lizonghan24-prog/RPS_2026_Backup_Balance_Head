#include "motor.h"

#include <string.h>

#define GM6020_ONLINE_TIMEOUT_MS        100U
#define M3508_ONLINE_TIMEOUT_MS         100U
#define LK_MOTOR_ONLINE_TIMEOUT_MS      100U

static gm6020_service_t *gm6020_registry[GM6020_REGISTRY_SIZE];
static m3508_service_t *m3508_registry[M3508_REGISTRY_SIZE];
static lk_motor_service_t *lk_registry[LK_MOTOR_REGISTRY_SIZE];

/* 计算 GM6020 反馈标准 ID。 */
static uint16_t Motor_Gm6020RxStdId(uint8_t motor_id)
{
    return (uint16_t)(0x204U + motor_id);
}

/* 计算 GM6020 控制帧标准 ID。 */
static uint16_t Motor_Gm6020TxStdId(uint8_t motor_id)
{
    return (uint16_t)((motor_id <= 4U) ? GM6020_GROUP_1_4_STDID : GM6020_GROUP_5_7_STDID);
}

/* 计算 GM6020 在控制帧里的槽位。 */
static uint8_t Motor_Gm6020TxSlot(uint8_t motor_id)
{
    return (uint8_t)((motor_id <= 4U) ? (motor_id - 1U) : (motor_id - 5U));
}

/* 计算 M3508 反馈标准 ID。 */
static uint16_t Motor_M3508RxStdId(uint8_t motor_id)
{
    return (uint16_t)(0x200U + motor_id);
}

/* 计算 M3508 控制帧标准 ID。 */
static uint16_t Motor_M3508TxStdId(uint8_t motor_id)
{
    return (uint16_t)((motor_id <= 4U) ? M3508_GROUP_1_4_STDID : M3508_GROUP_5_8_STDID);
}

/* 计算 M3508 在控制帧里的槽位。 */
static uint8_t Motor_M3508TxSlot(uint8_t motor_id)
{
    return (uint8_t)((motor_id <= 4U) ? (motor_id - 1U) : (motor_id - 5U));
}

/* 计算 LK 电机反馈标准 ID。 */
static uint16_t Motor_LkRxStdId(uint8_t motor_id)
{
    return (uint16_t)(LK_MOTOR_ID_BASE + motor_id);
}

/* 对有符号 16 位数做对称限幅。 */
static int16_t Motor_LimitS16(int32_t value, int16_t limit)
{
    if (value > (int32_t)limit)
    {
        return limit;
    }

    if (value < -(int32_t)limit)
    {
        return (int16_t)(-limit);
    }

    return (int16_t)value;
}

/* 在注册表中挂入一个 GM6020 服务对象。 */
static void Motor_UpdateTotalEcd16(uint8_t *initialized,
                                   int16_t *last_ecd,
                                   int32_t *total_ecd,
                                   uint16_t ecd,
                                   uint16_t encoder_cpr)
{
    int32_t last_ecd_i32;
    int32_t delta;

    if ((*initialized == 0U) || (last_ecd == NULL) || (total_ecd == NULL))
    {
        if ((initialized != NULL) && (last_ecd != NULL) && (total_ecd != NULL))
        {
            *initialized = 1U;
            *last_ecd = (int16_t)ecd;
            *total_ecd = 0;
        }
        return;
    }

    last_ecd_i32 = (int32_t)(*last_ecd);
    delta = (int32_t)ecd - last_ecd_i32;
    if (delta > (int32_t)(encoder_cpr / 2U))
    {
        delta -= (int32_t)encoder_cpr;
    }
    else if (delta < -(int32_t)(encoder_cpr / 2U))
    {
        delta += (int32_t)encoder_cpr;
    }

    *total_ecd += delta;
    *last_ecd = (int16_t)ecd;
}

static void Motor_UpdateTotalEcd32(uint8_t *initialized,
                                   int32_t *last_ecd,
                                   int32_t *total_ecd,
                                   uint16_t ecd,
                                   uint32_t encoder_cpr)
{
    int32_t delta;

    if ((*initialized == 0U) || (last_ecd == NULL) || (total_ecd == NULL))
    {
        if ((initialized != NULL) && (last_ecd != NULL) && (total_ecd != NULL))
        {
            *initialized = 1U;
            *last_ecd = (int32_t)ecd;
            *total_ecd = 0;
        }
        return;
    }

    delta = (int32_t)ecd - *last_ecd;
    if (delta > (int32_t)(encoder_cpr / 2U))
    {
        delta -= (int32_t)encoder_cpr;
    }
    else if (delta < -(int32_t)(encoder_cpr / 2U))
    {
        delta += (int32_t)encoder_cpr;
    }

    *total_ecd += delta;
    *last_ecd = (int32_t)ecd;
}

static HAL_StatusTypeDef Motor_RegisterGm6020ToTable(gm6020_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < GM6020_REGISTRY_SIZE; ++i)
    {
        if (gm6020_registry[i] == motor)
        {
            return HAL_OK;
        }
    }

    for (i = 0U; i < GM6020_REGISTRY_SIZE; ++i)
    {
        if (gm6020_registry[i] == NULL)
        {
            gm6020_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* 在注册表中挂入一个 M3508 服务对象。 */
static HAL_StatusTypeDef Motor_RegisterM3508ToTable(m3508_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < M3508_REGISTRY_SIZE; ++i)
    {
        if (m3508_registry[i] == motor)
        {
            return HAL_OK;
        }
    }

    for (i = 0U; i < M3508_REGISTRY_SIZE; ++i)
    {
        if (m3508_registry[i] == NULL)
        {
            m3508_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* 在注册表中挂入一个 LK 电机服务对象。 */
static HAL_StatusTypeDef Motor_RegisterLkToTable(lk_motor_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < LK_MOTOR_REGISTRY_SIZE; ++i)
    {
        if (lk_registry[i] == motor)
        {
            return HAL_OK;
        }
    }

    for (i = 0U; i < LK_MOTOR_REGISTRY_SIZE; ++i)
    {
        if (lk_registry[i] == NULL)
        {
            lk_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* 统一发送一帧标准 CAN 数据帧。 */
static HAL_StatusTypeDef Motor_SendRawFrame(CAN_HandleTypeDef *hcan,
                                            uint16_t std_id,
                                            const uint8_t *data,
                                            uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    uint8_t tx_buffer[8];

    if ((hcan == NULL) || (data == NULL) || (dlc > 8U))
    {
        return HAL_ERROR;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    memset(tx_buffer, 0, sizeof(tx_buffer));
    memcpy(tx_buffer, data, dlc);

    tx_header.StdId = std_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer, &mailbox);
}

/* 统一发送一帧 DJI 电机 8 字节控制帧。 */
static HAL_StatusTypeDef Motor_SendDjiGroupFrame(CAN_HandleTypeDef *hcan,
                                                 uint16_t std_id,
                                                 const int16_t slot_output[4])
{
    uint8_t tx_data[8];
    uint8_t i;

    if ((hcan == NULL) || (slot_output == NULL))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < 4U; ++i)
    {
        tx_data[i * 2U] = (uint8_t)((uint16_t)slot_output[i] >> 8);
        tx_data[i * 2U + 1U] = (uint8_t)slot_output[i];
    }

    return Motor_SendRawFrame(hcan, std_id, tx_data, 8U);
}

/* 根据传入的 GM6020 服务对象构建一帧控制输出。 */
static HAL_StatusTypeDef Motor_BuildGm6020Frame(gm6020_mode_t expected_mode,
                                                const gm6020_service_t *motor_a,
                                                const gm6020_service_t *motor_b,
                                                const gm6020_service_t *motor_c,
                                                const gm6020_service_t *motor_d,
                                                CAN_HandleTypeDef **hcan_out,
                                                uint16_t *std_id_out,
                                                int16_t slot_output[4],
                                                uint8_t *has_data_out)
{
    const gm6020_service_t *motors[4];
    const gm6020_service_t *motor;
    CAN_HandleTypeDef *hcan;
    uint16_t std_id;
    uint8_t i;
    uint8_t has_data;
    uint8_t slot_used[4];

    motors[0] = motor_a;
    motors[1] = motor_b;
    motors[2] = motor_c;
    motors[3] = motor_d;

    memset(slot_output, 0, sizeof(int16_t) * 4U);
    memset(slot_used, 0, sizeof(slot_used));
    hcan = NULL;
    std_id = 0U;
    has_data = 0U;

    for (i = 0U; i < 4U; ++i)
    {
        motor = motors[i];
        if (motor == NULL)
        {
            continue;
        }

        if ((motor->registered == 0U) || (motor->mode != expected_mode))
        {
            return HAL_ERROR;
        }

        if (has_data == 0U)
        {
            hcan = motor->hcan;
            std_id = motor->tx_std_id;
            has_data = 1U;
        }
        else if ((motor->hcan != hcan) || (motor->tx_std_id != std_id))
        {
            return HAL_ERROR;
        }

        if ((motor->tx_slot >= 4U) || (slot_used[motor->tx_slot] != 0U))
        {
            return HAL_ERROR;
        }

        slot_output[motor->tx_slot] = motor->target_output;
        slot_used[motor->tx_slot] = 1U;
    }

    *hcan_out = hcan;
    *std_id_out = std_id;
    *has_data_out = has_data;
    return HAL_OK;
}

/* 根据传入的 M3508 服务对象构建一帧控制输出。 */
static HAL_StatusTypeDef Motor_BuildM3508Frame(const m3508_service_t *motor_a,
                                               const m3508_service_t *motor_b,
                                               const m3508_service_t *motor_c,
                                               const m3508_service_t *motor_d,
                                               CAN_HandleTypeDef **hcan_out,
                                               uint16_t *std_id_out,
                                               int16_t slot_output[4],
                                               uint8_t *has_data_out)
{
    const m3508_service_t *motors[4];
    const m3508_service_t *motor;
    CAN_HandleTypeDef *hcan;
    uint16_t std_id;
    uint8_t i;
    uint8_t has_data;
    uint8_t slot_used[4];

    motors[0] = motor_a;
    motors[1] = motor_b;
    motors[2] = motor_c;
    motors[3] = motor_d;

    memset(slot_output, 0, sizeof(int16_t) * 4U);
    memset(slot_used, 0, sizeof(slot_used));
    hcan = NULL;
    std_id = 0U;
    has_data = 0U;

    for (i = 0U; i < 4U; ++i)
    {
        motor = motors[i];
        if (motor == NULL)
        {
            continue;
        }

        if (motor->registered == 0U)
        {
            return HAL_ERROR;
        }

        if (has_data == 0U)
        {
            hcan = motor->hcan;
            std_id = motor->tx_std_id;
            has_data = 1U;
        }
        else if ((motor->hcan != hcan) || (motor->tx_std_id != std_id))
        {
            return HAL_ERROR;
        }

        if ((motor->tx_slot >= 4U) || (slot_used[motor->tx_slot] != 0U))
        {
            return HAL_ERROR;
        }

        slot_output[motor->tx_slot] = motor->target_output;
        slot_used[motor->tx_slot] = 1U;
    }

    *hcan_out = hcan;
    *std_id_out = std_id;
    *has_data_out = has_data;
    return HAL_OK;
}

/* 更新一个 GM6020 服务对象的反馈。 */
static void Motor_UpdateGm6020Feedback(gm6020_service_t *motor, const uint8_t data[8])
{
    motor->ecd = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];

    Motor_UpdateTotalEcd16(&motor->initialized,
                           &motor->last_ecd,
                           &motor->total_ecd,
                           motor->ecd,
                           GM6020_ENCODER_CPR);

    motor->angle_deg = ((float)motor->ecd * 360.0f) / (float)GM6020_ENCODER_CPR;
    motor->total_angle_deg = ((float)motor->total_ecd * 360.0f) / (float)GM6020_ENCODER_CPR;
    motor->online = 1U;
    motor->last_update_tick = HAL_GetTick();
    motor->frame_count++;
}

/* 更新一个 M3508 服务对象的反馈。 */
static void Motor_UpdateM3508Feedback(m3508_service_t *motor, const uint8_t data[8])
{
    motor->ecd = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];

    Motor_UpdateTotalEcd16(&motor->initialized,
                           &motor->last_ecd,
                           &motor->total_ecd,
                           motor->ecd,
                           M3508_ENCODER_CPR);

    motor->angle_deg = ((float)motor->ecd * 360.0f) / (float)M3508_ENCODER_CPR;
    motor->total_angle_deg = ((float)motor->total_ecd * 360.0f) / (float)M3508_ENCODER_CPR;
    motor->online = 1U;
    motor->last_update_tick = HAL_GetTick();
    motor->frame_count++;
}

/* 用 LK 编码器值刷新单圈角和累计角。 */
static void Motor_UpdateLkAngleByEncoder(lk_motor_service_t *motor, uint16_t encoder)
{
    motor->encoder = encoder;
    Motor_UpdateTotalEcd32(&motor->initialized,
                           &motor->last_ecd,
                           &motor->total_ecd,
                           encoder,
                           LK_MOTOR_ENCODER_CPR);
    motor->circle_angle_deg = ((float)encoder * 360.0f) / (float)LK_MOTOR_ENCODER_CPR;
    motor->total_angle_deg = ((float)motor->total_ecd * 360.0f) / (float)LK_MOTOR_ENCODER_CPR;
}

/* 更新一个 LK 电机服务对象的反馈。 */
static void Motor_UpdateLkFeedback(lk_motor_service_t *motor, const uint8_t data[8])
{
    uint64_t raw_multi_angle;
    lk_motor_cmd_t cmd;

    cmd = (lk_motor_cmd_t)data[0];
    motor->last_rx_cmd = cmd;

    switch (cmd)
    {
        case LK_MOTOR_CMD_READ_PID:
            motor->angle_kp = data[2];
            motor->angle_ki = data[3];
            motor->speed_kp = data[4];
            motor->speed_ki = data[5];
            motor->iq_kp = data[6];
            motor->iq_ki = data[7];
            break;

        case LK_MOTOR_CMD_READ_ACC:
            motor->accel = (int32_t)((uint32_t)data[4]
                                   | ((uint32_t)data[5] << 8)
                                   | ((uint32_t)data[6] << 16)
                                   | ((uint32_t)data[7] << 24));
            break;

        case LK_MOTOR_CMD_READ_ENC:
            motor->encoder = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
            motor->encoder_raw = (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            motor->encoder_offset = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
            Motor_UpdateLkAngleByEncoder(motor, motor->encoder);
            break;

        case LK_MOTOR_CMD_READ_MULTI_ANGLE:
            raw_multi_angle = (uint64_t)data[1]
                            | ((uint64_t)data[2] << 8)
                            | ((uint64_t)data[3] << 16)
                            | ((uint64_t)data[4] << 24)
                            | ((uint64_t)data[5] << 32)
                            | ((uint64_t)data[6] << 40)
                            | ((uint64_t)data[7] << 48);
            motor->reply_multi_angle_deg = (float)raw_multi_angle * 0.01f;
            break;

        case LK_MOTOR_CMD_READ_SINGLE_ANGLE:
            motor->reply_single_angle_deg = (float)((uint32_t)data[4]
                                                 | ((uint32_t)data[5] << 8)
                                                 | ((uint32_t)data[6] << 16)
                                                 | ((uint32_t)data[7] << 24)) * 0.01f;
            break;

        case LK_MOTOR_CMD_READ_STATE_1:
            motor->temperature_c = (int8_t)data[1];
            motor->voltage_v = (float)((uint16_t)data[3] | ((uint16_t)data[4] << 8)) * 0.1f;
            motor->error_state = data[7];
            motor->voltage_low = (uint8_t)((data[7] & 0x01U) != 0U);
            motor->over_temp = (uint8_t)(((data[7] >> 3) & 0x01U) != 0U);
            break;

        case LK_MOTOR_CMD_READ_STATE_2:
        case LK_MOTOR_CMD_POWER_REPLY:
        case LK_MOTOR_CMD_CONTROL_TORQUE:
        case LK_MOTOR_CMD_CONTROL_SPEED:
        case LK_MOTOR_CMD_CONTROL_POS_1:
        case LK_MOTOR_CMD_CONTROL_POS_2:
        case LK_MOTOR_CMD_CONTROL_POS_3:
        case LK_MOTOR_CMD_CONTROL_POS_4:
        case LK_MOTOR_CMD_CONTROL_POS_5:
        case LK_MOTOR_CMD_CONTROL_POS_6:
            motor->temperature_c = (int8_t)data[1];

            if (cmd == LK_MOTOR_CMD_POWER_REPLY)
            {
                motor->power_w = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
            }
            else
            {
                motor->torque_nm = (float)((int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8))) * 0.0025f;
            }

            motor->speed_dps = (float)((int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8))) / 8.0f;
            Motor_UpdateLkAngleByEncoder(motor,
                                         (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8)));
            break;

        case LK_MOTOR_CMD_READ_STATE_3:
            motor->temperature_c = (int8_t)data[1];
            break;

        default:
            break;
    }

    motor->online = 1U;
    motor->last_update_tick = HAL_GetTick();
    motor->frame_count++;
}

/* 发送 LK 电机的简易 8 字节命令。 */
static HAL_StatusTypeDef Motor_LkSendCommand(lk_motor_service_t *motor, lk_motor_cmd_t cmd, const uint8_t payload[8])
{
    HAL_StatusTypeDef status;

    if ((motor == NULL) || (motor->registered == 0U) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    status = Motor_SendRawFrame(motor->hcan, motor->rx_std_id, payload, 8U);
    if (status == HAL_OK)
    {
        motor->last_tx_cmd = cmd;
    }

    return status;
}

void Motor_Init(void)
{
    memset(gm6020_registry, 0, sizeof(gm6020_registry));
    memset(m3508_registry, 0, sizeof(m3508_registry));
    memset(lk_registry, 0, sizeof(lk_registry));
}

HAL_StatusTypeDef Motor_RegisterGm6020CurrentLoop(gm6020_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U) || (motor_id > 7U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->registered = 1U;
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->rx_std_id = Motor_Gm6020RxStdId(motor_id);
    motor->tx_std_id = Motor_Gm6020TxStdId(motor_id);
    motor->tx_slot = Motor_Gm6020TxSlot(motor_id);
    motor->mode = GM6020_MODE_CURRENT_LOOP;
    return Motor_RegisterGm6020ToTable(motor);
}

HAL_StatusTypeDef Motor_RegisterGm6020VoltageLoop(gm6020_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U) || (motor_id > 7U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->registered = 1U;
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->rx_std_id = Motor_Gm6020RxStdId(motor_id);
    motor->tx_std_id = Motor_Gm6020TxStdId(motor_id);
    motor->tx_slot = Motor_Gm6020TxSlot(motor_id);
    motor->mode = GM6020_MODE_VOLTAGE_LOOP;
    return Motor_RegisterGm6020ToTable(motor);
}

HAL_StatusTypeDef Motor_RegisterM3508CurrentLoop(m3508_service_t *motor,
                                                 CAN_HandleTypeDef *hcan,
                                                 uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->registered = 1U;
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->rx_std_id = Motor_M3508RxStdId(motor_id);
    motor->tx_std_id = Motor_M3508TxStdId(motor_id);
    motor->tx_slot = Motor_M3508TxSlot(motor_id);
    return Motor_RegisterM3508ToTable(motor);
}

HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                   CAN_HandleTypeDef *hcan,
                                   uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->registered = 1U;
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->rx_std_id = Motor_LkRxStdId(motor_id);
    motor->last_tx_cmd = LK_MOTOR_CMD_NONE;
    motor->last_rx_cmd = LK_MOTOR_CMD_NONE;
    return Motor_RegisterLkToTable(motor);
}

void Motor_SetGm6020CurrentLoopOutput(gm6020_service_t *motor, int16_t output)
{
    if ((motor == NULL) || (motor->mode != GM6020_MODE_CURRENT_LOOP))
    {
        return;
    }

    motor->target_output = output;
}

void Motor_SetGm6020VoltageLoopOutput(gm6020_service_t *motor, int16_t output)
{
    if ((motor == NULL) || (motor->mode != GM6020_MODE_VOLTAGE_LOOP))
    {
        return;
    }

    motor->target_output = output;
}

void Motor_SetM3508CurrentLoopOutput(m3508_service_t *motor, int16_t output)
{
    if (motor == NULL)
    {
        return;
    }

    motor->target_output = output;
}

void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t iq)
{
    if (motor == NULL)
    {
        return;
    }

    motor->target_iq = Motor_LimitS16(iq, 1000);
}

HAL_StatusTypeDef Motor_SendGm6020CurrentLoopFrame(const gm6020_service_t *motor_a,
                                                   const gm6020_service_t *motor_b,
                                                   const gm6020_service_t *motor_c,
                                                   const gm6020_service_t *motor_d)
{
    CAN_HandleTypeDef *hcan;
    uint16_t std_id;
    int16_t slot_output[4];
    uint8_t has_data;
    HAL_StatusTypeDef status;

    status = Motor_BuildGm6020Frame(GM6020_MODE_CURRENT_LOOP,
                                    motor_a,
                                    motor_b,
                                    motor_c,
                                    motor_d,
                                    &hcan,
                                    &std_id,
                                    slot_output,
                                    &has_data);
    if ((status != HAL_OK) || (has_data == 0U))
    {
        return status;
    }

    return Motor_SendDjiGroupFrame(hcan, std_id, slot_output);
}

HAL_StatusTypeDef Motor_SendGm6020VoltageLoopFrame(const gm6020_service_t *motor_a,
                                                   const gm6020_service_t *motor_b,
                                                   const gm6020_service_t *motor_c,
                                                   const gm6020_service_t *motor_d)
{
    CAN_HandleTypeDef *hcan;
    uint16_t std_id;
    int16_t slot_output[4];
    uint8_t has_data;
    HAL_StatusTypeDef status;

    status = Motor_BuildGm6020Frame(GM6020_MODE_VOLTAGE_LOOP,
                                    motor_a,
                                    motor_b,
                                    motor_c,
                                    motor_d,
                                    &hcan,
                                    &std_id,
                                    slot_output,
                                    &has_data);
    if ((status != HAL_OK) || (has_data == 0U))
    {
        return status;
    }

    return Motor_SendDjiGroupFrame(hcan, std_id, slot_output);
}

HAL_StatusTypeDef Motor_SendM3508CurrentLoopFrame(const m3508_service_t *motor_a,
                                                  const m3508_service_t *motor_b,
                                                  const m3508_service_t *motor_c,
                                                  const m3508_service_t *motor_d)
{
    CAN_HandleTypeDef *hcan;
    uint16_t std_id;
    int16_t slot_output[4];
    uint8_t has_data;
    HAL_StatusTypeDef status;

    status = Motor_BuildM3508Frame(motor_a,
                                   motor_b,
                                   motor_c,
                                   motor_d,
                                   &hcan,
                                   &std_id,
                                   slot_output,
                                   &has_data);
    if ((status != HAL_OK) || (has_data == 0U))
    {
        return status;
    }

    return Motor_SendDjiGroupFrame(hcan, std_id, slot_output);
}

HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_MOTOR_ON;
    if (Motor_LkSendCommand(motor, LK_MOTOR_CMD_MOTOR_ON, data) == HAL_OK)
    {
        motor->output_enabled = 1U;
        return HAL_OK;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef Motor_LkSendPowerOff(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_MOTOR_OFF;
    if (Motor_LkSendCommand(motor, LK_MOTOR_CMD_MOTOR_OFF, data) == HAL_OK)
    {
        motor->output_enabled = 0U;
        return HAL_OK;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_MOTOR_STOP;
    if (Motor_LkSendCommand(motor, LK_MOTOR_CMD_MOTOR_STOP, data) == HAL_OK)
    {
        motor->output_enabled = 0U;
        return HAL_OK;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef Motor_LkSendClearError(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_CLEAR_ERROR;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_CLEAR_ERROR, data);
}

HAL_StatusTypeDef Motor_LkSendReadEncoderRequest(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_READ_ENC;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_READ_ENC, data);
}

HAL_StatusTypeDef Motor_LkSendReadMultiAngleRequest(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_READ_MULTI_ANGLE;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_READ_MULTI_ANGLE, data);
}

HAL_StatusTypeDef Motor_LkSendReadSingleAngleRequest(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_READ_SINGLE_ANGLE;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_READ_SINGLE_ANGLE, data);
}

HAL_StatusTypeDef Motor_LkSendReadState1Request(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_READ_STATE_1;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_READ_STATE_1, data);
}

HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_READ_STATE_2;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_READ_STATE_2, data);
}

HAL_StatusTypeDef Motor_LkSendReadState3Request(lk_motor_service_t *motor)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)LK_MOTOR_CMD_READ_STATE_3;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_READ_STATE_3, data);
}

HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor)
{
    return Motor_LkSendIqControlValue(motor, motor->target_iq);
}

HAL_StatusTypeDef Motor_LkSendIqControlValue(lk_motor_service_t *motor, int16_t iq)
{
    uint8_t data[8] = {0};
    int16_t limited_iq;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    limited_iq = Motor_LimitS16(iq, 1000);
    motor->target_iq = limited_iq;
    data[0] = (uint8_t)LK_MOTOR_CMD_CONTROL_TORQUE;
    data[4] = (uint8_t)((uint16_t)limited_iq & 0xFFU);
    data[5] = (uint8_t)(((uint16_t)limited_iq >> 8) & 0xFFU);
    motor->output_enabled = 1U;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_CONTROL_TORQUE, data);
}

HAL_StatusTypeDef Motor_LkSendSpeedControl(lk_motor_service_t *motor, float speed_dps)
{
    uint8_t data[8] = {0};
    int32_t speed_cmd;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->target_speed_dps = speed_dps;
    speed_cmd = (int32_t)(speed_dps * 100.0f);
    data[0] = (uint8_t)LK_MOTOR_CMD_CONTROL_SPEED;
    data[4] = (uint8_t)((uint32_t)speed_cmd & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)speed_cmd >> 8) & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)speed_cmd >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)speed_cmd >> 24) & 0xFFU);
    motor->output_enabled = 1U;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_CONTROL_SPEED, data);
}

HAL_StatusTypeDef Motor_LkSendAngleControl(lk_motor_service_t *motor, float angle_deg)
{
    uint8_t data[8] = {0};
    int32_t angle_cmd;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->target_angle_deg = angle_deg;
    angle_cmd = (int32_t)(angle_deg * 100.0f);
    data[0] = (uint8_t)LK_MOTOR_CMD_CONTROL_POS_1;
    data[4] = (uint8_t)((uint32_t)angle_cmd & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)angle_cmd >> 8) & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)angle_cmd >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)angle_cmd >> 24) & 0xFFU);
    motor->output_enabled = 1U;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_CONTROL_POS_1, data);
}

HAL_StatusTypeDef Motor_LkSendAngleSpeedLimitedControl(lk_motor_service_t *motor,
                                                       uint16_t max_speed_dps,
                                                       float angle_deg)
{
    uint8_t data[8] = {0};
    int32_t angle_cmd;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->target_max_speed_dps = max_speed_dps;
    motor->target_angle_deg = angle_deg;
    angle_cmd = (int32_t)(angle_deg * 100.0f);
    data[0] = (uint8_t)LK_MOTOR_CMD_CONTROL_POS_2;
    data[2] = (uint8_t)(max_speed_dps & 0xFFU);
    data[3] = (uint8_t)((max_speed_dps >> 8) & 0xFFU);
    data[4] = (uint8_t)((uint32_t)angle_cmd & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)angle_cmd >> 8) & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)angle_cmd >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)angle_cmd >> 24) & 0xFFU);
    motor->output_enabled = 1U;
    return Motor_LkSendCommand(motor, LK_MOTOR_CMD_CONTROL_POS_2, data);
}

void Motor_ProcessCanMessage(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *header,
                             const uint8_t data[8])
{
    uint8_t i;

    if ((hcan == NULL) || (header == NULL) || (data == NULL) || (header->IDE != CAN_ID_STD))
    {
        return;
    }

    for (i = 0U; i < GM6020_REGISTRY_SIZE; ++i)
    {
        if ((gm6020_registry[i] != NULL)
         && (gm6020_registry[i]->registered != 0U)
         && (gm6020_registry[i]->hcan == hcan)
         && (gm6020_registry[i]->rx_std_id == header->StdId))
        {
            Motor_UpdateGm6020Feedback(gm6020_registry[i], data);
            return;
        }
    }

    for (i = 0U; i < M3508_REGISTRY_SIZE; ++i)
    {
        if ((m3508_registry[i] != NULL)
         && (m3508_registry[i]->registered != 0U)
         && (m3508_registry[i]->hcan == hcan)
         && (m3508_registry[i]->rx_std_id == header->StdId))
        {
            Motor_UpdateM3508Feedback(m3508_registry[i], data);
            return;
        }
    }

    for (i = 0U; i < LK_MOTOR_REGISTRY_SIZE; ++i)
    {
        if ((lk_registry[i] != NULL)
         && (lk_registry[i]->registered != 0U)
         && (lk_registry[i]->hcan == hcan)
         && (lk_registry[i]->rx_std_id == header->StdId))
        {
            Motor_UpdateLkFeedback(lk_registry[i], data);
            return;
        }
    }
}

uint8_t Motor_Gm6020IsOnline(gm6020_service_t *motor)
{
    if (motor == NULL)
    {
        return 0U;
    }

    motor->online = (uint8_t)((motor->frame_count > 0U)
                           && ((HAL_GetTick() - motor->last_update_tick) < GM6020_ONLINE_TIMEOUT_MS));
    return motor->online;
}

uint8_t Motor_M3508IsOnline(m3508_service_t *motor)
{
    if (motor == NULL)
    {
        return 0U;
    }

    motor->online = (uint8_t)((motor->frame_count > 0U)
                           && ((HAL_GetTick() - motor->last_update_tick) < M3508_ONLINE_TIMEOUT_MS));
    return motor->online;
}

uint8_t Motor_LkIsOnline(lk_motor_service_t *motor)
{
    if (motor == NULL)
    {
        return 0U;
    }

    motor->online = (uint8_t)((motor->frame_count > 0U)
                           && ((HAL_GetTick() - motor->last_update_tick) < LK_MOTOR_ONLINE_TIMEOUT_MS));
    return motor->online;
}


