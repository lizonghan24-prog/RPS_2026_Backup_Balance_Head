#include "motor.h"

#include <string.h>

/* Motor communication service: register, parse feedback, pack/send control frames. */

/*
 * 电机服务层职责说明：
 * 1. 上层任务注册“某个电机对象在某一路 CAN、某个 ID 上”；
 * 2. BSP 收到 CAN 帧后调用 Motor_ProcessCanMessage()；
 * 3. 本文件按 hcan + StdId 找到对象，刷新反馈、在线标志和时间戳；
 * 4. 上层任务写 output 后，调用发送函数把同组电机一次性打包发出。
 *
 * DM4310 和 MG4310/LK 的协议细节分别在 motor_dm.c 和 motor_lk.c 中维护。
 *
 * 注意：本文件不负责 CAN 外设初始化，也不负责闭环 PID 计算。
 */

/*
 * DJI 常用标准帧 ID：
 * - M3508/M2006 反馈：0x201 ~ 0x208，对应电机 ID1 ~ ID8；
 * - M3508/M2006 控制：ID1~ID4 发 0x200，ID5~ID8 发 0x1FF；
 * - GM6020 反馈：0x205 ~ 0x20B，对应电机 ID1 ~ ID7；
 * - GM6020 电流控制：ID1~ID4 发 0x1FE，ID5~ID7 发 0x2FE；
 * - GM6020 电压控制：ID1~ID4 发 0x1FF，ID5~ID7 发 0x2FF。
 */
#define MOTOR_DJI_M3508_FEEDBACK_BASE           0x200U
#define MOTOR_DJI_GM6020_FEEDBACK_BASE          0x204U
#define MOTOR_DJI_M3508_CMD_LOW                 0x200U
#define MOTOR_DJI_M3508_CMD_HIGH                0x1FFU
#define MOTOR_DJI_GM6020_CURRENT_CMD_LOW        0x1FEU
#define MOTOR_DJI_GM6020_CURRENT_CMD_HIGH       0x2FEU
#define MOTOR_DJI_GM6020_VOLTAGE_CMD_LOW        0x1FFU
#define MOTOR_DJI_GM6020_VOLTAGE_CMD_HIGH       0x2FFU

#define MOTOR_MAX_GM6020_COUNT                  8U      /* GM6020 注册表容量，足够覆盖一条 CAN 上的常用 ID。 */
#define MOTOR_MAX_M3508_COUNT                   8U      /* M3508/M2006 注册表容量，对应 DJI ID1~ID8。 */

static gm6020_service_t *gm6020_registry[MOTOR_MAX_GM6020_COUNT];
static m3508_service_t *m3508_registry[MOTOR_MAX_M3508_COUNT];

/*
 * 注册表只保存对象指针，不申请也不释放内存。
 * 因此传进 Register 函数的对象必须是全局、静态或长期存在的任务上下文成员。
 */

/* 把大端格式的两个字节还原成 int16_t，DJI 反馈和控制帧都使用这种顺序。 */
static int16_t Motor_ReadInt16BE(const uint8_t *data)
{
    return (int16_t)((uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* 把 int16_t 拆成高字节在前、低字节在后的格式。 */
static void Motor_WriteInt16BE(uint8_t *data, uint8_t slot, int16_t value)
{
    uint8_t index;

    index = (uint8_t)(slot * 2U);
    data[index] = (uint8_t)((uint16_t)value >> 8);
    data[index + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

/* HAL CAN 的标准数据帧发送封装，所有 motor 层发送最终都走这里。 */
static HAL_StatusTypeDef Motor_SendStdFrame(CAN_HandleTypeDef *hcan,
                                            uint32_t std_id,
                                            const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if ((hcan == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}


/*
 * 注册表不是按 ID 直接下标访问，而是按“CAN 句柄 + StdId”匹配。
 * 这样同一个电机 ID 可以同时出现在 CAN1 和 CAN2 上，互不冲突。
 */
static HAL_StatusTypeDef Motor_RegisterGm6020Object(gm6020_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_MAX_GM6020_COUNT; ++i)
    {
        if ((gm6020_registry[i] == motor)
         || ((gm6020_registry[i] != NULL)
          && (gm6020_registry[i]->hcan == motor->hcan)
          && (gm6020_registry[i]->feedback_std_id == motor->feedback_std_id)))
        {
            gm6020_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < MOTOR_MAX_GM6020_COUNT; ++i)
    {
        if (gm6020_registry[i] == NULL)
        {
            gm6020_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/*
 * 把 M3508/M2006 对象挂入注册表。
 * 允许重复注册同一个对象，或用新对象覆盖同一路 CAN + 同一反馈 ID 的旧条目。
 */
static HAL_StatusTypeDef Motor_RegisterM3508Object(m3508_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_MAX_M3508_COUNT; ++i)
    {
        if ((m3508_registry[i] == motor)
         || ((m3508_registry[i] != NULL)
          && (m3508_registry[i]->hcan == motor->hcan)
          && (m3508_registry[i]->feedback_std_id == motor->feedback_std_id)))
        {
            m3508_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < MOTOR_MAX_M3508_COUNT; ++i)
    {
        if (m3508_registry[i] == NULL)
        {
            m3508_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* 刷新在线时间戳。这个函数只说明“收到过合法帧”，具体解析由外层完成。 */
static void Motor_TouchOnline(volatile uint8_t *online, volatile uint32_t *last_update_tick)
{
    *last_update_tick = HAL_GetTick();
    *online = 1U;
}

/*
 * 判断某个时间戳是否仍然有效。
 * HAL_GetTick() 是 uint32_t，直接做无符号减法可以自然处理 tick 回绕。
 */
static uint8_t Motor_IsRecent(uint32_t last_update_tick)
{
    return (uint8_t)((HAL_GetTick() - last_update_tick) <= MOTOR_ONLINE_TIMEOUT_MS);
}

/*
 * DJI 编码器跨零处理：
 * - 原始值从 8191 跳到 0 附近，说明正向跨零，实际增量要加 8192；
 * - 原始值从 0 跳到 8191 附近，说明反向跨零，实际增量要减 8192。
 *
 * 这样可以把单圈编码器变成连续角度，方便上层做位置闭环或调试观察。
 */
static void Motor_UpdateDjiFeedback(dji_motor_feedback_t *feedback, const uint8_t data[8])
{
    uint16_t raw_ecd;
    int16_t delta;

    raw_ecd = (uint16_t)Motor_ReadInt16BE(&data[0]);

    if (feedback->initialized == 0U)
    {
        feedback->initialized = 1U;
        feedback->raw_ecd = raw_ecd;
        feedback->last_raw_ecd = raw_ecd;
        feedback->ecd_bias = raw_ecd;
        feedback->ecd_delta = 0;
        feedback->round_count = 0;
        feedback->total_ecd = 0;
    }
    else
    {
        feedback->last_raw_ecd = feedback->raw_ecd;
        feedback->raw_ecd = raw_ecd;

        delta = (int16_t)((int32_t)feedback->raw_ecd - (int32_t)feedback->last_raw_ecd);
        if (delta < -MOTOR_DJI_ENCODER_HALF_RANGE)
        {
            delta = (int16_t)(delta + (int16_t)MOTOR_DJI_ENCODER_RANGE);
            feedback->round_count++;
        }
        else if (delta > MOTOR_DJI_ENCODER_HALF_RANGE)
        {
            delta = (int16_t)(delta - (int16_t)MOTOR_DJI_ENCODER_RANGE);
            feedback->round_count--;
        }

        feedback->ecd_delta = delta;
        feedback->total_ecd += delta;
    }

    feedback->speed_rpm = Motor_ReadInt16BE(&data[2]);
    feedback->given_current = Motor_ReadInt16BE(&data[4]);
    feedback->temperature = data[6];
    feedback->angle_deg = ((float)((int32_t)feedback->raw_ecd - (int32_t)feedback->ecd_bias)
                         * MOTOR_DJI_ENCODER_DEG_PER_TICK)
                        + ((float)feedback->round_count * 360.0f);
    feedback->total_angle_deg = (float)feedback->total_ecd * MOTOR_DJI_ENCODER_DEG_PER_TICK;
    feedback->speed_dps = (float)feedback->speed_rpm * 6.0f;
}

/* 把通用 DJI 反馈同步到 GM6020 便捷字段，减少上层直接访问 feedback 子结构的麻烦。 */
static void Motor_CopyDjiFeedbackToGm6020(gm6020_service_t *motor)
{
    motor->speed_rpm = motor->feedback.speed_rpm;
    motor->given_current = motor->feedback.given_current;
    motor->temperature = motor->feedback.temperature;
    motor->angle_deg = motor->feedback.angle_deg;
    motor->total_angle_deg = motor->feedback.total_angle_deg;
    motor->speed_dps = motor->feedback.speed_dps;
}

/* 把通用 DJI 反馈同步到 M3508 便捷字段。 */
static void Motor_CopyDjiFeedbackToM3508(m3508_service_t *motor)
{
    motor->speed_rpm = motor->feedback.speed_rpm;
    motor->given_current = motor->feedback.given_current;
    motor->temperature = motor->feedback.temperature;
    motor->angle_deg = motor->feedback.angle_deg;
    motor->total_angle_deg = motor->feedback.total_angle_deg;
    motor->speed_dps = motor->feedback.speed_dps;
}

/*
 * 按电机 ID 自动决定 DJI 控制帧应该走低组还是高组。
 * 传入的四个指针不是强制代表 slot1~slot4，而是根据 motor_id 放到正确字节位。
 */
static HAL_StatusTypeDef Motor_SendGm6020FrameByMode(gm6020_control_mode_t mode,
                                                     uint32_t low_std_id,
                                                     uint32_t high_std_id,
                                                     const gm6020_service_t *motor1,
                                                     const gm6020_service_t *motor2,
                                                     const gm6020_service_t *motor3,
                                                     const gm6020_service_t *motor4)
{
    const gm6020_service_t *motors[4];
    CAN_HandleTypeDef *hcan;
    uint8_t tx_data[8];
    uint8_t i;
    uint8_t high_group;
    uint8_t have_motor;
    uint8_t slot;
    uint32_t std_id;

    motors[0] = motor1;
    motors[1] = motor2;
    motors[2] = motor3;
    motors[3] = motor4;
    hcan = NULL;
    high_group = 0U;
    have_motor = 0U;
    memset(tx_data, 0, sizeof(tx_data));

    for (i = 0U; i < 4U; ++i)
    {
        /* NULL 表示该控制槽位输出 0。 */
        if (motors[i] == NULL)
        {
            continue;
        }

        /* 保护检查：对象必须有效、ID 范围正确、控制模式匹配。 */
        if ((motors[i]->hcan == NULL)
         || (motors[i]->motor_id < 1U)
         || (motors[i]->motor_id > 8U)
         || (motors[i]->control_mode != mode))
        {
            return HAL_ERROR;
        }

        if (hcan == NULL)
        {
            /* 第一台有效电机决定本帧使用哪一路 CAN、低组还是高组。 */
            hcan = motors[i]->hcan;
            high_group = (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U);
        }
        else if ((hcan != motors[i]->hcan)
              || (high_group != (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U)))
        {
            /* 同一帧不能混发不同 CAN 或不同 ID 分组。 */
            return HAL_ERROR;
        }

        /* ID1/5 放 slot0，ID2/6 放 slot1，以此类推。 */
        slot = (uint8_t)((motors[i]->motor_id - 1U) % 4U);
        Motor_WriteInt16BE(tx_data, slot, motors[i]->output);
        have_motor = 1U;
    }

    if (have_motor == 0U)
    {
        return HAL_ERROR;
    }

    std_id = (high_group != 0U) ? high_std_id : low_std_id;
    return Motor_SendStdFrame(hcan, std_id, tx_data);
}

/* M3508/C620/C610 的分组规则与 GM6020 类似，但控制帧 ID 不同。 */
static HAL_StatusTypeDef Motor_SendM3508FrameById(const m3508_service_t *motor1,
                                                  const m3508_service_t *motor2,
                                                  const m3508_service_t *motor3,
                                                  const m3508_service_t *motor4)
{
    const m3508_service_t *motors[4];
    CAN_HandleTypeDef *hcan;
    uint8_t tx_data[8];
    uint8_t i;
    uint8_t high_group;
    uint8_t have_motor;
    uint8_t slot;
    uint32_t std_id;

    motors[0] = motor1;
    motors[1] = motor2;
    motors[2] = motor3;
    motors[3] = motor4;
    hcan = NULL;
    high_group = 0U;
    have_motor = 0U;
    memset(tx_data, 0, sizeof(tx_data));

    for (i = 0U; i < 4U; ++i)
    {
        /* NULL 表示该控制槽位输出 0。 */
        if (motors[i] == NULL)
        {
            continue;
        }

        /* M3508/M2006 的有效 ID 范围是 1~8。 */
        if ((motors[i]->hcan == NULL)
         || (motors[i]->motor_id < 1U)
         || (motors[i]->motor_id > 8U))
        {
            return HAL_ERROR;
        }

        if (hcan == NULL)
        {
            /* 第一台有效电机决定本帧使用哪一路 CAN、低组还是高组。 */
            hcan = motors[i]->hcan;
            high_group = (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U);
        }
        else if ((hcan != motors[i]->hcan)
              || (high_group != (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U)))
        {
            /* 同一帧不能混发不同 CAN 或不同 ID 分组。 */
            return HAL_ERROR;
        }

        /* ID1/5 放 slot0，ID2/6 放 slot1，以此类推。 */
        slot = (uint8_t)((motors[i]->motor_id - 1U) % 4U);
        Motor_WriteInt16BE(tx_data, slot, motors[i]->output);
        have_motor = 1U;
    }

    if (have_motor == 0U)
    {
        return HAL_ERROR;
    }

    std_id = (high_group != 0U) ? MOTOR_DJI_M3508_CMD_HIGH : MOTOR_DJI_M3508_CMD_LOW;
    return Motor_SendStdFrame(hcan, std_id, tx_data);
}

/*
 * 清空 DJI 电机注册表。
 * 该函数通常在 BSP 初始化阶段调用一次，之后由控制任务/发射任务重新注册电机。
 */
void Motor_Init(void)
{
    memset(gm6020_registry, 0, sizeof(gm6020_registry));
    memset(m3508_registry, 0, sizeof(m3508_registry));
}

/*
 * 注册 GM6020 电流环对象。
 * ID1 的反馈帧是 0x205，后续 ID 依次递增。
 */
HAL_StatusTypeDef Motor_RegisterGm6020CurrentLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id < 1U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->feedback_std_id = MOTOR_DJI_GM6020_FEEDBACK_BASE + (uint32_t)motor_id;
    motor->control_mode = MOTOR_GM6020_CONTROL_CURRENT;

    return Motor_RegisterGm6020Object(motor);
}

/*
 * 注册 GM6020 电压环对象。
 * 反馈解析与电流环一致，区别只在发送时使用电压控制帧 ID。
 */
HAL_StatusTypeDef Motor_RegisterGm6020VoltageLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id < 1U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->feedback_std_id = MOTOR_DJI_GM6020_FEEDBACK_BASE + (uint32_t)motor_id;
    motor->control_mode = MOTOR_GM6020_CONTROL_VOLTAGE;

    return Motor_RegisterGm6020Object(motor);
}

/* 缓存 GM6020 电流环输出，不立即发送 CAN。 */
void Motor_SetGm6020CurrentLoopOutput(gm6020_service_t *motor, int16_t output)
{
    if ((motor != NULL) && (motor->control_mode == MOTOR_GM6020_CONTROL_CURRENT))
    {
        motor->output = output;
    }
}

/* 缓存 GM6020 电压环输出，不立即发送 CAN。 */
void Motor_SetGm6020VoltageLoopOutput(gm6020_service_t *motor, int16_t output)
{
    if ((motor != NULL) && (motor->control_mode == MOTOR_GM6020_CONTROL_VOLTAGE))
    {
        motor->output = output;
    }
}

/*
 * 发送 GM6020 电流环控制帧。
 * 传入的电机必须属于同一路 CAN 和同一 ID 分组。
 */
HAL_StatusTypeDef Motor_SendGm6020CurrentLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4)
{
    return Motor_SendGm6020FrameByMode(MOTOR_GM6020_CONTROL_CURRENT,
                                       MOTOR_DJI_GM6020_CURRENT_CMD_LOW,
                                       MOTOR_DJI_GM6020_CURRENT_CMD_HIGH,
                                       motor1,
                                       motor2,
                                       motor3,
                                       motor4);
}

/*
 * 发送 GM6020 电压环控制帧。
 * 分组规则和电流环相同，只是控制 StdId 不同。
 */
HAL_StatusTypeDef Motor_SendGm6020VoltageLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4)
{
    return Motor_SendGm6020FrameByMode(MOTOR_GM6020_CONTROL_VOLTAGE,
                                       MOTOR_DJI_GM6020_VOLTAGE_CMD_LOW,
                                       MOTOR_DJI_GM6020_VOLTAGE_CMD_HIGH,
                                       motor1,
                                       motor2,
                                       motor3,
                                       motor4);
}

/* 返回 GM6020 在线状态，超过超时时间未刷新则返回 0。 */
uint8_t Motor_Gm6020IsOnline(const gm6020_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return Motor_IsRecent(motor->last_update_tick);
}

/*
 * 注册 M3508/M2006 电流环对象。
 * ID1 的反馈帧是 0x201，ID8 的反馈帧是 0x208。
 */
HAL_StatusTypeDef Motor_RegisterM3508CurrentLoop(m3508_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id < 1U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->feedback_std_id = MOTOR_DJI_M3508_FEEDBACK_BASE + (uint32_t)motor_id;

    return Motor_RegisterM3508Object(motor);
}

/* 缓存 M3508/M2006 电流环输出，不立即发送 CAN。 */
void Motor_SetM3508CurrentLoopOutput(m3508_service_t *motor, int16_t output)
{
    if (motor != NULL)
    {
        motor->output = output;
    }
}

/* 发送一帧 M3508/M2006 电流环控制帧。 */
HAL_StatusTypeDef Motor_SendM3508CurrentLoopFrame(const m3508_service_t *motor1,
                                                  const m3508_service_t *motor2,
                                                  const m3508_service_t *motor3,
                                                  const m3508_service_t *motor4)
{
    return Motor_SendM3508FrameById(motor1, motor2, motor3, motor4);
}

/* 返回 M3508/M2006 在线状态。 */
uint8_t Motor_M3508IsOnline(const m3508_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return Motor_IsRecent(motor->last_update_tick);
}

/*
 * 处理并分发一帧 CAN 接收数据。
 * BSP 的 CAN FIFO 回调会把 HAL 读出的 rx_header/rx_data 传进来。
 */
void Motor_ProcessCanMessage(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *rx_header,
                             const uint8_t rx_data[8])
{
    uint8_t i;

    if ((hcan == NULL)
     || (rx_header == NULL)
     || (rx_data == NULL)
     || (rx_header->IDE != CAN_ID_STD)
     || (rx_header->RTR != CAN_RTR_DATA)
     || (rx_header->DLC < 7U))
    {
        return;
    }

    for (i = 0U; i < MOTOR_MAX_GM6020_COUNT; ++i)
    {
        if ((gm6020_registry[i] != NULL)
         && (gm6020_registry[i]->hcan == hcan)
         && (gm6020_registry[i]->feedback_std_id == rx_header->StdId))
        {
            Motor_UpdateDjiFeedback(&gm6020_registry[i]->feedback, rx_data);
            Motor_CopyDjiFeedbackToGm6020(gm6020_registry[i]);
            Motor_TouchOnline(&gm6020_registry[i]->online, &gm6020_registry[i]->last_update_tick);
            return;
        }
    }

    for (i = 0U; i < MOTOR_MAX_M3508_COUNT; ++i)
    {
        if ((m3508_registry[i] != NULL)
         && (m3508_registry[i]->hcan == hcan)
         && (m3508_registry[i]->feedback_std_id == rx_header->StdId))
        {
            Motor_UpdateDjiFeedback(&m3508_registry[i]->feedback, rx_data);
            Motor_CopyDjiFeedbackToM3508(m3508_registry[i]);
            Motor_TouchOnline(&m3508_registry[i]->online, &m3508_registry[i]->last_update_tick);
            return;
        }
    }

}

