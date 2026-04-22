#include "motor.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*
 * 电机服务层职责说明：
 * 1. 上层任务注册“某个电机对象在某一路 CAN、某个 ID 上”；
 * 2. BSP 收到 CAN 帧后调用 Motor_ProcessCanMessage()；
 * 3. 本文件按 hcan + StdId 找到对象，刷新反馈、在线标志和时间戳；
 * 4. 上层任务写 output 后，调用发送函数把同组电机一次性打包发出。
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

/*
 * LK / RMD 常用标准帧 ID 和命令字。
 * 电机 ID5 对应 StdId 0x145，也就是 0x140 + 5。
 */
#define MOTOR_LK_STD_ID_BASE                    0x140U
#define MOTOR_LK_CMD_STOP                       0x81U
#define MOTOR_LK_CMD_POWER_ON                   0x88U
#define MOTOR_LK_CMD_READ_STATE_2               0x9CU
#define MOTOR_LK_CMD_IQ_CONTROL                 0xA1U

#define MOTOR_MAX_GM6020_COUNT                  8U      /* GM6020 注册表容量，足够覆盖一条 CAN 上的常用 ID。 */
#define MOTOR_MAX_M3508_COUNT                   8U      /* M3508/M2006 注册表容量，对应 DJI ID1~ID8。 */
#define MOTOR_MAX_LK_COUNT                      8U      /* LK 注册表容量，当前拨盘只用 1 个，预留扩展。 */

static gm6020_service_t *gm6020_registry[MOTOR_MAX_GM6020_COUNT];
static m3508_service_t *m3508_registry[MOTOR_MAX_M3508_COUNT];
static lk_motor_service_t *lk_registry[MOTOR_MAX_LK_COUNT];

/*
 * 注册表只保存对象指针，不申请也不释放内存。
 * 因此传进 Register 函数的对象必须是全局、静态或长期存在的任务上下文成员。
 */

/* 把大端格式的两个字节还原成 int16_t，DJI 反馈和控制帧都使用这种顺序。 */
static int16_t Motor_ReadInt16BE(const uint8_t *data)
{
    return (int16_t)((uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* 把小端格式的两个字节还原成 int16_t，LK 状态帧使用这种顺序。 */
static int16_t Motor_ReadInt16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* 把小端格式的两个字节还原成 uint16_t，用来解析 LK 单圈编码器值。 */
static uint16_t Motor_ReadUint16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* 把 int16_t 拆成高字节在前、低字节在后的格式。 */
static void Motor_WriteInt16BE(uint8_t *data, uint8_t slot, int16_t value)
{
    uint8_t index;

    index = (uint8_t)(slot * 2U);
    data[index] = (uint8_t)((uint16_t)value >> 8);
    data[index + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

/* LK 控制命令里 iq 给定值放在 data[4]/data[5]，采用小端格式。 */
static void Motor_WriteInt16LE(uint8_t *data, uint8_t index, int16_t value)
{
    data[index] = (uint8_t)((uint16_t)value & 0xFFU);
    data[index + 1U] = (uint8_t)((uint16_t)value >> 8);
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
 * 旧标准库接口传入的是 CAN1 / CAN2 外设基地址。
 * HAL 发送函数需要 CAN_HandleTypeDef，这里把外设基地址映射回 CubeMX 生成的句柄。
 */
static CAN_HandleTypeDef *Motor_ResolveCanHandle(CAN_TypeDef *CANx)
{
    if (CANx == CAN1)
    {
        return &hcan1;
    }

    if (CANx == CAN2)
    {
        return &hcan2;
    }

    return NULL;
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

/*
 * 把 LK 对象挂入注册表。
 * LK 协议收发共用 0x140 + ID，所以匹配键使用 hcan + std_id。
 */
static HAL_StatusTypeDef Motor_RegisterLkObject(lk_motor_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_MAX_LK_COUNT; ++i)
    {
        if ((lk_registry[i] == motor)
         || ((lk_registry[i] != NULL)
          && (lk_registry[i]->hcan == motor->hcan)
          && (lk_registry[i]->std_id == motor->std_id)))
        {
            lk_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < MOTOR_MAX_LK_COUNT; ++i)
    {
        if (lk_registry[i] == NULL)
        {
            lk_registry[i] = motor;
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
 * LK 状态 2 反馈解析：
 * data[0] = 0x9C 或力矩控制返回的 0xA1；
 * data[1] = 温度；
 * data[2..3] = iq/力矩电流，小端；
 * data[4..5] = 速度，小端；
 * data[6..7] = 单圈编码器，小端。
 */
static void Motor_UpdateLkState2(lk_motor_service_t *motor, const uint8_t data[8])
{
    uint16_t raw_ecd;
    int32_t delta;

    raw_ecd = Motor_ReadUint16LE(&data[6]);

    if (motor->initialized == 0U)
    {
        motor->initialized = 1U;
        motor->raw_ecd = raw_ecd;
        motor->last_raw_ecd = raw_ecd;
        motor->ecd_bias = raw_ecd;
        motor->ecd_delta = 0;
        motor->round_count = 0;
        motor->total_ecd = 0;
    }
    else
    {
        motor->last_raw_ecd = motor->raw_ecd;
        motor->raw_ecd = raw_ecd;

        delta = (int32_t)motor->raw_ecd - (int32_t)motor->last_raw_ecd;
        if (delta < -MOTOR_LK_ENCODER_HALF_RANGE)
        {
            delta += (int32_t)MOTOR_LK_ENCODER_RANGE;
            motor->round_count++;
        }
        else if (delta > MOTOR_LK_ENCODER_HALF_RANGE)
        {
            delta -= (int32_t)MOTOR_LK_ENCODER_RANGE;
            motor->round_count--;
        }

        motor->ecd_delta = (int16_t)delta;
        motor->total_ecd += delta;
    }

    motor->temperature = data[1];
    motor->iq_feedback = Motor_ReadInt16LE(&data[2]);
    motor->speed_dps = (float)Motor_ReadInt16LE(&data[4]);
    motor->angle_deg = ((float)((int32_t)motor->raw_ecd - (int32_t)motor->ecd_bias)
                      * MOTOR_LK_ENCODER_DEG_PER_TICK)
                     + ((float)motor->round_count * 360.0f);
    motor->total_angle_deg = (float)motor->total_ecd * MOTOR_LK_ENCODER_DEG_PER_TICK;
}

/* DJI 四电机控制帧的原始发送函数，旧 Set_* 包装也会复用它。 */
static HAL_StatusTypeDef Motor_SendDjiRawFrame(CAN_HandleTypeDef *hcan,
                                               uint32_t std_id,
                                               int16_t output1,
                                               int16_t output2,
                                               int16_t output3,
                                               int16_t output4)
{
    uint8_t tx_data[8];

    memset(tx_data, 0, sizeof(tx_data));
    Motor_WriteInt16BE(tx_data, 0U, output1);
    Motor_WriteInt16BE(tx_data, 1U, output2);
    Motor_WriteInt16BE(tx_data, 2U, output3);
    Motor_WriteInt16BE(tx_data, 3U, output4);

    return Motor_SendStdFrame(hcan, std_id, tx_data);
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
 * 清空三类电机注册表。
 * 该函数通常在 BSP 初始化阶段调用一次，之后由控制任务/发射任务重新注册电机。
 */
void Motor_Init(void)
{
    memset(gm6020_registry, 0, sizeof(gm6020_registry));
    memset(m3508_registry, 0, sizeof(m3508_registry));
    memset(lk_registry, 0, sizeof(lk_registry));
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
 * 注册 LK/RMD 电机。
 * 标准帧 ID = 0x140 + motor_id，例如 ID5 对应 0x145。
 */
HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                    CAN_HandleTypeDef *hcan,
                                    uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U) || (motor_id > 0x7FU))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->std_id = MOTOR_LK_STD_ID_BASE + (uint32_t)motor_id;

    return Motor_RegisterLkObject(motor);
}

/* 缓存 LK iq/力矩输出，不立即发送 CAN。 */
void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t output)
{
    if (motor != NULL)
    {
        motor->iq_output = output;
    }
}

/* 发送 LK 上电/使能命令 0x88。 */
HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_POWER_ON;
    status = Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

/* 发送 LK 停止命令 0x81。 */
HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_STOP;
    status = Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
    if (status == HAL_OK)
    {
        motor->output_enabled = 0U;
    }

    return status;
}

/* 发送 LK 状态 2 查询命令 0x9C，用于刷新温度、速度、电流和编码器。 */
HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor)
{
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_READ_STATE_2;

    return Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
}

/* 发送 LK iq/力矩控制命令 0xA1。 */
HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_IQ_CONTROL;
    Motor_WriteInt16LE(tx_data, 4U, motor->iq_output);

    status = Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

/* 返回 LK 在线状态。 */
uint8_t Motor_LkIsOnline(const lk_motor_service_t *motor)
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

    if (rx_header->DLC < 8U)
    {
        return;
    }

    for (i = 0U; i < MOTOR_MAX_LK_COUNT; ++i)
    {
        if ((lk_registry[i] != NULL)
         && (lk_registry[i]->hcan == hcan)
         && (lk_registry[i]->std_id == rx_header->StdId))
        {
            if ((rx_data[0] == MOTOR_LK_CMD_READ_STATE_2)
             || (rx_data[0] == MOTOR_LK_CMD_IQ_CONTROL))
            {
                Motor_UpdateLkState2(lk_registry[i], rx_data);
            }
            else if (rx_data[0] == MOTOR_LK_CMD_POWER_ON)
            {
                lk_registry[i]->output_enabled = 1U;
            }
            else if (rx_data[0] == MOTOR_LK_CMD_STOP)
            {
                lk_registry[i]->output_enabled = 0U;
            }

            Motor_TouchOnline(&lk_registry[i]->online, &lk_registry[i]->last_update_tick);
            return;
        }
    }
}

void Set_GM6020_IQ1(CAN_TypeDef *CANx,
                    int16_t motor1_iq,
                    int16_t motor2_iq,
                    int16_t motor3_iq,
                    int16_t motor4_iq)
{
    /* 兼容旧接口：GM6020 ID1~ID4 电压控制帧，StdId = 0x1FF。 */
    (void)Motor_SendDjiRawFrame(Motor_ResolveCanHandle(CANx),
                                MOTOR_DJI_GM6020_VOLTAGE_CMD_LOW,
                                motor1_iq,
                                motor2_iq,
                                motor3_iq,
                                motor4_iq);
}

void Set_GM6020_IQ2(CAN_TypeDef *CANx,
                    int16_t motor5_iq,
                    int16_t motor6_iq,
                    int16_t motor7_iq,
                    int16_t motor8_iq)
{
    /* 兼容旧接口：GM6020 ID5~ID8 电压控制帧，StdId = 0x2FF。 */
    (void)Motor_SendDjiRawFrame(Motor_ResolveCanHandle(CANx),
                                MOTOR_DJI_GM6020_VOLTAGE_CMD_HIGH,
                                motor5_iq,
                                motor6_iq,
                                motor7_iq,
                                motor8_iq);
}

void Set_GM6020_Current_IQ1(CAN_TypeDef *CANx,
                            int16_t motor1_iq,
                            int16_t motor2_iq,
                            int16_t motor3_iq,
                            int16_t motor4_iq)
{
    /* 兼容旧接口：GM6020 ID1~ID4 电流控制帧，StdId = 0x1FE。 */
    (void)Motor_SendDjiRawFrame(Motor_ResolveCanHandle(CANx),
                                MOTOR_DJI_GM6020_CURRENT_CMD_LOW,
                                motor1_iq,
                                motor2_iq,
                                motor3_iq,
                                motor4_iq);
}

void Set_GM6020_Current_IQ2(CAN_TypeDef *CANx,
                            int16_t motor5_iq,
                            int16_t motor6_iq,
                            int16_t motor7_iq,
                            int16_t motor8_iq)
{
    /* 兼容旧接口：GM6020 ID5~ID8 电流控制帧，StdId = 0x2FE。 */
    (void)Motor_SendDjiRawFrame(Motor_ResolveCanHandle(CANx),
                                MOTOR_DJI_GM6020_CURRENT_CMD_HIGH,
                                motor5_iq,
                                motor6_iq,
                                motor7_iq,
                                motor8_iq);
}

void Set_C620andC610_IQ1(CAN_TypeDef *CANx,
                         int16_t motor1_iq,
                         int16_t motor2_iq,
                         int16_t motor3_iq,
                         int16_t motor4_iq)
{
    /* 兼容旧接口：C620/C610 ID1~ID4 电流控制帧，StdId = 0x200。 */
    (void)Motor_SendDjiRawFrame(Motor_ResolveCanHandle(CANx),
                                MOTOR_DJI_M3508_CMD_LOW,
                                motor1_iq,
                                motor2_iq,
                                motor3_iq,
                                motor4_iq);
}

void Set_C620andC610_IQ2(CAN_TypeDef *CANx,
                         int16_t motor5_iq,
                         int16_t motor6_iq,
                         int16_t motor7_iq,
                         int16_t motor8_iq)
{
    /* 兼容旧接口：C620/C610 ID5~ID8 电流控制帧，StdId = 0x1FF。 */
    (void)Motor_SendDjiRawFrame(Motor_ResolveCanHandle(CANx),
                                MOTOR_DJI_M3508_CMD_HIGH,
                                motor5_iq,
                                motor6_iq,
                                motor7_iq,
                                motor8_iq);
}
