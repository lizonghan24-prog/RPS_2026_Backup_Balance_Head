#include "motor_lk.h"
#include "CANBus_Task.h"

#include <string.h>

/*
 * 这个文件是旧工程留下来的 LK/MG/RMD 协议驱动。
 * 当前拨盘 MG4310 的实时控制也放在本文件的 Motor_Lk* 服务接口里；
 * 旧 LK_* 函数继续保留，避免原工程里依赖旧接口的调试代码失效。
 */

/*
 * 旧驱动原本依赖全局 CAN_Send_Data()。
 * 当前工程统一通过 CANBus_Task_SendStdFrame() 发送标准 8 字节数据帧。
 */
static HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan,
                                       uint16_t std_id,
                                       uint8_t *data,
                                       uint16_t length)
{
    if ((data == NULL) || (length != 8U))
    {
        return HAL_ERROR;
    }

    return CANBus_Task_SendStdFrame(hcan, std_id, data);
}

#define LK_SERVICE_ONLINE_TIMEOUT_MS            100U
#define LK_SERVICE_MAX_COUNT                    8U
#define LK_STD_ID_BASE                          0x140U
#define LK_CMD_STOP                             0x81U
#define LK_CMD_POWER_ON                         0x88U
#define LK_CMD_READ_STATE_2                     0x9CU
#define LK_CMD_IQ_CONTROL                       0xA1U

static lk_motor_service_t *lk_registry[LK_SERVICE_MAX_COUNT];

/* LK/MG4310 状态帧使用小端整数。 */
static int16_t LK_ReadInt16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* LK/MG4310 编码器是 16 bit 无符号小端。 */
static uint16_t LK_ReadUint16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* LK/MG4310 控制命令里的 iq 给定放在 data[4]/data[5]，采用小端格式。 */
static void LK_WriteInt16LE(uint8_t *data, uint8_t index, int16_t value)
{
    data[index] = (uint8_t)((uint16_t)value & 0xFFU);
    data[index + 1U] = (uint8_t)((uint16_t)value >> 8);
}

/* HAL tick 使用无符号减法，天然兼容 tick 回绕。 */
static uint8_t LK_IsRecent(uint32_t last_update_tick)
{
    return (uint8_t)((HAL_GetTick() - last_update_tick) <= LK_SERVICE_ONLINE_TIMEOUT_MS);
}

static void LK_TouchOnline(volatile uint8_t *online, volatile uint32_t *last_update_tick)
{
    *last_update_tick = HAL_GetTick();
    *online = 1U;
}

/* 按 hcan + StdId 注册 LK/MG4310 对象。 */
static HAL_StatusTypeDef LK_RegisterObject(lk_motor_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < LK_SERVICE_MAX_COUNT; ++i)
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

    for (i = 0U; i < LK_SERVICE_MAX_COUNT; ++i)
    {
        if (lk_registry[i] == NULL)
        {
            lk_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/*
 * LK/MG4310 状态 2 反馈解析：
 * data[0] = 0x9C 或力矩控制返回的 0xA1；
 * data[1] = 温度；
 * data[2..3] = iq/力矩电流，小端；
 * data[4..5] = 速度，小端；
 * data[6..7] = 单圈编码器，小端。
 */
static void LK_UpdateState2(lk_motor_service_t *motor, const uint8_t data[8])
{
    uint16_t raw_ecd;
    int32_t delta;

    raw_ecd = LK_ReadUint16LE(&data[6]);

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
    motor->iq_feedback = LK_ReadInt16LE(&data[2]);
    motor->speed_dps = (float)LK_ReadInt16LE(&data[4]);
    motor->angle_deg = ((float)((int32_t)motor->raw_ecd - (int32_t)motor->ecd_bias)
                      * MOTOR_LK_ENCODER_DEG_PER_TICK)
                     + ((float)motor->round_count * 360.0f);
    motor->total_angle_deg = (float)motor->total_ecd * MOTOR_LK_ENCODER_DEG_PER_TICK;
}

void Motor_LkServiceInit(void)
{
    memset(lk_registry, 0, sizeof(lk_registry));
}

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
    motor->std_id = LK_STD_ID_BASE + (uint32_t)motor_id;

    return LK_RegisterObject(motor);
}

void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t output)
{
    if (motor != NULL)
    {
        motor->iq_output = output;
    }
}

HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(data, 0, sizeof(data));
    data[0] = LK_CMD_POWER_ON;
    status = CAN_Send_Data(motor->hcan, (uint16_t)motor->std_id, data, 8U);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(data, 0, sizeof(data));
    data[0] = LK_CMD_STOP;
    status = CAN_Send_Data(motor->hcan, (uint16_t)motor->std_id, data, 8U);
    if (status == HAL_OK)
    {
        motor->output_enabled = 0U;
    }

    return status;
}

HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor)
{
    uint8_t data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(data, 0, sizeof(data));
    data[0] = LK_CMD_READ_STATE_2;
    return CAN_Send_Data(motor->hcan, (uint16_t)motor->std_id, data, 8U);
}

HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(data, 0, sizeof(data));
    data[0] = LK_CMD_IQ_CONTROL;
    LK_WriteInt16LE(data, 4U, motor->iq_output);

    status = CAN_Send_Data(motor->hcan, (uint16_t)motor->std_id, data, 8U);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

uint8_t Motor_LkIsOnline(const lk_motor_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return LK_IsRecent(motor->last_update_tick);
}

void Motor_LkProcessCanMessage(CAN_HandleTypeDef *hcan,
                               const CAN_RxHeaderTypeDef *rx_header,
                               const uint8_t rx_data[8])
{
    uint8_t i;

    if ((hcan == NULL)
     || (rx_header == NULL)
     || (rx_data == NULL)
     || (rx_header->IDE != CAN_ID_STD)
     || (rx_header->RTR != CAN_RTR_DATA)
     || (rx_header->DLC < 8U))
    {
        return;
    }

    for (i = 0U; i < LK_SERVICE_MAX_COUNT; ++i)
    {
        if ((lk_registry[i] != NULL)
         && (lk_registry[i]->hcan == hcan)
         && (lk_registry[i]->std_id == rx_header->StdId))
        {
            if ((rx_data[0] == LK_CMD_READ_STATE_2)
             || (rx_data[0] == LK_CMD_IQ_CONTROL))
            {
                LK_UpdateState2(lk_registry[i], rx_data);
            }
            else if (rx_data[0] == LK_CMD_POWER_ON)
            {
                lk_registry[i]->output_enabled = 1U;
            }
            else if (rx_data[0] == LK_CMD_STOP)
            {
                lk_registry[i]->output_enabled = 0U;
            }

            LK_TouchOnline(&lk_registry[i]->online, &lk_registry[i]->last_update_tick);
            return;
        }
    }
}

uint8_t LK_ROM_Protect=0;   //防止写ROM烧电机

/**
 * @brief LK电机数据接收处理
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_InfoRecv_Process(motor_lk_t *p_motor,uint8_t *p_data)
{
    switch (p_data[0])
    {
    case 0x30:LK_PID_Param_Recv(p_motor,p_data);break;
    case 0x31:LK_PID_Param_Recv(p_motor,p_data);break;
    case 0x32:LK_PID_Param_Recv(p_motor,p_data);break;
    case 0x33:LK_Acc_Recv(p_motor,p_data);break;
    case 0x90:LK_Encoder_Data_Recv(p_motor,p_data);break;
    case 0x19:LK_Encoder_Offset_Recv(p_motor,p_data);break;
    case 0x92:LK_Multi_Angle_Absolute_Recv(p_motor,p_data);break;
    case 0x94:LK_Single_Angle_Absolute_Recv(p_motor,p_data);break;
    case 0x9A:LK_Status1_Recv(p_motor,p_data);break;
    case 0x9C:LK_Status2_Recv(p_motor,p_data);break;
    case 0xA0:LK_T_Openloop_Control_Recv(p_motor,p_data);break;
    case 0xA1:LK_T_Closeloop_Control_Recv(p_motor,p_data);break;
    case 0xA2:LK_Speed_Closeloop_Control_Recv(p_motor,p_data);break;
    case 0xA3:LK_Angle_Closeloop_Control_Recv(p_motor,p_data);break;
    case 0xA4:LK_Angle_Closeloop_Control_Recv2(p_motor,p_data);break;
    case 0xA5:LK_Angle_Closeloop_Control_Recv3(p_motor,p_data);break;
    default:break;
    }
}

/**
 * @brief LK电机部分简单指令发送
 * 
 * @param hcan 
 * @param ID 
 * @param LK_Command 
 */
void LK_Send_Command(CAN_HandleTypeDef *hcan,int16_t ID,LK_Command_e LK_Command)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = LK_Command;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief LK电机PID参数接收 0x30 0x31 0x32
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_PID_Param_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->anglePidKp = p_data[2];
    p_motor->anglePidKi = p_data[3];
    p_motor->speedPidKp = p_data[4];
    p_motor->speedPidKi = p_data[5];
    p_motor->iqPidKp = p_data[6];
    p_motor->iqPidKi = p_data[7]; 
}

/**
 * @brief LK电机写入PID参数 RAM掉电参数失效 安全
 * 
 * @param hcan 
 * @param ID 
 */
void LK_Write_PID_Param_RAM(CAN_HandleTypeDef *hcan,int16_t ID,
	int8_t angKp,int8_t angKi,
	int8_t spdKp,int8_t spdKi,
	int8_t iqKp, int8_t iqKi)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0x31;
    data[1] = 0x00;
    data[2] = angKp;
    data[3] = angKi;
    data[4] = spdKp;
    data[5] = spdKi;
    data[6] = iqKp;
    data[7] = iqKi;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief LK电机写入PID参数 ROM掉电参数仍有效 危险 执行一次即可 跑循环等于报废
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_Write_PID_Param_ROM(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
    LK_ROM_Protect++;
    if(LK_ROM_Protect > 3)return;

    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0x32;
    data[1] = 0x00;
    data[2] = p_motor->angKp;
    data[3] = p_motor->angKi;
    data[4] = p_motor->spdKp;
    data[5] = p_motor->spdKi;
    data[6] = p_motor->iqKp;
    data[7] = p_motor->iqKi;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief LK电机加速度数据接收 data4最低位 0x33
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Acc_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    // p_motor->Accel |= (int32_t)(p_data[7]<<24);
    // p_motor->Accel |= (int32_t)(p_data[6]<<16); 
    // p_motor->Accel |= (int32_t)(p_data[5]<<8); 
    // p_motor->Accel |= (int32_t)(p_data[4]<<0); 
    p_motor->Accel = (int32_t)(p_data[7]<<24)
                            |(p_data[6]<<16)
                            |(p_data[5]<<8)
                            |(p_data[4]<<0); 
}

/**
 * @brief LK电机写加速度到 RAM 安全 0x34
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_Write_Acc_RAM_Set(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0x34;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&p_motor->Acc);
    data[5] = *((uint8_t *)(&p_motor->Acc)+1);
    data[6] = *((uint8_t *)(&p_motor->Acc)+2);
    data[7] = *((uint8_t *)(&p_motor->Acc)+3);

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief LK电机编码器数据接收 0x90
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Encoder_Data_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    // p_motor->encoder |= (uint16_t)(p_data[2]<<0);
    // p_motor->encoder |= (uint16_t)(p_data[3]<<8);
    // p_motor->encoderRaw |= (uint16_t)(p_data[4]<<0);
    // p_motor->encoderRaw |= (uint16_t)(p_data[5]<<8);
    // p_motor->encoderOffset |= (uint16_t)(p_data[6]<<0);
    // p_motor->encoderOffset |= (uint16_t)(p_data[7]<<8);
    p_motor->encoder = (uint16_t)(p_data[2]<<0)|(p_data[3]<<8);
    p_motor->encoderRaw = (uint16_t)(p_data[4]<<0)|(p_data[5]<<8);
    p_motor->encoderOffset = (uint16_t)(p_data[6]<<0)|(p_data[7]<<8);
}

/**
 * @brief LK电机 设置编码器的零偏到ROM 危险 注意别烧ROM 0x91
 * 
 * @param hcan 
 * @param ID 
 * @param offset 0~16383
 */
void LK_Encoder_Zero_Offset_ROM_Set(CAN_HandleTypeDef *hcan,int16_t ID,uint16_t offset)
{
    LK_ROM_Protect++;
    if(LK_ROM_Protect > 3)return;

    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0x91;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = *((uint8_t *)(&offset));
    data[7] = *((uint8_t *)(&offset)+1);

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief LK电机设置当前位置为零点到ROM 危险 0x19 
 * 
 * @param hcan 
 * @param ID 
 */
void LK_Encoder_Zero_ROM_Set(CAN_HandleTypeDef *hcan,int16_t ID)
{
    LK_ROM_Protect++;
    if(LK_ROM_Protect > 3)return;

    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0x19;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief LK电机编码器零偏接收
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Encoder_Offset_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->encoderOffset = (uint16_t)(p_data[6]<<0)|(p_data[7]<<8);
}

/**
 * @brief LK电机多圈绝对角度值接收  0x92
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Multi_Angle_Absolute_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    memcpy(&p_motor->motorAngle,p_data+1,7);
    p_motor->cnt_angle = p_motor->motorAngle * 0.01f;
}

/**
 * @brief LK电机单圈绝对角度值接收  0x94
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Single_Angle_Absolute_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    memcpy(&p_motor->circleAngle,p_data+4,4);       //0~36000  * (减速比-1)
    // p_motor->circle_angle = p_motor->circleAngle * 0.01f;
}

/**
 * @brief LK电机状态获取1 包括温度 电压 报错 0x9A
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Status1_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    p_motor->voltage = (uint16_t)(p_data[3]<<0)|(p_data[4]<<8);
    p_motor->errorState = p_data[7];
}

/**
 * @brief LK电机获取状态接收 包括 温度 电流 速度 编码器位置 0x9C
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Status2_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    // p_motor->iq |= (uint16_t)(p_data[2]<<0);
    // p_motor->iq |= (uint16_t)(p_data[3]<<8);
    // p_motor->speed |= (uint16_t)(p_data[4]<<0);
    // p_motor->speed |= (uint16_t)(p_data[5]<<8);
    // p_motor->encoder |= (uint16_t)(p_data[6]<<0);
    // p_motor->encoder |= (uint16_t)(p_data[7]<<8);

    p_motor->iq = (uint16_t)(p_data[2]<<0)|(p_data[3]<<8);
    p_motor->speed = (uint16_t)(p_data[4]<<0)|(p_data[5]<<8);
    p_motor->encoder = (uint16_t)(p_data[6]<<0)|(p_data[7]<<8);
}

/**
 * @brief ^-^ 0x9D 看三相电流的
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Status3_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    /**^-^ */
}

/**
 * @brief 转矩开环控制命令  该命令仅在 MS 上实现
 *        该命令中的控制值 powerControl 不受上位机(LK-Motor Tool)中的 Max Power 值限制
 *        0xA0
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_T_Openloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0xA0;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t*)(&p_motor->powerControl);
    data[5] = *((uint8_t*)(&p_motor->powerControl)+1);
    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 转矩开环控制回复 0xA0
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_T_Openloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    // p_motor->pow |= (int16_t)(p_data[2]<<0);
    // p_motor->pow |= (int16_t)(p_data[3]<<8);
    // p_motor->speed |= (int16_t)(p_data[4]<<0);
    // p_motor->speed |= (int16_t)(p_data[5]<<8);
    // p_motor->encoder |= (int16_t)(p_data[6]<<0);
    // p_motor->encoder |= (int16_t)(p_data[7]<<8);
    p_motor->pow = (int16_t)(p_data[2]<<0)|(p_data[3]<<8);
    p_motor->speed = (int16_t)(p_data[4]<<0)|(p_data[5]<<8);
    p_motor->encoder = (int16_t)(p_data[6]<<0)|(p_data[7]<<8);
}

/**
 * @brief 转矩闭环控制命令  该命令仅在 MF 和 MG 上实现
 *        该命令中的控制值 iqControl 不受上位机(LK-Motor Tool)中的 Max Torque Current 限制
 *        0xA1
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_T_Closeloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;


    p_motor->iqControl = 0;

    data[0] = 0xA1;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
//    data[4] = *(uint8_t*)(&p_motor->iqControl);
//    data[5] = *((uint8_t*)(&p_motor->iqControl)+1);
	data[4] = p_motor->iqControl&0xff;
	data[5] = (p_motor->iqControl&0xff00)>>8;
    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 转矩闭环控制回复 0xA1
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_T_Closeloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
//    p_motor->iq |= (int16_t)(p_data[2]<<0);
//    p_motor->iq |= (int16_t)(p_data[3]<<8);
//    p_motor->speed |= (int16_t)(p_data[4]<<0);
//    p_motor->speed |= (int16_t)(p_data[5]<<8);
//    p_motor->encoder |= (int16_t)(p_data[6]<<0);
//    p_motor->encoder |= (int16_t)(p_data[7]<<8);
	p_motor->iq = (int16_t)((p_data[2])|(p_data[3]<<8));
	p_motor->speed = (int16_t)((p_data[4])|(p_data[5]<<8));
    p_motor->encoder = (int16_t)((p_data[6])|(p_data[7]<<8));

    // uint16_t encoder_new;
    // encoder_new = (int16_t)((p_data[6])|(p_data[7]<<8));
    // encoder_new = (float)(encoder_new)/65535.0f*360.0f
    // if(encoder_new - p_motor->encoder > )
	// p_motor->encoder = ;
	
    //获取有偏移量的单圈绝对值角
	p_motor->circle_angle = (float)(p_motor->encoder)/65535.0f*360.0f;

    if(p_motor->circle_angle - p_motor->circle_anglr_last > 300)p_motor->round_cnt--;
    if(p_motor->circle_angle - p_motor->circle_anglr_last < -300)p_motor->round_cnt++;

    p_motor->circle_anglr_last = p_motor->circle_angle;

    p_motor->cnt_angle = p_motor->round_cnt*360.0f + p_motor->circle_angle + p_motor->offset;
    // if(p_motor->circle_angle > 360)p_motor->circle_angle -=360;
    // if(p_motor->circle_angle < 0)p_motor->circle_angle +=360;
}

/**
 * @brief 速度闭环控制 0xA2
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_Speed_Closeloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0xA2;
    data[1] = 0x00;
    // data[2] = p_motor->iqControl&0xff;
	// data[3] = (p_motor->iqControl&0xff00)>>8;
	
	//p_motor->speedControl  = 0;
	
    data[2] = 0;
	data[3] = 0;
    data[4] = *(uint8_t*)(&p_motor->speedControl);
    data[5] = *((uint8_t*)(&p_motor->speedControl)+1);
    data[6] = *((uint8_t*)(&p_motor->speedControl)+2);
    data[7] = *((uint8_t*)(&p_motor->speedControl)+3);
	
//    data[4] = p_motor.speedControl;
//    data[5] = p_motor.speedControl>>8;
//    data[6] = p_motor.speedControl>>16;
//    data[7] = p_motor.speedControl>>24;


    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 速度闭环控制 0xA2
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Speed_Closeloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    // p_motor->iq |= (int16_t)(p_data[2]<<0);
    // p_motor->iq |= (int16_t)(p_data[3]<<8);
    // p_motor->speed |= (int16_t)(p_data[4]<<0);
    // p_motor->speed |= (int16_t)(p_data[5]<<8);
    // p_motor->encoder |= (int16_t)(p_data[6]<<0);
    // p_motor->encoder |= (int16_t)(p_data[7]<<8);
    p_motor->iq = (int16_t)(p_data[2])|(p_data[3]<<8);
    p_motor->speed = (int16_t)(p_data[4])|(p_data[5]<<8);
    p_motor->encoder = (int16_t)(p_data[6])|(p_data[7]<<8);

    p_motor->circle_angle = (float)(p_motor->encoder)/65535.0f*360.0f;

    if(p_motor->circle_angle - p_motor->circle_anglr_last > 300)p_motor->round_cnt--;
    if(p_motor->circle_angle - p_motor->circle_anglr_last < -300)p_motor->round_cnt++;
    p_motor->circle_anglr_last = p_motor->circle_angle;
    p_motor->cnt_angle = p_motor->round_cnt*360.0f + p_motor->circle_angle + p_motor->offset;
}

/**
 * @brief 发送该命令以控制电机的位置(多圈角度)  0xA3
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_Angle_Closeloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
     uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0xA3;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t*)(&p_motor->angleControl);
    data[5] = *((uint8_t*)(&p_motor->angleControl)+1);
    data[6] = *((uint8_t*)(&p_motor->angleControl)+2);
    data[7] = *((uint8_t*)(&p_motor->angleControl)+3);

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 多圈角度  0xA3
 * 
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Angle_Closeloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    // p_motor->iq |= (int16_t)(p_data[2]<<0);
    // p_motor->iq |= (int16_t)(p_data[3]<<8);
    // p_motor->speed |= (int16_t)(p_data[4]<<0);
    // p_motor->speed |= (int16_t)(p_data[5]<<8);
    // p_motor->encoder |= (int16_t)(p_data[6]<<0);
    // p_motor->encoder |= (int16_t)(p_data[7]<<8);

    p_motor->iq = (int16_t)(p_data[2]<<0)|(p_data[3]<<8);
    p_motor->speed = (int16_t)(p_data[4]<<0)|(p_data[5]<<8);
    p_motor->encoder = (int16_t)(p_data[6]<<0)|(p_data[7]<<8);
}

/**
 * @brief   多圈    0xA4
 *          限速
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_Angle_Closeloop_Control_Command2(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
     uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0xA4;
    data[1] = 0x00;
    data[2] = *(uint8_t*)(&p_motor->maxSpeed);
    data[3] = *((uint8_t*)(&p_motor->maxSpeed)+1);
    data[4] = *(uint8_t*)(&p_motor->angleControl);
    data[5] = *((uint8_t*)(&p_motor->angleControl)+1);
    data[6] = *((uint8_t*)(&p_motor->angleControl)+2);
    data[7] = *((uint8_t*)(&p_motor->angleControl)+3);

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 多圈  限速    0xA4
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Angle_Closeloop_Control_Recv2(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    // p_motor->iq |= (int16_t)(p_data[2]<<0);
    // p_motor->iq |= (int16_t)(p_data[3]<<8);
    // p_motor->speed |= (int16_t)(p_data[4]<<0);
    // p_motor->speed |= (int16_t)(p_data[5]<<8);
    // p_motor->encoder |= (int16_t)(p_data[6]<<0);
    // p_motor->encoder |= (int16_t)(p_data[7]<<8);
    p_motor->iq = (int16_t)(p_data[2]<<0)|(p_data[3]<<8);
    p_motor->speed = (int16_t)(p_data[4]<<0)|(p_data[5]<<8);
    p_motor->encoder = (int16_t)(p_data[6]<<0)|(p_data[7]<<8);
}

/**
 * @brief 单圈 0~360 0xA5
 * 
 * @param hcan 
 * @param ID 
 * @param p_motor 
 */
void LK_Angle_Closeloop_Control_Command3(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x140 + ID;

    data[0] = 0xA5;
    data[1] = p_motor->spinDirection;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t*)(&p_motor->angleControl);
    data[5] = *((uint8_t*)(&p_motor->angleControl)+1);
    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 单圈 0xA5
 * 
 * @param p_motor 
 * @param p_data 
 */
void LK_Angle_Closeloop_Control_Recv3(motor_lk_t *p_motor,uint8_t *p_data)
{
    p_motor->temperature = p_data[1];
    // p_motor->iq |= (int16_t)(p_data[2]<<0);
    // p_motor->iq |= (int16_t)(p_data[3]<<8);
    // p_motor->speed |= (int16_t)(p_data[4]<<0);
    // p_motor->speed |= (int16_t)(p_data[5]<<8);
    // p_motor->encoder |= (int16_t)(p_data[6]<<0);
    // p_motor->encoder |= (int16_t)(p_data[7]<<8);
    p_motor->iq = (int16_t)(p_data[2]<<0)|(p_data[3]<<8);
    p_motor->speed = (int16_t)(p_data[4]<<0)|(p_data[5]<<8);
    p_motor->encoder = (int16_t)(p_data[6]<<0)|(p_data[7]<<8);

}

/**^-^  */
