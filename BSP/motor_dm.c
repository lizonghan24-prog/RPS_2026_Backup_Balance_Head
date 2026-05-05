#include "motor_dm.h"
#include "CANBus_Task.h"

#include <string.h>

/*
 * 达妙 DM 电机驱动分类文件。
 * 上半部分是本工程当前 yaw DM4310 使用的服务层：注册、在线检测、MIT 打包和反馈解析。
 * 下半部分保留旧工程原有的 DaMiao_* 接口，方便旧调试代码继续编译和临时使用。
 */

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DM4310_SERVICE_ONLINE_TIMEOUT_MS        100U
#define DM4310_SERVICE_RAD_TO_DEG               57.295779513082320876f
#define DM4310_SERVICE_MAX_COUNT                4U
#define DM4310_CMD_ENABLE                       0xFCU
#define DM4310_CMD_DISABLE                      0xFDU

static dm4310_service_t *dm4310_registry[DM4310_SERVICE_MAX_COUNT];

/* 浮点限幅。DM4310 MIT 编码前必须先把给定值夹进上位机配置范围内。 */
static float Dm4310_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/* 旧驱动里的 MIT 字段编码：把浮点物理量压缩成 bits 位无符号整数。 */
static uint16_t float_To_uint(float value, float min_value, float max_value, uint8_t bits)
{
    uint32_t max_raw;
    float limited_value;
    float scaled_value;

    if ((bits == 0U) || (bits >= 31U) || (max_value <= min_value))
    {
        return 0U;
    }

    limited_value = Dm4310_LimitFloat(value, min_value, max_value);

    max_raw = (1UL << bits) - 1UL;
    scaled_value = (limited_value - min_value) * (float)max_raw / (max_value - min_value);
    return (uint16_t)(scaled_value + 0.5f);
}

/* 旧驱动里的 MIT 字段解码：把 bits 位无符号整数还原成浮点物理量。 */
static float uint_To_float(uint16_t raw_value, float min_value, float max_value, uint8_t bits)
{
    uint32_t max_raw;

    if ((bits == 0U) || (bits >= 31U) || (max_value <= min_value))
    {
        return min_value;
    }

    max_raw = (1UL << bits) - 1UL;
    return ((float)raw_value * (max_value - min_value) / (float)max_raw) + min_value;
}

/*
 * 旧驱动原本依赖全局 CAN_Send_Data()。
 * 当前工程统一通过 CANBus_Task_SendStdFrame() 发标准 8 字节数据帧。
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

/* 按 hcan + 反馈 StdId 注册 DM4310 对象。 */
static HAL_StatusTypeDef Dm4310_RegisterObject(dm4310_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < DM4310_SERVICE_MAX_COUNT; ++i)
    {
        if ((dm4310_registry[i] == motor)
         || ((dm4310_registry[i] != NULL)
          && (dm4310_registry[i]->hcan == motor->hcan)
          && (dm4310_registry[i]->feedback_std_id == motor->feedback_std_id)))
        {
            dm4310_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < DM4310_SERVICE_MAX_COUNT; ++i)
    {
        if (dm4310_registry[i] == NULL)
        {
            dm4310_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* HAL tick 使用无符号减法，天然兼容 tick 回绕。 */
static uint8_t Dm4310_IsRecent(uint32_t last_update_tick)
{
    return (uint8_t)((HAL_GetTick() - last_update_tick) <= DM4310_SERVICE_ONLINE_TIMEOUT_MS);
}

/*
 * 解析 DM4310 MIT 反馈帧。
 * data[0] 高 4 bit 为错误码，低 4 bit 为电机 ID；
 * data[1..2] 为 16 bit 位置；
 * data[3] + data[4] 高 4 bit 为 12 bit 速度；
 * data[4] 低 4 bit + data[5] 为 12 bit 力矩。
 */
static void Dm4310_UpdateMitFeedback(dm4310_service_t *motor, const uint8_t data[8])
{
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t torque_raw;
    float position_rad;
    float velocity_rad_per_s;

    if (motor == NULL)
    {
        return;
    }

    pos_raw = (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[2]);
    vel_raw = (uint16_t)(((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4));
    torque_raw = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | (uint16_t)data[5]);

    position_rad = uint_To_float(pos_raw, motor->param.p_min_rad, motor->param.p_max_rad, 16U);
    velocity_rad_per_s = uint_To_float(vel_raw,
                                       motor->param.v_min_rad_per_s,
                                       motor->param.v_max_rad_per_s,
                                       12U);

    motor->err = (uint8_t)(data[0] >> 4);
    motor->t_mos = data[6];
    motor->t_rotor = data[7];
    motor->position_rad = position_rad * motor->feedback_angle_sign;
    motor->velocity_rad_per_s = velocity_rad_per_s * motor->feedback_speed_sign;
    motor->torque_nm = uint_To_float(torque_raw,
                                     motor->param.torque_min_nm,
                                     motor->param.torque_max_nm,
                                     12U);
    motor->angle_deg = motor->position_rad * DM4310_SERVICE_RAD_TO_DEG;
    motor->speed_dps = motor->velocity_rad_per_s * DM4310_SERVICE_RAD_TO_DEG;
    motor->last_update_tick = HAL_GetTick();
    motor->online = 1U;
}

void Motor_Dm4310ServiceInit(void)
{
    memset(dm4310_registry, 0, sizeof(dm4310_registry));
}

HAL_StatusTypeDef Motor_RegisterDm4310Mit(dm4310_service_t *motor,
                                          CAN_HandleTypeDef *hcan,
                                          uint8_t motor_id,
                                          uint32_t feedback_std_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U) || (motor_id > 0x7FU))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->command_std_id = (uint32_t)motor_id;
    motor->feedback_std_id = feedback_std_id;
    motor->param.p_min_rad = DM4310_POS_MIN_RAD;
    motor->param.p_max_rad = DM4310_POS_MAX_RAD;
    motor->param.v_min_rad_per_s = DM4310_VEL_MIN_RAD_PER_S;
    motor->param.v_max_rad_per_s = DM4310_VEL_MAX_RAD_PER_S;
    motor->param.kp_min = DM4310_KP_MIN;
    motor->param.kp_max = DM4310_KP_MAX;
    motor->param.kd_min = DM4310_KD_MIN;
    motor->param.kd_max = DM4310_KD_MAX;
    motor->param.torque_min_nm = DM4310_TORQUE_MIN_NM;
    motor->param.torque_max_nm = DM4310_TORQUE_MAX_NM;
    motor->feedback_angle_sign = 1.0f;
    motor->feedback_speed_sign = 1.0f;

    return Dm4310_RegisterObject(motor);
}

void Motor_SetDm4310MitTorque(dm4310_service_t *motor, float torque_nm)
{
    if (motor != NULL)
    {
        motor->position_ref_rad = 0.0f;
        motor->velocity_ref_rad_per_s = 0.0f;
        motor->kp_ref = 0.0f;
        motor->kd_ref = 0.0f;
        motor->torque_ref_nm = Dm4310_LimitFloat(torque_nm,
                                                 motor->param.torque_min_nm,
                                                 motor->param.torque_max_nm);
    }
}

static HAL_StatusTypeDef Dm4310_SendSpecialCommand(dm4310_service_t *motor, uint8_t command)
{
    HAL_StatusTypeDef status;
    uint8_t data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(data, 0xFF, sizeof(data));
    data[7] = command;
    status = CAN_Send_Data(motor->hcan, (uint16_t)motor->command_std_id, data, 8U);

    if (status == HAL_OK)
    {
        if (command == DM4310_CMD_ENABLE)
        {
            motor->output_enabled = 1U;
        }
        else if (command == DM4310_CMD_DISABLE)
        {
            motor->output_enabled = 0U;
        }
    }

    return status;
}

HAL_StatusTypeDef Motor_Dm4310SendEnable(dm4310_service_t *motor)
{
    return Dm4310_SendSpecialCommand(motor, DM4310_CMD_ENABLE);
}

HAL_StatusTypeDef Motor_Dm4310SendDisable(dm4310_service_t *motor)
{
    return Dm4310_SendSpecialCommand(motor, DM4310_CMD_DISABLE);
}

HAL_StatusTypeDef Motor_Dm4310SendMitControl(dm4310_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque_raw;
    uint8_t data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    pos_raw = float_To_uint(motor->position_ref_rad,
                            motor->param.p_min_rad,
                            motor->param.p_max_rad,
                            16U);
    vel_raw = float_To_uint(motor->velocity_ref_rad_per_s,
                            motor->param.v_min_rad_per_s,
                            motor->param.v_max_rad_per_s,
                            12U);
    kp_raw = float_To_uint(motor->kp_ref, motor->param.kp_min, motor->param.kp_max, 12U);
    kd_raw = float_To_uint(motor->kd_ref, motor->param.kd_min, motor->param.kd_max, 12U);
    torque_raw = float_To_uint(motor->torque_ref_nm,
                               motor->param.torque_min_nm,
                               motor->param.torque_max_nm,
                               12U);

    data[0] = (uint8_t)(pos_raw >> 8);
    data[1] = (uint8_t)(pos_raw & 0xFFU);
    data[2] = (uint8_t)(vel_raw >> 4);
    data[3] = (uint8_t)(((vel_raw & 0x0FU) << 4) | (kp_raw >> 8));
    data[4] = (uint8_t)(kp_raw & 0xFFU);
    data[5] = (uint8_t)(kd_raw >> 4);
    data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4) | (torque_raw >> 8));
    data[7] = (uint8_t)(torque_raw & 0xFFU);

    status = CAN_Send_Data(motor->hcan, (uint16_t)motor->command_std_id, data, 8U);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

uint8_t Motor_Dm4310IsOnline(const dm4310_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return Dm4310_IsRecent(motor->last_update_tick);
}

void Motor_Dm4310ProcessCanMessage(CAN_HandleTypeDef *hcan,
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

    for (i = 0U; i < DM4310_SERVICE_MAX_COUNT; ++i)
    {
        if ((dm4310_registry[i] != NULL)
         && (dm4310_registry[i]->hcan == hcan)
         && (dm4310_registry[i]->feedback_std_id == rx_header->StdId)
         && ((rx_data[0] & 0x0FU) == dm4310_registry[i]->motor_id))
        {
            Dm4310_UpdateMitFeedback(dm4310_registry[i], rx_data);
            return;
        }
    }
}

motor_dm_t DaMiao_1;

/**
 * @brief 达妙电机参数初始化
 * 
 */
void DaMiao_Motor_Init(motor_dm_t *p_motor,
                            DaMiao_Mode_e mode,
                            float P_min,float P_max,
                            float V_min,float V_max,
                            float Kp_min,float Kp_max,
                            float Kd_min,float Kd_max,
                            float t_min,float t_max
                            )
{
    p_motor->Mode = mode;
    p_motor->Param.P_MIN = P_min;
    p_motor->Param.P_MAX = P_max;
    p_motor->Param.V_MIN = V_min;
    p_motor->Param.V_MAX = V_max;
    p_motor->Param.KP_MIN = Kp_min;
    p_motor->Param.KP_MAX = Kp_max;
    p_motor->Param.KD_MIN = Kd_min;
    p_motor->Param.KD_MAX = Kd_max;
    p_motor->Param.T_MIN = t_min;
    p_motor->Param.T_MAX = t_max;
}

/**
 * @brief 达妙电机数据接收处理
 * 
 * @param p_motor 
 * @param p_data 
 */
void DaMiao_InfoRecv_Process(motor_dm_t *p_motor,uint8_t *p_data)
{
    p_motor->ID = (uint16_t)(p_data[0]&0x0f);
    p_motor->Err = (DaMiao_Err_e)(p_data[0]>>4);

//    p_motor->POS_Fdb = (float)((p_data[1]<<8 | p_data[2]) - 32367)*0.02186f; //*0.02186
//    p_motor->VEL_Fdb = (float)((p_data[3]<<4 | p_data[4]>>4) -2047)*0.02198f; //*0.02198
//    p_motor->T_Fdb = (float)((((p_data[4]&0x0f)<<8) | (p_data[5])) -2047)*0.01953125f; //*0.000977
    int pos_temp,vel_temp,t_temp;
    pos_temp = (p_data[1]<<8) | p_data[2];
    vel_temp = (p_data[3]<<4) | (p_data[4]>>4);
    t_temp = ((p_data[4]&0x0f)<<8) | (p_data[5]);
    p_motor->POS_Fdb = uint_To_float(pos_temp,p_motor->Param.P_MIN,p_motor->Param.P_MAX,16); 
    p_motor->VEL_Fdb = uint_To_float(vel_temp,p_motor->Param.V_MIN,p_motor->Param.V_MAX,12); 
    p_motor->T_Fdb   = uint_To_float(t_temp,p_motor->Param.T_MIN,p_motor->Param.T_MAX,12); 

    p_motor->T_MOS = p_data[6];
    p_motor->T_Rotor = p_data[7];

	
    //数据进一步处理
    static int Init_Cnt;Init_Cnt++;

    p_motor->angle_abs = p_motor->POS_Fdb * 180.0f/PI + p_motor->angle_offset;
    p_motor->speed_dps = p_motor->VEL_Fdb * 180.0f/PI;
    p_motor->t_fdb = p_motor->T_Fdb * 10000.0f;

    if(Init_Cnt > 50)
    {
        if(p_motor->angle_abs - p_motor->angle_last > 300)
        p_motor->round_cnt--;
        else if(p_motor->angle_abs - p_motor->angle_last < -300)
        p_motor->round_cnt++;

        Init_Cnt = 100;
    }
    
    p_motor->angle_cnt = 360*p_motor->round_cnt + p_motor->angle_abs;
    p_motor->angle_last = p_motor->angle_abs;
}

/**
 * @brief 达妙电机 位置模式控制信号发送
 * 
 * @param hcan 
 * @param ID 0x100 + ID
 * @param p_motor 
 */
void DaMiao_Position_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x100 + ID;

    uint8_t *pos,*vel;
    pos = (uint8_t*)(&p_motor->POS_Ref);
    vel = (uint8_t*)(&p_motor->VEL_Ref);
	
    data[0] = *pos;
    data[1] = *(pos+1);
    data[2] = *(pos+2);
    data[3] = *(pos+3);
    data[4] = *vel;
	data[5] = *(vel+1);
	data[6] = *(vel+2);
	data[7] = *(vel+3);
    CAN_Send_Data(hcan,ID,data,Length);

}

/**
 * @brief 达妙电机 MIT模式控制信号发送
 * 
 * @param hcan 
 * @param ID ID
 * @param p_motor 
 */
void DaMiao_MIT_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x000 + ID;
    /**角度制要转到弧度制发送 */
	uint16_t pos_temp,vel_temp,kp_temp,kd_temp,tor_temp;
	pos_temp = float_To_uint(p_motor->POS_Ref*(PI/180.0f),p_motor->Param.P_MIN,p_motor->Param.P_MAX,16);
	vel_temp = float_To_uint(p_motor->VEL_Ref,p_motor->Param.V_MIN,p_motor->Param.V_MAX,12);
    kp_temp  = float_To_uint(p_motor->Kp,p_motor->Param.KP_MIN,p_motor->Param.KP_MAX,12);
    kd_temp  = float_To_uint(p_motor->Kd,p_motor->Param.KD_MIN,p_motor->Param.KD_MAX,12);
    tor_temp = float_To_uint(p_motor->T_Ref,p_motor->Param.T_MIN,p_motor->Param.T_MAX,12);
	
    data[0] = (pos_temp>>8);
    data[1] = pos_temp;
    data[2] = (vel_temp>>4);
    data[3] = (uint8_t)(((vel_temp & 0x0FU) << 4) | (kp_temp >> 8));
    data[4] = kp_temp;
	data[5] = (kd_temp>>4);
	data[6] = (uint8_t)(((kd_temp & 0x0FU) << 4) | (tor_temp >> 8));
	data[7] = tor_temp;

    CAN_Send_Data(hcan,ID,data,Length);
}

void DaMiao_Speed_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
    ID = 0x200 + ID;

    uint8_t *vel;
    vel = (uint8_t*)(&p_motor->VEL_Ref);
    data[0] = *vel;
    data[1] = *(vel+1);
    data[2] = *(vel+2);
    data[3] = *(vel+3);
    data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
    CAN_Send_Data(hcan,ID,data,Length);
}

// void DaMiao_eMIT_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor)
// {
//     uint16_t Length = 0x08;
//     uint8_t data[8];
//     ID = 0x300 + ID;

//     uint8_t *p_des,*v_des,*i_des;
//     p_des = 
//     v_des = (uint8_t*)(&p_motor->VEL_Ref);
//     i_des = 
//     data[0] = *p_des;
//     data[1] = *(p_des+1);
//     data[2] = *(p_des+2);
//     data[3] = *(p_des+3);
//     data[4] = *v_des;
// 	data[5] = *(v_des+1);
// 	data[6] = *i_des;
// 	data[7] = *(i_des+1);
//     CAN_Send_Data(hcan,ID,data,Length);
// }


/**
 * @brief 达妙电机使能
 * 
 * @param ID 
 */
void DaMiao_Enable(CAN_HandleTypeDef *hcan,uint16_t ID)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
   
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 达妙电机失能
 * 
 * @param hcan 
 * @param ID 
 */
void DaMiao_Disable(CAN_HandleTypeDef *hcan,uint16_t ID)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
   
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 达妙电机零点设置
 * 
 * @param hcan 
 * @param ID 
 */
void DaMiao_Zero_Set(CAN_HandleTypeDef *hcan,uint16_t ID)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
   
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
    CAN_Send_Data(hcan,ID,data,Length);
}

/**
 * @brief 达妙电机错误清零
 * 
 * @param hcan 
 * @param ID 
 */
void DaMiao_Error_Clear(CAN_HandleTypeDef *hcan,uint16_t ID)
{
    uint16_t Length = 0x08;
    uint8_t data[8];
   
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
    CAN_Send_Data(hcan,ID,data,Length);
}
