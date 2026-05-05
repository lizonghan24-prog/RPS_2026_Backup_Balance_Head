#ifndef __MOTOR_LK_H__
#define __MOTOR_LK_H__

#include "stm32f4xx_hal.h"

/* LK/MG4310 状态 2 帧里的单圈编码器按 16 bit 解析。 */
#define MOTOR_LK_ENCODER_RANGE                  65536U
#define MOTOR_LK_ENCODER_HALF_RANGE             32768
#define MOTOR_LK_ENCODER_DEG_PER_TICK           (360.0f / (float)MOTOR_LK_ENCODER_RANGE)

typedef enum
{
    LK_AAA,
}LK_Err_e;

typedef enum
{
    Get_PID_Param           =0x30,
    Get_Acc             =0x33,
    Get_Encoder_Data        =0x90,
    Get_Multi_Angle_Absolute_Values    =0x92,    //多圈绝对角度值
    Get_Single_Angle_Absolute_Values    =0x94,  //单圈绝对角度值
    Get_Status_And_Err  =0x9A,      //获取电机温度 电压 报错
    Clear_Err_Flag_Command      =0x9B,  
    Get_Status_Two      =0x9C,     //获取电机温度 电压 转速 编码器位置
    Get_Status_Three            =0x9D,  //获取三相电流数据
    LK_Motor_Close       =0x80,  //关闭电机 清除电机运行状态和之前接收的控制指令    上电默认状态
    LK_Motor_Stop             =0x81,  //停止    不清除电机运行状态和之前接收的控制指令
    LK_Motor_ReOpen     =0x88,  //重新打开电机
}LK_Command_e;

typedef struct 
{
    int16_t ID;
    LK_Err_e Err;
    /**接收参数 */
    uint8_t anglePidKp;
    uint8_t anglePidKi;
    uint8_t speedPidKp;
    uint8_t speedPidKi;
    uint8_t iqPidKp;
    uint8_t iqPidKi;
    int32_t Accel;
    uint16_t encoder;   //0~16383   现在位置 原始+零偏
    uint16_t encoderRaw;    //0~16383 编码器原始位置
    uint16_t encoderOffset; //0~16383 编码器零偏
    int64_t motorAngle; //多圈绝对值角度0.01°/LSB
    uint32_t circleAngle; //单圈绝对值角度0.01°/LSB	数值范围是0~36000*(减速比-1)
    uint8_t temperature;    
    uint16_t voltage;
    uint8_t errorState;
    int16_t iq;     //电流 -2048~+2048  -33A~+33A
    int16_t speed;  //转速 1dps/LSB
    int16_t pow;    //功率 -1000~1000 
    /**设置参数 */
    uint8_t angKp;
    uint8_t angKi;
    uint8_t spdKp;
    uint8_t spdKi;
    uint8_t iqKp;
    uint8_t iqKi;
    int32_t Acc;
    int16_t powerControl;   //开环输出功率  -1000~1000 
    int16_t iqControl;  //闭环控制转矩电流  -2000~2000
    int32_t speedControl; // 对应0.01dps/LSB???
    int32_t angleControl;   // 对应0.01dps/LSB???   即36000 代表360
    int16_t maxSpeed;   
    uint8_t spinDirection;  //0x00顺时针 0x01逆时针
    /**阅读参数 */
    int16_t round_cnt;  //圈数
	float cnt_angle;	//多圈角
    float circle_angle;	//单圈角 0~360
    float circle_anglr_last;    //上一单圈角
	float offset;	//角度偏移量	//类比dji电机操作
}motor_lk_t;

/* LK/MG4310 服务对象。当前用于拨盘，按 RMD / LK 常见 CAN 协议处理。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 电机所在 CAN 句柄。 */
    uint8_t motor_id;                            /* LK/MG 电机 ID，StdId 通常为 0x140 + ID。 */
    uint32_t std_id;                             /* LK/MG 电机收发使用的标准 ID。 */
    uint8_t initialized;                         /* 是否已经用第一帧建立单圈编码器零点。 */
    uint16_t raw_ecd;                            /* 当前 16 bit 单圈编码器值。 */
    uint16_t last_raw_ecd;                       /* 上一帧 16 bit 单圈编码器值。 */
    uint16_t ecd_bias;                           /* 上电后第一帧编码器值，作为相对零点。 */
    int16_t ecd_delta;                           /* 本帧相对上一帧的最短编码器增量。 */
    int32_t round_count;                         /* 根据编码器跨零累计出的圈数。 */
    int32_t total_ecd;                           /* 从上电零点开始累计的连续编码器计数。 */
    int16_t iq_output;                           /* 上层缓存的 LK/MG iq/力矩控制输出。 */
    volatile uint8_t online;                     /* 最近一段时间内是否收到反馈。 */
    volatile uint8_t output_enabled;             /* 已经发送过上电/力矩控制，且未发送 stop。 */
    volatile uint32_t last_update_tick;          /* 最近一次收到反馈的 HAL tick。 */
    volatile uint8_t temperature;                /* 状态 2 反馈温度。 */
    volatile int16_t iq_feedback;                /* 状态 2 反馈 iq/力矩电流。 */
    volatile float speed_dps;                    /* 状态 2 反馈速度，单位按协议记为 deg/s。 */
    volatile float angle_deg;                    /* 以 ecd_bias 为零点的相对角度，单位 deg。 */
    volatile float total_angle_deg;              /* 从上电开始累计的连续角度，单位 deg。 */
} lk_motor_service_t;

void LK_InfoRecv_Process(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Send_Command(CAN_HandleTypeDef *hcan,int16_t ID,LK_Command_e LK_Command);
void LK_PID_Param_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Write_PID_Param_RAM(CAN_HandleTypeDef *hcan,int16_t ID,
	int8_t angKp,int8_t angKi,
	int8_t spdKp,int8_t spdKi,
	int8_t iqKp, int8_t iqKi);
void LK_Write_PID_Param_ROM(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_Acc_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Write_Acc_RAM_Set(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_Encoder_Data_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Encoder_Zero_Offset_ROM_Set(CAN_HandleTypeDef *hcan,int16_t ID,uint16_t offset);
void LK_Encoder_Zero_ROM_Set(CAN_HandleTypeDef *hcan,int16_t ID);
void LK_Encoder_Offset_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Multi_Angle_Absolute_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Single_Angle_Absolute_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Status1_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Status2_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Status3_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_T_Openloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_T_Openloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_T_Closeloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_T_Closeloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Speed_Closeloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_Speed_Closeloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Angle_Closeloop_Control_Command(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_Angle_Closeloop_Control_Recv(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Angle_Closeloop_Control_Command2(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_Angle_Closeloop_Control_Recv2(motor_lk_t *p_motor,uint8_t *p_data);
void LK_Angle_Closeloop_Control_Command3(CAN_HandleTypeDef *hcan,int16_t ID,motor_lk_t *p_motor);
void LK_Angle_Closeloop_Control_Recv3(motor_lk_t *p_motor,uint8_t *p_data);

/* 清空 LK/MG4310 服务注册表。通常在 BSP_Init() 中调用一次。 */
void Motor_LkServiceInit(void);

/* 注册一个 LK/MG4310 电机。StdId = 0x140 + motor_id。 */
HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                   CAN_HandleTypeDef *hcan,
                                   uint8_t motor_id);

/* 缓存 LK/MG4310 iq/力矩输出；实际发送由 Motor_LkSendIqControl() 完成。 */
void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t output);

/* 发送 LK/MG4310 上电 / 输出使能命令 0x88。 */
HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor);

/* 发送 LK/MG4310 停止命令 0x81。 */
HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor);

/* 发送 LK/MG4310 状态 2 查询命令 0x9C。 */
HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor);

/* 发送 LK/MG4310 iq/力矩控制命令 0xA1。 */
HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor);

/* 刷新并返回 LK/MG4310 在线状态。 */
uint8_t Motor_LkIsOnline(const lk_motor_service_t *motor);

/* CAN 接收帧分发入口。BSP/CANBus_Task 收到 CAN 帧后调用。 */
void Motor_LkProcessCanMessage(CAN_HandleTypeDef *hcan,
                               const CAN_RxHeaderTypeDef *rx_header,
                               const uint8_t rx_data[8]);

#endif
