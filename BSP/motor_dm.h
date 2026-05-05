/**
 * @file motor_dm.h
 * @author ishmael
 * @brief send 
 *          MIT CAN_ID
 *          POS&SPD 0x100+CAN_ID   
 *          SPD 0x200+CAN_ID
 *        recv 
 *          masterID 
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "stm32f4xx_hal.h"

/* 老达妙驱动的参数宏会直接用 PI，头文件里兜底定义，避免调用方还要自己补。 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* DM4310 MIT 帧里位置和速度使用弧度单位，任务层可以直接读取 deg / deg/s 便捷字段。 */
#define DM4310_POS_MIN_RAD                (-PI)
#define DM4310_POS_MAX_RAD                (PI)
#define DM4310_VEL_MIN_RAD_PER_S          (-45.0f)
#define DM4310_VEL_MAX_RAD_PER_S          (45.0f)
#define DM4310_KP_MIN                     0.0f
#define DM4310_KP_MAX                     500.0f
#define DM4310_KD_MIN                     0.0f
#define DM4310_KD_MAX                     5.0f
#define DM4310_TORQUE_MIN_NM              (-10.0f)
#define DM4310_TORQUE_MAX_NM              (10.0f)

//pmin pmax vmin vmax kpmin kpmax kdmin kdmax tmin tmax
#define DM4310_PARAM    -12.5f,12.5f,-30.0f,30.0f,0,0,0,0,-10.0f,10.0f
#define DM4310_PARAM_YAW_BIG    -PI,PI,-45.0f,45.0f,0,0,0,0,-10.0f,10.0f
//#define DM4310_PARAM_PITCH      -PI,PI,-5000.0f,5000.0f,0,0,0,0,-5.0f,5.0f  //位置速度模式参数
#define DM4310_PARAM_PITCH      -PI,PI,-500.0f,500.0f,0,0,0,0,-8.0f,8.0f  //MIT模式参数
//达妙电机需要多圈时 在上位机将Pmax设置为PI的倍数 /原本默认12.5多圈有明显误差 一圈约365°

typedef enum
{
    DM_DISABLE         =0,  //失能
    DM_ENABLE          =1,  //使能
    DM_U_MAX           =8,  //超压
    DM_U_MIN           =9,  //欠压
    DM_I_MAX           =0xA,    //过流
    DM_MOS_TEM_MAX     =0xB,    //mos过温
    DM_ROTOR_TEM_MAX   =0xC,    //线圈过温
    DM_CAN_BAG_LOST    =0xD,    //丢包
    DM_OVERLOAD        =0xE,    //过载
}DaMiao_Err_e;
typedef enum
{
    DM_MIT          =0x000, //mit模式
    DM_POS          =0x100, //位置速度模式
    DM_SPD          =0x200, //速度模式
    DM_eMIT         =0x300, //力位混控模式
}DaMiao_Mode_e;

typedef struct
{
    float P_MIN;
    float P_MAX;
    float V_MIN;
    float V_MAX;
    float KP_MIN;
    float KP_MAX;
    float KD_MIN;
    float KD_MAX;
    float T_MIN;
    float T_MAX;
}DaMiao_Param_t;

typedef struct
{
    int16_t ID;
    DaMiao_Err_e Err;
    DaMiao_Mode_e Mode;
    DaMiao_Param_t Param;//必须初始化

    /**达妙电机数据返回 */
    float POS_Fdb;  //弧度制
    float VEL_Fdb;  //rad/s (omega,w)
    float T_Fdb;    //N·M
    uint8_t T_MOS;
    uint8_t T_Rotor;

    //部分处理后数据    阅读 
    float angle_abs;    //单圈角 角度制 都是角度制
    float angle_last;   //上次单圈角
    float angle_cnt;    //多圈角
    float angle_offset; //偏移量    //初始化设置
    float round_cnt;    //圈数
    float speed_dps;    //degree/seconds
    float t_fdb;    //放缩10000倍 方便调参
    float t_ref;    //放缩10000倍
    /**达妙电机数据发送 */
    float Kp;
    float Kd;
    float POS_Ref;  //角度制
    float VEL_Ref;
    float T_Ref;
    float I_Ref;
}motor_dm_t;

/* DM4310 MIT 协议的参数范围。编码和解码都依赖这组上下限。 */
typedef struct
{
    float p_min_rad;                            /* MIT 位置下限，单位 rad。 */
    float p_max_rad;                            /* MIT 位置上限，单位 rad。 */
    float v_min_rad_per_s;                      /* MIT 速度下限，单位 rad/s。 */
    float v_max_rad_per_s;                      /* MIT 速度上限，单位 rad/s。 */
    float kp_min;                               /* MIT Kp 下限。 */
    float kp_max;                               /* MIT Kp 上限。 */
    float kd_min;                               /* MIT Kd 下限。 */
    float kd_max;                               /* MIT Kd 上限。 */
    float torque_min_nm;                        /* MIT 力矩下限，单位 N*m。 */
    float torque_max_nm;                        /* MIT 力矩上限，单位 N*m。 */
} dm4310_mit_param_t;

/* DM4310 服务对象。当前用于倒装 yaw 轴，走 MIT 力矩控制。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;                    /* 电机所在 CAN 句柄，例如 &hcan2。 */
    uint8_t motor_id;                           /* 达妙电机自身 ID，当前 yaw 为 ID1。 */
    uint32_t command_std_id;                    /* MIT 控制帧标准 ID，默认等于 motor_id。 */
    uint32_t feedback_std_id;                   /* 反馈帧标准 ID；如上位机改 MasterID，需要同步改这里。 */
    dm4310_mit_param_t param;                   /* MIT 编码/解码使用的参数范围。 */
    float feedback_angle_sign;                  /* 反馈角度方向补偿；倒装 yaw 设为 -1 后与 IMU yaw 同向。 */
    float feedback_speed_sign;                  /* 反馈速度方向补偿；倒装 yaw 设为 -1 后与 IMU gyro 同向。 */
    float position_ref_rad;                     /* 发送 MIT 帧时的位置给定，单位 rad。 */
    float velocity_ref_rad_per_s;               /* 发送 MIT 帧时的速度给定，单位 rad/s。 */
    float kp_ref;                               /* 发送 MIT 帧时的 Kp。力矩模式下通常为 0。 */
    float kd_ref;                               /* 发送 MIT 帧时的 Kd。力矩模式下通常为 0。 */
    float torque_ref_nm;                        /* 发送 MIT 帧时的力矩给定，单位 N*m。 */
    uint8_t err;                                /* 反馈帧高 4 bit 错误码。 */
    uint8_t t_mos;                              /* MOS 温度反馈。 */
    uint8_t t_rotor;                            /* 转子温度反馈。 */
    volatile uint8_t online;                    /* 最近一段时间内是否收到反馈。 */
    volatile uint8_t output_enabled;            /* 已发送过使能或 MIT 控制，且未发送失能。 */
    volatile uint32_t last_update_tick;         /* 最近一次收到反馈的 HAL tick。 */
    volatile float position_rad;                /* MIT 反馈位置，单位 rad。 */
    volatile float velocity_rad_per_s;          /* MIT 反馈速度，单位 rad/s。 */
    volatile float torque_nm;                   /* MIT 反馈力矩，单位 N*m。 */
    volatile float angle_deg;                   /* position_rad 换算出的角度，单位 deg。 */
    volatile float speed_dps;                   /* velocity_rad_per_s 换算出的角速度，单位 deg/s。 */
} dm4310_service_t;

void DaMiao_Motor_Init(motor_dm_t *p_motor,
                            DaMiao_Mode_e mode,
                            float P_min,float P_max,
                            float V_min,float V_max,
                            float Kp_min,float Kp_max,
                            float Kd_min,float Kd_max,
                            float t_min,float t_max
                            );
void DaMiao_InfoRecv_Process(motor_dm_t *p_motor,uint8_t *p_data);
void DaMiao_Position_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor);
void DaMiao_MIT_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor);
void DaMiao_Speed_Send(CAN_HandleTypeDef *hcan,uint16_t ID,motor_dm_t *p_motor);
void DaMiao_Enable(CAN_HandleTypeDef *hcan,uint16_t ID);
void DaMiao_Disable(CAN_HandleTypeDef *hcan,uint16_t ID);
void DaMiao_Zero_Set(CAN_HandleTypeDef *hcan,uint16_t ID);
void DaMiao_Error_Clear(CAN_HandleTypeDef *hcan,uint16_t ID);

/* 清空 DM4310 服务注册表。通常在 BSP_Init() 中调用一次。 */
void Motor_Dm4310ServiceInit(void);

/* 注册一个 DM4310 MIT 协议电机。feedback_std_id 需要和达妙上位机 MasterID 一致。 */
HAL_StatusTypeDef Motor_RegisterDm4310Mit(dm4310_service_t *motor,
                                          CAN_HandleTypeDef *hcan,
                                          uint8_t motor_id,
                                          uint32_t feedback_std_id);

/* 缓存 DM4310 MIT 力矩输出；实际发送由 Motor_Dm4310SendMitControl() 完成。 */
void Motor_SetDm4310MitTorque(dm4310_service_t *motor, float torque_nm);

/* 发送 DM4310 使能帧：FF FF FF FF FF FF FF FC。 */
HAL_StatusTypeDef Motor_Dm4310SendEnable(dm4310_service_t *motor);

/* 发送 DM4310 失能帧：FF FF FF FF FF FF FF FD。 */
HAL_StatusTypeDef Motor_Dm4310SendDisable(dm4310_service_t *motor);

/* 发送 DM4310 MIT 控制帧，命令值来自 motor 结构体里的缓存字段。 */
HAL_StatusTypeDef Motor_Dm4310SendMitControl(dm4310_service_t *motor);

/* 刷新并返回 DM4310 在线状态。 */
uint8_t Motor_Dm4310IsOnline(const dm4310_service_t *motor);

/* CAN 接收帧分发入口。BSP/CANBus_Task 收到 CAN 帧后调用。 */
void Motor_Dm4310ProcessCanMessage(CAN_HandleTypeDef *hcan,
                                   const CAN_RxHeaderTypeDef *rx_header,
                                   const uint8_t rx_data[8]);

#endif
