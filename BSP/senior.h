#ifndef __SENIOR_H
#define __SENIOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * senior 层是给上层算法看的“通用数据接口层”。
 *
 * 这个文件保留老框架里的通用结构体命名：
 * - Encoder / Encoder_cal：通用编码器反馈结构；
 * - general_gyro_t：通用陀螺仪/姿态反馈；
 * - steering_wheel_t / Mecanum_wheel_t / Omni_wheel_t / balance_t：不同底盘结构；
 * - friction_t / poke_t / hero_small_gimbal_t：发射、拨盘和小云台结构。
 *
 * 当前工程的实时控制已经主要走 BSP/motor.h 里的 HAL 电机服务对象。
 * senior 层更多用于兼容旧算法、集中摆放通用对象和做简单在线检测。
 */

/* ========================= GM6020 / 舵向电机机械零点配置 ========================= */
/* Pitch 轴 GM6020 编码器初始机械零点。 */
#define GMPitchEncoder_Offset                   6165
/* Yaw 轴 GM6020 编码器初始机械零点。 */
#define GMYawEncoder_Offset                     1412
/* 四个舵轮底盘转向电机的编码器初始机械零点。 */
#define GM1Encoder_Offset                       1437
#define GM2Encoder_Offset                       8042
#define GM3Encoder_Offset                       4141
#define GM4Encoder_Offset                       6732
/* 平步/关节类底盘电机的编码器初始机械零点。 */
#define JM1Encoder_Offset                       52321
#define JM2Encoder_Offset                       6143
#define JM3Encoder_Offset                       7456
#define JM4Encoder_Offset                       4570
/* 预留电机零点，默认未使用。 */
#define TM1Encoder_Offset                       0
#define TM2Encoder_Offset                       0

/* ========================= 通用编码器结构 ========================= */
#ifndef STRUCT_MOTOR
#define STRUCT_MOTOR

/* 编码器速度滑动平均缓冲长度。 */
#define RATE_BUF_SIZE                           6

typedef struct
{
    int32_t raw_value;                          /* 当前帧编码器原始值，未做跨零处理。 */
    int32_t last_raw_value;                     /* 上一帧编码器原始值。 */
    int32_t ecd_value;                          /* 跨零处理后的连续编码器值。 */
    int32_t diff;                               /* 当前帧与上一帧的原始差值。 */
    int32_t temp_count;                         /* 初始化或临时统计用计数。 */
    uint8_t buf_count;                          /* 速度滤波缓冲写入位置。 */
    int32_t ecd_bias;                           /* 上电或标定时记录的编码器零点。 */
    int32_t ecd_raw_rate;                       /* 由编码器差分得到的原始速度。 */
    int32_t rate_buf[RATE_BUF_SIZE];            /* 速度滤波缓冲。 */
    int32_t round_cnt;                          /* 编码器跨零累计圈数。 */
    int32_t can_cnt;                            /* 接收反馈帧次数，常用于初始化阶段判断。 */
    uint32_t heart_cnt;                         /* 最近一次收到有效反馈的 HAL tick。 */
} Encoder_cal;

typedef struct
{
    Encoder_cal cal_data;                       /* 编码器解析过程中的中间量和累计量。 */
    uint8_t if_online;                          /* 在线标志，1 在线，0 离线。 */
    int32_t filter_rate;                        /* 滤波后的速度原始值。 */
    double ecd_angle;                           /* 编码器相对角度，单位通常为 deg。 */
    int16_t rate_rpm;                           /* 电调反馈转速，单位 rpm。 */
    double angle;                               /* 给上层算法预留的角度反馈。 */
    double gyro;                                /* 给上层算法预留的角速度反馈。 */
    float Torque;                               /* 转矩或电流反馈，具体单位由模块协议决定。 */
    uint32_t temperature;                       /* 温度反馈。 */
} Encoder;

#endif /* STRUCT_MOTOR */

/* ========================= 通用姿态结构 ========================= */
#ifndef GENERAL_GYRO_T
#define GENERAL_GYRO_T
typedef struct
{
    float pitch_Angle;                          /* Pitch 姿态角，单位 deg。 */
    float yaw_Angle;                            /* Yaw 姿态角，单位 deg。 */
    float roll_Angle;                           /* Roll 姿态角，单位 deg。 */
    float pitch_Gyro;                           /* Pitch 轴角速度。 */
    float yaw_Gyro;                             /* Yaw 轴角速度。 */
    float roll_Gyro;                            /* Roll 轴角速度。 */
    float x_Acc;                                /* X 轴加速度。 */
    float y_Acc;                                /* Y 轴加速度。 */
    float z_Acc;                                /* Z 轴加速度。 */
} general_gyro_t;
#endif /* GENERAL_GYRO_T */

/* ========================= 通用底盘结构 ========================= */
typedef struct
{
    volatile Encoder Heading_Encoder[4];        /* 四个舵向/转向编码器。 */
    volatile Encoder Driving_Encoder[4];        /* 四个驱动轮编码器。 */
} steering_wheel_t;

typedef struct
{
    volatile Encoder Driving_Encoder[4];        /* 麦轮底盘四个驱动电机编码器。 */
} Mecanum_wheel_t;

typedef struct
{
    volatile Encoder Driving_Encoder[4];        /* 全向轮底盘四个驱动电机编码器。 */
} Omni_wheel_t;

typedef struct
{
    volatile Encoder joint_Encoder[4];          /* 平衡/关节结构的关节编码器。 */
    volatile Encoder Driving_Encoder[2];        /* 驱动轮编码器，约定左 0 右 1。 */
} balance_t;

/* ========================= 发射与云台结构 ========================= */
typedef struct
{
    volatile Encoder right_up_motor;            /* 右上摩擦轮电机。 */
    volatile Encoder left_up_motor;             /* 左上摩擦轮电机。 */
    volatile Encoder left_down_motor;           /* 左下摩擦轮电机。 */
    volatile Encoder right_down_motor;          /* 右下摩擦轮电机。 */
    volatile Encoder left_motor;                /* 双摩擦轮结构中的左电机。 */
    volatile Encoder right_motor;               /* 双摩擦轮结构中的右电机。 */
} friction_t;

typedef struct
{
    volatile Encoder right_poke;                /* 右拨盘。 */
    volatile Encoder left_poke;                 /* 左拨盘。 */
    volatile Encoder up_poke;                   /* 上拨盘。 */
    volatile Encoder down_poke;                 /* 下拨盘。 */
    volatile Encoder poke;                      /* 单拨盘。 */
} poke_t;

typedef struct
{
    volatile Encoder scope_encoder;             /* 瞄准镜电机编码器。 */
    volatile Encoder small_gimbal_encoder;      /* 小云台电机编码器。 */
} hero_small_gimbal_t;

/* ========================= 全局通用对象声明 ========================= */
extern general_gyro_t gimbal_gyro;
extern general_gyro_t chassis_gyro;
extern steering_wheel_t steering_wheel_chassis;
extern Mecanum_wheel_t Mecanum_chassis;
extern Omni_wheel_t Omni_wheel_chassis;
extern balance_t balance_chassis;
extern volatile Encoder Pitch_Encoder;
extern volatile Encoder yaw_Encoder;
extern hero_small_gimbal_t hero_small_gimbal;
extern friction_t general_friction;
extern poke_t general_poke;

/*
 * 根据编码器对象 cal_data.heart_cnt 判断在线状态。
 * heart_cnt 应在收到对应反馈帧时写入 HAL_GetTick()。
 */
void online_detective(volatile Encoder *v);

/*
 * 通用心跳在线检测。
 * 参数 heart_cnt 为最近一次收到数据的 HAL tick，返回 1 在线，0 离线。
 */
uint8_t remote_online_detective(uint32_t heart_cnt);

/* 软件复位入口，内部调用 NVIC_SystemReset()。 */
void SoftReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENIOR_H */
