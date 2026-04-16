#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define GM6020_ENCODER_CPR          8192U
#define GM6020_GROUP_1_4_STDID      0x1FFU
#define GM6020_GROUP_5_7_STDID      0x2FFU
#define GM6020_REGISTRY_SIZE        8U

#define M3508_ENCODER_CPR           8192U
#define M3508_GROUP_1_4_STDID       0x200U
#define M3508_GROUP_5_8_STDID       0x1FFU
#define M3508_REGISTRY_SIZE         8U

#define LK_MOTOR_ID_BASE            0x140U
#define LK_MOTOR_ENCODER_CPR        65536U
#define LK_MOTOR_REGISTRY_SIZE      4U

/* GM6020 工作模式定义。 */
typedef enum
{
    GM6020_MODE_CURRENT_LOOP = 0U,          /* GM6020 电流环模式。 */
    GM6020_MODE_VOLTAGE_LOOP = 1U           /* GM6020 电压环模式。 */
} gm6020_mode_t;

/* LK 电机命令字定义。 */
typedef enum
{
    LK_MOTOR_CMD_NONE = 0x00U,              /* 空命令，占位使用。 */
    LK_MOTOR_CMD_READ_PID = 0x30U,          /* 读取 PID 参数。 */
    LK_MOTOR_CMD_READ_ACC = 0x33U,          /* 读取加速度参数。 */
    LK_MOTOR_CMD_WRITE_ACC_RAM = 0x34U,     /* 写入加速度到 RAM。 */
    LK_MOTOR_CMD_MOTOR_OFF = 0x80U,         /* 电机掉使能。 */
    LK_MOTOR_CMD_MOTOR_STOP = 0x81U,        /* 电机停止。 */
    LK_MOTOR_CMD_MOTOR_ON = 0x88U,          /* 电机使能。 */
    LK_MOTOR_CMD_READ_ENC = 0x90U,          /* 读取编码器数据。 */
    LK_MOTOR_CMD_READ_MULTI_ANGLE = 0x92U,  /* 读取多圈角度。 */
    LK_MOTOR_CMD_READ_SINGLE_ANGLE = 0x94U, /* 读取单圈角度。 */
    LK_MOTOR_CMD_CLEAR_ANGLE = 0x95U,       /* 清零角度。 */
    LK_MOTOR_CMD_READ_STATE_1 = 0x9AU,      /* 读取状态 1。 */
    LK_MOTOR_CMD_CLEAR_ERROR = 0x9BU,       /* 清除错误标志。 */
    LK_MOTOR_CMD_READ_STATE_2 = 0x9CU,      /* 读取状态 2。 */
    LK_MOTOR_CMD_READ_STATE_3 = 0x9DU,      /* 读取状态 3。 */
    LK_MOTOR_CMD_POWER_REPLY = 0xA0U,       /* 功率回包。 */
    LK_MOTOR_CMD_CONTROL_TORQUE = 0xA1U,    /* 转矩/电流闭环控制。 */
    LK_MOTOR_CMD_CONTROL_SPEED = 0xA2U,     /* 速度闭环控制。 */
    LK_MOTOR_CMD_CONTROL_POS_1 = 0xA3U,     /* 位置闭环控制 1。 */
    LK_MOTOR_CMD_CONTROL_POS_2 = 0xA4U,     /* 位置闭环控制 2。 */
    LK_MOTOR_CMD_CONTROL_POS_3 = 0xA5U,     /* 位置闭环控制 3。 */
    LK_MOTOR_CMD_CONTROL_POS_4 = 0xA6U,     /* 位置闭环控制 4。 */
    LK_MOTOR_CMD_CONTROL_POS_5 = 0xA7U,     /* 位置闭环控制 5。 */
    LK_MOTOR_CMD_CONTROL_POS_6 = 0xA8U      /* 位置闭环控制 6。 */
} lk_motor_cmd_t;

/* 单个 GM6020 服务对象。 */
typedef struct
{
    uint8_t registered;                     /* 是否已经注册到电机服务表。 */
    CAN_HandleTypeDef *hcan;                /* 该电机挂载的 CAN 句柄。 */
    uint8_t motor_id;                       /* 电机逻辑 ID，范围 1~7。 */
    uint16_t rx_std_id;                     /* 该电机反馈标准帧 ID。 */
    uint16_t tx_std_id;                     /* 该电机所属控制帧标准 ID。 */
    uint8_t tx_slot;                        /* 该电机在 8 字节控制帧中的槽位索引。 */
    gm6020_mode_t mode;                     /* 当前服务对象对应的控制模式。 */
    int16_t target_output;                  /* 当前缓存的目标输出。 */
    uint8_t online;                         /* 在线标志，超时后会被刷新为 0。 */
    uint8_t initialized;                    /* 是否已经收到过第一帧反馈。 */
    uint16_t ecd;                           /* 当前编码器原始值。 */
    int16_t speed_rpm;                      /* 当前转速，单位 rpm。 */
    int16_t given_current;                  /* 电调反馈电流。 */
    uint8_t temperature;                    /* 电机温度。 */
    int32_t total_ecd;                      /* 累计编码器增量。 */
    int16_t last_ecd;                       /* 上一帧编码器原始值。 */
    float angle_deg;                        /* 当前单圈角度，单位 deg。 */
    float total_angle_deg;                  /* 当前累计角度，单位 deg。 */
    uint32_t last_update_tick;              /* 最近一次收到反馈的系统时刻。 */
    uint32_t frame_count;                   /* 成功解析的反馈帧数量。 */
} gm6020_service_t;

/* 单个 M3508 服务对象。 */
typedef struct
{
    uint8_t registered;                     /* 是否已经注册到电机服务表。 */
    CAN_HandleTypeDef *hcan;                /* 该电机挂载的 CAN 句柄。 */
    uint8_t motor_id;                       /* 电机逻辑 ID，范围 1~8。 */
    uint16_t rx_std_id;                     /* 该电机反馈标准帧 ID。 */
    uint16_t tx_std_id;                     /* 该电机所属控制帧标准 ID。 */
    uint8_t tx_slot;                        /* 该电机在 8 字节控制帧中的槽位索引。 */
    int16_t target_output;                  /* 当前缓存的目标电流输出。 */
    uint8_t online;                         /* 在线标志，超时后会被刷新为 0。 */
    uint8_t initialized;                    /* 是否已经收到过第一帧反馈。 */
    uint16_t ecd;                           /* 当前编码器原始值。 */
    int16_t speed_rpm;                      /* 当前转速，单位 rpm。 */
    int16_t given_current;                  /* 电调反馈电流。 */
    uint8_t temperature;                    /* 电机温度。 */
    int32_t total_ecd;                      /* 累计编码器增量。 */
    int16_t last_ecd;                       /* 上一帧编码器原始值。 */
    float angle_deg;                        /* 当前单圈角度，单位 deg。 */
    float total_angle_deg;                  /* 当前累计角度，单位 deg。 */
    uint32_t last_update_tick;              /* 最近一次收到反馈的系统时刻。 */
    uint32_t frame_count;                   /* 成功解析的反馈帧数量。 */
} m3508_service_t;

/* 单个 LK 电机服务对象。 */
typedef struct
{
    uint8_t registered;                     /* 是否已经注册到电机服务表。 */
    CAN_HandleTypeDef *hcan;                /* 该电机挂载的 CAN 句柄。 */
    uint8_t motor_id;                       /* 电机逻辑 ID，范围 1~32。 */
    uint16_t rx_std_id;                     /* 该电机反馈标准帧 ID。 */
    uint8_t online;                         /* 在线标志，超时后会被刷新为 0。 */
    uint8_t initialized;                    /* 是否已经收到过带编码器的第一帧反馈。 */
    uint8_t output_enabled;                 /* 当前是否认为电机处于输出使能状态。 */
    lk_motor_cmd_t last_tx_cmd;             /* 最近一次发送的命令字。 */
    lk_motor_cmd_t last_rx_cmd;             /* 最近一次解析到的命令字。 */
    int16_t target_iq;                      /* 当前缓存的目标 iq/转矩命令。 */
    float target_speed_dps;                 /* 当前缓存的目标速度命令，单位 deg/s。 */
    float target_angle_deg;                 /* 当前缓存的目标角度命令，单位 deg。 */
    uint16_t target_max_speed_dps;          /* 位置控制命令附带的最大速度限制。 */
    int16_t angle_kp;                       /* 电机内部角度环 Kp 反馈值。 */
    int16_t angle_ki;                       /* 电机内部角度环 Ki 反馈值。 */
    int16_t speed_kp;                       /* 电机内部速度环 Kp 反馈值。 */
    int16_t speed_ki;                       /* 电机内部速度环 Ki 反馈值。 */
    int16_t iq_kp;                          /* 电机内部电流环 Kp 反馈值。 */
    int16_t iq_ki;                          /* 电机内部电流环 Ki 反馈值。 */
    int32_t accel;                          /* 电机内部加速度参数反馈值。 */
    uint16_t encoder;                       /* 当前编码器原始值。 */
    uint16_t encoder_raw;                   /* 读取编码器命令返回的原始值。 */
    uint16_t encoder_offset;                /* 编码器零偏。 */
    int32_t total_ecd;                      /* 通过编码器回绕累计出的总增量。 */
    int32_t last_ecd;                       /* 上一帧编码器原始值。 */
    float circle_angle_deg;                 /* 当前单圈角度，单位 deg。 */
    float total_angle_deg;                  /* 通过编码器累计出的总角度，单位 deg。 */
    float reply_multi_angle_deg;            /* 读取多圈角命令返回的多圈角度，单位 deg。 */
    float reply_single_angle_deg;           /* 读取单圈角命令返回的单圈角度，单位 deg。 */
    int8_t temperature_c;                   /* 当前温度，单位摄氏度。 */
    float voltage_v;                        /* 当前母线电压，单位 V。 */
    uint8_t error_state;                    /* 错误标志位原始字节。 */
    uint8_t voltage_low;                    /* 低压标志。 */
    uint8_t over_temp;                      /* 过温标志。 */
    float torque_nm;                        /* 当前反馈转矩，单位 N*m。 */
    float speed_dps;                        /* 当前反馈速度，单位 deg/s。 */
    int16_t power_w;                        /* 当前反馈功率，单位 W。 */
    uint32_t last_update_tick;              /* 最近一次收到反馈的系统时刻。 */
    uint32_t frame_count;                   /* 成功解析的反馈帧数量。 */
} lk_motor_service_t;

/* 初始化电机服务框架，清空注册表。 */
void Motor_Init(void);

/* 注册一个 GM6020 电流环服务对象。 */
HAL_StatusTypeDef Motor_RegisterGm6020CurrentLoop(gm6020_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id);

/* 注册一个 GM6020 电压环服务对象。 */
HAL_StatusTypeDef Motor_RegisterGm6020VoltageLoop(gm6020_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id);

/* 注册一个 M3508 电流环服务对象。 */
HAL_StatusTypeDef Motor_RegisterM3508CurrentLoop(m3508_service_t *motor,
                                                 CAN_HandleTypeDef *hcan,
                                                 uint8_t motor_id);

/* 注册一个 LK 电机服务对象。 */
HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                   CAN_HandleTypeDef *hcan,
                                   uint8_t motor_id);

/* 设置 GM6020 电流环输出。 */
void Motor_SetGm6020CurrentLoopOutput(gm6020_service_t *motor, int16_t output);

/* 设置 GM6020 电压环输出。 */
void Motor_SetGm6020VoltageLoopOutput(gm6020_service_t *motor, int16_t output);

/* 设置 M3508 电流环输出。 */
void Motor_SetM3508CurrentLoopOutput(m3508_service_t *motor, int16_t output);

/* 设置 LK 电机 iq/转矩命令缓存。 */
void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t iq);

/* 发送一帧 GM6020 电流环控制帧，支持同一控制组内最多 4 个服务对象。 */
HAL_StatusTypeDef Motor_SendGm6020CurrentLoopFrame(const gm6020_service_t *motor_a,
                                                   const gm6020_service_t *motor_b,
                                                   const gm6020_service_t *motor_c,
                                                   const gm6020_service_t *motor_d);

/* 发送一帧 GM6020 电压环控制帧，支持同一控制组内最多 4 个服务对象。 */
HAL_StatusTypeDef Motor_SendGm6020VoltageLoopFrame(const gm6020_service_t *motor_a,
                                                   const gm6020_service_t *motor_b,
                                                   const gm6020_service_t *motor_c,
                                                   const gm6020_service_t *motor_d);

/* 发送一帧 M3508 电流环控制帧，支持同一控制组内最多 4 个服务对象。 */
HAL_StatusTypeDef Motor_SendM3508CurrentLoopFrame(const m3508_service_t *motor_a,
                                                  const m3508_service_t *motor_b,
                                                  const m3508_service_t *motor_c,
                                                  const m3508_service_t *motor_d);

/* LK 电机基础命令：使能输出。 */
HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor);

/* LK 电机基础命令：关闭输出。 */
HAL_StatusTypeDef Motor_LkSendPowerOff(lk_motor_service_t *motor);

/* LK 电机基础命令：停止电机。 */
HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor);

/* LK 电机基础命令：清除错误。 */
HAL_StatusTypeDef Motor_LkSendClearError(lk_motor_service_t *motor);

/* LK 电机查询命令：读取编码器。 */
HAL_StatusTypeDef Motor_LkSendReadEncoderRequest(lk_motor_service_t *motor);

/* LK 电机查询命令：读取多圈角度。 */
HAL_StatusTypeDef Motor_LkSendReadMultiAngleRequest(lk_motor_service_t *motor);

/* LK 电机查询命令：读取单圈角度。 */
HAL_StatusTypeDef Motor_LkSendReadSingleAngleRequest(lk_motor_service_t *motor);

/* LK 电机查询命令：读取状态 1。 */
HAL_StatusTypeDef Motor_LkSendReadState1Request(lk_motor_service_t *motor);

/* LK 电机查询命令：读取状态 2。 */
HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor);

/* LK 电机查询命令：读取状态 3。 */
HAL_StatusTypeDef Motor_LkSendReadState3Request(lk_motor_service_t *motor);

/* LK 电机控制命令：发送当前缓存的 iq/转矩控制量。 */
HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor);

/* LK 电机控制命令：直接按给定 iq/转矩发送。 */
HAL_StatusTypeDef Motor_LkSendIqControlValue(lk_motor_service_t *motor, int16_t iq);

/* LK 电机控制命令：按内部速度环发送目标速度。 */
HAL_StatusTypeDef Motor_LkSendSpeedControl(lk_motor_service_t *motor, float speed_dps);

/* LK 电机控制命令：按内部位置环发送目标角度。 */
HAL_StatusTypeDef Motor_LkSendAngleControl(lk_motor_service_t *motor, float angle_deg);

/* LK 电机控制命令：按内部位置环发送目标角度并附带速度上限。 */
HAL_StatusTypeDef Motor_LkSendAngleSpeedLimitedControl(lk_motor_service_t *motor,
                                                       uint16_t max_speed_dps,
                                                       float angle_deg);

/* 将一帧 CAN 反馈分发给已经注册的电机服务对象。 */
void Motor_ProcessCanMessage(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *header,
                             const uint8_t data[8]);

/* 刷新并返回 GM6020 在线状态。 */
uint8_t Motor_Gm6020IsOnline(gm6020_service_t *motor);

/* 刷新并返回 M3508 在线状态。 */
uint8_t Motor_M3508IsOnline(m3508_service_t *motor);

/* 刷新并返回 LK 电机在线状态。 */
uint8_t Motor_LkIsOnline(lk_motor_service_t *motor);

#endif
