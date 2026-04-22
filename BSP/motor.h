#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * motor 层只负责三件事：
 * 1. 记录每个电机挂在哪一路 CAN、逻辑 ID 是多少；
 * 2. 把 HAL CAN 接收到的反馈帧分发到对应电机对象；
 * 3. 按 DJI / LK 协议打包控制帧，并通过 HAL_CAN_AddTxMessage() 发出。
 *
 * 上层任务不需要关心具体 StdId，也不需要在中断里手动解析 CAN 数据。
 */

/* DJI 电机编码器一圈 8192 个计数。GM6020 / M3508 都按这个范围解析。 */
#define MOTOR_DJI_ENCODER_RANGE                 8192U
#define MOTOR_DJI_ENCODER_HALF_RANGE            4096
#define MOTOR_DJI_ENCODER_DEG_PER_TICK          (360.0f / (float)MOTOR_DJI_ENCODER_RANGE)

/* LK 电机状态 2 帧里的单圈编码器通常是 16 bit，按 65536 个计数解析一圈。 */
#define MOTOR_LK_ENCODER_RANGE                  65536U
#define MOTOR_LK_ENCODER_HALF_RANGE             32768
#define MOTOR_LK_ENCODER_DEG_PER_TICK           (360.0f / (float)MOTOR_LK_ENCODER_RANGE)

/* 超过这个时间没有收到反馈帧，就认为电机掉线。单位 ms，来源是 HAL_GetTick()。 */
#define MOTOR_ONLINE_TIMEOUT_MS                 100U

/*
 * GM6020 支持两类控制帧：
 * - 电流环：0x1FE / 0x2FE，给定值一般按 -16384 ~ 16384 使用；
 * - 电压环：0x1FF / 0x2FF，给定值一般按 -30000 ~ 30000 使用。
 */
typedef enum
{
    MOTOR_GM6020_CONTROL_CURRENT = 0U,
    MOTOR_GM6020_CONTROL_VOLTAGE = 1U
} gm6020_control_mode_t;

/* DJI 电机反馈的通用解析结果。 */
typedef struct
{
    uint8_t initialized;                         /* 是否已经用第一帧建立编码器零点。 */
    uint16_t raw_ecd;                            /* 当前单圈编码器原始值，范围 0~8191。 */
    uint16_t last_raw_ecd;                       /* 上一帧单圈编码器原始值。 */
    uint16_t ecd_bias;                           /* 上电后第一帧编码器值，作为相对零点。 */
    int16_t ecd_delta;                           /* 本帧相对上一帧的最短编码器增量。 */
    int32_t round_count;                         /* 根据编码器跨零判断累计出的圈数。 */
    int32_t total_ecd;                           /* 从上电零点开始累计的连续编码器计数。 */
    int16_t speed_rpm;                           /* 电调反馈转速，单位 rpm。 */
    int16_t given_current;                       /* 电调反馈转矩电流，单位由电调协议定义。 */
    uint8_t temperature;                         /* 电调反馈温度。 */
    float angle_deg;                             /* 以 ecd_bias 为零点的相对角度，单位 deg。 */
    float total_angle_deg;                       /* 从上电开始累计的连续角度，单位 deg。 */
    float speed_dps;                             /* 由 rpm 换算出的角速度，单位 deg/s。 */
} dji_motor_feedback_t;

/* GM6020 服务对象。由上层任务持有，motor 层在 CAN 中断回调里刷新反馈。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 电机所在 CAN 句柄，例如 &hcan1 / &hcan2。 */
    uint8_t motor_id;                            /* 电机拨码 ID，通常为 1~7。 */
    uint32_t feedback_std_id;                    /* 反馈帧标准 ID，GM6020 ID1 对应 0x205。 */
    gm6020_control_mode_t control_mode;          /* 当前注册成电流环还是电压环。 */
    dji_motor_feedback_t feedback;               /* 完整 DJI 反馈解析结果。 */
    int16_t output;                              /* 上层缓存的控制输出，发送时打包进控制帧。 */
    volatile uint8_t online;                     /* 最近 MOTOR_ONLINE_TIMEOUT_MS 内是否收到反馈。 */
    volatile uint32_t last_update_tick;          /* 最近一次收到反馈的 HAL tick。 */
    volatile int16_t speed_rpm;                  /* 便捷字段：反馈转速 rpm。 */
    volatile int16_t given_current;              /* 便捷字段：反馈转矩电流。 */
    volatile uint8_t temperature;                /* 便捷字段：反馈温度。 */
    volatile float angle_deg;                    /* 便捷字段：相对角度 deg。 */
    volatile float total_angle_deg;              /* 便捷字段：连续累计角度 deg。 */
    volatile float speed_dps;                    /* 便捷字段：角速度 deg/s。 */
} gm6020_service_t;

/* M3508 / M2006 使用 C620 / C610 电调，反馈格式与 DJI 通用格式一致。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 电机所在 CAN 句柄。 */
    uint8_t motor_id;                            /* 电机拨码 ID，通常为 1~8。 */
    uint32_t feedback_std_id;                    /* 反馈帧标准 ID，ID1 对应 0x201。 */
    dji_motor_feedback_t feedback;               /* 完整 DJI 反馈解析结果。 */
    int16_t output;                              /* 上层缓存的电流环输出。 */
    volatile uint8_t online;                     /* 最近 MOTOR_ONLINE_TIMEOUT_MS 内是否收到反馈。 */
    volatile uint32_t last_update_tick;          /* 最近一次收到反馈的 HAL tick。 */
    volatile int16_t speed_rpm;                  /* 便捷字段：反馈转速 rpm。 */
    volatile int16_t given_current;              /* 便捷字段：反馈转矩电流。 */
    volatile uint8_t temperature;                /* 便捷字段：反馈温度。 */
    volatile float angle_deg;                    /* 便捷字段：相对角度 deg。 */
    volatile float total_angle_deg;              /* 便捷字段：连续累计角度 deg。 */
    volatile float speed_dps;                    /* 便捷字段：角速度 deg/s。 */
} m3508_service_t;

/* LK 电机服务对象。当前用于拨盘，按 RMD / LK 常见 CAN 协议处理。 */
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 电机所在 CAN 句柄。 */
    uint8_t motor_id;                            /* LK 电机 ID，StdId 通常为 0x140 + ID。 */
    uint32_t std_id;                             /* LK 电机收发使用的标准 ID。 */
    uint8_t initialized;                         /* 是否已经用第一帧建立单圈编码器零点。 */
    uint16_t raw_ecd;                            /* 当前 16 bit 单圈编码器值。 */
    uint16_t last_raw_ecd;                       /* 上一帧 16 bit 单圈编码器值。 */
    uint16_t ecd_bias;                           /* 上电后第一帧编码器值，作为相对零点。 */
    int16_t ecd_delta;                           /* 本帧相对上一帧的最短编码器增量。 */
    int32_t round_count;                         /* 根据编码器跨零累计出的圈数。 */
    int32_t total_ecd;                           /* 从上电零点开始累计的连续编码器计数。 */
    int16_t iq_output;                           /* 上层缓存的 LK iq/力矩控制输出。 */
    volatile uint8_t online;                     /* 最近 MOTOR_ONLINE_TIMEOUT_MS 内是否收到反馈。 */
    volatile uint8_t output_enabled;             /* 已经发送过上电/力矩控制，且未发送 stop。 */
    volatile uint32_t last_update_tick;          /* 最近一次收到反馈的 HAL tick。 */
    volatile uint8_t temperature;                /* LK 状态 2 反馈温度。 */
    volatile int16_t iq_feedback;                /* LK 状态 2 反馈 iq/力矩电流。 */
    volatile float speed_dps;                    /* LK 状态 2 反馈速度，单位按协议记为 deg/s。 */
    volatile float angle_deg;                    /* 以 ecd_bias 为零点的相对角度，单位 deg。 */
    volatile float total_angle_deg;              /* 从上电开始累计的连续角度，单位 deg。 */
} lk_motor_service_t;

/*
 * 初始化 motor 服务层。
 *
 * 作用：
 * - 清空 GM6020 / M3508 / LK 三类电机的注册表；
 * - 不直接启动 CAN，CAN 的过滤器、启动和中断由 BSP_Init() 负责；
 * - 通常在 BSP_Init() 中、上层任务注册电机之前调用一次。
 */
void Motor_Init(void);

/*
 * 注册一个 GM6020 电机，并声明它后续使用电流环控制帧。
 *
 * 参数：
 * - motor：上层任务持有的 GM6020 服务对象；
 * - hcan：电机所在 CAN，例如 &hcan1 或 &hcan2；
 * - motor_id：电机拨码 ID，通常为 1~7。
 *
 * 注册后，Motor_ProcessCanMessage() 会根据 hcan + feedback_std_id 自动刷新该对象。
 */
HAL_StatusTypeDef Motor_RegisterGm6020CurrentLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id);

/*
 * 注册一个 GM6020 电机，并声明它后续使用电压环控制帧。
 *
 * GM6020 电压环使用 0x1FF / 0x2FF 控制帧；电流环使用 0x1FE / 0x2FE。
 * 两种模式的反馈帧 ID 相同，只是发送控制帧的标准 ID 不同。
 */
HAL_StatusTypeDef Motor_RegisterGm6020VoltageLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id);

/*
 * 缓存 GM6020 电流环输出。
 *
 * 这个函数只改 motor->output，不会立刻发 CAN。
 * 真正发帧由 Motor_SendGm6020CurrentLoopFrame() 完成，方便上层把同组电机一次打包发送。
 */
void Motor_SetGm6020CurrentLoopOutput(gm6020_service_t *motor, int16_t output);

/*
 * 缓存 GM6020 电压环输出。
 *
 * 用法与 Motor_SetGm6020CurrentLoopOutput() 相同，只是要求电机注册为电压环模式。
 */
void Motor_SetGm6020VoltageLoopOutput(gm6020_service_t *motor, int16_t output);

/*
 * 发送一帧 GM6020 电流环控制帧。
 *
 * 传入的对象必须在同一路 CAN、同一控制组：
 * - ID1~ID4 会打包到 0x1FE；
 * - ID5~ID7 会打包到 0x2FE。
 *
 * 允许传 NULL，表示该槽位输出 0；但至少要传入一个有效电机对象。
 */
HAL_StatusTypeDef Motor_SendGm6020CurrentLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4);

/*
 * 发送一帧 GM6020 电压环控制帧。
 *
 * 分组规则：
 * - ID1~ID4 会打包到 0x1FF；
 * - ID5~ID7 会打包到 0x2FF。
 */
HAL_StatusTypeDef Motor_SendGm6020VoltageLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4);

/* 刷新并返回 GM6020 在线状态。超过 MOTOR_ONLINE_TIMEOUT_MS 未收到反馈即认为离线。 */
uint8_t Motor_Gm6020IsOnline(const gm6020_service_t *motor);

/*
 * 注册一个 M3508 / M2006 电机，并声明它使用 C620 / C610 电流环控制帧。
 *
 * 参数 motor_id 为电机拨码 ID，通常为 1~8：
 * - ID1~ID4 的控制帧为 0x200；
 * - ID5~ID8 的控制帧为 0x1FF。
 */
HAL_StatusTypeDef Motor_RegisterM3508CurrentLoop(m3508_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id);

/*
 * 缓存 M3508 / M2006 电流环输出。
 *
 * 只修改服务对象里的 output 字段，不立即发送 CAN。
 */
void Motor_SetM3508CurrentLoopOutput(m3508_service_t *motor, int16_t output);

/*
 * 发送一帧 M3508 / M2006 电流环控制帧。
 *
 * 传入的对象必须在同一路 CAN、同一控制组。
 * NULL 表示对应槽位输出 0。
 */
HAL_StatusTypeDef Motor_SendM3508CurrentLoopFrame(const m3508_service_t *motor1,
                                                  const m3508_service_t *motor2,
                                                  const m3508_service_t *motor3,
                                                  const m3508_service_t *motor4);

/* 刷新并返回 M3508 / M2006 在线状态。 */
uint8_t Motor_M3508IsOnline(const m3508_service_t *motor);

/*
 * 注册一个 LK / RMD 协议电机。
 *
 * LK 的收发标准帧 ID 通常为 0x140 + motor_id。
 * 当前工程里拨盘用 CAN2 ID5，即标准帧 ID 0x145。
 */
HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                    CAN_HandleTypeDef *hcan,
                                    uint8_t motor_id);

/*
 * 缓存 LK 电机 iq/力矩输出。
 *
 * 这个函数只写 iq_output，不发 CAN；发帧使用 Motor_LkSendIqControl()。
 */
void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t output);

/* 发送 LK 上电 / 输出使能命令 0x88。 */
HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor);

/* 发送 LK 停止命令 0x81，并在发送成功后清 output_enabled 标志。 */
HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor);

/*
 * 发送 LK 状态 2 查询命令 0x9C。
 *
 * 状态 2 回包里包含温度、iq 反馈、速度和编码器值，
 * 当前拨盘在线检测和反馈刷新主要依赖这条命令。
 */
HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor);

/* 发送 LK iq/力矩控制命令 0xA1，命令值来自 motor->iq_output。 */
HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor);

/* 刷新并返回 LK 电机在线状态。 */
uint8_t Motor_LkIsOnline(const lk_motor_service_t *motor);

/*
 * CAN 接收帧统一分发入口。
 *
 * BSP 在 HAL_CAN_RxFifo0MsgPendingCallback() 中取出 CAN 帧后调用此函数。
 * motor 层会按 hcan + StdId 匹配注册表，找到对应对象后刷新反馈和在线时间戳。
 */
void Motor_ProcessCanMessage(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *rx_header,
                             const uint8_t rx_data[8]);

/*
 * 兼容旧工程里的 Set_* 函数名。
 * 参数仍然保留 CAN1 / CAN2 这种 CAN_TypeDef*，函数内部会转换成 HAL 的
 * CAN_HandleTypeDef*，最终仍然走 HAL_CAN_AddTxMessage()。
 */

/*
 * GM6020 电压模式 ID1~ID4 控制帧。
 * 对应 StdId = 0x1FF，四个参数依次写入 data[0..7]。
 */
void Set_GM6020_IQ1(CAN_TypeDef *CANx,
                    int16_t motor1_iq,
                    int16_t motor2_iq,
                    int16_t motor3_iq,
                    int16_t motor4_iq);

/*
 * GM6020 电压模式 ID5~ID8 控制帧。
 * 对应 StdId = 0x2FF。
 */
void Set_GM6020_IQ2(CAN_TypeDef *CANx,
                    int16_t motor5_iq,
                    int16_t motor6_iq,
                    int16_t motor7_iq,
                    int16_t motor8_iq);

/*
 * GM6020 电流模式 ID1~ID4 控制帧。
 * 对应 StdId = 0x1FE。
 */
void Set_GM6020_Current_IQ1(CAN_TypeDef *CANx,
                            int16_t motor1_iq,
                            int16_t motor2_iq,
                            int16_t motor3_iq,
                            int16_t motor4_iq);

/*
 * GM6020 电流模式 ID5~ID8 控制帧。
 * 对应 StdId = 0x2FE。
 */
void Set_GM6020_Current_IQ2(CAN_TypeDef *CANx,
                            int16_t motor5_iq,
                            int16_t motor6_iq,
                            int16_t motor7_iq,
                            int16_t motor8_iq);

/*
 * C620/C610 电调 ID1~ID4 电流控制帧。
 * 通常用于 M3508 / M2006，对应 StdId = 0x200。
 */
void Set_C620andC610_IQ1(CAN_TypeDef *CANx,
                         int16_t motor1_iq,
                         int16_t motor2_iq,
                         int16_t motor3_iq,
                         int16_t motor4_iq);

/*
 * C620/C610 电调 ID5~ID8 电流控制帧。
 * 通常用于 M3508 / M2006，对应 StdId = 0x1FF。
 */
void Set_C620andC610_IQ2(CAN_TypeDef *CANx,
                         int16_t motor5_iq,
                         int16_t motor6_iq,
                         int16_t motor7_iq,
                         int16_t motor8_iq);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
