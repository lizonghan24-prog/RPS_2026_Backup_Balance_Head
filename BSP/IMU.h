#ifndef IMU_H
#define IMU_H

#include "main.h"

/*
 * IMU module quick usage:
 * - Call IMU_Init() once at startup.
 * - Feed serial bytes using IMU_Process(data, len).
 * - Read latest parsed HI91 state by IMU_GetState().
 * - Use IMU_IsOnline() before trusting angle/rate feedback.
 * - Optional: call IMU_SendRecommendedConfig() once after boot.
 */

#define IMU_FRAME_SOF_0       0x5AU
#define IMU_FRAME_SOF_1       0xA5U
#define IMU_HI91_TAG          0x91U
#define IMU_HI91_PAYLOAD_LEN  76U

/* HI91 数据帧解析后的关键姿态信息。 */
typedef struct
{
    uint8_t online;                       /* 在线标志，超时后会被清零。 */
    uint8_t updated;                      /* 本轮是否收到过新 IMU 数据。 */
    uint8_t tag;                          /* 当前解析到的数据标签。 */
    uint16_t main_status;                 /* 主状态字。 */
    int8_t temperature_c;                 /* 温度，单位摄氏度。 */
    float air_pressure_pa;                /* 气压，单位 Pa。 */
    uint32_t system_time_ms;              /* 模块内部时间戳，单位 ms。 */
    float acc_g[3];                       /* 三轴加速度。 */
    float gyr_dps[3];                     /* 三轴角速度，单位 deg/s。 */
    float mag_ut[3];                      /* 三轴磁场强度，单位 uT。 */
    float euler_deg[3];                   /* 欧拉角，单位 deg。 */
    float quat[4];                        /* 四元数。 */
    uint32_t last_update_tick;            /* 最近一次成功解帧的系统时刻。 */
    uint32_t frame_count;                 /* 成功解析的 IMU 帧数量。 */
} imu_hi91_t;

/* Reset IMU parser and clear state snapshot. */
void IMU_Init(void);

/* Feed raw UART bytes to IMU frame parser. */
void IMU_Process(const uint8_t *data, uint16_t length);

/* Get pointer to latest parsed IMU state snapshot. */
const imu_hi91_t *IMU_GetState(void);

/* Return 1 when IMU data is fresh within timeout window. */
uint8_t IMU_IsOnline(void);

/* Send one ASCII command to IMU UART (CRLF auto-appended). */
HAL_StatusTypeDef IMU_SendCommand(const char *command);

/* Push recommended IMU config: 921600 baud, HI91, 1000 Hz. */
void IMU_SendRecommendedConfig(void);

#endif
