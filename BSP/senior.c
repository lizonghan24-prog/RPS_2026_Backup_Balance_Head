#include "senior.h"

/*
 * senior.c 是旧框架里的“嵌入层导读文件”。
 *

 * 这个文件保留通用全局对象和在线检测函数，方便旧算法或调试代码继续使用。
 */

#define SENIOR_ONLINE_TIMEOUT_MS                100U

/* ========================= 通用全局对象定义 ========================= */
general_gyro_t gimbal_gyro = {0};
general_gyro_t chassis_gyro = {0};

steering_wheel_t steering_wheel_chassis = {0};
Mecanum_wheel_t Mecanum_chassis = {0};
Omni_wheel_t Omni_wheel_chassis = {0};
balance_t balance_chassis = {0};

volatile Encoder Pitch_Encoder = {0};
volatile Encoder yaw_Encoder = {0};
hero_small_gimbal_t hero_small_gimbal = {0};

friction_t general_friction = {0};
poke_t general_poke = {0};

/*
 * 判断某个心跳时间戳是否还在有效期内。
 *
 */
static uint8_t Senior_IsTickRecent(uint32_t heart_cnt)
{
    return (uint8_t)((HAL_GetTick() - heart_cnt) <= SENIOR_ONLINE_TIMEOUT_MS);
}

/*
 * 编码器在线检测。
 *
 * 
 * - 收到该编码器对应的 CAN 反馈帧时，把 v->cal_data.heart_cnt 写成 HAL_GetTick()；
 * - 周期性调用 online_detective(&encoder)；
 * - 函数会更新 encoder.if_online。
 */
void online_detective(volatile Encoder *v)
{
    if (v == NULL)
    {
        return;
    }

    v->if_online = Senior_IsTickRecent(v->cal_data.heart_cnt);
}

/*
 * 通用在线检测。
 *
 * 适用于遥控、串口模块或任何只需要心跳时间戳的模块。
 * 返回值：1 在线，0 离线。
 */
uint8_t remote_online_detective(uint32_t heart_cnt)
{
    return Senior_IsTickRecent(heart_cnt);
}

/*
 * 软件复位。
 *
 * 该函数会触发 Cortex-M 内核系统复位，效果接近按下复位键。
 */
void SoftReset(void)
{
    NVIC_SystemReset();
}	
