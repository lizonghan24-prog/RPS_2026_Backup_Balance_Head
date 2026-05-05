#ifndef USART_CHASSIS_TRANSMIT_H
#define USART_CHASSIS_TRANSMIT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    USART_CHASSIS_TX_PAYLOAD_LEN = 31U,
    USART_CHASSIS_TX_FRAME_LEN = USART_CHASSIS_TX_PAYLOAD_LEN + 1U
};

typedef struct
{
    uint8_t if_follow_gim;
    uint8_t jump_cmd;
    uint8_t overstep_cmd;
    uint8_t chassis_mode;
    float yaw_Encoder_ecd_angle;
    int16_t cmd_leg_length;
    int16_t x;
    int16_t y;
    int16_t rotate_speed;
    uint8_t ctrl_mode;
} usart_chassis_data_t;

typedef __PACKED_STRUCT
{
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_barrel_heat_limit;
    uint16_t shooter_barrel_cooling_value;
    float bullet_speed_x_hat;
    float bullet_speed;
    uint8_t robot_level;
    uint8_t power_management_chassis_output;
    uint16_t current_HP;
    uint8_t robot_id;
    uint8_t allow_gimbal_init;
    int16_t remain_heat;
    uint8_t game_state;
} usart_gimbal_data_t;

enum
{
    USART_GIMBAL_RX_PAYLOAD_LEN = sizeof(usart_gimbal_data_t),
    USART_GIMBAL_RX_FRAME_LEN = USART_GIMBAL_RX_PAYLOAD_LEN + 1U
};

typedef struct
{
    uint8_t online;
    uint8_t updated;
    uint8_t crc_ok;
    uint32_t last_update_tick;
    uint32_t frame_count;
    uint32_t crc_error_count;
} usart_chassis_link_state_t;

void USART_Chassis_Init(void);
void USART_Chassis_Process(const uint8_t *data, uint16_t length);
uint8_t USART_Chassis_Crc8(const uint8_t *data, uint16_t length);
uint8_t USART_Chassis_IsOnline(void);
const usart_chassis_link_state_t *USART_Chassis_GetLinkState(void);

void usart_chassis_send(uint8_t if_follow_gim,
                        uint8_t jump_cmd,
                        uint8_t overstep_cmd,
                        uint8_t chassis_mode,
                        float yaw_encoder_angle,
                        float cmd_leg_length,
                        float x,
                        float y,
                        int16_t rotate_speed,
                        float roll,
                        uint8_t ctrl_mode,
                        uint8_t remote_online_flag,
                        uint8_t shoot_fric_wheel_run,
                        uint8_t Rollover_posture_cmd,
                        uint8_t low_jump_cmd,
                        uint8_t UI_auto_aim_state);

void usart_gimbal_receive(usart_gimbal_data_t *data, const uint8_t *DataAddress);

extern usart_chassis_data_t usart_chassis_data;
extern usart_gimbal_data_t usart_gimbal_data;
extern uint8_t databuff[USART_CHASSIS_TX_FRAME_LEN];
extern uint8_t lock_shoot_check;
extern float remain_heat_test;

#ifdef __cplusplus
}
#endif

#endif
