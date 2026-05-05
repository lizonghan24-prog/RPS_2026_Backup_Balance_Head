#include "USART_chassis_transmit.h"

#include "Control_Task.h"
#include "Shoot_Task.h"

#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define USART_CHASSIS_CRC8_INIT             0xFFU
#define USART_CHASSIS_ONLINE_TIMEOUT_MS     100U
#define USART_CHASSIS_DEG_TO_RAD            0.01745329251994329577f
#define USART_CHASSIS_TWO_PI                6.28318530717958647692f
#define USART_CHASSIS_RX_CACHE_SIZE         (USART_GIMBAL_RX_FRAME_LEN * 2U)

usart_chassis_data_t usart_chassis_data;
usart_gimbal_data_t usart_gimbal_data;
uint8_t databuff[USART_CHASSIS_TX_FRAME_LEN];
uint8_t lock_shoot_check;
float remain_heat_test;

static usart_chassis_link_state_t usart_chassis_link_state;
static uint8_t usart_chassis_rx_cache[USART_CHASSIS_RX_CACHE_SIZE];
static uint16_t usart_chassis_rx_cache_len;

static const uint8_t usart_chassis_crc8_table[256] =
{
    0x00U, 0x5EU, 0xBCU, 0xE2U, 0x61U, 0x3FU, 0xDDU, 0x83U, 0xC2U, 0x9CU, 0x7EU, 0x20U, 0xA3U, 0xFDU, 0x1FU, 0x41U,
    0x9DU, 0xC3U, 0x21U, 0x7FU, 0xFCU, 0xA2U, 0x40U, 0x1EU, 0x5FU, 0x01U, 0xE3U, 0xBDU, 0x3EU, 0x60U, 0x82U, 0xDCU,
    0x23U, 0x7DU, 0x9FU, 0xC1U, 0x42U, 0x1CU, 0xFEU, 0xA0U, 0xE1U, 0xBFU, 0x5DU, 0x03U, 0x80U, 0xDEU, 0x3CU, 0x62U,
    0xBEU, 0xE0U, 0x02U, 0x5CU, 0xDFU, 0x81U, 0x63U, 0x3DU, 0x7CU, 0x22U, 0xC0U, 0x9EU, 0x1DU, 0x43U, 0xA1U, 0xFFU,
    0x46U, 0x18U, 0xFAU, 0xA4U, 0x27U, 0x79U, 0x9BU, 0xC5U, 0x84U, 0xDAU, 0x38U, 0x66U, 0xE5U, 0xBBU, 0x59U, 0x07U,
    0xDBU, 0x85U, 0x67U, 0x39U, 0xBAU, 0xE4U, 0x06U, 0x58U, 0x19U, 0x47U, 0xA5U, 0xFBU, 0x78U, 0x26U, 0xC4U, 0x9AU,
    0x65U, 0x3BU, 0xD9U, 0x87U, 0x04U, 0x5AU, 0xB8U, 0xE6U, 0xA7U, 0xF9U, 0x1BU, 0x45U, 0xC6U, 0x98U, 0x7AU, 0x24U,
    0xF8U, 0xA6U, 0x44U, 0x1AU, 0x99U, 0xC7U, 0x25U, 0x7BU, 0x3AU, 0x64U, 0x86U, 0xD8U, 0x5BU, 0x05U, 0xE7U, 0xB9U,
    0x8CU, 0xD2U, 0x30U, 0x6EU, 0xEDU, 0xB3U, 0x51U, 0x0FU, 0x4EU, 0x10U, 0xF2U, 0xACU, 0x2FU, 0x71U, 0x93U, 0xCDU,
    0x11U, 0x4FU, 0xADU, 0xF3U, 0x70U, 0x2EU, 0xCCU, 0x92U, 0xD3U, 0x8DU, 0x6FU, 0x31U, 0xB2U, 0xECU, 0x0EU, 0x50U,
    0xAFU, 0xF1U, 0x13U, 0x4DU, 0xCEU, 0x90U, 0x72U, 0x2CU, 0x6DU, 0x33U, 0xD1U, 0x8FU, 0x0CU, 0x52U, 0xB0U, 0xEEU,
    0x32U, 0x6CU, 0x8EU, 0xD0U, 0x53U, 0x0DU, 0xEFU, 0xB1U, 0xF0U, 0xAEU, 0x4CU, 0x12U, 0x91U, 0xCFU, 0x2DU, 0x73U,
    0xCAU, 0x94U, 0x76U, 0x28U, 0xABU, 0xF5U, 0x17U, 0x49U, 0x08U, 0x56U, 0xB4U, 0xEAU, 0x69U, 0x37U, 0xD5U, 0x8BU,
    0x57U, 0x09U, 0xEBU, 0xB5U, 0x36U, 0x68U, 0x8AU, 0xD4U, 0x95U, 0xCBU, 0x29U, 0x77U, 0xF4U, 0xAAU, 0x48U, 0x16U,
    0xE9U, 0xB7U, 0x55U, 0x0BU, 0x88U, 0xD6U, 0x34U, 0x6AU, 0x2BU, 0x75U, 0x97U, 0xC9U, 0x4AU, 0x14U, 0xF6U, 0xA8U,
    0x74U, 0x2AU, 0xC8U, 0x96U, 0x15U, 0x4BU, 0xA9U, 0xF7U, 0xB6U, 0xE8U, 0x0AU, 0x54U, 0xD7U, 0x89U, 0x6BU, 0x35U
};

static void USART_Chassis_WriteI16BE(uint8_t *buffer, uint16_t offset, int16_t value)
{
    buffer[offset] = (uint8_t)((uint16_t)value >> 8);
    buffer[offset + 1U] = (uint8_t)value;
}

static void USART_Chassis_WriteI32BE(uint8_t *buffer, uint16_t offset, int32_t value)
{
    buffer[offset] = (uint8_t)((uint32_t)value >> 24);
    buffer[offset + 1U] = (uint8_t)((uint32_t)value >> 16);
    buffer[offset + 2U] = (uint8_t)((uint32_t)value >> 8);
    buffer[offset + 3U] = (uint8_t)value;
}

static void USART_Chassis_ShiftRxCacheLeft(uint16_t shift)
{
    if (shift >= usart_chassis_rx_cache_len)
    {
        usart_chassis_rx_cache_len = 0U;
        return;
    }

    memmove(usart_chassis_rx_cache,
            &usart_chassis_rx_cache[shift],
            (size_t)(usart_chassis_rx_cache_len - shift));
    usart_chassis_rx_cache_len = (uint16_t)(usart_chassis_rx_cache_len - shift);
}

uint8_t USART_Chassis_Crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc;
    uint16_t i;

    crc = USART_CHASSIS_CRC8_INIT;
    if (data == NULL)
    {
        return crc;
    }

    for (i = 0U; i < length; ++i)
    {
        crc = usart_chassis_crc8_table[crc ^ data[i]];
    }

    return crc;
}

static uint8_t USART_Chassis_FrameValid(const uint8_t *frame)
{
    return (uint8_t)(USART_Chassis_Crc8(frame, USART_GIMBAL_RX_PAYLOAD_LEN)
                  == frame[USART_GIMBAL_RX_PAYLOAD_LEN]);
}

static void USART_Chassis_TryDecodeCache(void)
{
    while (usart_chassis_rx_cache_len >= USART_GIMBAL_RX_FRAME_LEN)
    {
        if (USART_Chassis_FrameValid(usart_chassis_rx_cache) != 0U)
        {
            usart_gimbal_receive(&usart_gimbal_data, usart_chassis_rx_cache);
            USART_Chassis_ShiftRxCacheLeft(USART_GIMBAL_RX_FRAME_LEN);
        }
        else
        {
            usart_chassis_link_state.crc_ok = 0U;
            usart_chassis_link_state.crc_error_count++;
            USART_Chassis_ShiftRxCacheLeft(1U);
        }
    }
}

void USART_Chassis_Init(void)
{
    memset(&usart_chassis_data, 0, sizeof(usart_chassis_data));
    memset(&usart_gimbal_data, 0, sizeof(usart_gimbal_data));
    memset(databuff, 0, sizeof(databuff));
    memset(&usart_chassis_link_state, 0, sizeof(usart_chassis_link_state));
    memset(usart_chassis_rx_cache, 0, sizeof(usart_chassis_rx_cache));
    usart_chassis_rx_cache_len = 0U;
    remain_heat_test = 0.0f;
    lock_shoot_check = 0U;
}

void USART_Chassis_Process(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    for (i = 0U; i < length; ++i)
    {
        if (usart_chassis_rx_cache_len < USART_CHASSIS_RX_CACHE_SIZE)
        {
            usart_chassis_rx_cache[usart_chassis_rx_cache_len++] = data[i];
        }
        else
        {
            USART_Chassis_ShiftRxCacheLeft(1U);
            usart_chassis_rx_cache[usart_chassis_rx_cache_len++] = data[i];
        }
    }

    USART_Chassis_TryDecodeCache();
}

uint8_t USART_Chassis_IsOnline(void)
{
    return (uint8_t)((usart_chassis_link_state.frame_count > 0U)
                  && ((HAL_GetTick() - usart_chassis_link_state.last_update_tick)
                      < USART_CHASSIS_ONLINE_TIMEOUT_MS));
}

const usart_chassis_link_state_t *USART_Chassis_GetLinkState(void)
{
    usart_chassis_link_state.online = USART_Chassis_IsOnline();
    return &usart_chassis_link_state;
}

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
                        uint8_t UI_auto_aim_state)
{
    const gimbal_control_task_t *control_state;
    float yaw_0_2pi;
    int32_t yaw_scaled;
    int16_t roll_scaled;
    int16_t vx_scaled;
    int16_t vy_scaled;
    int16_t leg_length_scaled;

    if (huart3.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    yaw_0_2pi = fmodf(-yaw_encoder_angle * USART_CHASSIS_DEG_TO_RAD, USART_CHASSIS_TWO_PI);
    if (yaw_0_2pi < 0.0f)
    {
        yaw_0_2pi += USART_CHASSIS_TWO_PI;
    }

    yaw_scaled = (int32_t)(yaw_0_2pi * 10000.0f);
    roll_scaled = (int16_t)(roll * 100.0f);
    vx_scaled = (int16_t)(x * 100.0f);
    vy_scaled = (int16_t)(y * 100.0f);
    leg_length_scaled = (int16_t)(cmd_leg_length * 100.0f);
    control_state = Control_Task_GetState();

    databuff[0] = if_follow_gim;
    databuff[1] = jump_cmd;
    databuff[2] = chassis_mode;
    USART_Chassis_WriteI16BE(databuff, 3U, leg_length_scaled);
    USART_Chassis_WriteI16BE(databuff, 5U, vx_scaled);
    USART_Chassis_WriteI16BE(databuff, 7U, vy_scaled);
    USART_Chassis_WriteI16BE(databuff, 9U, rotate_speed);
    USART_Chassis_WriteI32BE(databuff, 11U, yaw_scaled);
    databuff[15] = ctrl_mode;
    USART_Chassis_WriteI16BE(databuff, 16U, roll_scaled);
    databuff[18] = overstep_cmd;
    databuff[19] = remote_online_flag;
    databuff[20] = shoot_fric_wheel_run;
    databuff[21] = Rollover_posture_cmd;
    databuff[22] = low_jump_cmd;
    databuff[23] = UI_auto_aim_state;
    databuff[24] = (uint8_t)((control_state != NULL) ? control_state->init_finished : 0U);
    databuff[25] = 0U;
    databuff[26] = 0U;
    databuff[27] = 0U;
    databuff[28] = 0U;
    databuff[29] = 0U;
    databuff[30] = lock_shoot_check;
    databuff[USART_CHASSIS_TX_PAYLOAD_LEN] = USART_Chassis_Crc8(databuff, USART_CHASSIS_TX_PAYLOAD_LEN);

    (void)HAL_UART_Transmit_DMA(&huart3, databuff, USART_CHASSIS_TX_FRAME_LEN);
}

void usart_gimbal_receive(usart_gimbal_data_t *data, const uint8_t *DataAddress)
{
    if ((data == NULL) || (DataAddress == NULL))
    {
        return;
    }

    memcpy(data, DataAddress, USART_GIMBAL_RX_PAYLOAD_LEN);
    if (data != &usart_gimbal_data)
    {
        memcpy(&usart_gimbal_data, data, USART_GIMBAL_RX_PAYLOAD_LEN);
    }

    usart_chassis_link_state.online = 1U;
    usart_chassis_link_state.updated = 1U;
    usart_chassis_link_state.crc_ok = 1U;
    usart_chassis_link_state.last_update_tick = HAL_GetTick();
    usart_chassis_link_state.frame_count++;

    if (usart_gimbal_data.remain_heat != 0)
    {
        remain_heat_test = (float)usart_gimbal_data.remain_heat;
    }

    if ((usart_gimbal_data.power_management_chassis_output == 0U)
     || (usart_gimbal_data.current_HP == 0U))
    {
        Shoot_Task_SetFrictionEnable(0U);
    }
}
