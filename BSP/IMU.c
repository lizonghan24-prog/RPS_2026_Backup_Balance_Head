#include "IMU.h"

#include <string.h>

extern UART_HandleTypeDef huart6;

#define IMU_RX_CACHE_SIZE      256U
#define IMU_FRAME_OVERHEAD     6U
#define IMU_ONLINE_TIMEOUT     50U

imu_hi91_t imu_state;
static uint8_t imu_rx_cache[IMU_RX_CACHE_SIZE];
static uint16_t imu_rx_cache_len;

/* 从小端字节流中读取 16 位无符号数。 */
static uint16_t IMU_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/* 从小端字节流中读取 32 位无符号数。 */
static uint32_t IMU_ReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

/* 按模块原始格式直接取出 float。 */
static float IMU_ReadFloatLE(const uint8_t *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

/* 官方二进制协议整帧 CRC 的更新方式。 */
static void IMU_Crc16Update(uint16_t *current_crc, const uint8_t *src, uint32_t length_in_bytes)
{
    uint32_t crc;
    uint32_t i;
    uint32_t j;

    crc = *current_crc;
    for (j = 0U; j < length_in_bytes; ++j)
    {
        crc ^= (uint32_t)src[j] << 8;
        for (i = 0U; i < 8U; ++i)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (crc << 1) ^ 0x1021U;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    *current_crc = (uint16_t)crc;
}

/* 丢掉缓存头部无效数据，或者移除已经消费过的完整帧。 */
static void IMU_ShiftCacheLeft(uint16_t shift)
{
    if (shift >= imu_rx_cache_len)
    {
        imu_rx_cache_len = 0U;
        return;
    }

    memmove(imu_rx_cache, &imu_rx_cache[shift], (size_t)(imu_rx_cache_len - shift));
    imu_rx_cache_len = (uint16_t)(imu_rx_cache_len - shift);
}

/* 按 HI91 的字段顺序把负载区写入状态结构体。 */
static void IMU_ParseHi91(const uint8_t *payload)
{
    uint8_t i;

    imu_state.tag = payload[0];
    imu_state.main_status = IMU_ReadU16LE(&payload[1]);
    imu_state.temperature_c = (int8_t)payload[3];
    imu_state.air_pressure_pa = IMU_ReadFloatLE(&payload[4]);
    imu_state.system_time_ms = IMU_ReadU32LE(&payload[8]);

    for (i = 0U; i < 3U; ++i)
    {
        imu_state.acc_g[i] = IMU_ReadFloatLE(&payload[12U + (i * 4U)]);
        imu_state.gyr_dps[i] = IMU_ReadFloatLE(&payload[24U + (i * 4U)]);
        imu_state.mag_ut[i] = IMU_ReadFloatLE(&payload[36U + (i * 4U)]);
        imu_state.euler_deg[i] = IMU_ReadFloatLE(&payload[48U + (i * 4U)]);
    }

    for (i = 0U; i < 4U; ++i)
    {
        imu_state.quat[i] = IMU_ReadFloatLE(&payload[60U + (i * 4U)]);
    }

    imu_state.online = 1U;
    imu_state.updated = 1U;
    imu_state.last_update_tick = HAL_GetTick();
    imu_state.frame_count++;
}

/* 在缓存里找完整帧，找到就按协议解掉。 */
static void IMU_TryDecodeCache(void)
{
    uint16_t payload_length;
    uint16_t frame_length;
    uint16_t frame_crc;
    uint16_t calc_crc;

    while (imu_rx_cache_len >= 2U)
    {
        if ((imu_rx_cache[0] != IMU_FRAME_SOF_0) || (imu_rx_cache[1] != IMU_FRAME_SOF_1))
        {
            IMU_ShiftCacheLeft(1U);
            continue;
        }

        if (imu_rx_cache_len < IMU_FRAME_OVERHEAD)
        {
            return;
        }

        payload_length = IMU_ReadU16LE(&imu_rx_cache[2]);
        frame_length = (uint16_t)(payload_length + IMU_FRAME_OVERHEAD);

        if ((payload_length == 0U) || (frame_length > IMU_RX_CACHE_SIZE))
        {
            IMU_ShiftCacheLeft(1U);
            continue;
        }

        if (imu_rx_cache_len < frame_length)
        {
            return;
        }

        calc_crc = 0U;
        IMU_Crc16Update(&calc_crc, imu_rx_cache, 4U);
        IMU_Crc16Update(&calc_crc, &imu_rx_cache[6], payload_length);
        frame_crc = IMU_ReadU16LE(&imu_rx_cache[4]);

        if (calc_crc == frame_crc)
        {
            if ((payload_length >= IMU_HI91_PAYLOAD_LEN) && (imu_rx_cache[6] == IMU_HI91_TAG))
            {
                IMU_ParseHi91(&imu_rx_cache[6]);
            }

            IMU_ShiftCacheLeft(frame_length);
            continue;
        }

        IMU_ShiftCacheLeft(1U);
    }
}

void IMU_Init(void)
{
    memset(&imu_state, 0, sizeof(imu_state));
    memset(imu_rx_cache, 0, sizeof(imu_rx_cache));
    imu_rx_cache_len = 0U;
}

void IMU_Process(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    /* 串口输入先放缓存，后面统一按协议找帧头和 CRC。 */
    for (i = 0U; i < length; ++i)
    {
        if (imu_rx_cache_len < IMU_RX_CACHE_SIZE)
        {
            imu_rx_cache[imu_rx_cache_len++] = data[i];
        }
        else
        {
            IMU_ShiftCacheLeft(1U);
            imu_rx_cache[imu_rx_cache_len++] = data[i];
        }
    }

    IMU_TryDecodeCache();
}

const imu_hi91_t *IMU_GetState(void)
{
    imu_state.online = IMU_IsOnline();
    return &imu_state;
}

uint8_t IMU_IsOnline(void)
{
    return (uint8_t)((imu_state.frame_count > 0U)
                  && ((HAL_GetTick() - imu_state.last_update_tick) < IMU_ONLINE_TIMEOUT));
}

HAL_StatusTypeDef IMU_SendCommand(const char *command)
{
    char tx_buffer[96];
    size_t length;

    if (command == NULL)
    {
        return HAL_ERROR;
    }

    length = strlen(command);
    if ((length + 2U) > sizeof(tx_buffer))
    {
        return HAL_ERROR;
    }

    /* 下发给 IMU 的 ASCII 命令统一补成 CRLF 结尾。 */
    memcpy(tx_buffer, command, length);
    if ((length < 2U) || (tx_buffer[length - 2U] != '\r') || (tx_buffer[length - 1U] != '\n'))
    {
        tx_buffer[length++] = '\r';
        tx_buffer[length++] = '\n';
    }

    return HAL_UART_Transmit(&huart6, (uint8_t *)tx_buffer, (uint16_t)length, 100U);
}

void IMU_SendRecommendedConfig(void)
{
    /* 配置：921600 波特率，HI91 数据，1000 Hz 输出。 */
    (void)IMU_SendCommand("LOG DISABLE");
    HAL_Delay(5U);
    (void)IMU_SendCommand("SERIALCONFIG 921600");
    HAL_Delay(5U);
    (void)IMU_SendCommand("LOG HI91 ONTIME 0.001");
    HAL_Delay(5U);
    (void)IMU_SendCommand("LOG ENABLE");
}
