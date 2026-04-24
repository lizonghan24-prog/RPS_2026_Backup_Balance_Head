#include "Remote.h"

#include <string.h>

/* VT remote parser: UART byte stream -> validated 21-byte frame -> state struct. */

#define REMOTE_FRAME_HEADER_0   0xA9U
#define REMOTE_FRAME_HEADER_1   0x53U
#define REMOTE_CRC_OFFSET       19U
#define REMOTE_ONLINE_TIMEOUT   100U

remote_state_t remote_state_data;
static uint8_t remote_rx_cache[REMOTE_FRAME_LEN];
static uint16_t remote_rx_cache_len;

/* VT03/VT13 的遥控帧使用 CRC16-CCITT-FALSE。 */
/* Match VTM remote CRC: reflected CRC16 (poly 0x8408, init 0xFFFF). */
static uint16_t Remote_Crc16Vtm(const uint8_t *data, uint16_t length)
{
    uint16_t crc;
    uint16_t i;
    uint16_t j;

    crc = 0xFFFFU;
    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i];
        for (j = 0U; j < 8U; ++j)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0x8408U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* 协议里有不少非字节对齐的字段，这里按小端位流提取。 */
static uint32_t Remote_ExtractBitsLE(const uint8_t *frame, uint16_t offset, uint8_t width)
{
    uint32_t value;
    uint8_t i;

    value = 0U;
    for (i = 0U; i < width; ++i)
    {
        if (((frame[(offset + i) / 8U] >> ((offset + i) % 8U)) & 0x01U) != 0U)
        {
            value |= 1UL << i;
        }
    }

    return value;
}

/* 缓冲区左移，用来丢掉无效头或者已经消费过的数据。 */
static void Remote_ShiftCacheLeft(uint16_t shift)
{
    if (shift >= remote_rx_cache_len)
    {
        remote_rx_cache_len = 0U;
        return;
    }

    memmove(remote_rx_cache,
            &remote_rx_cache[shift],
            (size_t)(remote_rx_cache_len - shift));
    remote_rx_cache_len = (uint16_t)(remote_rx_cache_len - shift);
}

/* 有些资料里 CRC 高低字节顺序不一致，这里两种都兼容。 */
/* CRC layout follows VTM frame: low byte first, high byte second. */
static uint8_t Remote_FrameValid(const uint8_t *frame)
{
    uint16_t calc_crc;
    uint16_t frame_crc_le;

    calc_crc = Remote_Crc16Vtm(frame, REMOTE_CRC_OFFSET);
    frame_crc_le = (uint16_t)frame[REMOTE_CRC_OFFSET]
                 | ((uint16_t)frame[REMOTE_CRC_OFFSET + 1U] << 8);

    return (uint8_t)(calc_crc == frame_crc_le);
}

/* 把 21 字节遥控帧拆成结构化状态。 */
static void Remote_DecodeFrame(const uint8_t *frame)
{
    uint8_t i;

    memcpy(remote_state_data.last_frame, frame, REMOTE_FRAME_LEN);

    remote_state_data.raw_channel[0] = (uint16_t)Remote_ExtractBitsLE(frame, 16U, 11U);
    remote_state_data.raw_channel[1] = (uint16_t)Remote_ExtractBitsLE(frame, 27U, 11U);
    remote_state_data.raw_channel[2] = (uint16_t)Remote_ExtractBitsLE(frame, 38U, 11U);
    remote_state_data.raw_channel[3] = (uint16_t)Remote_ExtractBitsLE(frame, 49U, 11U);

    for (i = 0U; i < 4U; ++i)
    {
        remote_state_data.channel[i] = (int16_t)remote_state_data.raw_channel[i]
                                     - (int16_t)REMOTE_STICK_CENTER;
    }

    remote_state_data.gear = (uint8_t)Remote_ExtractBitsLE(frame, 60U, 2U);
    remote_state_data.pause_pressed = (uint8_t)Remote_ExtractBitsLE(frame, 62U, 1U);
    remote_state_data.custom_left_pressed = (uint8_t)Remote_ExtractBitsLE(frame, 63U, 1U);
    remote_state_data.custom_right_pressed = (uint8_t)Remote_ExtractBitsLE(frame, 64U, 1U);
    remote_state_data.raw_dial = (uint16_t)Remote_ExtractBitsLE(frame, 65U, 11U);
    remote_state_data.dial = (int16_t)remote_state_data.raw_dial - (int16_t)REMOTE_STICK_CENTER;
    remote_state_data.trigger_pressed = (uint8_t)Remote_ExtractBitsLE(frame, 76U, 1U);
    remote_state_data.mouse_x = (int16_t)Remote_ExtractBitsLE(frame, 80U, 16U);
    remote_state_data.mouse_y = (int16_t)Remote_ExtractBitsLE(frame, 96U, 16U);
    remote_state_data.mouse_z = (int16_t)Remote_ExtractBitsLE(frame, 112U, 16U);
    remote_state_data.mouse_left_pressed = (uint8_t)(Remote_ExtractBitsLE(frame, 128U, 2U) != 0U);
    remote_state_data.mouse_right_pressed = (uint8_t)(Remote_ExtractBitsLE(frame, 130U, 2U) != 0U);
    remote_state_data.mouse_middle_pressed = (uint8_t)(Remote_ExtractBitsLE(frame, 132U, 2U) != 0U);
    remote_state_data.keyboard = (uint16_t)Remote_ExtractBitsLE(frame, 136U, 16U);

    remote_state_data.online = 1U;
    remote_state_data.updated = 1U;
    remote_state_data.crc_ok = 1U;
    remote_state_data.last_update_tick = HAL_GetTick();
    remote_state_data.frame_count++;
}

/* 尝试在缓存里对齐并解析一帧完整遥控数据。 */
static void Remote_TryDecodeCache(void)
{
    while (remote_rx_cache_len >= 2U)
    {
        if ((remote_rx_cache[0] != REMOTE_FRAME_HEADER_0)
         || (remote_rx_cache[1] != REMOTE_FRAME_HEADER_1))
        {
            Remote_ShiftCacheLeft(1U);
            continue;
        }

        if (remote_rx_cache_len < REMOTE_FRAME_LEN)
        {
            return;
        }

        if (Remote_FrameValid(remote_rx_cache) != 0U)
        {
            Remote_DecodeFrame(remote_rx_cache);
            Remote_ShiftCacheLeft(REMOTE_FRAME_LEN);
            continue;
        }

        Remote_ShiftCacheLeft(1U);
    }
}

void Remote_Init(void)
{
    memset(&remote_state_data, 0, sizeof(remote_state_data));
    memset(remote_rx_cache, 0, sizeof(remote_rx_cache));
    remote_rx_cache_len = 0U;
}

void Remote_Process(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    /* 串口数据可能被拆成多段送进来，所以先入缓存，再尝试组帧。 */
    for (i = 0U; i < length; ++i)
    {
        if (remote_rx_cache_len < REMOTE_FRAME_LEN)
        {
            remote_rx_cache[remote_rx_cache_len++] = data[i];
        }
        else
        {
            Remote_ShiftCacheLeft(1U);
            remote_rx_cache[remote_rx_cache_len++] = data[i];
        }

        Remote_TryDecodeCache();
    }
}

const remote_state_t *Remote_GetState(void)
{
    remote_state_data.online = Remote_IsOnline();
    return &remote_state_data;
}

uint8_t Remote_IsOnline(void)
{
    return (uint8_t)((remote_state_data.frame_count > 0U)
                  && ((HAL_GetTick() - remote_state_data.last_update_tick) < REMOTE_ONLINE_TIMEOUT));
}
