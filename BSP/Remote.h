#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define REMOTE_FRAME_LEN     21U
#define REMOTE_STICK_MIN     364U
#define REMOTE_STICK_CENTER  1024U
#define REMOTE_STICK_MAX     1684U

#define REMOTE_KEY_W      (1U << 0)
#define REMOTE_KEY_S      (1U << 1)
#define REMOTE_KEY_A      (1U << 2)
#define REMOTE_KEY_D      (1U << 3)
#define REMOTE_KEY_SHIFT  (1U << 4)
#define REMOTE_KEY_CTRL   (1U << 5)
#define REMOTE_KEY_Q      (1U << 6)
#define REMOTE_KEY_E      (1U << 7)
#define REMOTE_KEY_R      (1U << 8)
#define REMOTE_KEY_F      (1U << 9)
#define REMOTE_KEY_G      (1U << 10)
#define REMOTE_KEY_Z      (1U << 11)
#define REMOTE_KEY_X      (1U << 12)
#define REMOTE_KEY_C      (1U << 13)
#define REMOTE_KEY_V      (1U << 14)
#define REMOTE_KEY_B      (1U << 15)

/* 图传遥控器的三挡开关定义。 */
typedef enum
{
    REMOTE_SWITCH_C = 0U,
    REMOTE_SWITCH_N = 1U,
    REMOTE_SWITCH_S = 2U
} remote_switch_t;

/* 图传模块解析后的完整遥控状态。 */
typedef struct
{
    uint8_t online;                               /* 在线标志，超时后会被清零。 */
    uint8_t updated;                              /* 本轮是否收到过新遥控数据。 */
    uint8_t crc_ok;                               /* 最近一帧 CRC 是否校验通过。 */
    uint8_t gear;                                 /* 三挡拨杆档位。 */
    uint8_t pause_pressed;                        /* 暂停键状态。 */
    uint8_t custom_left_pressed;                  /* 左侧自定义按键状态。 */
    uint8_t custom_right_pressed;                 /* 右侧自定义按键状态。 */
    uint8_t trigger_pressed;                      /* 扳机状态。 */
    uint8_t mouse_left_pressed;                   /* 鼠标左键状态。 */
    uint8_t mouse_right_pressed;                  /* 鼠标右键状态。 */
    uint8_t mouse_middle_pressed;                 /* 鼠标中键状态。 */
    uint16_t raw_channel[4];                      /* 四个主通道原始值。 */
    int16_t channel[4];                           /* 四个主通道去中值后的结果。 */
    uint16_t raw_dial;                            /* 拨轮原始值。 */
    int16_t dial;                                 /* 拨轮去中值后的结果。 */
    int16_t mouse_x;                              /* 鼠标 X 轴增量。 */
    int16_t mouse_y;                              /* 鼠标 Y 轴增量。 */
    int16_t mouse_z;                              /* 鼠标 Z 轴增量。 */
    uint16_t keyboard;                            /* 键盘位图。 */
    uint32_t last_update_tick;                    /* 最近一次成功解帧的系统时刻。 */
    uint32_t frame_count;                         /* 成功解析的遥控帧数量。 */
    uint8_t last_frame[REMOTE_FRAME_LEN];         /* 最近一次收到的完整原始帧。 */
} remote_state_t;

/* 遥控模块状态清零。 */
void Remote_Init(void);

/* 向遥控解析器喂入一段串口原始数据。 */
void Remote_Process(const uint8_t *data, uint16_t length);

/* 获取当前遥控状态。 */
const remote_state_t *Remote_GetState(void);

/* 查询遥控是否在超时窗口内。 */
uint8_t Remote_IsOnline(void);

#endif
