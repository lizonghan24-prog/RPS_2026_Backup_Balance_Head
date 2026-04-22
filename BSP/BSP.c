#include "BSP.h"
#include "Control_Task.h"
#include "Shoot_Task.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

#define BSP_IMU_DMA_BUFFER_SIZE       256U
#define BSP_IMU_RING_BUFFER_SIZE      2048U
#define BSP_REMOTE_DMA_BUFFER_SIZE    64U
#define BSP_REMOTE_RING_BUFFER_SIZE   256U

/* 简单环形缓冲，只负责把 DMA 收到的数据先存起来。 */
typedef struct
{
    uint8_t *buffer;                       /* 缓冲区起始地址。 */
    uint16_t size;                         /* 缓冲区总长度。 */
    volatile uint16_t head;               /* 写指针。 */
    volatile uint16_t tail;               /* 读指针。 */
} bsp_ring_buffer_t;

/* 一路串口 DMA 接收链路的完整描述。 */
typedef struct
{
    UART_HandleTypeDef *huart;            /* 这一路对应哪个串口。 */
    uint8_t *dma_buffer;                  /* DMA 循环接收区。 */
    uint16_t dma_buffer_size;             /* DMA 接收区长度。 */
    uint16_t dma_last_pos;                /* 上一次搬运到哪一位。 */
    bsp_ring_buffer_t ring;               /* 软件环形缓冲。 */
} bsp_uart_dma_channel_t;

static uint8_t imu_dma_buffer[BSP_IMU_DMA_BUFFER_SIZE];
static uint8_t imu_ring_storage[BSP_IMU_RING_BUFFER_SIZE];
static uint8_t remote_dma_buffer[BSP_REMOTE_DMA_BUFFER_SIZE];
static uint8_t remote_ring_storage[BSP_REMOTE_RING_BUFFER_SIZE];
static uint8_t imu_poll_buffer[128];
static uint8_t remote_poll_buffer[64];

static bsp_uart_dma_channel_t imu_uart_channel;
static bsp_uart_dma_channel_t remote_uart_channel;

/* 初始化一个空的环形缓冲。 */
static void BSP_RingInit(bsp_ring_buffer_t *ring, uint8_t *buffer, uint16_t size)
{
    ring->buffer = buffer;
    ring->size = size;
    ring->head = 0U;
    ring->tail = 0U;
    memset(buffer, 0, size);
}

/* 往环形缓冲里写一段数据，写满就停。 */
static uint16_t BSP_RingWrite(bsp_ring_buffer_t *ring, const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint16_t next_head;

    for (i = 0U; i < length; ++i)
    {
        next_head = (uint16_t)((ring->head + 1U) % ring->size);
        if (next_head == ring->tail)
        {
            break;
        }

        ring->buffer[ring->head] = data[i];
        ring->head = next_head;
    }

    return i;
}

/* 从环形缓冲里读一段数据出来给协议层消费。 */
static uint16_t BSP_RingRead(bsp_ring_buffer_t *ring, uint8_t *data, uint16_t max_length)
{
    uint16_t count;

    count = 0U;
    while ((count < max_length) && (ring->tail != ring->head))
    {
        data[count++] = ring->buffer[ring->tail];
        ring->tail = (uint16_t)((ring->tail + 1U) % ring->size);
    }

    return count;
}

/* 根据串口句柄找到对应的 DMA 通道描述。 */
static bsp_uart_dma_channel_t *BSP_FindUartChannel(UART_HandleTypeDef *huart)
{
    if (huart == imu_uart_channel.huart)
    {
        return &imu_uart_channel;
    }

    if (huart == remote_uart_channel.huart)
    {
        return &remote_uart_channel;
    }

    return NULL;
}

/* 把 DMA 新收到的数据搬进软件环形缓冲。 */
static void BSP_MoveDmaToRing(bsp_uart_dma_channel_t *channel)
{
    uint16_t dma_pos;

    if ((channel == NULL) || (channel->huart == NULL) || (channel->huart->hdmarx == NULL))
    {
        return;
    }

    dma_pos = (uint16_t)(channel->dma_buffer_size - __HAL_DMA_GET_COUNTER(channel->huart->hdmarx));
    if (dma_pos == channel->dma_last_pos)
    {
        return;
    }

    if (dma_pos > channel->dma_last_pos)
    {
        (void)BSP_RingWrite(&channel->ring,
                            &channel->dma_buffer[channel->dma_last_pos],
                            (uint16_t)(dma_pos - channel->dma_last_pos));
    }
    else
    {
        (void)BSP_RingWrite(&channel->ring,
                            &channel->dma_buffer[channel->dma_last_pos],
                            (uint16_t)(channel->dma_buffer_size - channel->dma_last_pos));
        if (dma_pos > 0U)
        {
            (void)BSP_RingWrite(&channel->ring, channel->dma_buffer, dma_pos);
        }
    }

    channel->dma_last_pos = dma_pos;
}

/* 启动一路串口的 DMA 循环接收，并打开空闲中断。 */
static void BSP_StartUartDmaChannel(bsp_uart_dma_channel_t *channel)
{
    channel->dma_last_pos = 0U;
    memset(channel->dma_buffer, 0, channel->dma_buffer_size);

    if (HAL_UART_Receive_DMA(channel->huart, channel->dma_buffer, channel->dma_buffer_size) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(channel->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(channel->huart->hdmarx, DMA_IT_TC);
    __HAL_UART_CLEAR_IDLEFLAG(channel->huart);
    __HAL_UART_ENABLE_IT(channel->huart, UART_IT_IDLE);
}

/* 串口异常后，整条 DMA 接收链路直接重启。 */
static void BSP_RestartUartDmaChannel(bsp_uart_dma_channel_t *channel)
{
    if (channel == NULL)
    {
        return;
    }

    (void)HAL_UART_DMAStop(channel->huart);
    BSP_StartUartDmaChannel(channel);
}

/* CAN 过滤器先全放开，后面按标准帧 ID 在软件里分发。 */
static HAL_StatusTypeDef BSP_ConfigCanFilter(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

/* 配好过滤器以后，启动 CAN 并打开 FIFO0 接收中断。 */
static void BSP_InitCan(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    if (BSP_ConfigCanFilter(hcan, filter_bank) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 从软件环形缓冲持续取数，交给对应的协议解析函数。 */
static void BSP_PollSerialChannel(bsp_uart_dma_channel_t *channel,
                                  uint8_t *scratch,
                                  uint16_t scratch_size,
                                  void (*consumer)(const uint8_t *data, uint16_t length))
{
    uint16_t bytes_read;

    do
    {
        bytes_read = BSP_RingRead(&channel->ring, scratch, scratch_size);
        if (bytes_read > 0U)
        {
            consumer(scratch, bytes_read);
        }
    } while (bytes_read == scratch_size);
}

void BSP_Init(void)
{
    Remote_Init();
    IMU_Init();
    Motor_Init();

    /* USART3 给 IMU，缓存稍大一点，避免 1 kHz 数据堆积。 */
    BSP_RingInit(&imu_uart_channel.ring, imu_ring_storage, BSP_IMU_RING_BUFFER_SIZE);
    imu_uart_channel.huart = &huart3;
    imu_uart_channel.dma_buffer = imu_dma_buffer;
    imu_uart_channel.dma_buffer_size = BSP_IMU_DMA_BUFFER_SIZE;

    /* USART6 给图传遥控，缓存可以相对小一些。 */
    BSP_RingInit(&remote_uart_channel.ring, remote_ring_storage, BSP_REMOTE_RING_BUFFER_SIZE);
    remote_uart_channel.huart = &huart6;
    remote_uart_channel.dma_buffer = remote_dma_buffer;
    remote_uart_channel.dma_buffer_size = BSP_REMOTE_DMA_BUFFER_SIZE;

    BSP_StartUartDmaChannel(&imu_uart_channel);
    BSP_StartUartDmaChannel(&remote_uart_channel);

    BSP_InitCan(&hcan1, 0U);
    BSP_InitCan(&hcan2, 14U);

    /* TIM6 触发 1 kHz 控制周期。 */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

void BSP_Poll(void)
{
    /* 主循环只做“取数据并分发”，不直接碰 DMA 计数器。 */
    BSP_PollSerialChannel(&imu_uart_channel, imu_poll_buffer, sizeof(imu_poll_buffer), IMU_Process);
    BSP_PollSerialChannel(&remote_uart_channel, remote_poll_buffer, sizeof(remote_poll_buffer), Remote_Process);
}

void BSP_HandleUartIdle(UART_HandleTypeDef *huart)
{
    bsp_uart_dma_channel_t *channel;

    channel = BSP_FindUartChannel(huart);
    if (channel == NULL)
    {
        return;
    }

    /* 进空闲中断说明这段串口数据基本收完了，这时搬运最省事。 */
    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
     && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        BSP_MoveDmaToRing(channel);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        Motor_ProcessCanMessage(hcan, &rx_header, rx_data);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* TIM6 每次到期直接执行控制任务。 */
    if (htim->Instance == TIM6)
    {
        Control_Task_Run();
        Shoot_Task_Run();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_dma_channel_t *channel;

    channel = BSP_FindUartChannel(huart);
    if (channel != NULL)
    {
        BSP_RestartUartDmaChannel(channel);
    }
}
