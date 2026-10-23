/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_uart.c
  * @brief   USART1 传输实现：逐字节中断 + 帧解析 + 最新点缓存
  *
  * 帧格式: 0xAA 0x55 | X低 | X高 | Y低 | Y高 | 校验(前 6 字节异或)
  * 坐标 int16 小端。
  *
  * 只保留最新一个点: 上位机 1ms 发一次, 取点端 10kHz 消费,
  * 正常情况下每次取点前都只会有 1 个新点, 不需要队列。
  * 若新点到来时旧点还没被取走, 旧点被覆盖并累加覆盖计数。
  *
  * 以后换 SPI 时，新增 comm_spi.c 实现 comm.h 的同一组接口，
  * 从工程中移除本文件即可。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "comm.h"
#include "usart.h"
#include "main.h"

/* USER CODE BEGIN 0 */
#define COMM_FRAME_LEN   7U
#define COMM_HEAD0       0xAAU
#define COMM_HEAD1       0x55U

typedef struct
{
  int16_t x;
  int16_t y;
} comm_point_t;

/* 双槽: 写者 (串口中断) 写另一个槽, 写完再用 comm_pub 发布。
   读者 (DMA 中断) 优先级更高, 读取期间写者不会运行, 因此不会撕裂。 */
static comm_point_t comm_slot[2];
static volatile uint8_t comm_wr_idx = 0U;    /* 写者下一次使用的槽 */
static volatile uint8_t comm_pub = 0U;       /* 已写完、可读的槽 */
static volatile uint8_t comm_ready = 0U;     /* 1 = 有新点未取 */
static volatile uint32_t comm_crc_err = 0U;

volatile uint32_t g_comm_last_rx_cycle = 0U;  /* 调试计时用 */

static volatile uint8_t comm_rx_byte;
static uint8_t comm_rx_frame[COMM_FRAME_LEN];
static uint8_t comm_rx_len = 0U;

static void comm_point_push(int16_t x, int16_t y);
static void comm_parse_byte(uint8_t b);
/* USER CODE END 0 */

void Comm_Init(void)
{
  MX_USART1_UART_Init();

  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&comm_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

bool Comm_PopPoint(int16_t *x, int16_t *y)
{
  uint8_t idx;

  if (comm_ready == 0U)
  {
    return false;
  }

  idx = comm_pub;                           /* 读已发布（写完）的槽 */
  __DMB();
  *x = comm_slot[idx].x;
  *y = comm_slot[idx].y;
  comm_ready = 0U;                          /* 取走后清标志 */
  return true;
}

uint32_t Comm_GetCrcErrorCount(void)
{
  return comm_crc_err;
}

/* USER CODE BEGIN 1 */

static void comm_point_push(int16_t x, int16_t y)
{
  comm_wr_idx ^= 1U;
  comm_slot[comm_wr_idx].x = x;             /* 写另一个槽, 不影响读者 */
  comm_slot[comm_wr_idx].y = y;
  __DMB();                                  /* 数据先落地, 再发布 */
  comm_pub = comm_wr_idx;
  comm_ready = 1U;
}

static void comm_parse_byte(uint8_t b)
{
  if (comm_rx_len == 0U)
  {
    if (b == COMM_HEAD0)
    {
      comm_rx_frame[comm_rx_len++] = b;
    }
    return;
  }

  if (comm_rx_len == 1U)
  {
    if (b == COMM_HEAD1)
    {
      comm_rx_frame[comm_rx_len++] = b;
    }
    else if (b != COMM_HEAD0)               /* 允许 AA AA 55 */
    {
      comm_rx_len = 0U;
    }
    return;
  }

  comm_rx_frame[comm_rx_len++] = b;

  if (comm_rx_len < COMM_FRAME_LEN)
  {
    return;
  }

  {
    uint8_t cs = 0U;

    for (uint8_t i = 0U; i < (COMM_FRAME_LEN - 1U); i++)
    {
      cs ^= comm_rx_frame[i];
    }

    if (cs == comm_rx_frame[COMM_FRAME_LEN - 1U])
    {
      int16_t x = (int16_t)((uint16_t)comm_rx_frame[2] |
                            ((uint16_t)comm_rx_frame[3] << 8));
      int16_t y = (int16_t)((uint16_t)comm_rx_frame[4] |
                            ((uint16_t)comm_rx_frame[5] << 8));

      g_comm_last_rx_cycle = DWT->CYCCNT;   /* 一帧收完的时刻，供延时测量 */
      comm_point_push(x, y);
    }
    else
    {
      comm_crc_err++;
    }

    comm_rx_len = 0U;
  }
}

/**
  * @brief 收完一个字节: 解析后立刻重挂接收
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    comm_parse_byte(comm_rx_byte);
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&comm_rx_byte, 1U);
  }
}

/**
  * @brief 溢出/帧错误后重新挂接收, 否则会永久停收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&comm_rx_byte, 1U);
  }
}

/* USER CODE END 1 */
