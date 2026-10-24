/**
  ******************************************************************************
  * @file    comm_uart.c
  * @brief   USART1 传输实现：逐字节中断 + 帧解析 + 大环形缓冲区
  *
  * 帧格式: 0xAA 0x55 | X低 | X高 | Y低 | Y高 | 激光 | 校验
  *         前 7 字节逐字节异或得到校验字节；坐标 int16 小端。
  *
  * 解析在串口中断里完成，一个完整目标点作为一个整体写入环形缓冲区；
  * 主循环通过 Comm_PopPoint() 取用，因此上位机可以一次下发很多个目标点。
  *
  * 以后换 SPI 时，新增 comm_spi.c 实现 comm.h 的同一组接口，
  * 从工程中移除本文件即可。
  ******************************************************************************
  */

#include "comm.h"
#include "usart.h"
#include "main.h"
#include "ringbuffer.h"

/* USER CODE BEGIN 0 */
#define COMM_FRAME_LEN   8U
#define COMM_HEAD0       0xAAU
#define COMM_HEAD1       0x55U
#define COMM_LASER_Pos   6U

/* 环形缓冲区大小（必须是 2 的幂）。8192 B ≈ 1365 个目标点。 */
#define COMM_RING_SIZE   8192U

/* 一条记录整体读写，不能有隐式填充变化 */
_Static_assert(sizeof(CommPoint) == 6U, "CommPoint must be 6 bytes");

static RingBuffer *comm_ring = NULL;

volatile uint32_t g_comm_last_rx_cycle = 0U;  /* 调试计时用 */

static volatile uint32_t comm_crc_err = 0U;

static volatile uint8_t comm_rx_byte;
static uint8_t comm_rx_frame[COMM_FRAME_LEN];
static uint8_t comm_rx_len = 0U;

static void comm_parse_byte(uint8_t b);
/* USER CODE END 0 */

void Comm_Init(void)
{
  comm_ring = RingBuffer_Init(COMM_RING_SIZE);
  if (comm_ring == NULL)
  {
    Error_Handler();            /* 堆不够或容量非法 */
  }

  MX_USART1_UART_Init();

  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&comm_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

bool Comm_PopPoint(CommPoint *pt)
{
  if (pt == NULL)
  {
    return false;
  }

  return RingBuffer_Read(comm_ring, pt, sizeof(CommPoint)) == sizeof(CommPoint);
}

uint32_t Comm_GetCrcErrorCount(void)
{
  return comm_crc_err;
}

/* USER CODE BEGIN 1 */

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
      CommPoint pt;

      pt.x = (int16_t)((uint16_t)comm_rx_frame[2] |
                       ((uint16_t)comm_rx_frame[3] << 8));
      pt.y = (int16_t)((uint16_t)comm_rx_frame[4] |
                       ((uint16_t)comm_rx_frame[5] << 8));
      pt.laser = (comm_rx_frame[COMM_LASER_Pos] != 0U) ? 1U : 0U;

      g_comm_last_rx_cycle = DWT->CYCCNT;   /* 一帧收完的时刻，供延时测量 */
      (void)RingBuffer_Write(comm_ring, &pt, sizeof(pt));
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
