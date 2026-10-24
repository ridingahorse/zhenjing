/**
  ******************************************************************************
  * @file    comm.h
  * @brief   上位机通信层接口（传输介质无关）
  *
  * 目前实现为 USART1（comm_uart.c）。以后换成 SPI 等介质时，
  * 只需新增一份实现同一组接口的文件并替换编译，上层无需改动。
  *
  * 收到的目标点进入大环形缓冲区，主循环按需取用。
  ******************************************************************************
  */
#ifndef __COMM_H__
#define __COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** 一个目标点 */
typedef struct
{
  int16_t x;
  int16_t y;
  uint8_t laser;      /* 0 = 关，1 = 开 */
} CommPoint;

/**
  * @brief 初始化底层传输、分配环形缓冲区并开始接收
  */
void Comm_Init(void);

/**
  * @brief 取出一个目标点
  * @retval true  取到点；false 当前没有待处理目标
  */
bool Comm_PopPoint(CommPoint *pt);

/**
  * @brief 校验失败的帧数（调试用）
  */
uint32_t Comm_GetCrcErrorCount(void);

/**
  * @brief 最近一帧收完时的 DWT 周期计数（调试计时用，需先开 DWT）
  */
extern volatile uint32_t g_comm_last_rx_cycle;

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H__ */
