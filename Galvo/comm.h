/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm.h
  * @brief   上位机通信层接口（传输介质无关）
  *
  * 目前实现为 USART1（comm_uart.c）。以后换成 SPI 等介质时，
  * 只需新增一份实现同一组接口的文件并替换编译，Galvo 层无需改动。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __COMM_H__
#define __COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
  * @brief 初始化底层传输并开始接收
  */
void Comm_Init(void);

/**
  * @brief 取出最新坐标（每个点只会被取到一次）
  * @retval true  有新点；false 自上次取出后没有新数据
  */
bool Comm_PopPoint(int16_t *x, int16_t *y);

/**
  * @brief 校验失败的帧数（调试用）
  */
uint32_t Comm_GetCrcErrorCount(void);

/**
  * @brief 最近一帧收完时的 DWT 周期计数（调试用，需先开 DWT）
  */
extern volatile uint32_t g_comm_last_rx_cycle;

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H__ */
