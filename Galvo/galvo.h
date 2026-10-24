/**
  ******************************************************************************
  * @file    galvo.h
  * @brief   XY2-100 振镜输出层：API、标定参数
  *
  * 数据来源由 comm.h 提供（Comm_PopPoint），本层不关心是串口还是 SPI。
  ******************************************************************************
  */
#ifndef __GALVO_H__
#define __GALVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- 标定参数，全部待实测 ------------------------------------------------ */

/* 满量程坐标（|x| = 32767）对应的光机角度，由场镜决定，待实测。
   默认 0.35 rad（±20°），使未标定时坐标线性映射到命令字，不会饱和。 */
#define GALVO_FULL_SCALE_RAD   0.35f

/* 等效焦距，与坐标单位一致：|x| / (2f) = 角度 */
#define GALVO_F                (32767.0f / (2.0f * GALVO_FULL_SCALE_RAD))

/* 每弧度对应的 16bit 命令数 */
#define GALVO_COUNTS_PER_RAD   (32767.0f / GALVO_FULL_SCALE_RAD)
#define GALVO_CMD_CENTER       32768.0f           /* 命令字零点 */
#define GALVO_ALPHA0           0.0f               /* 坐标为 0 时 X 轴角度(rad) */
#define GALVO_BETA0            0.0f               /* 坐标为 0 时 Y 轴角度(rad) */

/**
  * @brief 预填中心帧、注册回调并启动 DMA/TIM1 连续输出
  *
  * 调用前需完成 MX_DMA_Init / MX_GPIO_Init / MX_TIM1_Init。
  * 启动后不再停止 DMA 或重配 TIM1，保证波形连续。
  */
void Galvo_Init(void);

/**
  * @brief 主循环调用：取一个最新坐标，换算并编码到 stage，发布给 DMA 中断
  *
  * 没有新坐标时直接返回。函数本身不阻塞。
  */
void Galvo_Task(void);

/**
  * @brief 设置 3 bit 控制字：0b001 标准数据，0b011 串行链路测试
  */
void Galvo_SetControlCode(uint8_t code);

/**
  * @brief 调试计数
  */
uint32_t Galvo_GetDmaErrorCount(void);

/* ---- 计时（调试观测） ---------------------------------------------------- */

/**
  * @brief 打开 DWT 周期计数器（168 MHz，1 µs = 168 个周期）
  *
  * 必须在 Comm_Init / Galvo_Init 之前调用，否则时间戳无效。
  */
void Galvo_DwtInit(void);

/* DMA 中断里一次 memcpy(stage → A/B 半区) 的耗时，单位 DWT 周期 */
extern volatile uint32_t g_dma_copy_cycles;

/* 串口收完一帧 → 该点写进 DMA 缓冲区的总耗时，单位 DWT 周期 */
extern volatile uint32_t g_rx_to_dma_cycles;

#ifdef __cplusplus
}
#endif

#endif /* __GALVO_H__ */
