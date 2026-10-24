/**
  ******************************************************************************
  * @file    ringbuffer.h
  * @brief   柔性数组环形缓冲区（单生产者 / 单消费者，无锁）
  *
  * 容量在 Init 时通过 malloc 分配，调用者传入的 size 必须是 2 的幂。
  * 读、写都是"整体成功或整体失败"：空间/数据不够时返回 0，不产生半条记录。
  *
  * 使用约束：
  *   - Write 只能由一个上下文调用（例如串口中断）；
  *   - Read 只能由另一个上下文调用（例如主循环）；
  *   - 不要在两者之外的代码里并发调用。
  ******************************************************************************
  */
#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint32_t size;              /* 容量，必须是 2 的幂 */
  volatile uint32_t head;     /* 生产者写 */
  volatile uint32_t tail;     /* 消费者写 */
  uint8_t data[];             /* 柔性数组，长度 = size */
} RingBuffer;

/**
  * @brief 分配并初始化环形缓冲区
  * @param  size 容量（字节），必须是 2 的幂
  * @retval 成功返回句柄；malloc 失败或 size 非法返回 NULL
  */
RingBuffer *RingBuffer_Init(uint32_t size);

/**
  * @brief 释放缓冲区
  */
void RingBuffer_Deinit(RingBuffer *rb);

/**
  * @brief 写入一批数据（整体成功或整体失败）
  * @retval 写入字节数；空间不足返回 0
  */
uint32_t RingBuffer_Write(RingBuffer *rb, const void *src, uint32_t len);

/**
  * @brief 读出一批数据（整体成功或整体失败）
  * @retval 读出字节数；数据不足返回 0
  */
uint32_t RingBuffer_Read(RingBuffer *rb, void *dst, uint32_t len);

/**
  * @brief 擦除：丢弃全部已缓存数据
  */
void RingBuffer_Clear(RingBuffer *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUFFER_H__ */
