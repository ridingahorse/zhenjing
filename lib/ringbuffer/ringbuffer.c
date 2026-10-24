/**
  ******************************************************************************
  * @file    ringbuffer.c
  * @brief   柔性数组环形缓冲区实现
  ******************************************************************************
  */

#include "ringbuffer.h"

#include <stdlib.h>
#include <string.h>

#include "stm32f4xx.h"      /* __DMB() */

RingBuffer *RingBuffer_Init(uint32_t size)
{
  RingBuffer *rb;

  /* 必须是非 0 的 2 的幂 */
  if ((size == 0U) || ((size & (size - 1U)) != 0U))
  {
    return NULL;
  }

  /* 柔性数组: 头结构 + 数据区一次分配 */
  rb = (RingBuffer *)malloc(sizeof(RingBuffer) + size);
  if (rb == NULL)
  {
    return NULL;
  }

  rb->size = size;
  rb->head = 0U;
  rb->tail = 0U;
  return rb;
}

void RingBuffer_Deinit(RingBuffer *rb)
{
  free(rb);
}

uint32_t RingBuffer_Write(RingBuffer *rb, const void *src, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)src;
  uint32_t mask, used, space, first;

  if ((rb == NULL) || (p == NULL) || (len == 0U))
  {
    return 0U;
  }

  mask = rb->size - 1U;
  used = rb->head - rb->tail;
  space = rb->size - used;

  /* 整体写: 空间不够就一个字节都不写 */
  if (len > space)
  {
    return 0U;
  }

  first = rb->size - (rb->head & mask);
  if (first > len)
  {
    first = len;
  }

  memcpy(&rb->data[rb->head & mask], p, first);
  if (len > first)
  {
    memcpy(&rb->data[0], p + first, len - first);
  }

  __DMB();                    /* 数据先落地，再发布 head */
  rb->head += len;
  return len;
}

uint32_t RingBuffer_Read(RingBuffer *rb, void *dst, uint32_t len)
{
  uint8_t *p = (uint8_t *)dst;
  uint32_t mask, used, first;

  if ((rb == NULL) || (p == NULL) || (len == 0U))
  {
    return 0U;
  }

  used = rb->head - rb->tail;

  /* 整体读: 数据不够就读不出来 */
  if (len > used)
  {
    return 0U;
  }

  mask = rb->size - 1U;
  first = rb->size - (rb->tail & mask);
  if (first > len)
  {
    first = len;
  }

  __DMB();                    /* 确认 head 发布的数据已可见 */
  memcpy(p, &rb->data[rb->tail & mask], first);
  if (len > first)
  {
    memcpy(p + first, &rb->data[0], len - first);
  }

  rb->tail += len;
  return len;
}

void RingBuffer_Clear(RingBuffer *rb)
{
  if (rb != NULL)
  {
    rb->tail = rb->head;
  }
}
