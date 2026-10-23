/**
  ******************************************************************************
  * @file    galvo_lut.h
  * @brief   二维角度校正表（当前全 0）
  *
  * 表存放在 Flash（const，.rodata），不占 RAM。每张 128×128×int16 = 32 KB，
  * 两张共 64 KB。
  *
  * 索引只覆盖 |坐标| 的正象限：表[行 = |y|>>8][列 = |x|>>8]，
  * 单位 1 LSB = 1 µrad（见 GALVO_LUT_UNIT）。符号由 galvo.c 的 sgn() 处理。
  *
  * 标定完成后由上位机工具生成完整初始化列表替换这里的 {0}。
  ******************************************************************************
  */
#ifndef __GALVO_LUT_H__
#define __GALVO_LUT_H__

#include <stdint.h>

#define GALVO_LUT_SIZE   128
#define GALVO_LUT_UNIT   1e-6f      /* 每个 LSB 对应的弧度 */

static const int16_t galvo_lut_alpha[GALVO_LUT_SIZE][GALVO_LUT_SIZE] = {{0}};
static const int16_t galvo_lut_beta [GALVO_LUT_SIZE][GALVO_LUT_SIZE] = {{0}};

#endif /* __GALVO_LUT_H__ */
