/**
  ******************************************************************************
  * @file    scurve.h
  * @brief   S 曲线轨迹规划（纯数学，不涉及硬件）
  *
  * 从起点到目标点生成 SCURVE_POINTS 个规划点（含终点），
  * 位置按三次 smoothstep 变化：s(u) = 3u² − 2u³，u = k / N。
  * 首尾速度为 0，等效于 S 形加减速。
  *
  * 时间节奏（点间隔、到位稳定时间）由调用者控制，本模块只负责几何插值。
  ******************************************************************************
  */
#ifndef __SCURVE_H__
#define __SCURVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- 可一键修改的规划参数 ------------------------------------------------ */
#define SCURVE_POINTS       7U      /* 每段轨迹的规划点数（含终点） */
#define SCURVE_INTERVAL_US  100U    /* 相邻规划点的最小更新间隔 */
#define SCURVE_SETTLE_US    200U    /* 到达目标点后的稳定时间 */

typedef struct
{
  int32_t sx;                   /* 起点 */
  int32_t sy;
  int32_t tx;                   /* 目标点 */
  int32_t ty;
  uint32_t step;                /* 已输出的规划点序号 0..SCURVE_POINTS */
} SCurve;

/**
  * @brief 开始一段新的轨迹
  */
void SCurve_Start(SCurve *c, int16_t sx, int16_t sy, int16_t tx, int16_t ty);

/**
  * @brief 取下一个规划点
  * @retval true  输出有效；false 本段已走完（或参数非法）
  */
bool SCurve_Next(SCurve *c, int16_t *x, int16_t *y);

/**
  * @brief 本段是否已到达目标点
  */
bool SCurve_Done(const SCurve *c);

#ifdef __cplusplus
}
#endif

#endif /* __SCURVE_H__ */
