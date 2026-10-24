/**
  ******************************************************************************
  * @file    scurve.c
  * @brief   S 曲线轨迹规划实现
  ******************************************************************************
  */

#include "scurve.h"

#include <math.h>

void SCurve_Start(SCurve *c, int16_t sx, int16_t sy, int16_t tx, int16_t ty)
{
  if (c == NULL)
  {
    return;
  }

  c->sx = sx;
  c->sy = sy;
  c->tx = tx;
  c->ty = ty;
  c->step = 0U;
}

bool SCurve_Next(SCurve *c, int16_t *x, int16_t *y)
{
  float u, s;

  if ((c == NULL) || (x == NULL) || (y == NULL))
  {
    return false;
  }

  if (c->step >= SCURVE_POINTS)
  {
    return false;
  }

  c->step++;

  /* u: 0..1，第 SCURVE_POINTS 个点恰好等于目标点 */
  u = (float)c->step / (float)SCURVE_POINTS;
  s = u * u * (3.0f - 2.0f * u);

  *x = (int16_t)lroundf((float)c->sx + (float)(c->tx - c->sx) * s);
  *y = (int16_t)lroundf((float)c->sy + (float)(c->ty - c->sy) * s);
  return true;
}

bool SCurve_Done(const SCurve *c)
{
  return (c != NULL) && (c->step >= SCURVE_POINTS);
}
