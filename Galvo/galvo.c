/**
  ******************************************************************************
  * @file    galvo.c
  * @brief   XY2-100 输出：坐标 → 角度 → 命令字 → 20 拍 BSRR 编码 → 循环 DMA
  *
  * 分工：
  *   - 主循环 Galvo_Task()：取点、换算、编码到 stage[]，发布代号；
  *   - DMA 中断：帧边界把 stage[] 拷到刚发完的那半区（A/B 乒乓）；
  *   - DMA 硬件：循环读取 40 个 word 写 GPIOB->BSRR，永不停止。
  * 因此波形连续性由硬件保证，软件只在帧边界改数据。
  ******************************************************************************
  */

#include "galvo.h"
#include "comm.h"
#include "scurve.h"
#include "laser.h"
#include "tim.h"
#include "main.h"

#include <string.h>
#include <math.h>

/* ---- XY2-100 帧结构 ------------------------------------------------------ */
#define XY_BEATS        20U                 /* 一帧 20 个时钟周期            */
#define XY_BUF_WORDS    (XY_BEATS * 2U)     /* 乒乓双缓冲: 两帧共 40 个 word */

/* GPIOB->BSRR 位掩码 (低 16 位=置位, 高 16 位=复位) */
#define M_SYNC          (1UL << 7)          /* PB7 */
#define M_X             (1UL << 5)          /* PB5 */
#define M_Y             (1UL << 6)          /* PB6 */

/* 一个 BSRR 控制字: 同时确定 SYNC/X/Y 三条线的电平 */
#define BSRR_LEVEL(mask, lvl)  ((lvl) ? (mask) : ((mask) << 16))

/* ---- 双半区缓冲 ---------------------------------------------------------- */
static uint32_t galvo_frame[2][XY_BEATS];   /* DMA 搬运区，[0]=A 半区，[1]=B 半区 */
static uint32_t galvo_stage[XY_BEATS];      /* 主循环编码区 */

/* 代号：stage 写完才自增；半区拷贝完才跟上，主循环据此判断能否覆盖 stage */
static volatile uint32_t galvo_stage_gen = 0U;
static volatile uint32_t galvo_a_gen = 0U;
static volatile uint32_t galvo_b_gen = 0U;

static uint8_t galvo_ctrl = 0x1U;           /* 控制字 001 */

static volatile uint32_t galvo_dma_err = 0U;

/* ---- S 曲线规划 ---------------------------------------------------------- */
typedef enum
{
  GALVO_IDLE = 0,               /* 等待下一个目标点 */
  GALVO_MOVING                  /* 正在按 S 曲线走向目标点 */
} galvo_plan_state_t;

static SCurve galvo_scurve;
static galvo_plan_state_t galvo_plan_state = GALVO_IDLE;
static int16_t galvo_cur_x = 0;            /* 当前输出的坐标 */
static int16_t galvo_cur_y = 0;
static uint32_t galvo_last_point_cycle = 0U;   /* 上一个规划点的输出时刻 */
static uint32_t galvo_last_arrive_cycle = 0U;  /* 到达上一个目标点的时刻 */
static uint8_t  galvo_laser_off_pending = 0U;  /* 已到位，待稳定结束后关光 */

/* µs → DWT 周期（168 MHz 下 1 µs = 168 周期） */
static uint32_t galvo_us_to_cycles(uint32_t us)
{
  return us * (SystemCoreClock / 1000000U);
}

/* ---- 调试计时 ---- */
volatile uint32_t g_dma_copy_cycles = 0U;
volatile uint32_t g_rx_to_dma_cycles = 0U;

/* ===================== 临时测试：S 曲线日志（测完整块删除） =============== */
#define GALVO_TEST_LOG   1
#if GALVO_TEST_LOG
#define GTEST_SEGS       16U

volatile uint32_t gtest_t[GTEST_SEGS][SCURVE_POINTS];   /* 7 个点的输出时刻 */
volatile int16_t  gtest_x[GTEST_SEGS][SCURVE_POINTS];
volatile int16_t  gtest_y[GTEST_SEGS][SCURVE_POINTS];
volatile uint32_t gtest_dt[GTEST_SEGS][SCURVE_POINTS];  /* 与上一个点的间隔 */
volatile uint32_t gtest_settle[GTEST_SEGS];             /* 末点 → 稳定到期 */
volatile uint32_t gtest_total[GTEST_SEGS];              /* 首点 → 稳定到期 */
volatile uint8_t  gtest_seg = 0U;                       /* 已记录的段数 */

static uint32_t   gtest_last_t = 0U;
static uint8_t    gtest_pending_seg = 0U;               /* 等待记录稳定时间的段 */
static volatile uint8_t gtest_settle_pending = 0U;
#endif
/* ===================== 临时测试结束 ====================================== */

/* ---- 内部函数 ------------------------------------------------------------ */
static uint16_t galvo_axis_cmd(int32_t d, float offset);
static void galvo_build_stage(int16_t x, int16_t y, uint32_t *buf);
static void galvo_encode(uint16_t cx, uint16_t cy, uint32_t *buf);
static void galvo_dma_half_cb(DMA_HandleTypeDef *hdma);
static void galvo_dma_cplt_cb(DMA_HandleTypeDef *hdma);
static void galvo_dma_error_cb(DMA_HandleTypeDef *hdma);

#if GALVO_TEST_LOG
static void gtest_record(uint32_t now, uint32_t step, int16_t x, int16_t y);
#endif

/* ---- 坐标换算 ------------------------------------------------------------ */

/**
  * @brief 单轴：α = sgn(d) × |d|/(2f)，再换算成 16bit 命令字
  *
  * @param  d      有符号坐标
  * @param  offset 该轴的零点角度 α0/β0
  */
static uint16_t galvo_axis_cmd(int32_t d, float offset)
{
  float ang = (float)(d < 0 ? -d : d) / (2.0f * GALVO_F);
  int32_t cmd;

  if (d < 0) { ang = -ang; }

  cmd = (int32_t)lroundf((ang - offset) * GALVO_COUNTS_PER_RAD + GALVO_CMD_CENTER);

  if (cmd < 0)      { cmd = 0; }
  if (cmd > 65535)  { cmd = 65535; }

  return (uint16_t)cmd;
}

/**
  * @brief 把一对坐标换算成命令字并编码成一帧 20 个 BSRR 控制字
  */
static void galvo_build_stage(int16_t x, int16_t y, uint32_t *buf)
{
  uint16_t cx = galvo_axis_cmd(x, GALVO_ALPHA0);
  uint16_t cy = galvo_axis_cmd(y, GALVO_BETA0);

  galvo_encode(cx, cy, buf);
}

/**
  * @brief 编码一帧 XY2-100
  *
  * 20 bit, MSB first:
  *   bit 0..2   C2 C1 C0（控制字）
  *   bit 3..18  16bit 命令字，offset binary
  *   bit 19     偶校验（对前 19 位异或）
  * SYNC 前 19 拍为高，第 20 拍拉低。
  */
static void galvo_encode(uint16_t cx, uint16_t cy, uint32_t *buf)
{
  uint32_t ctrl = ((uint32_t)galvo_ctrl & 0x7U) << 16;
  uint32_t wx = ctrl | cx;
  uint32_t wy = ctrl | cy;
  uint32_t px = 0U, py = 0U;

  for (uint32_t i = 0U; i < 19U; i++)
  {
    uint32_t bx = (wx >> (18U - i)) & 1U;
    uint32_t by = (wy >> (18U - i)) & 1U;

    px ^= bx;
    py ^= by;

    /* 三条线每个节拍都必须明确置位或复位，BSRR 没有"保持"语义 */
    buf[i] = BSRR_LEVEL(M_SYNC, 1U) | BSRR_LEVEL(M_X, bx) | BSRR_LEVEL(M_Y, by);
  }

  buf[19] = BSRR_LEVEL(M_SYNC, 0U) | BSRR_LEVEL(M_X, px) | BSRR_LEVEL(M_Y, py);
}

#if GALVO_TEST_LOG
/* 记录一个规划点（只在主循环调用，写内存，不影响波形） */
static void gtest_record(uint32_t now, uint32_t step, int16_t x, int16_t y)
{
  uint8_t s = gtest_seg;
  uint32_t k;

  if ((s >= GTEST_SEGS) || (step == 0U) || (step > SCURVE_POINTS))
  {
    return;
  }

  k = step - 1U;

  gtest_t[s][k] = now;
  gtest_x[s][k] = x;
  gtest_y[s][k] = y;
  gtest_dt[s][k] = (gtest_last_t == 0U) ? 0U : (now - gtest_last_t);
  gtest_last_t = now;

  if (k == (SCURVE_POINTS - 1U))
  {
    gtest_pending_seg = s;
    gtest_settle_pending = 1U;
    gtest_seg = (uint8_t)(s + 1U);      /* 等于 GTEST_SEGS 后停止记录 */
  }
}
#endif

/* ---- DMA 中断：帧边界搬运 ------------------------------------------------ */

/* 上半区(A)刚发完，DMA 正在发 B，可以重填 A */
static void galvo_dma_half_cb(DMA_HandleTypeDef *hdma)
{
  if (hdma->Instance == DMA2_Stream5)
  {
    if (galvo_a_gen != galvo_stage_gen)
    {
      uint32_t a = DWT->CYCCNT;             /* 拷贝前 */
      uint32_t b;

      memcpy(galvo_frame[0], galvo_stage, sizeof(galvo_stage));
      __DMB();
      galvo_a_gen = galvo_stage_gen;

      b = DWT->CYCCNT;                      /* 拷贝后 */
      g_dma_copy_cycles = b - a;
      g_rx_to_dma_cycles = b - g_comm_last_rx_cycle;
    }
  }
}

/* 下半区(B)刚发完，DMA 绕回 A，可以重填 B */
static void galvo_dma_cplt_cb(DMA_HandleTypeDef *hdma)
{
  if (hdma->Instance == DMA2_Stream5)
  {
    if (galvo_b_gen != galvo_stage_gen)
    {
      uint32_t a = DWT->CYCCNT;             /* 拷贝前 */
      uint32_t b;

      memcpy(galvo_frame[1], galvo_stage, sizeof(galvo_stage));
      __DMB();
      galvo_b_gen = galvo_stage_gen;

      b = DWT->CYCCNT;                      /* 拷贝后 */
      g_dma_copy_cycles = b - a;
      g_rx_to_dma_cycles = b - g_comm_last_rx_cycle;
    }
  }
}

/* 只记录故障：恢复操作（Abort/Start）会造成断拍 */
static void galvo_dma_error_cb(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  galvo_dma_err++;
}

/* ---- 对外 API ------------------------------------------------------------ */

void Galvo_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Galvo_Init(void)
{
  galvo_stage_gen = 0U;
  galvo_a_gen = 0U;
  galvo_b_gen = 0U;

  /* 规划器从原点、静止状态开始 */
  galvo_plan_state = GALVO_IDLE;
  galvo_cur_x = 0;
  galvo_cur_y = 0;
  galvo_last_point_cycle = 0U;
  galvo_last_arrive_cycle = 0U;
  galvo_laser_off_pending = 0U;

  Laser_Off();                 /* 上电默认光闸关闭 */

  /* 先填中心帧，避免 DMA 启动后先发出未初始化数据 */
  galvo_build_stage(0, 0, galvo_stage);
  memcpy(galvo_frame[0], galvo_stage, sizeof(galvo_stage));
  memcpy(galvo_frame[1], galvo_stage, sizeof(galvo_stage));

  (void)HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_HALFCPLT_CB_ID,
                                 galvo_dma_half_cb);
  (void)HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_CPLT_CB_ID,
                                 galvo_dma_cplt_cb);
  (void)HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_ERROR_CB_ID,
                                 galvo_dma_error_cb);

  /* 让 TIM1 更新事件产生 DMA 请求 (DIER.UDE) */
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

  if (HAL_DMA_Start_IT(&hdma_tim1_up,
                       (uint32_t)galvo_frame,
                       (uint32_t)&GPIOB->BSRR,
                       XY_BUF_WORDS) != HAL_OK)
  {
    Error_Handler();
  }

  /* 高级定时器必须开 MOE，否则 PE9 无输出 */
  __HAL_TIM_MOE_ENABLE(&htim1);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Galvo_Task(void)
{
  uint32_t now;
  int16_t x, y;

  now = DWT->CYCCNT;

#if GALVO_TEST_LOG
  /* 稳定时间到期时记录一次（不依赖后面是否还有新点） */
  if ((gtest_settle_pending != 0U) &&
      ((now - galvo_last_arrive_cycle) >= galvo_us_to_cycles(SCURVE_SETTLE_US)))
  {
    uint8_t s = gtest_pending_seg;

    gtest_settle[s] = now - galvo_last_arrive_cycle;
    gtest_total[s] = (gtest_t[s][SCURVE_POINTS - 1U] - gtest_t[s][0]) + gtest_settle[s];
    gtest_settle_pending = 0U;
  }
#endif

  /* 相邻规划点的最小间隔 */
  if ((now - galvo_last_point_cycle) < galvo_us_to_cycles(SCURVE_INTERVAL_US))
  {
    return;
  }

  if (galvo_plan_state == GALVO_IDLE)
  {
    CommPoint pt;

    /* 到达目标点后的稳定时间，过了才允许写入下一段并切换激光 */
    if ((now - galvo_last_arrive_cycle) < galvo_us_to_cycles(SCURVE_SETTLE_US))
    {
      return;
    }

    /* 稳定结束，上一段才算真正完成：此时关光 */
    if (galvo_laser_off_pending != 0U)
    {
      Laser_Off();
      galvo_laser_off_pending = 0U;
    }

    if (!Comm_PopPoint(&pt))
    {
      return;                  /* 队列为空，保持当前位置（激光已在上面关闭） */
    }

    /* 激光状态属于"移动到该目标点"这一段，在段起点开启 */
    if (pt.laser != 0U)
    {
      Laser_On();
    }

    SCurve_Start(&galvo_scurve, galvo_cur_x, galvo_cur_y, pt.x, pt.y);
    galvo_plan_state = GALVO_MOVING;
  }

  /* 上一代还没铺满两个半区时先不动 stage（等 DMA 中断追上） */
  if ((galvo_a_gen != galvo_stage_gen) || (galvo_b_gen != galvo_stage_gen))
  {
    return;
  }

  if (!SCurve_Next(&galvo_scurve, &x, &y))
  {
    galvo_plan_state = GALVO_IDLE;
    return;
  }

  galvo_build_stage(x, y, galvo_stage);
  galvo_cur_x = x;
  galvo_cur_y = y;
  galvo_last_point_cycle = now;

#if GALVO_TEST_LOG
  gtest_record(now, galvo_scurve.step, x, y);
#endif

  if (SCurve_Done(&galvo_scurve))
  {
    galvo_plan_state = GALVO_IDLE;          /* 已到达目标点 */
    galvo_last_arrive_cycle = now;
    galvo_laser_off_pending = 1U;           /* 稳定时间结束后再关光 */
  }

  __DMB();                     /* stage 写完，再发布代号 */
  galvo_stage_gen++;
}

void Galvo_SetControlCode(uint8_t code)
{
  galvo_ctrl = (uint8_t)(code & 0x7U);
}

uint32_t Galvo_GetDmaErrorCount(void)
{
  return galvo_dma_err;
}
