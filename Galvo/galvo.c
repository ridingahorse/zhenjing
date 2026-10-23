/**
  ******************************************************************************
  * @file    galvo.c
  * @brief   XY2-100 输出：坐标 → 角度 → 命令字 → 20 拍 BSRR 编码 → 循环 DMA
  *
  * 分工：
  *   - 主循环 Galvo_Task()：取点、查 LUT、换算、编码到 stage[]，发布代号；
  *   - DMA 中断：帧边界把 stage[] 拷到刚发完的那半区（A/B 乒乓）；
  *   - DMA 硬件：循环读取 40 个 word 写 GPIOB->BSRR，永不停止。
  * 因此波形连续性由硬件保证，软件只在帧边界改数据。
  ******************************************************************************
  */

#include "galvo.h"
#include "galvo_lut.h"
#include "comm.h"
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

static volatile uint32_t galvo_drop = 0U;
static volatile uint32_t galvo_dma_err = 0U;

/* ---- 调试计时 ---- */
volatile uint32_t g_dma_copy_cycles = 0U;
volatile uint32_t g_rx_to_dma_cycles = 0U;

/* ---- 内部函数 ------------------------------------------------------------ */
static float galvo_lut_sample(const int16_t lut[GALVO_LUT_SIZE][GALVO_LUT_SIZE],
                              uint32_t ax, uint32_t ay);
static uint16_t galvo_axis_cmd(int32_t d, uint32_t ax, uint32_t ay,
                               const int16_t lut[GALVO_LUT_SIZE][GALVO_LUT_SIZE],
                               float offset);
static void galvo_build_stage(int16_t x, int16_t y, uint32_t *buf);
static void galvo_encode(uint16_t cx, uint16_t cy, uint32_t *buf);
static void galvo_dma_half_cb(DMA_HandleTypeDef *hdma);
static void galvo_dma_cplt_cb(DMA_HandleTypeDef *hdma);
static void galvo_dma_error_cb(DMA_HandleTypeDef *hdma);

/* ---- LUT ---------------------------------------------------------------- */

/**
  * @brief 双线性插值取样，ax/ay 为坐标绝对值（0..32768）
  * @retval 校正量（弧度），未标定时为 0
  */
static float galvo_lut_sample(const int16_t lut[GALVO_LUT_SIZE][GALVO_LUT_SIZE],
                              uint32_t ax, uint32_t ay)
{
  uint32_t ix, iy;
  float fx, fy, v00, v10, v01, v11, top, bot;

  if (ax > 32767U) { ax = 32767U; }
  if (ay > 32767U) { ay = 32767U; }

  ix = ax >> 8;
  iy = ay >> 8;

  /* 插值需要 i+1，索引限制在 0..126；末段的分数部分截到 1.0 */
  if (ix > (GALVO_LUT_SIZE - 2U)) { ix = GALVO_LUT_SIZE - 2U; }
  if (iy > (GALVO_LUT_SIZE - 2U)) { iy = GALVO_LUT_SIZE - 2U; }

  fx = (float)(ax - (ix << 8)) * (1.0f / 256.0f);
  fy = (float)(ay - (iy << 8)) * (1.0f / 256.0f);
  if (fx > 1.0f) { fx = 1.0f; }
  if (fy > 1.0f) { fy = 1.0f; }

  v00 = (float)lut[iy][ix];
  v10 = (float)lut[iy][ix + 1U];
  v01 = (float)lut[iy + 1U][ix];
  v11 = (float)lut[iy + 1U][ix + 1U];

  top = v00 + (v10 - v00) * fx;
  bot = v01 + (v11 - v01) * fx;
  return (top + (bot - top) * fy) * GALVO_LUT_UNIT;
}

/**
  * @brief 单轴：α = sgn(d) × (|d|/(2f) + LUT)，再换算成 16bit 命令字
  *
  * @param  d      有符号坐标
  * @param  ax/ay  坐标绝对值，用于查表
  * @param  offset 该轴的零点角度 α0/β0
  */
static uint16_t galvo_axis_cmd(int32_t d, uint32_t ax, uint32_t ay,
                               const int16_t lut[GALVO_LUT_SIZE][GALVO_LUT_SIZE],
                               float offset)
{
  float ang = (float)ax / (2.0f * GALVO_F) + galvo_lut_sample(lut, ax, ay);
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
  uint32_t ax = (uint32_t)(x < 0 ? -(int32_t)x : (int32_t)x);
  uint32_t ay = (uint32_t)(y < 0 ? -(int32_t)y : (int32_t)y);

  uint16_t cx = galvo_axis_cmd(x, ax, ay, galvo_lut_alpha, GALVO_ALPHA0);
  uint16_t cy = galvo_axis_cmd(y, ax, ay, galvo_lut_beta,  GALVO_BETA0);

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
  int16_t x, y;

  if (!Comm_PopPoint(&x, &y))
  {
    return;
  }

  /* 上一代还没铺满两个半区时不动 stage（正常速率下永远不会命中） */
  if ((galvo_a_gen != galvo_stage_gen) || (galvo_b_gen != galvo_stage_gen))
  {
    galvo_drop++;               /* 该点丢弃，下次发新点会覆盖 */
    return;
  }

  galvo_build_stage(x, y, galvo_stage);

  __DMB();                     /* stage 写完，再发布代号 */
  galvo_stage_gen++;
}

void Galvo_SetControlCode(uint8_t code)
{
  galvo_ctrl = (uint8_t)(code & 0x7U);
}

uint32_t Galvo_GetDropCount(void)
{
  return galvo_drop;
}

uint32_t Galvo_GetDmaErrorCount(void)
{
  return galvo_dma_err;
}
