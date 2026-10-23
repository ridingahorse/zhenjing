# 振镜控制卡设计方案

| 项目 | 内容 |
|---|---|
| 版本 | v0.1 |
| 日期 | 2026-10-23 |
| 平台 | STM32F407VET6 / 168 MHz |
| 状态 | 设计评审，待实现 |

---

## 1. 目标

控制卡通过串口接收上位机下发的 XY 坐标，在卡内完成「坐标 → 振镜角度 → 16 bit 角度命令字」的换算，并按 XY2-100 时序以 **2 MHz 连续时钟、100 kHz 帧率** 把 XY 角度命令发送给振镜驱动。

核心约束：

- 输出波形**任何时刻不允许中断**；每个时钟节拍（500 ns）必须有一个新的 BSRR 控制字写入，不允许出现空拍。
- 数据更新允许有旧帧过渡，最大 500 µs（本方案实际最大 10 µs）。
- 主循环允许被阻塞；波形连续性不得依赖主循环的实时性。

---

## 2. 系统框图

```
                  ┌──────────────────────────────────────────────────────┐
   上位机          │ STM32F407                                            │
  ┌───────┐        │                                                      │
  │ 串口  │─TX────▶│ PA10 USART1_RX ─▶ 帧解析 ─▶ 最新点缓存               │
  └───────┘        │                                │                     │
                   │                                ▼                     │
                   │                          Galvo_Task()                │
                   │                     查 LUT / 算角度 / 编码           │
                   │                                │                     │
                   │                                ▼                     │
                   │                          stage[20]                  │
                   │                                │                     │
                   │                  HT/TC 中断拷贝 │                     │
                   │                                ▼                     │
                   │           frame[2][20] = 连续 40 word 循环缓冲        │
                   │                    │                                 │
                   │      TIM1_UP ─────▶│ DMA2_Stream5 (循环, 40 word)   │
                   │      (2 MHz)       │                                 │
                   │                    ▼                                 │
                   │            GPIOB->BSRR  (PB7 SYNC / PB5 X / PB6 Y)   │
                   │                                                      │
                   │      TIM1_CH1 (PE9) ─▶ 2 MHz CLK                     │
                   └──────────────────────────────────────────────────────┘
```

关键点：**TIM1 的更新事件直接触发 DMA 搬 BSRR，全程不经过 CPU**。CPU 只负责往缓冲区里准备数据，波形由硬件自行维持。

---

## 3. 硬件资源

| 信号 | 引脚 | 配置 |
|---|---|---|
| CLK | PE9 | TIM1_CH1，AF1，PWM1，2 MHz |
| SYNC | PB7 | GPIO 推挽输出，由 DMA 写 BSRR 驱动 |
| X_DATA | PB5 | 同上 |
| Y_DATA | PB6 | 同上 |
| UART TX | PA9 | USART1_TX，AF7 |
| UART RX | PA10 | USART1_RX，AF7 |

外部串口参数：**115200 8N1，无流控**。

> 注意：PB6 已用于 Y_DATA，因此不能使用 PB6/PB7 那组 USART1 复用引脚。

### 3.1 定时器配置

| 参数 | 值 | 说明 |
|---|---|---|
| 计数时钟 | 168 MHz | APB2 定时器时钟 |
| PSC | 0 | |
| ARR | 83 | 84 个计数 = 500 ns → 更新事件 2 MHz |
| CCR1 | 42 | 250 ns 高 / 250 ns 低 → CLK 2 MHz |
| 模式 | PWM1 | CNT=0 上升沿，CNT=42 下降沿 |
| MOE | 使能 | 高级定时器必须，否则 PE9 无输出 |
| UDE | 使能 | 更新事件产生 DMA 请求 |

数据更新发生在 CNT=0（上升沿），振镜在 CNT=42（下降沿）采样，建立时间 250 ns，远大于协议要求的 50 ns。

### 3.2 DMA 配置

| 参数 | 值 |
|---|---|
| 流/通道 | DMA2_Stream5 / Channel 6（TIM1_UP） |
| 方向 | 内存 → 外设 |
| 源 | `frame`（40 word，自增） |
| 目的 | `GPIOB->BSRR`（不自增） |
| 数据宽度 | Word / Word |
| 模式 | **Circular** |
| 长度 | 40 |
| FIFO | 关闭 |
| 优先级 | 建议 HIGH（当前工程为 LOW，可改） |
| NVIC | DMA2_Stream5，抢占优先级 0 |

**初始化后不再调用 `HAL_DMA_Abort/Start`，不修改 M0AR/NDTR，不停止 TIM1。** 这是波形连续的前提。

---

## 4. 软件结构

```
Galvo/
  comm.h         // 传输无关接口：Comm_Init / Comm_PopPoint
  comm_uart.c    // 当前传输实现：USART1 + 帧解析 + 最新点缓存
  galvo.h        // 对外 API、参数宏
  galvo.c        // 帧编码、DMA 回调、LUT 换算、双半区管理
  galvo_lut.h    // 校准表（128×128 ×2，当前全 0）与表参数
```

CubeMX 生成的 `MX_TIM1_Init` / `MX_DMA_Init` / `MX_USART1_UART_Init` 保持不变；`stm32f4xx_it.c` 中的中断向量只调用 HAL。

**传输层与业务层解耦**：`Galvo` 只依赖 `Comm_PopPoint()`，不知道数据来自串口、SPI 还是以太网。以后更换介质时，新写一个 `comm_spi.c` 实现同一组 `comm.h` 接口，把 `comm_uart.c` 从编译列表里去掉即可，`galvo.c` 不用改。

### 4.1 对外 API

```c
/* comm.h —— 传输无关 */
void     Comm_Init(void);                     /* 初始化底层传输并开始接收 */
bool     Comm_PopPoint(int16_t *x, int16_t *y);/* 取最新坐标，无新点时返回 false */
uint32_t Comm_GetCrcErrorCount(void);         /* 校验错误计数 */

/* galvo.h */
void     Galvo_Init(void);                    /* 填中心帧、注册回调、启动 DMA/TIM */
void     Galvo_Task(void);                    /* 主循环调用：出队、换算、编码 */
void     Galvo_SetControlCode(uint8_t code);  /* 0b001 标准 / 0b011 串口链路测试 */
```

`comm_uart.c` 负责逐字节中断、帧解析入队；`HAL_UART_RxCpltCallback` 解析完立刻重挂接收。

---

## 5. 数据流

1. **接收**：USART1 每收到 1 字节进入中断，状态机解析 `AA 55 ...` 帧；校验通过后把 `(x, y)`（共 4 字节）写入最新点缓存，并置「有新点」标志。
2. **转换（主循环）**：`Galvo_Task()` 取最新点，查 LUT 计算 α、β，换算成 16 bit 命令字，编码为 20 个 BSRR 控制字写入 `stage[20]`，最后发布新的 `stage_gen`。
3. **分发（DMA 中断）**：HT/TC 中断在帧边界把 `stage[]` 拷贝到「刚发完、下一轮才会被读」的那一半（A 或 B）。
4. **输出（纯硬件）**：DMA 循环读取 40 word，每个 TIM1 更新事件搬一个 word 到 `GPIOB->BSRR`；TIM1_CH1 同时翻转 CLK。

主循环被阻塞时，输出停留在上一帧，但**波形不会中断**；恢复后直接跳到最新坐标。

### 5.1 最新点缓存与 LUT 的区别

这两个「表」经常被混淆，职责完全不同：

| | 最新点缓存 | LUT（校准表） |
|---|---|---|
| 内容 | 上位机刚发来的 `(x, y)` 坐标 | 每个坐标处的角度校正量 |
| 尺寸 | 2 槽 × 4 B（双缓冲） | 128 × 128 × 2 轴 × 2 B |
| 何时写 | 串口中断每收到一帧写一次（覆盖旧值） | 标定完成后写死，重新编译烧录 |
| 何时读 | 每取一次新点 | 每换算一个点读 4 个值（双线性） |
| 存储 | RAM（运行时数据） | Flash（`const`，掉电保留） |
| 无新数据/全 0 时 | 保持上一坐标 | 退化为纯线性 f-theta 映射 |

一句话：**最新点缓存只存「下一步要画到哪」，LUT 是「怎么把点画准」的静态系数表。**

用两个槽而不是一个 `x`/`y` 变量，是因为两个中断有抢占关系：取点端（DMA 中断，优先级 0）可以打断写点端（串口中断，优先级 1）。若共用一个变量，取点可能拿到「新的 x + 旧的 y」。双槽让写者总写另一槽、写完置标志，读者永远读完整的一组。

---

## 6. 串口协议（当前传输实现）

> 这一节描述 `comm_uart.c` 的帧格式。协议本身属于传输层，换成 SPI 或其他介质时可以整体替换，`Galvo` 层不受影响。

### 6.1 帧格式

```
偏移  0     1     2     3     4     5     6
     AA    55    XL    XH    YL    YH    CS
```

- X、Y：`int16`，**小端**，范围 −32768 ~ +32767
- CS：前 6 字节逐字节异或
- 帧之间无间隔要求

### 6.2 示例

| 坐标 | 字节流 |
|---|---|
| (0, 0) | `AA 55 00 00 00 00 FF` |
| (1000, −1000) | `AA 55 E8 03 18 FC F0` |
| (20000, 0) | `AA 55 20 4E 00 00 91` |
| (−20000, 0) | `AA 55 E0 B1 00 00 AE` |
| (32767, −32768) | `AA 55 FF 7F 00 80 FF` |

### 6.3 速率

单帧 7 字节 = 70 bit。115200 bps 下最短间隔约 **0.61 ms/帧**（≈1642 帧/s）。取点端 10 kHz 消费，比串口快一个数量级，正常情况下每个点都会被取走。

### 6.4 解析容错

- 任意位置重新搜索 `AA 55`，支持 `AA AA 55`。
- 校验失败丢弃当前帧并重新同步。
- 新点覆盖尚未取走的旧点（只保留最新坐标）。

---

## 7. XY2-100 输出时序

### 7.1 帧结构（每轴 20 bit，MSB first）

| 节拍 | 内容 | SYNC |
|---|---|---|
| 0 | C2 | 1 |
| 1 | C1 | 1 |
| 2 | C0 | 1 |
| 3..18 | D15 … D0（16 bit 角度命令） | 1 |
| 19 | 偶校验 P | **0** |

- 标准数据传输：`{C2,C1,C0} = {0,0,1}`
- 串行数据链路测试：`{0,1,1}`
- 偶校验：对前 19 位（含控制字）异或，使 20 位中 1 的个数为偶数。
- X、Y 两条数据线在同一节拍并行输出各自的数据。

### 7.2 时序

```
CLK   _/‾\_/‾\_/‾\ ... 20 个周期
         ↑ 上升沿：数据更新（DMA 写 BSRR）
         ↓ 下降沿：振镜采样
SYNC  ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____   （前 19 拍高，第 20 拍低）
```

| 参数 | 值 |
|---|---|
| CLK 频率 | 2 MHz（周期 500 ns） |
| 帧长 | 20 bit × 500 ns = 10 µs |
| 帧率 | 100 kHz |
| 数据建立时间 | 250 ns |

### 7.3 BSRR 控制字

```c
#define PIN_SYNC (1UL << 7)   /* PB7 */
#define PIN_X    (1UL << 5)   /* PB5 */
#define PIN_Y    (1UL << 6)   /* PB6 */

/* BSRR 低 16 位写 1 = 置高，高 16 位写 1 = 拉低 */
#define SET(p) ((p))
#define RST(p) ((p) << 16)

word = (sync ? SET(PIN_SYNC) : RST(PIN_SYNC))
     | (x    ? SET(PIN_X)    : RST(PIN_X))
     | (y    ? SET(PIN_Y)    : RST(PIN_Y));
```

每拍三条线都必须显式置位或复位，不能省略（BSRR 没有「保持」语义）。

---

## 8. 双半区 DMA 方案

### 8.1 缓冲区

```c
#define XY_BEATS 20U

uint32_t frame[2][XY_BEATS];   /* 连续 40 word：frame[0] = A 区，frame[1] = B 区 */
uint32_t stage[XY_BEATS];      /* 主循环计算区 */

volatile uint32_t stage_gen;   /* 主循环发布的新帧代号 */
volatile uint32_t a_gen;       /* A 区已应用到第几代 */
volatile uint32_t b_gen;       /* B 区已应用到第几代 */
```

DMA 源地址固定为 `(uint32_t)frame`，长度 40，循环模式。**不需要切换 DMA 地址**。

### 8.2 为什么不能手动切 DMA

2 MHz 下每 500 ns 必须搬一个 word。手动切换需要：

```
停流 → 等 EN=0 → 改 M0AR → 重新使能
```

裸寄存器操作约 100~200 ns，走 HAL 则 0.5~1 µs，期间 TIM1 的更新请求无人响应，至少丢一拍，数据线保持上一拍电平，整帧错位。这违反「中断不超过 50 ns」的约束。

循环模式由 DMA 硬件在 NDTR 归零时自动重装，流始终使能，**空档为 0**。

### 8.3 帧边界与回调

40 word 循环的硬件行为：

```
t(µs)      0────10────20────30────40
DMA 发送   [ A  ]  [ B  ]  [ A  ]  [ B  ]
中断         —       HT     TC     HT
```

- **HT**：A 刚发完，DMA 开始发 B。此时 A 空闲，**下一次被读是 10 µs 后**。
- **TC**：B 刚发完，DMA 绕回 A。B 空闲，下次被读也是 10 µs 后。

### 8.4 拷贝放在 HT/TC 中断，不放在主循环

```c
/* HT：A 刚发完 */
static void galvo_dma_half_cb(DMA_HandleTypeDef *hdma)
{
    if (a_gen != stage_gen) {
        memcpy(frame[0], stage, sizeof(stage));
        __DMB();
        a_gen = stage_gen;
    }
}

/* TC：B 刚发完 */
static void galvo_dma_cplt_cb(DMA_HandleTypeDef *hdma)
{
    if (b_gen != stage_gen) {
        memcpy(frame[1], stage, sizeof(stage));
        __DMB();
        b_gen = stage_gen;
    }
}
```

理由：

- 波形连续性由 DMA 硬件保证，**不依赖**这个拷贝；
- 但拷贝有硬截止时间：那一半 10 µs 后就会被再次读取。放在优先级 0 的 DMA 中断里，80 字节拷贝约 0.5~1 µs，余量 ≥9 µs；放在主循环则可能被其他操作拖过截止时间。
- 中断里只做拷贝，**LUT 查表和编码绝不放进中断**。

### 8.5 主循环发布

```c
static void galvo_task(void)
{
    int16_t x, y;

    if (!Comm_PopPoint(&x, &y)) {
        return;
    }

    /* 上一代还没铺满两个区时不覆盖 stage（按主机速率永远不会命中） */
    if (a_gen != stage_gen || b_gen != stage_gen) {
        galvo_dbg.drop++;
        return;
    }

    build_stage(x, y, stage);   /* LUT + 角度换算 + 20 拍编码 */
    __DMB();
    stage_gen++;
}
```

`stage_gen` 在 `stage[]` 写完之后才自增，中断看到新代号时数据一定完整；主循环通过 `a_gen/b_gen` 确认上一代已铺满两区，保证不会与中断同时操作 `stage[]`。整个过程**不需要关中断**。

### 8.6 换帧延迟

| 场景 | 延迟 |
|---|---|
| 新点到达 → 主循环算完 | 几十 µs（取决于主循环） |
| 算完 → 第一半被发送 | ≤10 µs |
| 算完 → 两半都更新 | ≤20 µs |
| 旧帧最长停留 | **10 µs**（要求 ≤500 µs） |

### 8.7 死守的规则

1. 初始化后不调用 `HAL_DMA_Abort/Start/Stop`，不重配 M0AR/NDTR；
2. 运行中不改 TIM1 的 ARR/CCR；
3. 除 DMA 外，任何代码都不写 `GPIOB->BSRR`，也不对 PB5/6/7 调 `HAL_GPIO_WritePin`；
4. DMA 出错中断只记录故障，恢复操作会造成断拍，仅在故障时使用。

---

## 9. 坐标 → 角度 → 命令字

### 9.1 公式

```
α = sgn(x_d) × ( |x_d| / (2f) + LUT_α(|x_d|, |y_d|) )
β = sgn(y_d) × ( |y_d| / (2f) + LUT_β(|x_d|, |y_d|) )

cmd = round( (α − α0) × K ) + 32768
```

| 符号 | 含义 | 初值 |
|---|---|---|
| f | 等效焦距（与坐标单位一致），`|x|/(2f)` 得到角度 | 由满量程反推：32767 / (2 × 0.35) ≈ 46810 |
| LUT_α / LUT_β | 二维角度校正表（弧度） | 全 0 |
| α0 / β0 | 坐标为 0 时的轴角度 | 先给 0，实测填写 |
| K | 每弧度对应的 16 bit 命令数 | 待标定 |

`sgn(0)` 取 `+1`。

该公式隐含一个假设：校正量关于 X、Y 轴对称（奇函数），因此 LUT 只需存 `|x|,|y|` 的正象限，符号由 `sgn()` 处理。如果实测发现四个象限的畸变不对称，需要改成带符号索引或分象限存表。

### 9.2 LUT 尺寸决策

原始需求为 512×512，但：

- 512×512 × 2 轴 × `int16` = **1 MB**；
- F407VET6 只有 512 KB Flash + 192 KB RAM，放不下。

**决定：改为 128×128 × 2 轴 `int16`，共 64 KB，放 Flash（`const`），双线性插值。**

- 坐标步长 = 32768 / 128 = 256 counts；
- 1 LUT LSB = 1 µrad，覆盖 ±32.767 mrad 校正量；
- 校正场是平滑的，128×128 + 插值足以覆盖场镜畸变；
- 如果后续确认不够，可升到 256×256（每轴 128 KB，共 256 KB，仍可放入 512 KB Flash），但需要重新评估余量。

### 9.3 表存储

```c
#define GALVO_LUT_SIZE   128
#define GALVO_LUT_UNIT   1e-6f     /* 每 LSB 的弧度数 */

static const int16_t galvo_lut_alpha[GALVO_LUT_SIZE][GALVO_LUT_SIZE] = {0};
static const int16_t galvo_lut_beta [GALVO_LUT_SIZE][GALVO_LUT_SIZE] = {0};
```

未标定时全 0，配上默认的 `GALVO_FULL_SCALE_RAD = 0.35`，映射退化为恒等：`cmd = 32768 + x`（`x = 32767` → `0xFFFF`，`x = −32767` → `0x0001`）。这既是一个安全的默认行为，也方便先用逻辑分析仪直接核对链路。

**表放在 Flash，不占 RAM。** `const` 修饰后链接到 `.rodata`，64 KB 全部落在 Flash 里；运行时不改写，也不需要 RAM 副本。上位机 1 ms 发一次新点，而主循环换算一个点只需要查 4 个 `int16`（双线性插值）加几次浮点运算，远小于 1 ms，所以「Flash 读比 RAM 慢」在这里完全不是瓶颈。唯一代价是标定值更新必须重新烧录；如果将来需要在线更新，再改成从 Flash 里某个独立扇区加载到 RAM 副本即可。

### 9.4 索引与双线性插值

```c
/* |x| 上限 32768，先 clamp；>>8 得到 0..127，插值时索引限制到 126 */
static uint32_t lut_idx(uint32_t a)
{
    if (a > 32767U) a = 32767U;
    return (a >> 8) > 126U ? 126U : (a >> 8);
}

/* 小数部分，限制在 [0,1]，避免顶端越界
   fx = clamp((|x| - (i << 8)) / 256.0f, 0.0f, 1.0f)
   fy 同理

   value = (1-fx)(1-fy)·L[i][j] + fx(1-fy)·L[i+1][j]
         + (1-fx)fy·L[i][j+1] + fx·fy·L[i+1][j+1] */
```

注意 `x = −32768` 时 `|x| = 32768`，必须 clamp。最顶部 256 个坐标会落在最后一行（`i=126, fx=1`），相当于把最后一格外延，可接受。

### 9.5 命令字换算

```c
float alpha = (float)sign_x * ((float)abs_x / (2.0f * GALVO_F)
                               + lut_sample(galvo_lut_alpha, abs_x, abs_y) * GALVO_LUT_UNIT);
float beta  = (float)sign_y * ((float)abs_y / (2.0f * GALVO_F)
                               + lut_sample(galvo_lut_beta,  abs_x, abs_y) * GALVO_LUT_UNIT);

int32_t cx = lroundf((alpha - GALVO_ALPHA0) * GALVO_COUNTS_PER_RAD) + GALVO_CMD_CENTER;
int32_t cy = lroundf((beta  - GALVO_BETA0 ) * GALVO_COUNTS_PER_RAD) + GALVO_CMD_CENTER;

cx = clamp(cx, 0, 65535);
cy = clamp(cy, 0, 65535);
```

- `GALVO_CMD_CENTER` 固定 32768；
- F407 带单精度 FPU，工程已使用 hard-float，主循环里用浮点没有性能问题；
- 若后续要固定点优化，可把 `1/(2f)`、`K`、LUT 单位统一折算成「命令数 / 坐标 count」，但当前没有必要。

### 9.6 标定流程

1. 表全 0 时先验证链路：线性的坐标→角度→命令字映射本身就可以用逻辑分析仪逐点核对，不需要额外的直通模式；
2. 在视场里打网格点，测量实际光斑位置；
3. 反算每个网格点的角度误差，写入 `galvo_lut_*.h`（由标定工具生成初始化列表）；
4. 实测 `α0 / β0 / K / f`，更新宏；
5. 重新编译烧录，复测。

表值全 0 时，系统输出严格的线性映射，可用于对照。

---

## 10. 中断、并发与优先级

| 中断 | 优先级 | 职责 |
|---|---|---|
| DMA2_Stream5 | 0（最高） | 每帧边界把 `stage` 拷到空闲半区 |
| USART1 | 1 | 逐字节解析，更新最新点 |
| SysTick | 15 | HAL 时基 |

- DMA 中断可抢占串口中断；拷贝只有 80 字节，不会造成串口丢字节（115200 下字节间隔 87 µs）。
- 主循环与中断之间只用 `volatile` 变量和「写另一槽 + 置标志」的双槽交接，不需要关中断。
- **关中断不会停止 DMA 和 TIM1**，所以即使将来加入短临界区，也不影响波形。

---

## 11. 关键参数表

```c
/* 时序 */
#define XY_BEATS              20U
#define XY_FRAME_PERIOD_US    10U
#define XY_FRAME_RATE_HZ      100000U

/* LUT */
#define GALVO_LUT_SIZE        128
#define GALVO_LUT_UNIT        1e-6f

/* 标定 */
/* 默认 0.35 rad 满量程：未标定时坐标线性映射到命令字，不会饱和 */
#define GALVO_FULL_SCALE_RAD  0.35f    /* 待标定 */
#define GALVO_F               (32767.0f / (2.0f * GALVO_FULL_SCALE_RAD))
#define GALVO_COUNTS_PER_RAD  (32767.0f / GALVO_FULL_SCALE_RAD)
#define GALVO_CMD_CENTER      32768.0f
#define GALVO_ALPHA0          0.0f     /* 待标定 */
#define GALVO_BETA0           0.0f     /* 待标定 */
```

---

## 12. 资源预算

| 项 | 大小 | 位置 |
|---|---|---|
| LUT（128×128×2，int16） | 64 KB | Flash（`.rodata`） |
| 代码 + HAL | 约 20–25 KB | Flash |
| `frame` + `stage` | 240 B | RAM |
| 最新点缓存（2 槽 × 4 B） | 8 B | RAM |
| HAL 句柄/栈等 | 约 2–3 KB | RAM |

F407VET6 资源：512 KB Flash / 192 KB RAM，余量充足。

---

## 13. 启动流程

1. `HAL_Init()`、`SystemClock_Config()`；
2. `MX_DMA_Init()`、`MX_GPIO_Init()`、`MX_TIM1_Init()`；
3. `Comm_Init()`：初始化当前传输（USART1）并启动接收；
4. `Galvo_Init()`：
   1. 用中心坐标（0,0）预填 `frame[0]` 和 `frame[1]`，**避免 DMA 启动后先发未初始化数据**；
   2. 注册 DMA HT/TC/Error 回调；
   3. `__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE)`；
   4. `HAL_DMA_Start_IT(&hdma_tim1_up, frame, &GPIOB->BSRR, 40)`；
   5. `__HAL_TIM_MOE_ENABLE(&htim1)`、`HAL_TIM_PWM_Start()`、`HAL_TIM_Base_Start()`；
5. `while (1) { Galvo_Task(); }`。

---

## 14. 测试与验收

### 14.1 波形检查

- PE9：2.000 MHz 方波，占空比 50%，**连续无缺口**；
- 每 10 µs 一帧，每帧 20 个时钟周期；
- SYNC：连续 19 拍高 + 1 拍低；
- X/Y 在 CLK 上升沿后更新，下降沿前稳定 ≥250 ns。

### 14.2 协议解码

用逻辑分析仪按 `SYNC / CLK / X / Y` 解码：

- 帧头应为 `001`；
- 16 bit 数据为命令字（MSB first）；
- 第 20 拍为偶校验，且 SYNC 为低。

### 14.3 功能检查

| 用例 | 期望 |
|---|---|
| 发送 `(0,0)`（LUT 全 0） | 数据位 = `0x8000` |
| 发送 `(1000,-1000)` | X = `0x83E8`，Y = `0x7C18` |
| 连续发送圆轨迹 | 光斑连续画圆，无跳变 |
| 停止发送 | 保持最后坐标，波形不中断 |
| 发送错误校验帧 | 被丢弃，不产生动作 |
| 连续发送 1 分钟 | 无丢帧、无溢出（观察计数） |
| 拔掉串口 | 输出保持，CLK 不中断 |

### 14.4 调试计数

```c
volatile struct {
    uint32_t rx_ok;
    uint32_t rx_crc_err;
    uint32_t drop;
    uint32_t dma_error;
} galvo_dbg;
```

建议在调试器中观察这些计数；`drop` 长期为 0，`dma_error` 必须为 0。当前传输层的校验错误由 `Comm_GetCrcErrorCount()` 提供。

---

## 15. 约束与风险

| 项 | 说明 |
|---|---|
| 不得停止 DMA/TIM1 | 任何重配都会造成 ≥100 ns 的断拍 |
| 不得在别处写 BSRR | 会与 DMA 输出冲突 |
| LUT 放 Flash | 标定值需重新烧录；若需在线更新，需另做 Flash 写入或 RAM 副本 |
| 传输层可替换 | 上层只依赖 `comm.h`；新增介质时实现同一接口，勿在 `Galvo` 里直接调用 HAL 串口 API |
| 单端信号 | 若硬件使用差分驱动（RS-422），MCU 仍输出单端，由外部驱动芯片转换 |
| 上电顺序 | 先用中心帧预填缓冲，再启动 DMA |
| 浮点 | 必须保持 hard-float 编译选项（已验证） |
| 坐标单位 | 上位机下发的是控制卡坐标系下的 16 bit 值，不是物理角度 |

---

## 16. 后续扩展

- **XY2-100-E**：18 bit 数据、扩展命令通道，需要改帧长为 22 bit 并重新核算时序；
- **回读通道 STATUS**：可作为输入引脚 + 定时器捕获实现；
- **多轴 / Z 轴**：复用同一套编码与 DMA 结构，增加数据线；
- **更换传输介质**：新增 `comm_spi.c` / `comm_eth.c` 实现 `comm.h`，替换编译列表中的 `comm_uart.c`；
- **在线标定**：通过串口下发标定表，写入外部 Flash 或内部备份区。

---

## 附录 A：一帧编码示例（x 命令 = 0x8000）

```
20 bit 序列（发送顺序）:
0011 0000 0000 0000 0000
│││└────────┬────────┘└ parity = 0
│││         └─ 16 bit 数据 0x8000
││└─ C1 = 0
│└── C2 = 0
└─── C0 = 1

SYNC: 1111 1111 1111 1111 1110
      （前 19 拍高，第 20 拍低）
```

校验位 = 前 19 位异或 = 1（C0）^ 1（D15）= 0。

## 附录 B：串口发送参考（Python）

```python
import serial, struct, functools

ser = serial.Serial('COM3', 115200)

def send(x, y):
    f = bytes([0xAA, 0x55]) + struct.pack('<hh', x, y)
    f += bytes([functools.reduce(int.__xor__, f, 0)])
    ser.write(f)
```
