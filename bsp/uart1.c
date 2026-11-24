/**
 * @file    uart1.c
 * @brief   USART1 环形缓冲收发实现（单生产者 / 单消费者，无需加锁）
 *
 * ── 环形缓冲是怎么工作的 ──────────────────────────────────────────────
 *   把 s_buf 看成一个首尾相接的圈：
 *
 *        tail(读)            head(写)
 *          ↓                   ↓
 *      ┌───┬───┬───┬───┬───┬───┬───┬───┐
 *      │   │ D │ A │ T │ A │   │   │   │
 *      └───┴───┴───┴───┴───┴───┴───┴───┘
 *           已经有数据的区间
 *
 *   - 中断只动 head（往里放）
 *   - 任务只动 tail（往外取）
 *   - 两者互不写对方的变量，所以**不需要关中断、不需要互斥量**
 *
 * ── 为什么"留一个空位" ────────────────────────────────────────────────
 *   判断"满"最省事的办法是让 head 永远追不上 tail：
 *   当 (head+1)%SIZE == tail 就认为满了。代价是浪费 1 个字节容量，
 *   换来每次判断只要一条比较指令（在中断里很值）。
 */
#include "uart1.h"
#include "usart.h"      /* 为了拿到 huart1 这个句柄 */

/* 缓冲大小：必须是 2 的幂。
 * 115200 下一字节约 86.8µs，20ms 任务周期内最多来 230 字节，
 * 512 给了 2 倍余量（应付任务偶尔被拖慢）。 */
#define RX_SIZE   512U

static uint8_t  s_buf[RX_SIZE];

/* head：中断里写，任务里读；tail：任务里写，中断里读。
 * 两者都是 16 位对齐，Cortex-M4 上读写是原子的，所以不用加临界区。 */
static uint16_t s_head;
static uint16_t s_tail;

/* HAL 单字节接收的落点。
 * ★必须是 static：HAL 会记住这个地址，中断来了往这里写。
 *   如果写成局部变量，函数一返回栈就被人覆盖，HAL 还在往那块内存写，
 *   症状是"偶尔丢字节/偶尔乱码"，极难查。 */
static uint8_t  s_byte;

/* volatile：这个值被中断修改、被任务读取。
 * 不加的话编译器可能把它缓存在寄存器里，你永远看到旧值。 */
static volatile uint32_t s_rx_total;

/* 接收错误次数（ORE/FE/NE）。ORE 是一类"悄无声息把接收停掉"的错误，
 * 必须计数，否则你在任务里看到的永远只是"没数据"。 */
static volatile uint32_t s_err_total;

/* 清掉 RXNE / ORE 残留标志。
 * ★顺序不能反：必须"先读 SR，再读 DR"，这一对读操作才清得掉 ORE。
 *   只读 DR 或只读 SR 都清不干净，症状是"刚重新武装就立刻又报溢出"。 */
static void rx_flags_clear(void)
{
    volatile uint32_t tmp;
    tmp = huart1.Instance->SR;
    tmp = huart1.Instance->DR;
    (void)tmp;
}

/* 武装一次"收 1 字节进中断"。已经武装过时 HAL 返回 HAL_BUSY，无副作用。 */
static void rx_arm(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &s_byte, 1U);
}

void Uart1_OnRxError(void)
{
    s_err_total++;

    /* 关键顺序：先清 ORE，再重新武装。
     * 只重新武装不清标志的话，下一次中断进来第一件事还是报 ORE，
     * HAL 又会调 UART_EndRxTransfer 把接收关掉，等于白武装。 */
    rx_flags_clear();
    rx_arm();
}

void Uart1_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_rx_total = 0U;
    s_err_total = 0U;

    /* 本工程 CubeMX 已经在 USART1 的 MspInit 里调过
     *   HAL_NVIC_SetPriority(USART1_IRQn, ...);
     *   HAL_NVIC_EnableIRQ(USART1_IRQn);
     * 所以这里不用再开。
     * 如果哪天重新生成 CubeMX 把 NVIC 的勾去掉了，症状是"一个字节都收不到"，
     * 那时把上面两行补回来即可。 */

    /* 启动前先把上电/开串口时残留的 RXNE、ORE 清掉。
     * ★不做这一步的经典故障：串口助手在板子复位前就已经打开，
     *   线上早来了一两个字节（RXNE=1，再来一个就 ORE=1）。
     *   我们这边一武装中断，HAL 立刻按"溢出错误"处理，
     *   UART_EndRxTransfer() 把 RXNEIE/EIE 全关掉，接收从此永久停摆，
     *   而 TX 完全正常 —— 就是你现在的现象：心跳照发，数据一个字节都收不到。
     *   驱动里 ErrorCallback 是空的，所以这个过程一点提示都没有。 */
    rx_flags_clear();
    rx_arm();
}

void Uart1_OnRxByteFromIsr(void)
{
    /* ★中断里绝对不能调 HAL_UART_Transmit★
     * 它是阻塞发送：要等 TXE/TC 标志，超时靠 HAL_GetTick()。
     * 中断上下文里如果时基中断优先级更低，GetTick 不再前进，
     * 超时就永远不到 → 死在这个中断里，任务再也不跑。
     * 实测现象：一收到字节，心跳就停了，板子像"死"了。
     * 要做回显/调试输出，放在任务里发，不能放中断。 */

    /* 先算出"写进去之后 head 会在哪" */
    uint16_t next = (uint16_t)((s_head + 1U) % RX_SIZE);

    /* next == tail 说明再写就追上读指针了 → 缓冲满。
     * 满了的策略：丢掉这个新字节，保住已有的数据。
     * 反过来"丢旧数据"会破坏正在解析的半帧，更难恢复。 */
    if (next != s_tail) {
        s_buf[s_head] = s_byte;
        s_head = next;
    }

    s_rx_total++;

    /* ★关键：HAL 的 IT 模式是"一次性"的，
     *   收满 1 个字节、回调完就停了。不重新调用，第二个字节开始全丢。
     *   现象是"只收到第一字节，后面没了"。 */
    rx_arm();
}

int Uart1_Read(uint8_t *buf, int len)
{
    int n = 0;

    if ((buf == 0) || (len <= 0)) {
        return 0;
    }

    /* tail 追 head，追到为止 */
    while ((n < len) && (s_tail != s_head)) {
        buf[n] = s_buf[s_tail];
        n++;

        /* 加一后回绕。RX_SIZE 是 2 的幂，编译器会把 % 优化成 & (RX_SIZE-1) */
        s_tail = (uint16_t)((s_tail + 1U) % RX_SIZE);
    }
    return n;
}

void Uart1_Write(const uint8_t *buf, int len)
{
    if ((buf == 0) || (len <= 0)) {
        return;
    }
    /* 阻塞发送，超时 100ms（26 字节在 115200 下只要 2.3ms，100ms 足够宽松）。
     * 最后那个 (void) 表示"我不关心返回值"，避免编译告警。 */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100U);
}

uint32_t Uart1_RxTotal(void)
{
    return s_rx_total;
}

uint32_t Uart1_ErrTotal(void)
{
    return s_err_total;
}
