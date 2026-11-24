/**
 * @file    uart1.h
 * @brief   USART1（上下位机通信口）的收发封装
 *
 * ── 这个模块存在的意义 ────────────────────────────────────────────────
 *   USART1 的中断每来 1 个字节就触发一次（115200 下大约每 86.8µs 一次），
 *   而我们的解析任务 20ms 才跑一次。两边节奏差了 200 多倍。
 *
 *   所以这层的唯一职责是：**把"随时会来的字节"安全地存起来，
 *   让任务能"按自己的节奏、按顺序"取走**。
 *   它完全不认识协议（不知道帧头、不知道 CRC），换协议也不用改它。
 *
 * ── 数据流 ────────────────────────────────────────────────────────────
 *   USART1 硬件 → HAL 中断 → Uart1_OnRxByteFromIsr() → 环形缓冲
 *                                                         ↓
 *                                          Uart1_Read() ← 任务里取
 */
#ifndef UART1_H
#define UART1_H

#include <stdint.h>

/**
 * @brief 初始化：启动"单字节中断接收"
 * @note  必须在 MX_USART1_UART_Init() 之后调用（任务里调正好满足）。
 *        本工程 CubeMX 已经打开了 USART1 的 NVIC，所以这里不用再开中断。
 */
void Uart1_Init(void);

/**
 * @brief 中断服务用：收下这 1 个字节，并重新武装下一次接收
 * @note  只能在 HAL_UART_RxCpltCallback 里调用（中断上下文）。
 *        ★不要漏掉重新武装，HAL 的 IT 模式是"一次性"的。
 */
void Uart1_OnRxByteFromIsr(void);

/**
 * @brief 从缓冲里取字节
 * @param buf  取到哪里
 * @param len  最多取几个
 * @return     实际取到的个数（0 表示当前没数据）
 * @note  任务上下文调用。返回值 < len 不代表出错，只是"现在只有这么多"。
 */
int  Uart1_Read(uint8_t *buf, int len);

/**
 * @brief 发送一段字节（阻塞，等发完才返回）
 * @note  一帧 26 字节在 115200 下约 2.3ms。
 *        现在只有一个任务在发，阻塞没问题；将来如果多个任务都要发，
 *        必须加互斥或改成发送队列，否则两帧字节会交叉。
 */
void Uart1_Write(const uint8_t *buf, int len);

/**
 * @brief 接收出错后的恢复：清 ORE/RXNE 残留标志 + 重新武装接收
 * @note  只能在 HAL_UART_ErrorCallback 里调用（中断上下文）。
 *        HAL 遇到 ORE 会调 UART_EndRxTransfer() 把接收彻底关掉、
 *        并且不重新武装，所以必须在这里手动恢复，否则接收会永久停摆。
 */
void Uart1_OnRxError(void);

/**
 * @brief 累计收到过多少字节（只增不减）
 * @note  调试用：这个数在涨 = 硬件通了；不涨 = 线/波特率/中断有问题。
 *        它是排查"到底收没收到"的第一道分诊指标。
 */
uint32_t Uart1_RxTotal(void);

/**
 * @brief 累计接收错误次数（ORE/FE/NE）
 * @note  调试用：如果它一直不涨而 Uart1_RxTotal 也不涨，
 *        说明线上真的没信号；如果它在涨，是线上有信号但被错误吃掉了。
 */
uint32_t Uart1_ErrTotal(void);

#endif /* UART1_H */
