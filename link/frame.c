/**
 * @file    frame.c
 * @brief   协议实现：CRC16、组帧、拆帧状态机
 *
 * ── 拆帧状态机的六个状态 ──────────────────────────────────────────────
 *
 *   0 ──看到 0xAA──▶ 1 ──看到 0x55──▶ 2 ──收满8字节头──▶ 3
 *   ▲                 │                                  │ 收满 len 字节
 *   │                 └─不是 0x55：退回 0（或停在1）       ▼
 *   │                                                     4 ──收 CRC 低──▶ 5
 *   │                                                                     │ 收 CRC 高并校验
 *   └──────────────── 校验通过：回调 / 校验失败：丢弃 ◀──────────────────┘
 *
 * ── 为什么要状态机 ────────────────────────────────────────────────────
 *   帧是变长的，而且字节流可能从半截开始（上位机刚复位、上电杂波、丢过字节）。
 *   状态机的价值不在"正常情况"，而在**异常之后能自己爬回来**：
 *   找不到帧头就一直丢字节（成本极低），找到就重新同步。
 */
#include "frame.h"
#include <string.h>

/* 收帧缓冲：按"最大可能的帧"开静态数组，放在 .bss，不占任务栈。
 * 大小 = 8 字节头 + 最大载荷 224 + 2 字节 CRC。
 * ★不要改成局部数组：任务栈只有 4KB，放个 234 字节的局部数组很危险。 */
static uint8_t  s_rx[FRAME_HDR_LEN + FRAME_MAX_PAYLOAD + 2U];

static uint16_t s_idx;      /* 当前已收了多少字节 */
static uint16_t s_need;     /* 这一帧还差多少（从头里读出来的 len） */
static uint8_t  s_state;    /* 状态机当前状态 0~5 */
static uint32_t s_ok;       /* 成功帧计数 */
static uint32_t s_bad;      /* 失败帧计数 */

/**
 * CRC16-CCITT 逐位实现。
 * 为什么不用查表：这个接口每秒最多算几十次，逐位够快，代码短、易核对。
 * 算法：每字节先异或到高 8 位，然后左移 8 次，溢出位就异或多项式 0x1021。
 */
uint16_t Frame_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;     /* 初值固定 0xFFFF，两端必须一致 */
    uint16_t i;
    uint8_t  b;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0U; b < 8U; b++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * 组帧：把字段按顺序填进 out，最后算 CRC 追加到末尾。
 * 注意每个多字节字段都是"低字节在前"（小端），
 * 所以 seq/len/crc 都写成 (v & 0xFF) 再 ((v>>8) & 0xFF)。
 */
uint16_t Frame_Encode(uint8_t type, const uint8_t *payload, uint16_t len,
                      uint16_t seq, uint8_t *out)
{
    uint16_t crc;
    uint16_t total;

    /* 参数校验：输出缓冲非空、长度不超上限 */
    if ((out == 0) || (len > FRAME_MAX_PAYLOAD)) {
        return 0U;
    }

    /* ---- 固定 8 字节头 ---- */
    out[0] = FRAME_MAGIC0;
    out[1] = FRAME_MAGIC1;
    out[2] = FRAME_VERSION;
    out[3] = type;

    out[4] = (uint8_t)(seq & 0xFFU);            /* seq 低字节 */
    out[5] = (uint8_t)((seq >> 8) & 0xFFU);     /* seq 高字节 */

    out[6] = (uint8_t)(len & 0xFFU);            /* len 低字节 */
    out[7] = (uint8_t)((len >> 8) & 0xFFU);     /* len 高字节 */

    /* ---- 载荷 ---- */
    if (len > 0U) {
        memcpy(&out[FRAME_HDR_LEN], payload, len);
    }

    /* ---- CRC：从 ver（out[2]）开始，算 6 字节头剩余部分 + 载荷 ---- */
    crc = Frame_Crc16(&out[2], (uint16_t)(6U + len));

    out[FRAME_HDR_LEN + len]      = (uint8_t)(crc & 0xFFU);
    out[FRAME_HDR_LEN + len + 1U] = (uint8_t)((crc >> 8) & 0xFFU);

    total = (uint16_t)(FRAME_HDR_LEN + len + 2U);
    return total;
}

void Frame_Reset(void)
{
    s_idx = 0U;
    s_need = 0U;
    s_state = 0U;
    s_ok = 0U;
    s_bad = 0U;
}

/**
 * 每收到一个字节调一次。
 * 这个函数没有任何阻塞、没有内存分配，所以调用它的任务不会被卡住。
 */
void Frame_Feed(uint8_t b, frame_rx_cb_t cb)
{
    switch (s_state) {
    /* ---- 状态0：找第一个帧头 0xAA ---- */
    case 0U:
        if (b == FRAME_MAGIC0) {
            s_state = 1U;
        }
        /* 不是 0xAA 就丢掉，什么都不做。这是"最便宜的重同步" */
        break;

    /* ---- 状态1：找第二个帧头 0x55 ---- */
    case 1U:
        if (b == FRAME_MAGIC1) {
            s_rx[0] = FRAME_MAGIC0;
            s_rx[1] = FRAME_MAGIC1;
            s_idx = 2U;                 /* 已经有 2 个字节了 */
            s_state = 2U;
        } else {
            /* 来的又是 0xAA：说明可能是 "AA AA 55" 这种，停在状态1 再等一次
             * 否则退回状态0 重新找 */
            s_state = (b == FRAME_MAGIC0) ? 1U : 0U;
        }
        break;

    /* ---- 状态2：收固定头（还剩 6 字节：ver type seq2 len2） ---- */
    case 2U:
        s_rx[s_idx] = b;
        s_idx++;

        if (s_idx >= FRAME_HDR_LEN) {
            /* 头收齐了，现在才知道这一帧有多长（小端拼接） */
            uint16_t len = (uint16_t)((uint16_t)s_rx[6] | ((uint16_t)s_rx[7] << 8));

            /* ★安全边界：版本不对 或 长度超上限 → 整帧丢弃，重新找帧头。
             * 如果不校验 len，对端发个 60000，往下写就越界踩内存了，
             * 症状是"跑一会儿随机 HardFault"。 */
            if ((s_rx[2] != FRAME_VERSION) || (len > FRAME_MAX_PAYLOAD)) {
                s_state = 0U;
                s_bad++;
            } else {
                s_need = len;
                /* len==0 时没有载荷，必须跳过"收载荷"状态直接去收 CRC。
                 * 否则状态 3 会把 CRC 低字节当成载荷吃掉，整帧错位一格，
                 * 空载荷帧就永远校验失败、收不到 ACK。 */
                s_state = (len == 0U) ? 4U : 3U;
            }
        }
        break;

    /* ---- 状态3：收载荷 ---- */
    case 3U:
        s_rx[s_idx] = b;
        s_idx++;
        /* 收满 8 + len 字节，转去收 CRC */
        if (s_idx >= (uint16_t)(FRAME_HDR_LEN + s_need)) {
            s_state = 4U;
        }
        break;

    /* ---- 状态4：收 CRC 低字节 ---- */
    case 4U:
        s_rx[s_idx] = b;
        s_idx++;
        s_state = 5U;
        break;

    /* ---- 状态5：收 CRC 高字节，然后校验 ---- */
    case 5U: {
        uint16_t crc_recv;
        uint16_t crc_calc;

        s_rx[s_idx] = b;
        s_idx++;

        /* 刚才这两个字节（s_idx-2, s_idx-1）就是收到的 CRC，小端拼回来 */
        crc_recv = (uint16_t)((uint16_t)s_rx[s_idx - 2U] |
                              ((uint16_t)s_rx[s_idx - 1U] << 8));

        /* 自己按同样范围再算一遍 */
        crc_calc = Frame_Crc16(&s_rx[2], (uint16_t)(6U + s_need));

        if (crc_calc == crc_recv) {
            s_ok++;
            /* 校验通过：把 type 和载荷交给上层。
             * 注意 payload 指向内部缓冲，回调里要用完就处理掉，别存指针。 */
            if (cb != 0) {
                /* seq 来自帧头 s_rx[4..5]（小端），回传给上层，ACK 好原样带回 */
                cb(s_rx[3],
                   (uint16_t)((uint16_t)s_rx[4] | ((uint16_t)s_rx[5] << 8U)),
                   &s_rx[FRAME_HDR_LEN], s_need);
            }
        } else {
            /* CRC 错：整帧丢弃。
             * 只丢这一帧、不清空缓冲，因为后面可能紧跟着好帧。 */
            s_bad++;
        }

        /* 不管成功失败，都回到"找帧头"状态 */
        s_state = 0U;
        break;
    }

    default:
        s_state = 0U;
        break;
    }
}

uint32_t Frame_OkCount(void)  { return s_ok; }
uint32_t Frame_BadCount(void) { return s_bad; }
