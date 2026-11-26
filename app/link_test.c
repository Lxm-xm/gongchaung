/**
 * @file    link_test.c
 * @brief   USART1 通信最小闭环：收帧 → 校验 → 回 ACK + 500ms 心跳
 *
 * ── 一次完整的交互长这样 ──────────────────────────────────────────────
 *
 *   上位机                          下位机（本文件）
 *   ──────                          ────────────────
 *   发 AA 55 01 14 00 00 00 00 EC C6
 *        │
 *        └── 字节流 ──▶ Uart1 缓冲 ──▶ Frame_Feed 逐字节拆
 *                                        │ CRC 通过
 *                                        ▼
 *                                   on_frame(type=0x14)
 *                                        │
 *   收 AA 55 01 82 xx xx 02 00 14 00 yy yy
 *        ◀── send_frame(MSG_ACK, {0x14, 0x00}) ┘
 *
 *
 * ── 心跳的两层含义 ────────────────────────────────────────────────────
 *   车 → Pi：每 500ms 发 EVENT(evt=0)，证明"车还活着"
 *   Pi → 车：Pi 也要发，否则车会认为链路断了（断链策略：300ms 告警 / 1500ms 停车）
 *   只做单向是常见错误。
 */
#include "link_test.h"
#include "uart1.h"
#include "frame.h"
#include "nav.h"          /* NavPoint_t / Nav_PushStage / Nav_Count */
#include "main.h"          /* HAL_GetTick()：毫秒计数 */
#include <string.h>        /* memcpy */
#include "cmsis_os.h"

/* 心跳周期。500ms 配合断链阈值：连丢 3 个心跳（1.5s）才判断开。
 * 太快浪费带宽，太慢则车跑偏了你才知道。 */
#define HB_MS   4000U

/* 发送序号：每发一帧 +1，用来配对和排查丢帧。
 * 16 位，回绕一次要 65536 帧，比赛期间不会回绕。 */
static uint16_t s_seq;

/* 上一次发心跳的时刻（毫秒） */
static uint32_t s_last_hb;

/*0x13 变量*/
typedef enum {
    MISSION_IDLE = 0, /* 空闲\未启动 */
    MISSION_RUN = 1, /* 启动 */
    MISSION_HOLD = 2, /* 暂停 */
    MISSION_DONE = 3, /* 完成 */
    MISSION_FAULT = 4 /* 故障 */
}mission_state_t;

static mission_state_t s_mission = MISSION_IDLE; /* 当前任务状态 */

/*0x11 变量*/
static uint16_t s_last_stage = 0xFFFFU; /* 还没有收到过0x11 */
static uint8_t  s_last_wp    = 0U;

/**
 * 处理 0x11 段任务表（解析段任务，把航点入队）
 * @return 0 成功 / 1 队列满 / 2 载荷不对
 */
static uint8_t handle_stage_plan(const uint8_t *pl, uint16_t len)
{
    frame_stage_hdr_t h;
    NavPoint_t pts[8];                 /* 单段最多 8 个点，够用 */
    uint16_t n;
    uint16_t i;

    if ((pl == 0) || (len < sizeof(frame_stage_hdr_t))) {
        return 2U;                     /* 长度至少要有 12 字节头 */
    }
    memcpy(&h, pl, sizeof(h));         /* ★用 memcpy，不要强转指针 */

    if (h.stage >= NAV_MAX_STAGE)  return 2U;
    if (h.wp_count > 8U)           return 2U;

    /* 实际航点数 = (总长 - 头) / 每个航点大小；必须和头里声明的 wp_count 一致 */
    n = (uint16_t)((len - sizeof(h)) / sizeof(frame_wp_t));
    if (n != (uint16_t)h.wp_count) return 2U;

    /* 逐个把航点拷出来、转 float、填进 pts[]*/
    for (i = 0U; i < n; i++) {
        frame_wp_t w;
        memcpy(&w, pl + sizeof(h) + i * sizeof(frame_wp_t), sizeof(w));
        pts[i].x_mm    = (float)w.x_mm;
        pts[i].y_mm    = (float)w.y_mm;
        pts[i].yaw_deg = (float)w.yaw_ddeg / 10.0f;
        pts[i].stage   = h.stage;
        pts[i].flags   = w.flags;
    }

    if ((n > 0U) && (Nav_PushStage(h.stage, pts, (uint8_t)n) != 0U)) {
        return 1U;                     /* 队列满 */
    }

    s_last_stage = h.stage;
    s_last_wp    = (uint8_t)n;
    return 0U;
}

/**
 * 处理 0x13 任务命令（开始/暂停/继续/中止/清空）
 * @return 0 成功；2 载荷不对；3 未知命令   ← 这个值会写进 ACK 的 result 字节
 */
static uint8_t handle_mission_cmd(const uint8_t *pl, uint16_t len)
{
    if ((pl == 0) || (len < 1U)) {
        return 2U;                        /* 命令字都没带 */
    }

    switch (pl[0]) {
    case 1U:                              /* START */
        s_mission = MISSION_RUN;
        return 0U;

    case 2U:                              /* PAUSE */
        s_mission = MISSION_HOLD;
        return 0U;

    case 3U:                              /* RESUME */
        s_mission = MISSION_RUN;
        return 0U;

    case 4U:                              /* ABORT */
    case 5U:                              /* CLEAR */
        s_mission = MISSION_IDLE;
        return 0U;

    default:
        return 3U;                        /* 未知命令：状态不动 */
    }
}

/**
 * 组帧并发送一条命令。
 * 注意：这个函数是"发一帧"，不负责重发，也不等对方 ACK。
 */
static void send_frame(uint8_t type, const uint8_t *pl, uint16_t len)
{
    /* 栈上开缓冲：最大帧 = 8 头 + 224 载荷 + 2 CRC = 234 字节。
     * 任务栈有 4KB，放这个是安全的。 */
    uint8_t  buf[FRAME_HDR_LEN + FRAME_MAX_PAYLOAD + 2U];

    /* Frame_Encode 会填好所有字段并算 CRC，返回总长度 */
    uint16_t n = Frame_Encode(type, pl, len, s_seq, buf);

    if (n > 0U) {
        Uart1_Write(buf, (int)n);   /* 阻塞发送，约 0.7ms（14 字节） */
        s_seq++;                    /* 序号只在真正发出去后才递增 */
    }
}

/**
 * 收到一帧"CRC 校验通过"的数据后，由 Frame_Feed 回调进来。
 * 现在只做一件事：把收到的命令字原样回一个 ACK。
 */
static void on_frame(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint8_t result = 0U;   /* 0=成功，非0留给以后表示失败原因 */
       
    switch (type) {
    case MSG_MISSION_CMD:
        result = handle_mission_cmd(payload, len);
        break;
        
        case MSG_HEARTBEAT:
        result = 0U; /* 任务码不做处理，直接回 ACK */
        break;
        
        case MSG_STAGE_PLAN:
        if (len < sizeof(frame_stage_hdr_t)) {
        result = 2U;
        } else {
        result = handle_stage_plan(payload, len);
        }
        break;
        
        default:
        /* 0x10 / 0x11 / 0x12 第 2 步再处理，现在先照旧回 ACK */
        result = 0U;
        break;
    }
      {
    uint8_t ack[2];
    ack[0] = type;      /* 把命令字原样带回：上位机据此知道"哪条命令被确认了" */
    ack[1] = result;    /* 结果码：0 = 成功，非 0 留给以后表示失败原因 */
    send_frame(MSG_ACK, ack, 2U);
      }
}

void LinkTest_Init(void)
{
    s_seq = 0U;
    s_last_hb = 0U;
    Frame_Reset();      /* 清拆帧状态机与统计 */
    Uart1_Init();       /* 启动 USART1 单字节中断接收 */
}

void LinkTest_Step(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  b;

    /* 心跳载荷 5 字节（【调试期】临时改成带计数，方便在串口助手里直接看接收状态）：
     *   hb[0] = evt = 0        → 心跳标志，和原来一致
     *   hb[1] = 收到的原始字节数（Uart1_RxTotal 的低字节，256 回绕）
     *   hb[2] = 校验通过的帧数（Frame_OkCount 的低字节）
     *   hb[3] = 校验失败的帧数（Frame_BadCount 的低字节）
     *   hb[4] = 接收错误次数（Uart1_ErrTotal 的低字节，ORE/FE/NE）
     * 上位机侧如果是按固定 4 字节解析的，注意这里长度变成了 5，帧里的 len 字段会跟着变。
     * 定位完把 hb[1..4] 恢复成 0、长度改回 4 即可。 */
    uint8_t  hb[9];   

    /* ── 1) 把环形缓冲里积压的字节全部取出来喂给拆帧器 ──
     * 一次取 1 个字节是为了复用 Frame_Feed 的逐字节接口，
     * 简单可靠；字节数不多（20ms 内最多 230 个），这点开销可以接受。 */
    while (Uart1_Read(&b, 1) == 1) {
        Frame_Feed(b, on_frame);
    }

    /* ── 2) 到点就发心跳 ──
     * 用"当前时间 - 上次时间 >= 周期"而不是定时器，
     * 这样即使任务偶尔被拖慢，也不会累积漂移。 */
    if ((now - s_last_hb) >= HB_MS) {
        s_last_hb = now;

        hb[0] = 0U;                            /* evt=0：心跳 */
        hb[1] = (uint8_t)Uart1_RxTotal();      /* 收到多少原始字节 */
        hb[2] = (uint8_t)Frame_OkCount();      /* 多少帧 CRC 通过 */
        hb[3] = (uint8_t)Frame_BadCount();     /* 多少帧帧头/版本/长度/CRC 出错 */
        hb[4] = (uint8_t)Uart1_ErrTotal();     /* 多少字节被 ORE/FE/NE 吃掉 */
        hb[5] = (uint8_t)s_mission;          /* ← 新增：任务状态 */
        hb[6] = (uint8_t)Nav_Count();            /* 队列里有几个点 */
        hb[7] = (s_last_stage > 0xFFU) ? 0xFFU : (uint8_t)s_last_stage;  /* 最近段号 */
        hb[8] = s_last_wp;                       /* 最近一帧的航点数 */
        send_frame(MSG_EVENT, hb, 9U);   /* evt=0 即心跳 */
    }
}
