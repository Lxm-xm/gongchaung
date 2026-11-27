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
 *   收 AA 55 01 82 xx xx 04 00 ss ss 14 00 yy yy
 *        ◀── send_frame(MSG_ACK, {seq, 0x14, 0x00}) ┘
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

/* 心跳周期。正式值 500ms：配合断链阈值，连丢 3 个心跳（1.5s）才判断开。
 * 太快浪费带宽，太慢则上位机要很久才发现车已经死了。
 * ★调试期临时放大到 4000，方便在串口助手里看清 ACK；比赛前必须改回 500。 */
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

/* 当前执行的段号：收到 0x11 时更新，将来 STAGE_DONE 时 +1 */
static uint8_t  s_cur_stage;

/* 故障码，位定义见 frame.h 的 FAULT_xxx */
static uint16_t s_fault;

/* ── 断链检测 ──
 * 只在上位机“应该一直在”的时候检查（也就是车真的在跑的时候），
 * 停在台架上调试时几秒不发命令是很正常的，不能因此报故障。
 * 阈值：> 300ms 告警（TODO：等有屏幕后闪灯），> 1500ms 直接停车并置故障。 */
static uint32_t s_last_pi_ms;    /* 最近一次收到上位机任何帧的时刻 */

/* ── 停稳（STABLE）上报 ──
 * Pi 收到 STABLE 才做 AMCL 纠偏，所以这个事件必须由“车确实停了”触发。
 * 现在底盘还没做，先用 s_need_stable 手动置位（暂停时会置），
 * 等 chassis.c 做好，把这段的判定换成 Chassis_IsStill() 即可。 */
static uint8_t  s_need_stable;   /* 1 = 需要报一次停稳 */
static uint8_t  s_stable_sent;   /* 1 = 这一轮已经报过了 */
static uint8_t  s_still_armed;   /* 1 = 停稳计时已经起跑 */
static uint32_t s_still_since;   /* 开始静止的时刻 */

/* 调试开关：验收 ACK/队列时先关掉 POSE，免得 20Hz 刷屏看不见别的帧 */
#define POSE_ENABLE   0
static uint32_t s_last_pose_ms;
static uint32_t s_pose_cnt;          /* 累计发了几帧，调试用 */

/* 前向声明：send_frame 定义在文件下方，send_pose 要先用到它 */
static void send_frame(uint8_t type, const uint8_t *pl, uint16_t len);

/* 0x80位姿上报,20Hz */
#if (POSE_ENABLE != 0)
static void send_pose(void)
{
    frame_pose_t p;

    /* ── 假数据 ──
     * 等 CAN 和陀螺仪接上，把下面几行换成真实值：
     *   p.x_mm / p.y_mm / p.yaw_ddeg  ← 里程计 + 陀螺仪
     *   p.vx_mmps / p.vy_mmps         ← 底盘
     *   p.enc_mask / p.imu_ok         ← 在线状态
     */
    p.x_mm      = 0;
    p.y_mm      = 0;
    p.yaw_ddeg  = 0;
    p.vx_mmps   = 0;
    p.vy_mmps   = 0;
    p.state     = (uint8_t)s_mission;   /* ← 接第 1 步做的状态机 */
    p.stage     = s_cur_stage;          /* ← 当前第几段（收到 0x11 时更新） */
    p.enc_mask  = 0;                    /* 还没有编码器 */
    p.imu_ok    = 0;                    /* 还没有陀螺仪 */
    p.fault     = s_fault;              /* 断链 / 电机掉线 / 打滑等 */

    send_frame(MSG_POSE, (const uint8_t *)&p, (uint16_t)sizeof(p));
    s_pose_cnt++;
}
#endif  /* POSE_ENABLE */

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

    /* ★先确认"头之后正好是整数个航点"。
     * 少了这一步：如果 len = 12 + 8n + 3（尾巴带垃圾），
     * 下面的除法会向下取整成 n，而它可能恰好等于 wp_count，
     * 于是“带垃圾尾巴的帧”也会被当成合法帧接受。 */
    if (((len - sizeof(frame_stage_hdr_t)) % sizeof(frame_wp_t)) != 0U) {
        return 2U;
    }

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

    /* ★用 Nav_ReplaceStage 而不是 Nav_PushStage：上位机重发同一段时，
     * 先把该段旧点摘掉再加新点，否则会重复入队、车会重复跑。 */
    if (Nav_ReplaceStage(h.stage, pts, (uint8_t)n) != 0U) {
        return 1U;                     /* 队列满 */
    }

    s_last_stage  = h.stage;
    s_last_wp     = (uint8_t)n;
    s_cur_stage   = h.stage;
    s_need_stable = 0U;      /* 来了新航点，车马上要走，不再需要报“停稳” */
    return 0U;
}

/**
 * 处理 0x12 航点补丁（追加 / 替换当前段剩余）
 * @return 0 成功 / 1 队列满 / 2 载荷不对 / 3 未知 mode
 */
static uint8_t handle_waypoint_patch(const uint8_t *pl, uint16_t len)
{
    frame_patch_hdr_t h;
    NavPoint_t pts[8];
    uint16_t   n;
    uint16_t   i;

    if ((pl == 0) || (len < sizeof(frame_patch_hdr_t))) {
        return 2U;
    }
    memcpy(&h, pl, sizeof(h));

    if (h.stage >= NAV_MAX_STAGE) return 2U;
    if (h.wp_count > 8U)          return 2U;

    /* 和 0x11 一样：头之后必须是整数个航点 */
    if (((len - sizeof(frame_patch_hdr_t)) % sizeof(frame_wp_t)) != 0U) {
        return 2U;
    }
    n = (uint16_t)((len - sizeof(h)) / sizeof(frame_wp_t));
    if (n != (uint16_t)h.wp_count) return 2U;

    for (i = 0U; i < n; i++) {
        frame_wp_t w;
        memcpy(&w, pl + sizeof(h) + i * sizeof(frame_wp_t), sizeof(w));
        pts[i].x_mm    = (float)w.x_mm;
        pts[i].y_mm    = (float)w.y_mm;
        pts[i].yaw_deg = (float)w.yaw_ddeg / 10.0f;
        pts[i].stage   = h.stage;
        pts[i].flags   = w.flags;
    }

    if (h.mode == WP_MODE_REPLACE) {
        /* 替换：AMCL 纠偏后重规划，把这一段的剩余点整体换掉 */
        if (Nav_ReplaceStage(h.stage, pts, (uint8_t)n) != 0U) return 1U;
    } else if (h.mode == WP_MODE_APPEND) {
        /* 追加：绕障，在队尾补点，原有进度不动 */
        if ((n > 0U) && (Nav_PushStage(h.stage, pts, (uint8_t)n) != 0U)) return 1U;
    } else {
        return 3U;                      /* 未知 mode */
    }

    s_cur_stage   = h.stage;
    s_need_stable = 0U;
    return 0U;
}

/**
 * 处理 0x15 视觉对位偏移
 * @return 0 成功 / 2 载荷不对
 *
 * 现在只做“校验 + 应答”：底盘和抓取还没做，收到偏移也无处可用。
 * 等 chassis.c 有了横向移动能力，这里改成：
 *     Chassis_MoveBy(v.dx_mm, v.dy_mm);   // 阻塞或非阻塞都行
 *     动作完成后 send_event(EVT_VISION_DONE, s_cur_stage, 0);
 */
static uint8_t handle_vision_adjust(const uint8_t *pl, uint16_t len)
{
    frame_vision_t v;

    if ((pl == 0) || (len < sizeof(frame_vision_t))) {
        return 2U;
    }
    memcpy(&v, pl, sizeof(v));

    /* 简单校验：偏移量不该是天文数字（±32000mm 超出场地范围，说明帧错位） */
    if ((v.dx_mm > 32000) || (v.dx_mm < -32000)) return 2U;
    if ((v.dy_mm > 32000) || (v.dy_mm < -32000)) return 2U;

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
    case 1U:                              /* START：开始新一轮 */
        if (s_mission != MISSION_IDLE) {
            return 1U;                    /* 已经在跑/暂停/故障：拒绝重复启动 */
        }
        Nav_Clear();                      /* ★必须清：上一轮的残留航点不清掉，
                                           *   第二轮会从"上一轮剩下的点"开始走 */
        s_cur_stage   = 0U;
        s_fault       = 0U;
        s_need_stable = 0U;
        s_mission     = MISSION_RUN;
        return 0U;

    case 2U:                              /* PAUSE：暂停 */
        if (s_mission != MISSION_RUN) {
            return 1U;                    /* 没在跑，谈何暂停 */
        }
        s_mission     = MISSION_HOLD;
        s_need_stable = 1U;               /* 停下后要主动告诉 Pi“我停稳了” */
        s_stable_sent = 0U;
        s_still_armed = 0U;
        return 0U;

    case 3U:                              /* RESUME：从暂停处继续，什么都不清 */
        if (s_mission != MISSION_HOLD) {
            return 1U;                    /* 没在暂停，谈何继续 */
        }
        s_mission     = MISSION_RUN;
        s_need_stable = 0U;
        return 0U;

    case 4U:                              /* ABORT：中止 */
    case 5U:                              /* CLEAR：清空 */
        s_mission     = MISSION_IDLE;
        s_fault       = 0U;
        s_need_stable = 0U;
        Nav_Clear();
        return 0U;

    case 6U:                              /* HOLD：原地别动（AMCL 算不出位姿时用） */
        s_mission     = MISSION_HOLD;
        s_need_stable = 1U;
        s_stable_sent = 0U;
        s_still_armed = 0U;
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
static void on_frame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t result = 0U;   /* 0=成功，非0表示失败原因（见 ACK 的 result 码） */

    /* 只要收到任何一帧（包括心跳），就说明上位机还活着 */
    s_last_pi_ms = HAL_GetTick();

    switch (type) {
    case MSG_MISSION_CMD:
        result = handle_mission_cmd(payload, len);
        break;

    case MSG_TASK_CODE:
        /* 任务码只用于屏显：必须正好 20 字节。
         * 长度不对就回 2，别让"发错的帧"被当成成功。 */
        result = (len == 20U) ? 0U : 2U;
        break;

    case MSG_STAGE_PLAN:
        if (len < sizeof(frame_stage_hdr_t)) {
            result = 2U;
        } else {
            result = handle_stage_plan(payload, len);
        }
        break;

    case MSG_WAYPOINT_ADD:
        if (len < sizeof(frame_patch_hdr_t)) {
            result = 2U;
        } else {
            result = handle_waypoint_patch(payload, len);
        }
        break;

    case MSG_VISION_ADJUST:
        if (len < sizeof(frame_vision_t)) {
            result = 2U;
        } else {
            result = handle_vision_adjust(payload, len);
        }
        break;

    case MSG_HEARTBEAT:
        result = 0U;                   /* 上位机心跳：不用做事，回 ACK 即可 */
        break;

    default:
        result = 3U;                   /* ★未知命令要如实回 3，不能一律回 0 */
        break;
    }

    {
        uint8_t ack[4];
        ack[0] = (uint8_t)(seq & 0xFFU);         /* 回传命令 seq 低字节 */
        ack[1] = (uint8_t)((seq >> 8) & 0xFFU);  /* 回传命令 seq 高字节 */
        ack[2] = type;                            /* 命令字原样带回 */
        ack[3] = result;                          /* 结果码：0 = 成功 */
        send_frame(MSG_ACK, ack, 4U);
    }
}

/**
 * @brief 初始化：重置拆帧状态机 + 启动 USART1 接收
 * @note  在任务里、MX_USART1_UART_Init() 之后调用一次。
 */
void LinkTest_Init(void)
{
    s_seq = 0U;
    s_last_hb = 0U;
    s_last_pose_ms = 0U;
    s_pose_cnt = 0U;

    s_cur_stage   = 0U;
    s_fault       = 0U;
    s_last_pi_ms  = HAL_GetTick();   /* 从"现在"开始计时，避免上电就判断链 */

    s_need_stable = 0U;
    s_stable_sent = 0U;
    s_still_armed = 0U;
    s_still_since = 0U;

    Frame_Reset();      /* 清拆帧状态机与统计 */
    Nav_Init();         /* 清航点队列 */
    Uart1_Init();       /* 启动 USART1 单字节中断接收 */
}

/**
 * @brief 主循环体：收帧 + 定时发心跳
 * @note  每 20ms 调一次。
 *        20ms 是权衡：115200 下一周期最多积压 230 字节（缓冲 512 够用），
 *        而每秒只唤醒 50 次，CPU 开销可忽略。
 */
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

    /* ① 收帧 */
    while (Uart1_Read(&b, 1) == 1) {
        Frame_Feed(b, on_frame);
    }

    /* ② 位姿 20Hz：每 50ms 一帧（和心跳完全不同频，不能放进同一个 if） */
#if (POSE_ENABLE != 0)
    if ((now - s_last_pose_ms) >= 50U) {
        s_last_pose_ms = now;
        send_pose();
    }
#endif

    /* ③ 停稳上报（新方案的关键：Pi 收到 STABLE 才做 AMCL 纠偏）
     * 现在的"停稳"判定是简化的：底盘还没做，先认为"一旦被要求停，
     * 100ms 后就一定停得住"。等 chassis.c 做好，把下面这句话换成
     *     if (s_need_stable && (s_stable_sent == 0U) && Chassis_IsStill())
     * 其余逻辑不用改。 */
    if (s_need_stable != 0U) {
        if (s_stable_sent == 0U) {
            if (s_still_armed == 0U) {
                s_still_armed = 1U;
                s_still_since = now;
            }
            if ((now - s_still_since) >= 100U) {   /* 连续静止 100ms 才算停稳 */
                uint8_t ev[3];
                ev[0] = EVT_STABLE;                /* evt = 5 */
                ev[1] = s_cur_stage;
                ev[2] = 0U;
                send_frame(MSG_EVENT, ev, 3U);
                s_stable_sent = 1U;                /* 一轮只报一次，别刷屏 */
            }
        }
    } else {
        s_still_armed = 0U;
        s_stable_sent = 0U;
    }

    /* ④ 断链检测：只在"车在跑"时检查
     *    > 300ms：告警（TODO：等有屏幕后闪灯；现在先不打扰上位机）
     *    > 1500ms：停车 + 置故障，等上位机回来说 RESUME 才继续 */
    if (s_mission == MISSION_RUN) {
        uint32_t age = now - s_last_pi_ms;
        if (age > 1500U) {
            s_mission     = MISSION_HOLD;
            s_fault      |= FAULT_LINK_LOST;
            s_need_stable = 1U;                /* 停车了，通知 Pi 我停稳了 */
            s_stable_sent = 0U;
            s_still_armed = 0U;
        }
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
