/**
 * @file    frame.h
 * @brief   上下位机通信协议：帧格式、CRC、组帧、拆帧
 *
 * ── 帧长什么样 ────────────────────────────────────────────────────────
 *
 *   字节:   0    1    2     3     4  5    6  7    8 ... 8+len-1   末尾2字节
 *        ┌────┬────┬────┬────┬─────┬─────┬───────────┬──────────┐
 *        │ AA │ 55 │ver │type│ seq │ len │  payload  │  crc16   │
 *        └────┴────┴────┴────┴─────┴─────┴───────────┴──────────┘
 *         帧头(2)      多字节都是小端          变长        小端
 *
 *   - 帧头 AA 55：用来在字节流里"重新对齐"，收到之前的内容全丢
 *   - ver       ：协议版本，对不上直接丢帧（防止旧程序连新程序）
 *   - type      ：这条命令干什么（见下面 MSG_xxx）
 *   - seq       ：发送序号，每次发一帧 +1，用来配对和排查丢帧
 *   - len       ：payload 长度，0~224。★收到后必须先校验上限再用
 *   - crc16     ：CRC16-CCITT，覆盖范围是 ver..payload（6+len 字节），
 *                 不含帧头也不含自己，两端必须完全一致
 *
 * ── 为什么多字节用小端 ────────────────────────────────────────────────
 *   STM32 和树莓派（ARM64/x86）都是小端，直接内存拷贝即可，
 *   不需要做字节序转换。Python 侧用 struct 的 '<' 前缀就是这个意思。
 */
#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

/* ---------------- 帧结构常量 ---------------- */
#define FRAME_MAGIC0        0xAAU   /* 帧头第 1 字节 */
#define FRAME_MAGIC1        0x55U   /* 帧头第 2 字节 */
#define FRAME_VERSION       0x01U   /* 协议版本 */
#define FRAME_MAX_PAYLOAD   224U    /* payload 上限，超了直接丢帧（防越界） */
#define FRAME_HDR_LEN       8U      /* 帧头 + ver + type + seq + len 共 8 字节 */

/* ---------------- 命令字：上位机 → 下位机 ---------------- */
#define MSG_TASK_CODE       0x10U   /* 4 组任务码，仅用于串口屏显示 */
#define MSG_STAGE_PLAN      0x11U   /* 某一段的任务：动作 + 颜色 + 环号 + 航点 */
#define MSG_WAYPOINT_ADD    0x12U   /* 比赛中临时追加航点（改道） */
#define MSG_MISSION_CMD     0x13U   /* 开始 / 暂停 / 继续 / 中止 / 清空 */
#define MSG_HEARTBEAT       0x14U   /* 上位机心跳（证明它还活着） */

/* ---------------- 命令字：下位机 → 上位机 ---------------- */
#define MSG_POSE            0x80U   /* 位姿上报，20Hz */
#define MSG_EVENT           0x81U   /* 事件上报（到位/抓放成功/故障/心跳） */
#define MSG_ACK             0x82U   /* 命令应答 */

/* ── 0x11 段任务表：头 12 字节 ── */
typedef struct __attribute__((packed)) {
    uint8_t stage;      /* 0..7 */
    uint8_t action;     /* 1扫码 2抓 3放 4码垛 5回库 */
    uint8_t wp_count;   /*后面跟几个航向*/
    uint8_t reserved;
    uint8_t color[3];   /* 三个物料的颜色顺序 1..6 */
    uint8_t ring[3];    /* 三个放置环号 1..6 */
    uint8_t reserved2[2];
}frame_stage_hdr_t;    /*12字节*/  

/*——单个航点8字节——*/
typedef struct __attribute__((packed)) {
    int16_t x_mm;
    int16_t y_mm;
    int16_t yaw_ddeg;    /* 0.1°，900 = 90.0° */
    uint8_t flags;       /* bit0 到位停 / bit1 可横移 / bit2 末点 */
    uint8_t reserved;
} frame_wp_t;     


/**
 * @brief 收帧回调原型
 * @param type     命令字（就是帧里的 type 字段）
 * @param payload  载荷首地址（指向内部缓冲，回调返回后失效，别存指针）
 * @param len      载荷长度
 * @note  回调在调用 Frame_Feed 的那个任务上下文里执行，不在中断里。
 */
typedef void (*frame_rx_cb_t)(uint8_t type, const uint8_t *payload, uint16_t len);

/**
 * @brief 算 CRC16-CCITT（初值 0xFFFF，多项式 0x1021，不反射、不异或输出）
 * @note  自检向量：Frame_Crc16((const uint8_t*)"123456789", 9) == 0x29B1
 *        如果这个不对，后面所有帧都会"收得到但校验失败"。
 */
uint16_t Frame_Crc16(const uint8_t *data, uint16_t len);

/**
 * @brief 把一条命令组装成完整的一帧
 * @param type     命令字
 * @param payload  载荷（可为 0，当 len=0 时）
 * @param len      载荷长度，必须 <= FRAME_MAX_PAYLOAD
 * @param seq      发送序号
 * @param out      输出缓冲，调用者保证至少 8+len+2 字节
 * @return         这一帧的总字节数；参数非法返回 0
 */
uint16_t Frame_Encode(uint8_t type, const uint8_t *payload, uint16_t len,
                      uint16_t seq, uint8_t *out);

/** @brief 清空拆帧状态机与统计（初始化时调一次） */
void Frame_Reset(void);

/**
 * @brief 喂一个字节给拆帧状态机
 * @param b   刚收到的字节
 * @param cb  凑齐一帧且 CRC 正确时回调；可以传 0 表示只校验不处理
 * @note  这个函数是"吃字节"的，不阻塞、不分配内存，
 *        所以既能在任务里调，也能在中断里调（我们是任务里调）。
 */
void Frame_Feed(uint8_t b, frame_rx_cb_t cb);

/** @brief 收到并通过 CRC 的总帧数（调试用：在涨说明协议通了） */
uint32_t Frame_OkCount(void);

/** @brief 帧头/版本/长度/CRC 出错的总次数（调试用：在涨说明有脏数据或对端不一致） */
uint32_t Frame_BadCount(void);

#endif /* FRAME_H */
