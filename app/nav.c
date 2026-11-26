/*一个航点队列（存解析好的点，等以后底盘去执行）*/
#include "nav.h"

static NavPoint_t s_nav_pts[NAV_MAX_POINTS]; /* 队列 */
static uint16_t s_head;  /* 读位置 */
static uint16_t s_tail;  /* 写位置 */
static uint16_t s_count; /* 队列中有多少个点 */

void Nav_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
}

void Nav_Clear(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
}

/*把一段 n 个点追加进队尾*/
uint8_t Nav_PushStage(uint8_t stage, const NavPoint_t *pts,uint8_t n)
{
     uint8_t i;

    if ((pts == 0) || (n == 0U)) {
        return 0U;
    }
    if (s_count + n > NAV_MAX_POINTS) {
        return 1U;                       /* 放不下：整段拒绝，别塞一半 */
    }

    /* TODO（可以先不做）：覆盖同段旧点。
     *   现在简化为"只追加"，Pi 重发同一段会重复入队。
     *   完整做法：把队列里 stage == 本段 的点全部摘掉，再追加。 */

    for (i = 0U; i < n; i++) {
        s_nav_pts[s_tail] = pts[i];
        s_nav_pts[s_tail].stage = stage;
        s_tail = (uint16_t)((s_tail + 1U) % NAV_MAX_POINTS);
        s_count++;
    }
    return 0U;
}

 /* 获取当前队列中的航点数量 */
uint16_t Nav_Count(void)
{
    return s_count;
}

bool Nav_Empty(void)
{
    return (s_count == 0U);
}

/* 看队头那个点（不删除） */
bool Nav_Current(NavPoint_t *out)
{
    if ((out == 0) || (s_count == 0U)) return false;
    *out = s_nav_pts[s_head];
    return true;
}

/* 弹出队头（执行完一个点就调它) */
bool Nav_Advance(void)
{
    if (s_count == 0U) return false;
    s_head = (uint16_t)((s_head + 1U) % NAV_MAX_POINTS);
    s_count--;
    return true;
}