#ifndef NAV_H
#define NAV_H

#include <stdint.h>
#include <stdbool.h>

#define NAV_MAX_POINTS   48U    /* 队列总容量 */
#define NAV_MAX_STAGE     8U    /* 段号范围 0..7 */

typedef struct {
    float   x_mm;
    float   y_mm;
    float   yaw_deg;
    uint8_t stage;
    uint8_t flags;
} NavPoint_t;

void Nav_Init(void);
void Nav_Clear(void);
uint8_t Nav_PushStage(uint8_t stage, const NavPoint_t *pts, uint8_t n); /* 0=成功 1=队列满 */
uint16_t Nav_Count(void);
bool Nav_Empty(void);
bool Nav_Current(NavPoint_t *out);
bool Nav_Advance(void);

#endif