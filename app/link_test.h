/**
 * @file    link_test.h
 * @brief   USART1 通信链路的"最小闭环"验收程序
 *
 * ── 这一轮只做三件事，故意不做业务 ────────────────────────────────────
 *   1) 把串口收到的字节喂给拆帧状态机
 *   2) 收到一帧合法数据 → 回一个 ACK
 *   3) 每 500ms 主动发一次心跳
 *
 * ── 为什么先做这么少 ──────────────────────────────────────────────────
 *   故障隔离。如果一上来就写"解析航点→启动任务→电机转"，
 *   链路不通时你有 5 个可能原因（串口/波特率/CRC/解析/业务），要试一天。
 *   只回 ACK，可能原因只剩 3 个，而且有明确判据：
 *     发对帧不回 → 串口或波特率问题
 *     发错帧也回 → CRC 没生效（范围算错）
 *     两个都对   → 协议层可信，可以往上加业务
 */
#ifndef LINK_TEST_H
#define LINK_TEST_H

#include <stdint.h>
#include <stdbool.h>

static void on_frame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t len);
static void send_frame(uint8_t type, const uint8_t *pl, uint16_t len);
static uint8_t handle_mission_cmd(const uint8_t *pl, uint16_t len);
static uint8_t handle_vision_adjust(const uint8_t *pl, uint16_t len);
static uint8_t handle_stage_plan(const uint8_t *pl, uint16_t len);
static uint8_t handle_waypoint_patch(const uint8_t *pl, uint16_t len);
static void send_pose(void);
void LinkTest_Init(void);


void LinkTest_Step(void);

#endif /* LINK_TEST_H */
