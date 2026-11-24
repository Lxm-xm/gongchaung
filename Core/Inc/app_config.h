#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ============================================================================
 * 本文件是"整车配置总开关"
 * ----------------------------------------------------------------------------
 * 里面全是编译期开关：改完之后必须重新编译 + 烧录 + 复位，
 * 运行中改是没有任何效果的（因为对应的代码已经被编译器删掉了）。
 *
 * 开关一览（按重要性排）：
 *   APP_IMU_MODEL           : 陀螺仪型号二选一，选错会编译不过（下面有 #error 兜底）
 *   APP_RADAR_ENABLED       : 有没有激光雷达，0 时不初始化 UART5，A* 用固定地图
 *   APP_MANUAL_TASK_ENABLED : 1 = 不等扫码直接用下面四组写死的数据（调试用）
 *   APP_MANUAL_TASK_GROUP1~4: 写死的四组任务数据
 *
 * 文件末尾还定义了两个枚举和一个全局结构体，用来在 Watch 窗口里
 * 观察"发车流程走到哪一步了、卡在哪"。主流程的调试全靠它们。
 * ========================================================================== */

/*
 * 陀螺仪型号：APP_IMU_HWT101 或 APP_IMU_CY61E，修改后重新编译烧录。
 * 两种模块都接 USART6（PC6 TX、PC7 RX），115200，8N1。
 * 底盘和视觉仍从同一接口读取航向角。
 */
/* 下面这两个宏只是"型号编号"，0 和 1 本身没有含义，
 * 真正的意思要配合 #if APP_IMU_MODEL == APP_IMU_CY61E 去看。
 * 之所以编号从 0 开始、且写成 0U/1U：0 是默认值，万一宏没被定义，
 * #if 会把它当 0 处理，等于自动退回 HWT101，不会出现"两个分支都不成立"。
 */
#define APP_IMU_HWT101 0U
#define APP_IMU_CY61E  1U
#ifndef APP_IMU_MODEL
#define APP_IMU_MODEL  APP_IMU_CY61E
#endif

/* 这一段的写法值得单独理解，它是"允许外部覆盖 + 非法值拦截"的经典套路：
 *   #ifndef APP_IMU_MODEL  → 如果编译选项里没定义（例如 Keil 的 -DAPP_IMU_MODEL=0）
 *   #define ... CY61E      → 才用这里的默认值；外部定义过就什么都不做
 *   紧接着的 #if + #error  → 值既不是 0 也不是 1 时，编译直接报错停下
 * 为什么必须有 #error：不成立的分支不会被编译器检查，
 * 万一写错成 2U，两套 IMU 代码都不会被编译，你会得到一个"没有陀螺仪"的固件，
 * 编译却能通过 —— 这种错误在车上极难排查，所以在这里提前拦住。
 */
#if APP_IMU_MODEL != APP_IMU_HWT101 && APP_IMU_MODEL != APP_IMU_CY61E
#error APP_IMU_MODEL_must_be_APP_IMU_HWT101_or_APP_IMU_CY61E
#endif

/*
 * 雷达开关（后续只改这里即可恢复）：
 * 0：当前未接雷达，不初始化UART5、不发扫描指令；
 *    按地图无新增障碍运行A*，保留固定地图约束和原有转向代价。
 * 1：接回雷达后，恢复UART5初始化、启动扫描和障碍绕行规划。
 * 无雷达测试前必须确认路线确实没有新增障碍，不能自动躲避未知物体。
 */
/* 这个开关影响的是"编不编"而不是"跑不跑"，改动范围有三个文件：
 *   main.c / freertos.c : 是否调用 UART5 初始化、雷达启动、障碍回传处理
 *   path_plan.c         : 障碍边映射 grid_to_edges() 拿到的 grid_count 恒为 0
 *   radar_startup.c     : 无雷达时走 RadarStartup_PlanWithoutRadar()，
 *                         它和雷达版共用同一套 A*，只是障碍表为空
 * 换句话说：关掉雷达不会改变规划算法，只是"看不见新障碍"，只能按固定地图走，
 * 所以关雷达前必须人工确认路线上没有会撞的东西。
 */
#ifndef APP_RADAR_ENABLED
#define APP_RADAR_ENABLED 0
#endif

/* 任务数据来源：1=手动测试，不等待扫码；0=恢复实际扫码。
 * 手动模式仍会行驶到扫码区，再按下列四组任务继续完整流程。
 * 只跳过扫码等待，不跳过后续视觉定位、抓取和码垛动作。
 */
/* 为什么需要这个开关：赛前调试时不想每次都等扫码（扫码要停车、要光照、还要等帧），
 * 于是把扫码结果写死成 APP_MANUAL_TASK_GROUP1~4，车照样跑完扫码区，
 * 但"扫码结果"直接用手写值，后面视觉定位/抓取/码垛全部照常执行。
 * 注意它只跳过"等码"这一段，不跳过任何动作 —— 想跑空路线请去 freertos.c 看 case1~6。
 */
#ifndef APP_MANUAL_TASK_ENABLED
#define APP_MANUAL_TASK_ENABLED 1
#endif

/* 每组是三位十进制数，三位不能重复；修改后重新编译、烧录并复位。
 * 第1/3组：两圈取料颜色，各位范围1~6，两组必须是相同颜色的排列。
 * 第2/4组：两圈加工圆环，各位范围1~3；第一圈暂存沿用第2组，
 * 第二圈暂存自动按颜色匹配第一层，不直接使用第4组放置顺序。
 * 当前123+123+123+123：两圈均取颜色1/2/3，加工圆环均为1/2/3。
 */
/* 【补充说明】上面那段注释里写的 "当前123+123+123+123" 是旧版本的数值，
 * 现在真正编译进固件的是下面四个宏：组1=642、组2=123、组3=462、组4=321，
 * 请以宏为准，不要以那段注释为准。
 *
 * 三位的读法（以 642 为例）：百位=第1个取料位置的颜色6，十位=颜色4，个位=颜色2。
 * 也就是"第1/2/3个槽位分别取什么颜色"，数字本身就是颜色编号 1~6 红黄蓝绿黑浅蓝。
 * 组2/组4 是加工圆环编号 1~3（车上三个工位圆环）。
 * 两条硬约束：同组三位不能重复；组1 和组3 必须是同一批颜色的不同排列，
 * 否则第二圈取不到和第一圈相同的颜色，暂存同色码垛会失败。
 */
#define APP_MANUAL_TASK_GROUP1 642U
#define APP_MANUAL_TASK_GROUP2 123U
#define APP_MANUAL_TASK_GROUP3 462U
#define APP_MANUAL_TASK_GROUP4 321U

/* 主流程选区/准备状态，可在Watch查看g_app_start。按键只在WAIT_KEY读取。 */
/* g_app_start.status 就是"发车流程的进度条"，顺序固定：
 *   WAIT_KEY      等待按键（整个程序里只有这一刻会读 PC13/PD13，之后不再读）
 *        ↓ 读到键
 *   WAIT_FEEDBACK 等底盘任务回应"我已经停了"（底盘任务 20ms 一拍，所以很快）
 *        ↓
 *   WAIT_POSE     把地图初始位姿改成该区的位姿，并等序号握手完成
 *                 （2 区必须改，否则地图以为车在 1 区，第一步就会撞墙）
 *        ↓
 *   PLANNING      雷达扫描 / 无雷达时直接按固定地图跑 A*
 *        ↓
 *   RUNNING       交给 PathFollower 执行 8 段路线
 *        ↓
 *   DONE          全流程结束
 * 任何一步超时或失败 → ERROR，具体原因看 g_app_start.fault（下一个枚举）。
 *
 * 调试技巧：车不动的时候，先在 Watch 里看 g_app_start.status，
 * 卡在哪个状态就说明问题出在那一步，不用满车乱查。
 */
typedef enum {
    APP_START_WAIT_KEY = 0,
    APP_START_WAIT_FEEDBACK,
    APP_START_WAIT_POSE,
    APP_START_PLANNING,
    APP_START_RUNNING,
    APP_START_DONE,
    APP_START_ERROR
} AppStart_Status_t;

/* 故障码与出问题的那一步一一对应，看到哪个就查哪个：
 *   BAD_ZONE         : 按键读到的值既不是 1 也不是 2（按键坏或引脚被占用）
 *   FEEDBACK_TIMEOUT : 等底盘"已停车"回应超过 START_FEEDBACK_TIMEOUT_MS(5000ms)
 *   CHASSIS          : 底盘自检没过（编码器丢失 / 航向不可信 / 内部故障）
 *   POSE_QUEUE       : 位姿修正指令没能塞进长度为 4 的命令队列（队列被占满）
 *   POSE_TIMEOUT     : 位姿修正发出去 1000ms 内没有握手完成
 *   PLAN             : A* 找不到可行路径（通常是起点/终点被障碍边包住）
 *   FOLLOWER         : 路径执行层报错（某一段路线超时或到不了位）
 */
typedef enum {
    APP_START_FAULT_NONE = 0,
    APP_START_FAULT_BAD_ZONE,
    APP_START_FAULT_FEEDBACK_TIMEOUT,
    APP_START_FAULT_CHASSIS,
    APP_START_FAULT_POSE_QUEUE,
    APP_START_FAULT_POSE_TIMEOUT,
    APP_START_FAULT_PLAN,
    APP_START_FAULT_FOLLOWER
} AppStart_Fault_t;

/* 这个全局对象是"主流程 ↔ 人"之间的唯一窗口，定义在 freertos.c，声明在这里。
 *   status       : 当前走到哪一步（见上面第一个枚举）
 *   fault        : 出错原因（见上面第二个枚举）
 *   selected_zone: 本次发车用的是哪个启停区，0=还没按键
 *   pose_sequence: 这个字段是和底盘任务握手的凭据，必须理解它
 *
 * 序号握手是什么：往底盘命令队列里塞一条"把位姿改成 X, Y, 航向 Z"的命令时，
 * 队列里的命令会分到一个递增序号，函数立刻返回这个序号，但此时车还没动、
 * 地图也还没改。只有当底盘任务真正处理完那条命令后，
 * chassis_nav 里的 completed_sequence 才会等于这个序号。
 * 所以"发出去 ≠ 生效"，中间必须等两者相等（超时 1000ms 就判 POSE_TIMEOUT）。
 */
typedef struct {
    AppStart_Status_t status;
    AppStart_Fault_t fault;
    uint8_t selected_zone; /* 0=未按键；1=PC13；2=PD13，上电只选择一次。 */
    uint32_t pose_sequence; /* 必须等电机任务completed_sequence匹配才允许规划。 */
} AppStart_State_t;

extern AppStart_State_t g_app_start;

#endif /* APP_CONFIG_H */
