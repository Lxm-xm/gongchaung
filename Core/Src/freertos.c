/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   FreeRTOS 任务创建与启动
  * @attention
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "link_test.h"      /* 第 6 步：把串口链路挂进任务 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId defaultTaskHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 互斥量：以后两个任务同时访问同一份数据（比如航点队列）时在这里建 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 信号量：以后"中断通知任务"时用（比轮询省 CPU） */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 软件定时器 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 消息队列：以后 CAN 收包、串口帧跨任务传递时用 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 以后在这里加第二个任务，例如 20ms 底盘任务：
   *   osThreadDef(chassisTask, StartChassisTask, osPriorityHigh, 0, 1024);
   *   chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);
   */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务：现在跑 USART1 通信链路
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;       /* 参数没用，加 (void) 避免告警 */

  /* ── 初始化只做一次，所以放在 for 之前 ──
   * 注意顺序：LinkTest_Init() 内部会调 Uart1_Init()，
   * 而 Uart1_Init() 依赖 MX_USART1_UART_Init() 已经执行过——
   * 那是在 main() 里、调度器启动之前做的，所以这里一定满足。 */
  LinkTest_Init();

  /* ── 任务主循环 ──
   * ★FreeRTOS 的任务绝对不允许"跑完就返回"，必须是死循环。
   *   如果返回了，内核会触发 configASSERT 或者跑飞。
   *
   * osDelay(20) 让出 CPU 20ms。为什么不是 osDelay(1)：
   *   20ms 和我们的数据节奏匹配（心跳 500ms、以后位姿 20Hz），
   *   1ms 只是白白增加调度次数，没有收益。 */
  for(;;)
  {
    LinkTest_Step();    /* 收帧（拆包+校验+回ACK） + 每 500ms 发心跳 */
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

