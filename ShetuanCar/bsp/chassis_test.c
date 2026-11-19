/**
 * @file    chassis_test.c
 * @brief   用 USER 逐项验证俯视 X 形麦轮底盘的十种动作。
 *
 * 文件结构：
 *   1. chassis_test_read_key()  — 对 USER 消抖并识别按下边沿
 *   2. chassis_test_start()     — 将动作方向转换为慢衰减驱动指令
 *   3. chassis_test_disable()   — 停机、取消当前动作并显示状态
 *   4. ChassisTest_Init()       — 初始化输出和 OLED 待机界面
 *   5. ChassisTest_Task()       — 切换动作并管理释放、等待、试跑和停机
 */

#include "chassis_test.h"
#include "main.h"
#include "motor_drive.h"
#include "OLED.h"

// 有效 PWM 占空比。提高可增加落地起步驱动力，也会提升试跑速度和滑行距离。
#define CHASSIS_TEST_DUTY_PERCENT 30

// USER 松开后预留 800 ms 让操作手离开车体并让上一动作滑行减速。
// 单次输出 700 ms，增大时动作更明显，同时试跑需要更大空地。
#define CHASSIS_TEST_SETTLE_MS 800U
#define CHASSIS_TEST_RUN_MS 700U
// PE0 上拉、低有效，连续 25 ms 确认状态，避免一次按压跳过动作。
#define CHASSIS_TEST_DEBOUNCE_MS 25U

typedef struct
{
  char label[17];
  int8_t wheel_direction[MOTOR_DRIVE_WHEEL_COUNT];
} ChassisTestAction;

// ================================================================
// 1. 俯视 X 形麦轮的动作指令
//
//    下标依次为 A 左后、B 右后、C 右前、D 左前。
//    +1 使车轮顶部朝车头转，-1 反向，0 滑行。斜移使用一对对角轮，
//    原地转向使用左右侧反向。底层 AT8236 极性由独立标定系数转换。
// ================================================================
static ChassisTestAction actions[]=
{
  {"01 Forward      ",{1,1,1,1}},
  {"02 Reverse      ",{-1,-1,-1,-1}},
  {"03 Strafe left  ",{1,-1,1,-1}},
  {"04 Strafe right ",{-1,1,-1,1}},
  {"05 Front left   ",{1,0,1,0}},
  {"06 Front right  ",{0,1,0,1}},
  {"07 Rear left    ",{0,-1,0,-1}},
  {"08 Rear right   ",{-1,0,-1,0}},
  {"09 Turn left    ",{-1,1,1,-1}},
  {"10 Turn right   ",{1,-1,-1,1}}
};

#define CHASSIS_TEST_ACTION_COUNT (sizeof(actions)/sizeof(actions[0]))

// 实测只有 B 右后轮需要反转驱动指令。这里用电机方向系数，编码器系数用于轮速反馈。
static const int8_t motor_direction[MOTOR_DRIVE_WHEEL_COUNT]={1,-1,1,1};

typedef enum
{
  CHASSIS_TEST_IDLE,
  CHASSIS_TEST_WAIT_RELEASE,
  CHASSIS_TEST_SETTLING,
  CHASSIS_TEST_RUNNING
} ChassisTestPhase;

// 主循环独占这些状态。初始索引让第一次按键回绕到 01 前进，之后每次按下推进一项。
static uint8_t selected_action=CHASSIS_TEST_ACTION_COUNT-1U;
static ChassisTestPhase phase=CHASSIS_TEST_IDLE;
static uint32_t phase_since=0U;
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static uint32_t key_since=0U;
static GPIO_PinState last_enable=GPIO_PIN_RESET;

/**
 * @brief  对 USER 按键消抖并记录稳定状态。
 * @param  now 当前 HAL 节拍，单位为 ms。
 * @return 1 表示出现一次稳定的按下边沿，0 表示没有新的按下事件。
 */
static uint8_t chassis_test_read_key(uint32_t now)
{
  GPIO_PinState level=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);

  if(level!=raw_key)
  {
    raw_key=level;
    key_since=now;
  }
  if(stable_key!=raw_key&&(now-key_since)>=CHASSIS_TEST_DEBOUNCE_MS)
  {
    stable_key=raw_key;
    return stable_key==GPIO_PIN_RESET;
  }
  return 0U;
}

static void chassis_test_start(void)
{
  int8_t duty[MOTOR_DRIVE_WHEEL_COUNT];
  uint8_t wheel;

  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    // 本测试使用固定开环占空比验证动作方向，各轮实际转速随负载而变。
    duty[wheel]=(int8_t)(actions[selected_action].wheel_direction[wheel]*
                         motor_direction[wheel]*CHASSIS_TEST_DUTY_PERCENT);
  }
  MotorDrive_RunRaw(duty);
  phase=CHASSIS_TEST_RUNNING;
  phase_since=HAL_GetTick();
}

static void chassis_test_disable(void)
{
  MotorDrive_StopOutputs();
  phase=CHASSIS_TEST_IDLE;
  last_enable=GPIO_PIN_RESET;
  OLED_ShowString(3,1,"Stopped         ");
  OLED_ShowString(4,1,"Enable switch   ");
}

// ================================================================
// 2. USER 动作切换
//
//    PD3 高电平允许运行，低电平立即撤去输出并取消本项。
//    按下 USER 先停机再换项，松开后等待，随后短跑一次。恢复使能后等待新的按下边沿。
// ================================================================
void ChassisTest_Init(void)
{
  MotorDrive_Init();
  raw_key=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);
  stable_key=raw_key;
  key_since=HAL_GetTick();
  last_enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);

  OLED_ShowString(1,1,"X chassis test  ");
  OLED_ShowString(2,1,"Action: -       ");
  OLED_ShowString(3,1,"Stopped         ");
  OLED_ShowString(4,1,last_enable==GPIO_PIN_SET?"USER: Next move ":"Enable switch   ");
}

void ChassisTest_Task(void)
{
  uint32_t now=HAL_GetTick();
  uint8_t pressed=chassis_test_read_key(now);
  GPIO_PinState enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);

  // ------- 1. 使能开关优先于动作切换和运行 -------
  if(enable==GPIO_PIN_RESET)
  {
    if(last_enable==GPIO_PIN_SET)  chassis_test_disable();
    return;
  }
  if(last_enable==GPIO_PIN_RESET)
  {
    last_enable=enable;
    stable_key=raw_key;  // 恢复使能时已按住的 USER 需先释放，避免恢复使能即启动。
    OLED_ShowString(4,1,"USER: Next move ");
    return;
  }

  // ------- 2. 按下即停机并选择下一个动作 -------
  if(pressed!=0U)
  {
    MotorDrive_StopOutputs();
    selected_action=(uint8_t)((selected_action+1U)%CHASSIS_TEST_ACTION_COUNT);
    phase=CHASSIS_TEST_WAIT_RELEASE;
    OLED_ShowString(2,1,actions[selected_action].label);
    OLED_ShowString(3,1,"Release USER    ");
    return;
  }

  // ------- 3. 松键延时后短跑，到时自动撤去驱动 -------
  if(phase==CHASSIS_TEST_WAIT_RELEASE&&stable_key==GPIO_PIN_SET)
  {
    phase=CHASSIS_TEST_SETTLING;
    OLED_ShowString(3,1,"Starting...     ");
    phase_since=HAL_GetTick();
  }
  else if(phase==CHASSIS_TEST_SETTLING&&raw_key==GPIO_PIN_SET&&
          (now-phase_since)>=CHASSIS_TEST_SETTLE_MS)
  {
    OLED_ShowString(3,1,"Running         ");
    // 刷屏期间电机仍停机。再次读 PD3，保证刷屏时拨回停止也能取消起跑。
    if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET)
    {
      chassis_test_disable();
      return;
    }
    chassis_test_start();
  }
  else if(phase==CHASSIS_TEST_RUNNING&&(now-phase_since)>=CHASSIS_TEST_RUN_MS)
  {
    MotorDrive_StopOutputs();
    phase=CHASSIS_TEST_IDLE;
    OLED_ShowString(3,1,"Done            ");
  }
}
