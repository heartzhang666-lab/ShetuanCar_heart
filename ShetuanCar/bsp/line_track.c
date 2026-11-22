/**
 * @file    line_track.c
 * @brief   八路灰度闭环处理曲线、折角、交叉点及横线终点停车。
 *
 * 文件结构：
 *   1. 目标与状态管理       — 原子提交左右目标、停机和重新起跑
 *   2. 灰度外环与元素识别   — 位置 PD、折角寻线、交叉计数及终点识别
 *   3. USER 与主循环调度   — 消抖、停车保持、通信及 OLED
 *   4. TIM6 速度内环       — 固定 10 ms 四轮调速与独立停机联锁
 */

#include "line_track.h"
#include <stdio.h>
#include "gray_line.h"
#include "line_tuning.h"
#include "main.h"
#include "motor_drive.h"
#include "OLED.h"
#include "tim.h"
#include "wheel_speed.h"

// 外环为 20 ms，内环为 10 ms。外环 D 差分按实际帧间隔折算回 20 ms，
// 避免 OLED 或串口造成的周期变化直接改变 D 的强度。目标超过 100 ms 未刷新即停机。
#define TRACK_SCAN_MS 20U
#define TRACK_STALE_MS 100U
#define TRACK_DEBOUNCE_MS 25U
#define TRACK_OLED_MS 120U
#define TRACK_RACE_LIMIT_MS 120000U
#define TRACK_FINISH_HOLD_MS 2000U
// 只有四轮连续 200 个 10 ms 周期均无编码器增量，才确认静止保持完成；有增量就重计。
#define TRACK_FINISH_STILL_TICKS (TRACK_FINISH_HOLD_MS/10U)

// 单位为编码器计数/10 ms，沿用已验证速度环；以下外环参数均为待实车调整的初值。
// |e| 越大基础速度越低，保留更大回线余量；左右轮均不反转，内侧最小速度为 2。
#define TRACK_FORWARD_SPEED 20
#define TRACK_SLOW_SPEED 12
#define TRACK_MIN_SPEED 2
#define TRACK_CURVE_SLOWDOWN 1.2f
#define TRACK_TURN_OUTER_SPEED 18U
#define TRACK_TURN_INNER_SPEED 2U
#define TRACK_SEARCH_TURN 3

// 连续帧计数只在成功采到的一帧更新，不按主循环次数累加。
// 2 帧名义约 40 ms，5 帧约 100 ms；实际帧间隔可变，保护超时另按 HAL 毫秒计时。
// 增大确认帧数可抑制噪声，但窄横线或高速通过时可能漏检，须落地观察 M 位图。
#define TRACK_CONFIRM_FRAMES 2U
#define TRACK_CLEAR_FRAMES 5U
#define TRACK_START_TIMEOUT_MS 1500U
#define TRACK_ELEMENT_TIMEOUT_MS 1500U
#define TRACK_TURN_MIN_MS 80U
// 离开交叉点后至少 500 ms 才允许下一次计数/终点识别，抑制边界抖动重复事件。
// 本值是当前赛道的联调初值；过大会漏掉紧邻元素，过小会降低重复计数抑制效果。
#define TRACK_CROSS_REARM_MS 500U
#define TRACK_LOST_STOP_MS 800U
#define TRACK_CENTER_ERROR 2.0f
#define TRACK_EDGE_ERROR 5.0f

// 已与用户核对规则附图：一圈两次经过同一个中央交叉点，之后的全宽横线为终点。
// 这是传感器事件计数，不按时间或预存轨迹跑圈。换赛道拓扑必须重新核对本值。
#define TRACK_CROSSINGS_PER_LAP 2U

typedef enum
{
  TRACK_READY,
  TRACK_RUN,
  TRACK_START,
  TRACK_CROSS,
  TRACK_TURN_LEFT,
  TRACK_TURN_RIGHT,
  TRACK_LOST,
  TRACK_FINISH_HOLD,
  TRACK_FINISHED,
  TRACK_USER_STOP,
  TRACK_LOST_STOP,
  TRACK_ADC_STOP,
  TRACK_DISABLED,
  TRACK_STALE_STOP,
  TRACK_ELEMENT_STOP,
  TRACK_RACE_STOP
} LineTrackStatus;

typedef enum
{
  TRACK_FOLLOW,
  TRACK_PASS_CROSS,
  TRACK_SEEK_CORNER
} LineTrackPhase;

typedef struct
{
  uint16_t left;
  uint16_t right;
  uint32_t stamp;
  uint8_t armed;
  uint8_t brake;
} LineTrackCommand;

// 以下状态由主循环独占，每次 USER 重新起跑清零，停车时清除运动阶段与差分历史。
// departed 屏蔽起跑横线；crossings 仅在确认离开交叉区后增加，终点保持期间冻结。
static uint8_t running=0U;
static uint8_t departed=0U;
static uint8_t have_line=0U;
static uint8_t lost=0U;
static uint8_t derivative_valid=0U;
static uint8_t crossings=0U;
static uint8_t clear_frames=0U;
static uint8_t cross_frames=0U;
static uint8_t finish_frames=0U;
static uint8_t corner_frames=0U;
static int8_t corner_candidate=0;
static int8_t turn_direction=0;
static uint32_t lost_since=0U;
static uint32_t phase_since=0U;
static uint32_t cross_exit_since=0U;
static uint32_t scan_since=0U;
static uint32_t oled_since=0U;
static uint32_t error_since=0U;
static float last_error=0.0f;
static int16_t last_turn=0;
static LineTrackStatus status=TRACK_READY;
static LineTrackPhase phase=TRACK_FOLLOW;
static GrayLineReading frame;

// PE0 上拉、低有效。按住只触发一次；终点保持期间也消费按键边沿，避免保持结束自动重启。
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static uint32_t key_since=0U;

// 主循环在短临界区写 command，TIM6 只读目标并更新四个独立 PI 状态。
// race_since 在未授权时由主循环设定，TIM6 独立检查 120 s，防止主循环阻塞时超时继续跑。
// drive_active/brake_active/still_ticks 由 TIM6 写、主循环读；四轮连续无计数满 2 s 才结束。
static volatile LineTrackCommand command={0U,0U,0U,0U,0U};
static volatile uint32_t race_since=0U;
static volatile LineTrackStatus interrupt_stop=TRACK_READY;
static volatile uint8_t drive_active=0U;
static volatile uint8_t brake_active=0U;
static volatile uint16_t still_ticks=0U;
static WheelSpeedController controller[MOTOR_DRIVE_WHEEL_COUNT];

// ================================================================
// 1. 目标与状态管理
//
//    左侧 A/D，右侧 B/C，正速均为前进，极性由已标定驱动与编码器模块处理。
//    TIM6 已撤销授权后，主循环中的旧采样结果不得重新打开电机。
// ================================================================
static void line_track_publish(uint16_t left,uint16_t right,uint32_t now)
{
  uint32_t interrupts=__get_PRIMASK();

  __disable_irq();
  if(interrupt_stop==TRACK_READY||running==0U)
  {
    command.left=left;
    command.right=right;
    command.stamp=now;
    command.armed=running;
    command.brake=(uint8_t)(status==TRACK_FINISH_HOLD);
  }
  __set_PRIMASK(interrupts);
}

static void line_track_stop(LineTrackStatus reason,uint32_t now)
{
  running=0U;
  lost=0U;
  derivative_valid=0U;
  phase=TRACK_FOLLOW;
  status=reason;
  line_track_publish(0U,0U,now);
}

/**
 * @brief  主循环开始一次新试跑，清除上一圈的元素计数和 PID 外环历史。
 * @param  now 当前毫秒节拍；所有持续时间用无符号差值计算，允许自然回绕。
 */
static void line_track_start(uint32_t now)
{
  running=1U;
  departed=0U;
  have_line=0U;
  lost=0U;
  derivative_valid=0U;
  crossings=0U;
  cross_exit_since=now;
  clear_frames=0U;
  cross_frames=0U;
  finish_frames=0U;
  corner_frames=0U;
  corner_candidate=0;
  turn_direction=0;
  last_turn=0;
  race_since=now;
  phase_since=now;
  phase=TRACK_FOLLOW;
  status=TRACK_START;
  scan_since=now-TRACK_SCAN_MS;
}

static float line_track_abs(float value)
{
  return value<0.0f?-value:value;
}

static uint8_t line_track_black_count(uint8_t mask)
{
  uint8_t count=0U;

  while(mask!=0U)
  {
    count+=(uint8_t)(mask&1U);
    mask>>=1U;
  }
  return count;
}

// 普通单线重新落入中间四个探头附近才确认离开元素，边缘单点不算回线成功。
static uint8_t line_track_centered(void)
{
  return (uint8_t)(frame.state==GRAY_LINE_NORMAL&&(frame.mask&0x3CU)!=0U&&
                   line_track_abs(frame.error)<=TRACK_CENTER_ERROR);
}

// ================================================================
// 2. 灰度外环与赛道元素识别
//
//    传感器位于前轮前约 5 cm。折角按实时边缘与回线条件处理，不把 5 cm
//    擅自换算为固定延时或编码器距离；轮径、每圈计数和底盘转动中心尚未量定。
//    交叉点两侧都有黑线时保持直行，避免对两条分支的平均重心作大幅转向。
// ================================================================
/**
 * @brief  正常曲线的位置 PD，并按横向误差降低基础速度。
 * @param  now 当前帧完成采样的毫秒节拍。
 * @param  slow 非零时把基础速度上限限制为低速，用于离开起跑区和交叉区回线。
 */
static void line_track_follow(uint32_t now,uint8_t slow)
{
  LineTuningGains gains=LineTuning_GetGains();
  float difference=0.0f;
  float correction;
  float limit;
  int16_t base=TRACK_FORWARD_SPEED-(int16_t)(line_track_abs(frame.error)*TRACK_CURVE_SLOWDOWN);

  if(base<TRACK_SLOW_SPEED)  base=TRACK_SLOW_SPEED;
  if(slow!=0U)  base=TRACK_SLOW_SPEED;
  if(derivative_valid!=0U&&(now-error_since)!=0U)
  {
    difference=(frame.error-last_error)*(float)TRACK_SCAN_MS/(float)(now-error_since);
  }
  correction=gains.kp*frame.error+gains.kd*difference;
  limit=(float)(base-TRACK_MIN_SPEED);
  if(correction>limit)  correction=limit;
  if(correction<-limit)  correction=-limit;
  last_turn=(int16_t)(correction+(correction>=0.0f?0.5f:-0.5f));
  last_error=frame.error;
  error_since=now;
  derivative_valid=1U;
  have_line=1U;
  line_track_publish((uint16_t)(base+last_turn),(uint16_t)(base-last_turn),now);
}

static void line_track_turn(uint32_t now)
{
  // 只用已验证的前向 PI。外侧 18、内侧 2，避免未经验证的反向速度闭环。
  // 与原地反转相比转弯半径较大，能否通过实物直角需按 5 cm 前伸和轮距实测。
  status=turn_direction>0?TRACK_TURN_RIGHT:TRACK_TURN_LEFT;
  if(turn_direction>0)  line_track_publish(TRACK_TURN_OUTER_SPEED,TRACK_TURN_INNER_SPEED,now);
  else  line_track_publish(TRACK_TURN_INNER_SPEED,TRACK_TURN_OUTER_SPEED,now);
}

static void line_track_set_lost(uint32_t now)
{
  int16_t turn=last_error>TRACK_CENTER_ERROR?TRACK_SEARCH_TURN:
               (last_error<-TRACK_CENTER_ERROR?-TRACK_SEARCH_TURN:0);

  if(lost==0U)
  {
    lost=1U;
    lost_since=now;
  }
  derivative_valid=0U;
  status=TRACK_LOST;
  if(have_line==0U)  line_track_publish(0U,0U,now);
  else  line_track_publish((uint16_t)(TRACK_SLOW_SPEED+turn),(uint16_t)(TRACK_SLOW_SPEED-turn),now);
}

/**
 * @brief  每次成功采样后处理一帧；元素确认与计数仅在本函数推进。
 * @param  now 本帧采样完成时间，单位 ms。
 *
 * 终点使用八探头全黑连续两帧，必须已离开起点且确认通过两次交叉点。
 * 横线应覆盖八探头；窄到不足两帧时需降速或调整确认值，不用延时冒充识别。
 */
static void line_track_process(uint32_t now)
{
  uint8_t centered=line_track_centered();
  uint8_t junction=(uint8_t)(frame.state==GRAY_LINE_WIDE&&
                            line_track_black_count(frame.mask)>=4U&&
                            (frame.mask&0x07U)!=0U&&(frame.mask&0xE0U)!=0U);
  int8_t candidate=0;

  // ------- 1. 终点制动后保持零目标，ADC 仍照常采样并接受异常停机 -------
  if(status==TRACK_FINISH_HOLD)
  {
    line_track_publish(0U,0U,now);
    return;
  }

  // ------- 2. 起跑横线只允许低速驶离，不计成交叉点或终点 -------
  if(departed==0U)
  {
    status=TRACK_START;
    if(frame.state!=GRAY_LINE_LOST)  lost=0U;
    if(centered!=0U)
    {
      if(clear_frames<TRACK_CLEAR_FRAMES)  clear_frames++;
      if(clear_frames>=TRACK_CLEAR_FRAMES)
      {
        departed=1U;
        clear_frames=0U;
        status=TRACK_RUN;
      }
    }
    else  clear_frames=0U;
    if((now-race_since)>=TRACK_START_TIMEOUT_MS&&departed==0U)
    {
      line_track_stop(TRACK_ELEMENT_STOP,now);
      return;
    }
    if(frame.state==GRAY_LINE_NORMAL)  line_track_follow(now,1U);
    else if(frame.state==GRAY_LINE_WIDE)
    {
      derivative_valid=0U;
      line_track_publish(TRACK_SLOW_SPEED,TRACK_SLOW_SPEED,now);
    }
    else  line_track_set_lost(now);
    return;
  }

  // ------- 3. 已进入折角或交叉区，必须重新确认窄线，或超时停车 -------
  if(phase!=TRACK_FOLLOW)
  {
    lost=0U;  // 元素内部允许短暂空白，使用独立的元素超时，不沿用上一段丢线计时。
    if((now-phase_since)>=TRACK_ELEMENT_TIMEOUT_MS)
    {
      line_track_stop(TRACK_ELEMENT_STOP,now);
      return;
    }
    if(centered!=0U&&(phase!=TRACK_SEEK_CORNER||(now-phase_since)>=TRACK_TURN_MIN_MS))
    {
      if(clear_frames<TRACK_CLEAR_FRAMES)  clear_frames++;
    }
    else  clear_frames=0U;
    if(clear_frames>=TRACK_CLEAR_FRAMES)
    {
      if(phase==TRACK_PASS_CROSS)
      {
        if(crossings<TRACK_CROSSINGS_PER_LAP)  crossings++;
        cross_exit_since=now;
      }
      phase=TRACK_FOLLOW;
      clear_frames=0U;
      derivative_valid=0U;
      status=TRACK_RUN;
      line_track_follow(now,1U);
      return;
    }
    if(phase==TRACK_SEEK_CORNER)  line_track_turn(now);
    else
    {
      status=TRACK_CROSS;
      // 出交叉点重新见到单线后即低速 PD；交叉内部、短暂空白均维持直行。
      if(frame.state==GRAY_LINE_NORMAL)  line_track_follow(now,1U);
      else
      {
        derivative_valid=0U;
        line_track_publish(TRACK_SLOW_SPEED,TRACK_SLOW_SPEED,now);
      }
    }
    return;
  }

  // ------- 4. 两次交叉点都离开后，下一条连续全黑横线触发终点制动 -------
  if(crossings>=TRACK_CROSSINGS_PER_LAP&&(now-cross_exit_since)>=TRACK_CROSS_REARM_MS&&frame.mask==0xFFU)
  {
    lost=0U;
    if(finish_frames<TRACK_CONFIRM_FRAMES)  finish_frames++;
    derivative_valid=0U;
    if(finish_frames>=TRACK_CONFIRM_FRAMES)
    {
      status=TRACK_FINISH_HOLD;
      line_track_publish(0U,0U,now);
    }
    else  line_track_publish(TRACK_SLOW_SPEED,TRACK_SLOW_SPEED,now);
    return;
  }
  finish_frames=0U;

  // ------- 5. 双侧宽黑确认交叉；只在离开后计一次，不重复累加内部帧 -------
  if(junction!=0U)
  {
    if(crossings<TRACK_CROSSINGS_PER_LAP)  lost=0U;
    if(crossings!=0U&&(now-cross_exit_since)<TRACK_CROSS_REARM_MS)
    {
      cross_frames=0U;
      derivative_valid=0U;
      line_track_publish(TRACK_SLOW_SPEED,TRACK_SLOW_SPEED,now);
      return;
    }
    if(crossings>=TRACK_CROSSINGS_PER_LAP)
    {
      // 已经过两次交叉后，部分探头先压到终点横线时不能锁入第三次交叉状态。
      // 等待后续全黑帧；若一直无法确认，使用丢线时限停车，避免长时间盲行。
      line_track_set_lost(now);
      return;
    }
    if(cross_frames<TRACK_CONFIRM_FRAMES)  cross_frames++;
    corner_frames=0U;
    derivative_valid=0U;
    status=TRACK_CROSS;
    if(cross_frames>=TRACK_CONFIRM_FRAMES)
    {
      phase=TRACK_PASS_CROSS;
      phase_since=now;
      clear_frames=0U;
      cross_frames=0U;
    }
    line_track_publish(TRACK_SLOW_SPEED,TRACK_SLOW_SPEED,now);
    return;
  }
  cross_frames=0U;

  // ------- 6. 单侧宽黑或单线抵达最外侧，持续两帧后闭环寻角 -------
  // bit0 最左，bit7 最右。左三路黑且右两路无黑为左折角候选，右侧镜像。
  // 双侧都有黑优先按交叉处理；普通圆弧偏到边缘也会低速寻线，不能据此宣称识别了 90°。
  if(frame.state==GRAY_LINE_WIDE&&line_track_black_count(frame.mask)>=3U)
  {
    if((frame.mask&0x07U)==0x07U&&(frame.mask&0xC0U)==0U)  candidate=-1;
    else if((frame.mask&0xE0U)==0xE0U&&(frame.mask&0x03U)==0U)  candidate=1;
  }
  else if(frame.state==GRAY_LINE_NORMAL)
  {
    if(frame.error<=-TRACK_EDGE_ERROR&&(frame.mask&0x01U)!=0U)  candidate=-1;
    else if(frame.error>=TRACK_EDGE_ERROR&&(frame.mask&0x80U)!=0U)  candidate=1;
  }
  if(candidate!=0)
  {
    if(candidate!=corner_candidate)  corner_frames=0U;
    corner_candidate=candidate;
    if(corner_frames<TRACK_CONFIRM_FRAMES)  corner_frames++;
    if(corner_frames>=TRACK_CONFIRM_FRAMES)
    {
      phase=TRACK_SEEK_CORNER;
      phase_since=now;
      turn_direction=candidate;
      clear_frames=0U;
      corner_frames=0U;
      derivative_valid=0U;
      lost=0U;
      line_track_turn(now);
      return;
    }
  }
  else
  {
    corner_candidate=0;
    corner_frames=0U;
  }

  // ------- 7. 普通曲线使用 PD，宽黑不明时只短时低速通过，丢线按历史方向找线 -------
  if(frame.state==GRAY_LINE_LOST)
  {
    line_track_set_lost(now);
    return;
  }
  if(frame.state==GRAY_LINE_WIDE)
  {
    // 不明宽线沿用丢线时限，避免噪声或整片黑底使车辆无限向前行驶。
    line_track_set_lost(now);
    return;
  }
  lost=0U;
  status=TRACK_RUN;
  line_track_follow(now,0U);
}

// ================================================================
// 3. USER、OLED 和主循环调度
// ================================================================
static void line_track_draw(void)
{
  static char *titles[]=
  {
    "Line track ready","Tracking        ","Leaving start   ","Cross: straight ",
    "Corner: left    ","Corner: right   ","Line lost       ","Finish hold 2s  ",
    "Finished        ","USER stop       ","Lost: stop      ","ADC error: stop ",
    "Enable switch   ","Frame timeout   ","Element timeout ","Race 120s: stop "
  };
  LineTuningGains gains=LineTuning_GetGains();
  char text[17];

  OLED_ShowString(1,1,titles[status]);
  (void)snprintf(text,sizeof(text),"E%+5.2f M%02X X%u  ",(double)frame.error,
                 (unsigned int)frame.mask,(unsigned int)crossings);
  OLED_ShowString(2,1,text);
  (void)snprintf(text,sizeof(text),"P%4.2f D%5.2f    ",(double)gains.kp,(double)gains.kd);
  OLED_ShowString(3,1,text);
  OLED_ShowString(4,1,status==TRACK_FINISH_HOLD?"Hold: no restart":(running?"USER: Stop      ":"USER: Start     "));
  oled_since=HAL_GetTick();
}

static uint8_t line_track_key_pressed(uint32_t now)
{
  GPIO_PinState level=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);

  if(level!=raw_key)
  {
    raw_key=level;
    key_since=now;
  }
  if(stable_key!=raw_key&&(now-key_since)>=TRACK_DEBOUNCE_MS)
  {
    stable_key=raw_key;
    return stable_key==GPIO_PIN_RESET;
  }
  return 0U;
}

void LineTrack_Init(void)
{
  GrayLine_Init();
  WheelSpeed_Init();
  LineTuning_Init();
  raw_key=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);
  stable_key=raw_key;
  key_since=HAL_GetTick();
  if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET)  status=TRACK_DISABLED;
  line_track_draw();

  // TIM6 为 84 MHz/(84×10000)=100 Hz，清除初始化更新标志后开启中断。
  __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
  if(HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK)  Error_Handler();
}

void LineTrack_Task(void)
{
  uint32_t now=HAL_GetTick();
  uint8_t pressed=line_track_key_pressed(now);
  GPIO_PinState enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);
  LineTrackStatus abort_reason=interrupt_stop;

  // ------- 1. 联锁先于按键；中断停机与按键同一轮出现时不得立即重新起跑 -------
  if(abort_reason!=TRACK_READY)
  {
    interrupt_stop=TRACK_READY;
    line_track_stop(abort_reason,now);
    pressed=0U;
  }
  if(enable==GPIO_PIN_RESET)
  {
    if(status!=TRACK_DISABLED)  line_track_stop(TRACK_DISABLED,now);
    pressed=0U;
    stable_key=raw_key;
  }
  else if(status==TRACK_DISABLED)  status=TRACK_READY;
  if(status==TRACK_FINISH_HOLD)  pressed=0U;
  if(pressed!=0U)
  {
    if(running!=0U)  line_track_stop(TRACK_USER_STOP,now);
    else  line_track_start(now);
  }
  if(running!=0U&&(now-race_since)>=TRACK_RACE_LIMIT_MS)  line_track_stop(TRACK_RACE_STOP,now);
  if(running!=0U&&lost!=0U&&(now-lost_since)>=TRACK_LOST_STOP_MS)  line_track_stop(TRACK_LOST_STOP,now);

  // ------- 2. TIM6 制动期间确认四轮连续无计数 2 s，USER 不能提前跳过 -------
  // 任意轮出现一个计数就重新计时。编码器只能反映轮子，打滑、外力拖动和 30 mm 偏差仍需实测。
  if(status==TRACK_FINISH_HOLD&&brake_active!=0U&&still_ticks>=TRACK_FINISH_STILL_TICKS)
  {
    line_track_stop(TRACK_FINISHED,now);
  }

  // ------- 3. 完整采样后更新目标，ADC 失败时不使用旧帧继续跑 -------
  if((now-scan_since)>=TRACK_SCAN_MS)
  {
    scan_since=now;
    if(GrayLine_Read(&frame)==0U)
    {
      if(running!=0U)  line_track_stop(TRACK_ADC_STOP,now);
    }
    else if(running!=0U)  line_track_process(HAL_GetTick());
  }

  // ------- 4. 停机后才允许标定；软件串行显示期间保持 TIM6 正常抢占 -------
  LineTuning_Task((uint8_t)(running!=0U||drive_active!=0U||brake_active!=0U));
  if((HAL_GetTick()-oled_since)>=TRACK_OLED_MS)  line_track_draw();
}

// ================================================================
// 4. TIM6 速度内环与停机联锁
//
//    中断独立检查 PD3、120 s 和目标超时；异常停机使用原驱动的撤输出接口。
//    仅终点保持采用四路 command=0 的慢衰减制动，与 AT8236 的 00 滑行停机区分。
// ================================================================
void LineTrack_ControlTick(void)
{
  LineTrackStatus reason=TRACK_READY;
  uint8_t wheel;

  if(command.armed!=0U)
  {
    if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET)  reason=TRACK_DISABLED;
    else if((HAL_GetTick()-race_since)>=TRACK_RACE_LIMIT_MS)  reason=TRACK_RACE_STOP;
    else if((HAL_GetTick()-command.stamp)>=TRACK_STALE_MS)  reason=TRACK_STALE_STOP;
    if(reason!=TRACK_READY)
    {
      command.armed=0U;
      interrupt_stop=reason;
    }
  }
  if(command.armed!=0U&&command.brake!=0U)
  {
    if(brake_active==0U)
    {
      for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
      {
        MotorDrive_SetWheelDuty(wheel,MotorDrive_ForwardDirection[wheel],0U);
        WheelSpeed_Reset(wheel,&controller[wheel]);
      }
      still_ticks=0U;
      drive_active=0U;
      brake_active=1U;
    }
    else
    {
      uint8_t stationary=1U;

      // 每拍读取完整四轮，任一非零计数重置保持时间，不用延时冒充实测静止。
      for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
      {
        if(WheelSpeed_Read(wheel,&controller[wheel])!=0)  stationary=0U;
      }
      if(stationary==0U)  still_ticks=0U;
      else if(still_ticks<TRACK_FINISH_STILL_TICKS)  still_ticks++;
    }
    return;
  }
  if(command.armed==0U||command.left==0U||command.right==0U)
  {
    if(drive_active!=0U||brake_active!=0U)
    {
      MotorDrive_StopOutputs();
      drive_active=0U;
      brake_active=0U;
      still_ticks=0U;
    }
    return;
  }
  if(drive_active==0U)
  {
    for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)  WheelSpeed_Reset(wheel,&controller[wheel]);
    brake_active=0U;
    drive_active=1U;
    return;  // 下一拍取得完整 10 ms 的计数后再输出，避免首次采样时间不确定。
  }
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    uint16_t target=(wheel==0U||wheel==3U)?command.left:command.right;
    (void)WheelSpeed_Update(wheel,target,&controller[wheel]);
  }
}
