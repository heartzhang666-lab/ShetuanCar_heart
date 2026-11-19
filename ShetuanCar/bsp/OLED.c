/**
 * @file    OLED.c
 * @brief   通过 C30D 初版六针接口驱动 128×64 SSD1306 OLED。
 *
 * 文件结构：
 *   1. oled_write_byte()      — 通过 GPIO 发送命令或显示数据
 *   2. OLED_SetCursor()       — 设置页地址和像素列地址
 *   3. OLED_Clear()           — 将全部八页显示 RAM 清零
 *   4. OLED_Show*()           — 按 8×16 字模显示文本和多进制数字
 *   5. OLED_Init()            — 硬复位并配置显示方向和页寻址
 *   6. OLED_*Line/Reverse()   — 清行、填充和反白显示
 */

#include "main.h"
#include "OLED.h"
#include "OLED_Font.h"

/*
 * C30D 六针 OLED 接口没有片选脚，以下四根信号由 MCU 推挽输出，无内部上下拉：
 *   PD14（SCLK）空闲为低，OLED 在上升沿锁存数据。
 *   PD13（SDIN）先发送 bit7，最后发送 bit0。
 *   PD12（RST）低电平复位，高电平释放。
 *   PD11（DC）低电平选择命令，高电平选择显示数据。
 */
#define OLED_W_SCL(x) HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,(x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define OLED_W_SDA(x) HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,(x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define OLED_W_RST(x) HAL_GPIO_WritePin(OLED_RST_GPIO_Port,OLED_RST_Pin,(x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define OLED_W_DC(x)  HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,(x)?GPIO_PIN_SET:GPIO_PIN_RESET)

// ================================================================
// 1. 四线串行写入
//
//    ① SCLK 保持低电平 → 更新 SDIN，为上升沿建立数据。
//    ② 拉高 SCLK → SSD1306 锁存当前位，随后将下一位移到 bit7。
//    ③ 字节发送结束 → SCLK 恢复低电平，便于下一字节先切换 DC。
// ================================================================
static void oled_write_byte(uint8_t data,uint8_t is_data)
{
  uint8_t i;

  OLED_W_DC(is_data);
  for(i=0;i<8;i++)
  {
    OLED_W_SCL(0);
    OLED_W_SDA((data&0x80U)!=0U);
    OLED_W_SCL(1);
    data<<=1;
  }
  OLED_W_SCL(0);
}

static void OLED_WriteCommand(uint8_t command)
{
  oled_write_byte(command,0);
}

static void OLED_WriteData(uint8_t data)
{
  oled_write_byte(data,1);
}

// 页号 Y 为 0～7，每页高 8 像素。列号 X 为 0～127，拆成高低四位写入控制器。
static void OLED_SetCursor(uint8_t Y,uint8_t X)
{
  OLED_WriteCommand((uint8_t)(0xB0U|Y));
  OLED_WriteCommand((uint8_t)(0x10U|((X&0xF0U)>>4)));
  OLED_WriteCommand((uint8_t)(X&0x0FU));
}

void OLED_Clear(void)
{
  uint8_t i,j;

  for(j=0;j<8;j++)
  {
    OLED_SetCursor(j,0);
    for(i=0;i<128;i++)
    {
      OLED_WriteData(0x00);
    }
  }
}

void OLED_ShowChar(uint8_t Line,uint8_t Column,char Char)
{
  uint8_t i;

  // 画面只有 4 行、16 列字符，越界时跳过写入以免覆盖其他位置。
  if(Line<1U||Line>4U||Column<1U||Column>16U)  return;
  if(Char<' '||Char>'~')  Char='?';

  // 每个 8×16 字符跨两页，字模的前 8 字节是上半部，后 8 字节是下半部。
  OLED_SetCursor((uint8_t)((Line-1U)*2U),(uint8_t)((Column-1U)*8U));
  for(i=0;i<8;i++)
  {
    OLED_WriteData(OLED_F8x16[Char-' '][i]);
  }

  OLED_SetCursor((uint8_t)((Line-1U)*2U+1U),(uint8_t)((Column-1U)*8U));
  for(i=0;i<8;i++)
  {
    OLED_WriteData(OLED_F8x16[Char-' '][i+8U]);
  }
}

void OLED_ShowString(uint8_t Line,uint8_t Column,char *String)
{
  uint8_t i;

  if(String==0)  return;
  for(i=0;String[i]!='\0'&&Column+i<=16U;i++)
  {
    OLED_ShowChar(Line,(uint8_t)(Column+i),String[i]);
  }
}

static uint32_t OLED_Pow(uint32_t X,uint32_t Y)
{
  uint32_t Result=1;

  while(Y--)
  {
    Result*=X;
  }

  return Result;
}

void OLED_ShowNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length)
{
  uint8_t i;

  for(i=0;i<Length;i++)
  {
    OLED_ShowChar(Line,(uint8_t)(Column+i),(char)(Number/OLED_Pow(10U,Length-i-1U)%10U+'0'));
  }
}

void OLED_ShowSignedNum(uint8_t Line,uint8_t Column,int32_t Number,uint8_t Length)
{
  uint8_t i;
  uint32_t Number1;

  if(Number>=0)
  {
    OLED_ShowChar(Line,Column,'+');
    Number1=(uint32_t)Number;
  }
  else
  {
    OLED_ShowChar(Line,Column,'-');
    Number1=(uint32_t)(-(Number+1))+1U;
  }

  for(i=0;i<Length;i++)
  {
    OLED_ShowChar(Line,(uint8_t)(Column+i+1U),(char)(Number1/OLED_Pow(10U,Length-i-1U)%10U+'0'));
  }
}

void OLED_ShowHexNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length)
{
  uint8_t i,SingleNumber;

  for(i=0;i<Length;i++)
  {
    SingleNumber=(uint8_t)(Number/OLED_Pow(16U,Length-i-1U)%16U);
    if(SingleNumber<10U)  OLED_ShowChar(Line,(uint8_t)(Column+i),(char)(SingleNumber+'0'));
    else  OLED_ShowChar(Line,(uint8_t)(Column+i),(char)(SingleNumber-10U+'A'));
  }
}

void OLED_ShowBinNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length)
{
  uint8_t i;

  for(i=0;i<Length;i++)
  {
    OLED_ShowChar(Line,(uint8_t)(Column+i),(char)(Number/OLED_Pow(2U,Length-i-1U)%2U+'0'));
  }
}

// ================================================================
// 2. OLED 初始化
//
//    上电后 PD12 保持低电平 100ms 完成硬复位，再采用轮趣官方 C30D 例程的
//    页寻址、A1 反向段映射和 C8 反向 COM 扫描。整屏旋转 180°必须同时翻转水平 SEG 与
//    垂直 COM 两个方向；只改 C0/C8 会形成单轴翻转，使原本可读的字符再次呈镜像。
// ================================================================
void OLED_Init(void)
{
  // ------- 1. 硬复位并等待控制器就绪 -------
  OLED_W_SCL(0);
  OLED_W_SDA(0);
  OLED_W_DC(1);
  OLED_W_RST(0);
  HAL_Delay(100);  // 沿用旧驱动的 100 ms 低电平，为模块上电和复位留出时间。
  OLED_W_RST(1);
  HAL_Delay(10);   // 释放复位后等待 10 ms，再发送首条命令。

  // ------- 2. 配置面板和页寻址 -------
  OLED_WriteCommand(0xAE);  // 初始化期间关闭显示，避免随机 RAM 内容闪屏。
  OLED_WriteCommand(0xD5);
  OLED_WriteCommand(0x50);  // 官方例程的显示时钟分频与振荡频率。
  OLED_WriteCommand(0xA8);
  OLED_WriteCommand(0x3F);  // 1/64 驱动，匹配 128x64 面板。
  OLED_WriteCommand(0xD3);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0x8D);
  OLED_WriteCommand(0x14);  // 开启内部电荷泵。
  OLED_WriteCommand(0x20);
  OLED_WriteCommand(0x02);  // 页寻址模式，与 OLED_SetCursor() 的写法一致。
  OLED_WriteCommand(0xA1);  // 反转 SEG0~SEG127，配合 C8 完成 180°旋转而非单轴镜像。
  OLED_WriteCommand(0xC8);  // 反转 COM0~COM63，必须与 A1 成对使用。
  OLED_WriteCommand(0xDA);
  OLED_WriteCommand(0x12);
  OLED_WriteCommand(0x81);
  OLED_WriteCommand(0xEF);
  OLED_WriteCommand(0xD9);
  OLED_WriteCommand(0xF1);
  OLED_WriteCommand(0xDB);
  OLED_WriteCommand(0x30);
  OLED_WriteCommand(0xA4);
  OLED_WriteCommand(0xA6);
  // ------- 3. 开启正常显示并清屏 -------
  OLED_WriteCommand(0xAF);
  OLED_Clear();
}

// ================================================================
// 3. 反白显示扩展
//
//    一行文本对应两个 SSD1306 page，清行、填充和反白都按两个 page 处理。
//    反白字符对 8x16 字模逐字节取反，保持原工程菜单选中项的显示语义。
// ================================================================
void OLED_ClearLine(uint8_t Line)
{
  uint8_t i;
  uint8_t page;

  if(Line<1U||Line>4U)  return;
  page=(uint8_t)((Line-1U)*2U);

  OLED_SetCursor(page,0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0x00);
  }

  OLED_SetCursor((uint8_t)(page+1U),0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0x00);
  }
}

void OLED_FillLine(uint8_t Line)
{
  uint8_t i;
  uint8_t page;

  if(Line<1U||Line>4U)  return;
  page=(uint8_t)((Line-1U)*2U);

  OLED_SetCursor(page,0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0xFF);
  }

  OLED_SetCursor((uint8_t)(page+1U),0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0xFF);
  }
}

void OLED_ShowCharReverse(uint8_t Line,uint8_t Column,char Char)
{
  uint8_t i;

  // 画面只有 4 行、16 列字符，越界时跳过写入以免覆盖其他位置。
  if(Line<1U||Line>4U||Column<1U||Column>16U)  return;
  if(Char<' '||Char>'~')  Char='?';

  OLED_SetCursor((uint8_t)((Line-1U)*2U),(uint8_t)((Column-1U)*8U));
  for(i=0;i<8;i++)
  {
    OLED_WriteData((uint8_t)~OLED_F8x16[Char-' '][i]);
  }

  OLED_SetCursor((uint8_t)((Line-1U)*2U+1U),(uint8_t)((Column-1U)*8U));
  for(i=0;i<8;i++)
  {
    OLED_WriteData((uint8_t)~OLED_F8x16[Char-' '][i+8U]);
  }
}

void OLED_ShowStringReverse(uint8_t Line,uint8_t Column,char *String)
{
  uint8_t i;

  if(String==0)  return;
  for(i=0;String[i]!='\0'&&Column+i<=16U;i++)
  {
    OLED_ShowCharReverse(Line,(uint8_t)(Column+i),String[i]);
  }
}
