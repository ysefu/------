#ifndef __BSP_PIN_H
#define __BSP_PIN_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：bsp_pin.h
 * 用处：全工程【唯一】的引脚与通道映射表。所有驱动 .c 文件只引用本文件的宏，
 *       禁止在驱动里直接写 GPIOA / GPIO_Pin_x 这类字面量。
 * 对应外设：GPIOA / GPIOB / TIM1_CH1 / TIM2_CH2 / USART1
 * 对应器件：1 个 LED、2 个独立按键、旋转编码器、OLED、USB-TTL、SG90 舵机、MPU6050
 * 备注：1) 本文件是【纯宏定义头文件，没有对应的 .c】，只放"位置信息"（哪个脚、哪个通道）
 *       2) 引脚电平请用标准库已有的 Bit_SET / Bit_RESET 表示，
 *          模式请用 GPIO_Mode_IPU / GPIO_Mode_IPD / GPIO_Mode_AF_PP 等，不另造符号
 *       3) 禁用脚：PB6 / PB7（本板已被占用，全工程不使用）
 *----------------------------------------------------------------------------*/

/*============================================================================
 * 一、LED（PA1共用呼吸与报警）
 *   LED1 = PA1 = TIM2_CH2：承担上电3秒变速呼吸与闹钟1Hz闪烁
 *   极性：高电平点亮（IO 经各自限流电阻接 LED 正极，负极接 GND）
 *============================================================================*/
#define LED1_PORT               GPIOA
#define LED1_PIN                GPIO_Pin_1


#define LED_ACTIVE_LEVEL        Bit_SET         /* 高电平点亮 */
#define LED1_TIM_CH             2               /* PA1 对应 TIM2 通道 2（用 TIM_SetCompare2） */

/*============================================================================
 * 二、独立按键（有且仅有两个）
 *   KEY1 = PB5 ：另一端接 VCC → 下拉输入，空闲=低、按下=高
 *   KEY2 = PB11：另一端接 GND → 上拉输入，空闲=高、按下=低
 *============================================================================*/
#define KEY1_PORT               GPIOB
#define KEY1_PIN                GPIO_Pin_5
#define KEY1_MODE               GPIO_Mode_IPD
#define KEY1_ACTIVE_LEVEL       Bit_SET         /* 按下为高 */

#define KEY2_PORT               GPIOB
#define KEY2_PIN                GPIO_Pin_11
#define KEY2_MODE               GPIO_Mode_IPU
#define KEY2_ACTIVE_LEVEL       Bit_RESET       /* 按下为低 */

/*============================================================================
 * 三、旋转编码器（只使用 A、B 两相旋转输出，不使用按压开关）
 *   A(CLK) = PB14     B(DT) = PB15
 *   采样方式：TIM3 每 500us 中断采样（见 Encoder.c）
 *============================================================================*/
#define ENC_A_PORT              GPIOB
#define ENC_A_PIN               GPIO_Pin_14
#define ENC_B_PORT              GPIOB
#define ENC_B_PIN               GPIO_Pin_15

/*============================================================================
 * 四、OLED（0.96 寸 I2C，软件 IIC）
 *   SCL = PB8    SDA = PB9
 *   注意：OLED.c 里已经写死这两个脚（用户固定方案），此处仅作记录，
 *         不要在别的模块里再配置 PB8 / PB9
 *============================================================================*/
#define OLED_SCL_PORT           GPIOB
#define OLED_SCL_PIN            GPIO_Pin_8
#define OLED_SDA_PORT           GPIOB
#define OLED_SDA_PIN            GPIO_Pin_9

/*============================================================================
 * 五、USB-TTL 串口（USART1）
 *   PA9  = USART1_TX → 接 USB-TTL 的 RXD
 *   PA10 = USART1_RX ← 接 USB-TTL 的 TXD
 *   参数：115200 bit/s，8 位数据，1 位停止，无校验
 *============================================================================*/
#define SERIAL_USART            USART1
#define SERIAL_GPIO             GPIOA
#define SERIAL_TX_PORT          GPIOA
#define SERIAL_TX_PIN           GPIO_Pin_9
#define SERIAL_RX_PORT          GPIOA
#define SERIAL_RX_PIN           GPIO_Pin_10
#define SERIAL_BAUDRATE         115200

/*============================================================================
 * 六、SG90 舵机（PWM 控制）
 *   信号线 = PA8 = TIM1_CH1
 *   舵机电源必须独立供电（不要从开发板或 ST-Link 取电），且与开发板共地
 *============================================================================*/
#define SERVO_PORT              GPIOA
#define SERVO_PIN               GPIO_Pin_8
#define SERVO_TIM               TIM1
#define SERVO_TIM_CH            1               /* TIM1 通道 1（用 TIM_SetCompare1） */

/*============================================================================
 * 七、MPU6050（软件 IIC，独立于 OLED 的第二条软件 IIC 总线）
 *   SCL = PB12    SDA = PB13
 *   AD0 接 GND → 器件地址 0x68
 *============================================================================*/
#define IMU_SCL_PORT            GPIOB
#define IMU_SCL_PIN             GPIO_Pin_12
#define IMU_SDA_PORT            GPIOB
#define IMU_SDA_PIN             GPIO_Pin_13

/*============================================================================
 * 八、光敏AO（3.3V模拟输入）
 *============================================================================*/
#define LIGHT_AO_PORT           GPIOA
#define LIGHT_AO_PIN            GPIO_Pin_0
#define LIGHT_ADC               ADC1
#define LIGHT_ADC_CHANNEL       ADC_Channel_0

/*============================================================================
 * 九、全工程禁用脚 / 空闲备用脚（开工前必看）
 *
 *   禁用脚：
 *     PB6 / PB7 —— 本板已被占用（用户实测），全工程不得使用
 *     PA13 / PA14 —— SWDIO / SWCLK，下载调试用，不得占用
 *     PA15 / PB3 / PB4 —— JTAG 复用，本方案未关闭 JTAG，不得占用
 *     PC13 / PC14 / PC15 —— 板载 RTC 与低速晶振相关，未使用
 *
 *   空闲备用脚（要加功能时优先用这些）：
 *     PA2、PA3、PA4、PA5、PA6、PA7、PA11、PA12、PB0、PB1、PB10
 *
 *   冲突检查结论：
 *     · 本项目共占用 PA0、PA1、PA8、PA9、PA10、PB5、PB8、PB9、
 *       PB11、PB12、PB13、PB14、PB15，两两不重复
 *     · 未使用 PB6 / PB7，未被 JTAG 脚占用，未使用 IIC1 默认脚
 *     · ADC 通道：ADC1_IN0/PA0=光敏AO，不占用定时器、无通道冲突
 *               TIM3 = 编码器 500us 采样 → 三个定时器互不冲突
 *============================================================================*/

#endif
