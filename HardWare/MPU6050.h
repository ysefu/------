#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：MPU6050.c / MPU6050.h
 * 用处：读取 MPU6050 的三轴陀螺仪（°/s）与三轴加速度计，并算出姿态角
 *       （IMU 数据查看页显示三轴角速度；串口 IMU ? 返回同源的姿态角，精度 0.1°）
 * 对应外设：两条普通 GPIO 模拟的软件 IIC —— SCL = PB12，SDA = PB13
 * 对应器件：MPU6050（GY-521 模块），AD0 接 GND → 器件地址 0x68
 * 备注：1) 软件 IIC 的延时决定了总线速度，本文件用 2us 半周期（约 250kHz）
 *       2) 上电必须做零偏校准（静止约 0.4 秒），否则角度会一直漂
 *       3) 姿态角用"互补滤波"：陀螺仪积分（快、会漂）+ 加速度计（慢、不漂）
 *       4) 只读一次的 14 字节连读，软件 IIC 约 2~3ms，放在 20ms 周期的任务里
 *          完全不阻塞时钟、按键、闹钟与舵机 PWM
 *----------------------------------------------------------------------------*/

/* 零偏校准采样次数（每次采样间隔 2ms，200 次约 0.4 秒，期间保持静止） */
#define MPU6050_CALI_TIMES      200

/* 驱动是否可用：1=检测到器件  0=没检测到（OLED 会显示 IMU ERR） */
typedef enum
{
	MPU_OK    = 0,                  /* 器件正常 */
	MPU_ERROR = 1                   /* 器件无响应（查线 / 查供电） */
}MPU_Status_t;

/* 仅启动期间的主循环协作回调；传0解除，不从中断调用，不允许回调重入IMU。 */
void MPU6050_SetStartupHook(void (*Hook)(void));

/* 初始化：配置软件 IIC 引脚、复位器件、设量程、读取 WHO_AM_I */
MPU_Status_t MPU6050_Init(void);

/* 读取 WHO_AM_I 寄存器（0x68 为正常值，可用于查线） */
uint8_t MPU6050_WhoAmI(void);

/* 零偏校准：必须让模块保持静止，内部采 MPU6050_CALI_TIMES 次求平均 */
void MPU6050_Calibrate(void);

/* 读一次原始数据（14 字节连读）并换算成三轴角速度和倾斜角 */
uint8_t MPU6050_ReadRaw(void);

/* 姿态解算任务：主循环里每 20ms 调一次（互补滤波 + 角度积分） */
void MPU6050_Update(void);

/* 器件是否可用：1=可用 */
uint8_t MPU6050_IsReady(void);

/* 取三轴角速度，单位 0.1°/s（如 1234 表示 123.4°/s） */
void MPU6050_GetGyroDps10(int16_t *X, int16_t *Y, int16_t *Z);

/* 取姿态角，单位 0.1°（如 -123 表示 -12.3°） */
void MPU6050_GetAngle10(int16_t *Roll, int16_t *Pitch, int16_t *Yaw);

/* 功能：查询闭环数据健康；无参数，返回1表示已校准且100ms内有有效姿态采样。 */
uint8_t MPU6050_HasFreshSample(void);
/* 功能：查询静止零偏校准是否成功；无参数，返回1成功、0失败。 */
uint8_t MPU6050_IsCalibrated(void);
/* 功能：读取成功姿态更新的样本序号；无参数，返回累计采样数。 */
uint32_t MPU6050_GetSampleCount(void);
/* 取Roll/Pitch姿态变化率（单位0.1度/秒），含侧装坐标变换，供定角阻尼。 */
void MPU6050_GetTiltRate10(int16_t *RollRate,int16_t *PitchRate);
#endif
