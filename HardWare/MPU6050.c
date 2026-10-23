#include "stm32f10x.h"                  // Device header
#include <math.h>
#include "bsp_pin.h"
#include "Delay.h"
#include "MPU6050.h"
#include "Timer.h"

/*----------------------------------------------------------------------------
 * 文件：MPU6050.c / MPU6050.h
 * 用处：软件 IIC 驱动 MPU6050，输出三轴角速度与姿态角
 * 对应外设：软件 IIC（SCL = PB12，SDA = PB13，都配成开漏输出）
 * 对应器件：MPU6050 / GY-521，AD0 接 GND → 地址 0x68
 * 备注：1) 【为什么用软件 IIC 而不是硬件 IIC】软件 IIC 时序完全可控、不挑引脚，
 *          OLED 已经用了 PB8/PB9 的软件 IIC，这里再开一条独立的 PB12/PB13，
 *          两条总线互不干扰（若挂在同一条总线上，OLED 刷新会打断 IMU 的连读）
 *       2) 【上拉电阻】GY-521 模块自带的 4.7k 上拉就够用，不用外接；
 *          如果你的模块没有上拉，总线会一直读不到数据（现象：WHO_AM_I=0）
 *       3) 【关键时序】读写寄存器都没有死等，全部带超时计数，器件没接也不会卡死
 *       4) 陀螺仪量程 ±2000°/s → 灵敏度 16.4 LSB/(°/s)
 *          加速度计量程 ±2g      → 灵敏度 16384 LSB/g
 *       5) 互补滤波系数：0.98 信陀螺仪（快）、0.02 信加速度计（稳）
 *      6) 调用顺序：MPU6050_Init() → MPU6050_ReadRaw() → 静止 →
 *          MPU6050_Calibrate() → 之后循环 MPU6050_Update()
 *----------------------------------------------------------------------------*/

/* ---- MPU6050 寄存器地址（芯片手册里的固定值，不是我们自造的引脚/位掩码） ---- */
#define MPU6050_ADDR            0x68
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CFG    0x1B
#define MPU6050_REG_ACCEL_CFG   0x1C
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_WHO_AM_I    0x75
#define MPU6050_REG_ACCEL_XOUT  0x3B

#define MPU6050_WHOAMI_VALUE    0x68    /* 正常应读回 0x68（部分兼容片为 0x70/0x71/0x72/0x73） */

#define MPU6050_IIC_DELAY_US    2       /* 软件 IIC 半周期延时（约 250kHz） */
#define MPU6050_ACK_TIMEOUT     6000    /* 等 ACK 的超时计数，超时说明器件没应答 */
#define MPU6050_BUS_TIMEOUT     12000   /* 等 SCL 被释放的超时计数（从机时钟拉伸兜底） */

#define MPU6050_GYRO_LSB_PER_DPS    16.4f
#define MPU6050_ACCEL_LSB_PER_G     16384.0f
#define MPU6050_COMP_ALPHA          0.98f   /* 互补滤波：陀螺仪权重 */

/*============================================================================
 * 一、软件 IIC 底层（只在本文件内部使用，全部 static）
 *============================================================================*/

/**
  * @brief  内部使用：SCL 引脚输出电平
  * @param  Level Bit_SET=高  Bit_RESET=低
  * @retval 无
  */
static void IIC_SclWrite(BitAction Level)
{
	GPIO_WriteBit(IMU_SCL_PORT,IMU_SCL_PIN,Level);
}

/**
  * @brief  内部使用：SDA 引脚输出电平
  * @param  Level Bit_SET=高  Bit_RESET=低
  * @retval 无
  */
static void IIC_SdaWrite(BitAction Level)
{
	GPIO_WriteBit(IMU_SDA_PORT,IMU_SDA_PIN,Level);
}

/**
  * @brief  内部使用：读 SDA 电平（主机接收从机数据用）
  * @param  无
  * @retval Bit_SET=高  Bit_RESET=低
  */
static BitAction IIC_SdaRead(void)
{
	return GPIO_ReadInputDataBit(IMU_SDA_PORT,IMU_SDA_PIN);
}

/**
  * @brief  内部使用：等 SCL 真的变成高电平（有的从机会拉伸时钟）
  * @param  无
  * @retval 无
  */
static void IIC_WaitSclHigh(void)
{
	uint16_t Timeout = MPU6050_BUS_TIMEOUT;

	while(GPIO_ReadInputDataBit(IMU_SCL_PORT,IMU_SCL_PIN) == Bit_RESET)
	{
		Timeout--;
		if(Timeout == 0)
		{
			break;                  //超时就算了，绝不在这里死等
		}
	}
}

/**
  * @brief  内部使用：IIC 起始条件（SCL 高时 SDA 由高变低）
  * @param  无
  * @retval 无
  */
static void IIC_Start(void)
{
	IIC_SdaWrite(Bit_SET);
	IIC_SclWrite(Bit_SET);
	Delay_us(MPU6050_IIC_DELAY_US);
	IIC_SdaWrite(Bit_RESET);
	Delay_us(MPU6050_IIC_DELAY_US);
	IIC_SclWrite(Bit_RESET);
	Delay_us(MPU6050_IIC_DELAY_US);
}

/**
  * @brief  内部使用：IIC 停止条件（SCL 高时 SDA 由低变高）
  * @param  无
  * @retval 无
  */
static void IIC_Stop(void)
{
	IIC_SdaWrite(Bit_RESET);
	IIC_SclWrite(Bit_SET);
	IIC_WaitSclHigh();
	Delay_us(MPU6050_IIC_DELAY_US);
	IIC_SdaWrite(Bit_SET);
	Delay_us(MPU6050_IIC_DELAY_US);
}

/**
  * @brief  内部使用：发送一个字节，并读从机的应答
  * @param  Byte 要发送的字节
  * @retval 0=收到应答(ACK)   1=没有应答(NACK)或超时
  */
static uint8_t IIC_SendByte(uint8_t Byte)
{
	uint8_t i;
	uint8_t Ack;
	uint16_t Timeout;

	for(i=0;i<8;i++)
	{
		IIC_SdaWrite((Byte & 0x80) ? Bit_SET : Bit_RESET);
		Byte <<= 1;
		Delay_us(MPU6050_IIC_DELAY_US);
		IIC_SclWrite(Bit_SET);
		IIC_WaitSclHigh();
		Delay_us(MPU6050_IIC_DELAY_US);
		IIC_SclWrite(Bit_RESET);
		Delay_us(MPU6050_IIC_DELAY_US);
	}

	/* 第 9 个时钟：读从机应答 */
	IIC_SdaWrite(Bit_SET);                  //开漏输出高 = 释放总线，交给从机拉低
	Delay_us(MPU6050_IIC_DELAY_US);
	IIC_SclWrite(Bit_SET);
	IIC_WaitSclHigh();
	Delay_us(MPU6050_IIC_DELAY_US);

	Ack = 1;
	Timeout = MPU6050_ACK_TIMEOUT;
	while(Timeout > 0)
	{
		if(IIC_SdaRead() == Bit_RESET)      //被从机拉低 → 应答
		{
			Ack = 0;
			break;
		}
		Timeout--;
	}

	IIC_SclWrite(Bit_RESET);
	Delay_us(MPU6050_IIC_DELAY_US);

	return Ack;
}

/**
  * @brief  内部使用：接收一个字节
  * @param  Ack 主机收到后要不要回应答：1=回 ACK（还要继续读），0=回 NACK（最后一个字节）
  * @retval 收到的字节
  */
static uint8_t IIC_ReceiveByte(uint8_t Ack)
{
	uint8_t i;
	uint8_t Byte = 0;

	IIC_SdaWrite(Bit_SET);                  //释放 SDA 由从机驱动

	for(i=0;i<8;i++)
	{
		Byte <<= 1;
		IIC_SclWrite(Bit_SET);
		IIC_WaitSclHigh();
		Delay_us(MPU6050_IIC_DELAY_US);

		if(IIC_SdaRead() == Bit_SET)
		{
			Byte |= 0x01;
		}

		IIC_SclWrite(Bit_RESET);
		Delay_us(MPU6050_IIC_DELAY_US);
	}

	/* 第 9 个时钟：主机回应答 */
	IIC_SdaWrite(Ack ? Bit_RESET : Bit_SET);
	Delay_us(MPU6050_IIC_DELAY_US);
	IIC_SclWrite(Bit_SET);
	IIC_WaitSclHigh();
	Delay_us(MPU6050_IIC_DELAY_US);
	IIC_SclWrite(Bit_RESET);
	IIC_SdaWrite(Bit_SET);
	Delay_us(MPU6050_IIC_DELAY_US);

	return Byte;
}

/**
  * @brief  内部使用：向 MPU6050 的某个寄存器写一个字节
  * @param  Reg   寄存器地址
  * @param  Value 要写入的值
  * @retval 0=成功   1=器件没应答
  */
static uint8_t MPU_WriteReg(uint8_t Reg, uint8_t Value)
{
	uint8_t Err = 0;

	IIC_Start();
	Err |= IIC_SendByte((uint8_t)(MPU6050_ADDR << 1) | 0x00);   //地址 + 写
	Err |= IIC_SendByte(Reg);
	Err |= IIC_SendByte(Value);
	IIC_Stop();

	return Err ? 1 : 0;
}

/**
  * @brief  内部使用：从 MPU6050 连续读若干个寄存器
  * @param  Reg    起始寄存器地址
  * @param  Buf    读出的数据存放处
  * @param  Length 读几个字节
  * @retval 0=成功   1=器件没应答
  */
static uint8_t MPU_ReadRegs(uint8_t Reg, uint8_t *Buf, uint8_t Length)
{
	uint8_t i;
	uint8_t Err = 0;

	IIC_Start();
	Err |= IIC_SendByte((uint8_t)(MPU6050_ADDR << 1) | 0x00);   //地址 + 写
	Err |= IIC_SendByte(Reg);

	IIC_Start();                                                //重复起始条件
	Err |= IIC_SendByte((uint8_t)(MPU6050_ADDR << 1) | 0x01);   //地址 + 读

	for(i=0;i<Length;i++)
	{
		Buf[i] = IIC_ReceiveByte((i < (Length-1)) ? 1 : 0);     //最后一个字节回 NACK
	}

	IIC_Stop();

	return Err ? 1 : 0;
}

/*============================================================================
 * 二、模块状态（只在本文件内部使用）
 *============================================================================*/
static uint8_t MpuCalibrated=0,MpuHaveSample=0,MpuAngleInitialized=0;
static uint32_t MpuLastSampleTick=0,MpuSampleCount=0;
static uint8_t  MpuReady    = 0;            /* 1=器件正常 */
static uint8_t  MpuWhoAmI   = 0;            /* WHO_AM_I 读回值（查线用） */

static int16_t  AccRaw[3]   = {0,0,0};      /* 加速度原始值 X/Y/Z */
static int16_t  GyroRaw[3]  = {0,0,0};      /* 陀螺仪原始值 X/Y/Z */
static int16_t  GyroBias[3] = {0,0,0};      /* 陀螺仪零偏（校准得到） */

static float    AngleRoll  = 0.0f;          /* 绕 X 轴姿态角（度） */
static float    AnglePitch = 0.0f;          /* 绕 Y 轴姿态角（度） */
static float    AngleYaw   = 0.0f;          /* 绕 Z 轴积分角（度，会缓慢漂移，属正常） */

/*============================================================================
 * 三、对外接口
 *============================================================================*/

/**
  * @brief  初始化：配置软件 IIC 引脚、唤醒器件、设量程、读 WHO_AM_I
  * @param  无
  * @retval MPU_OK=器件正常   MPU_ERROR=没有检测到器件
  */
static void (*MpuStartupHook)(void)=0;
void MPU6050_SetStartupHook(void (*Hook)(void)){MpuStartupHook=Hook;}
/* 器件稳定等待拆成1ms片段；每次总线交易结束后才推进OLED和呼吸灯。 */
static void MPU_StartupWait(uint32_t Ms)
{
 while(Ms--){Delay_ms(1);if(MpuStartupHook)MpuStartupHook();}
}
MPU_Status_t MPU6050_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	/* SCL/SDA 都配成"开漏输出"：输出低能拉低总线，输出高等于释放总线，
	   由上拉电阻（GY-521 模块自带 4.7k）把电平拉到 3.3V */
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin=IMU_SCL_PIN | IMU_SDA_PIN;
	GPIO_Init(IMU_SCL_PORT,&GPIO_InitStruct);

	IIC_SdaWrite(Bit_SET);
	IIC_SclWrite(Bit_SET);
	MPU_StartupWait(50);                               //等模块内部上电稳定

	MpuReady = 0;
	MpuCalibrated=0; MpuHaveSample=0; MpuAngleInitialized=0; MpuSampleCount=0;
	MPU_WriteReg(MPU6050_REG_PWR_MGMT_1,0x80);  //复位器件
	MPU_StartupWait(100);
	MPU_WriteReg(MPU6050_REG_PWR_MGMT_1,0x01);  //退出睡眠，选 PLL 时钟
	MPU_StartupWait(50);

	if(MPU_WriteReg(MPU6050_REG_SMPLRT_DIV,9) != 0)     //1kHz/(1+9) = 100Hz 采样率
	{
		return MPU_ERROR;
	}
	MPU_WriteReg(MPU6050_REG_CONFIG,0x03);              //3 阶低通，44Hz 带宽
	MPU_WriteReg(MPU6050_REG_GYRO_CFG,0x18);            //陀螺仪 ±2000°/s
	MPU_WriteReg(MPU6050_REG_ACCEL_CFG,0x00);           //加速度计 ±2g

	MpuWhoAmI = MPU6050_WhoAmI();

	/* 兼容常见的国产替代片：0x68 是正品，0x70~0x73 是兼容型号，都算可用 */
	if(MpuWhoAmI == MPU6050_WHOAMI_VALUE ||
	   (MpuWhoAmI >= 0x70 && MpuWhoAmI <= 0x73))
	{
		MpuReady = 1;
	}
	else
	{
		MpuReady = 0;
	}

	return MpuReady ? MPU_OK : MPU_ERROR;
}

/**
  * @brief  读取 WHO_AM_I 寄存器
  * @param  无
  * @retval 读回的值（正常为 0x68；读回 0x00 或 0xFF 说明总线没通）
  */
uint8_t MPU6050_WhoAmI(void)
{
	uint8_t Value = 0;

	MPU_ReadRegs(MPU6050_REG_WHO_AM_I,&Value,1);

	return Value;
}

/**
  * @brief  读一次原始数据（14 字节连读），并算出加速度计的倾斜角
  * @param  无
  * @retval 0=成功   1=读失败（器件没应答）
  */
uint8_t MPU6050_ReadRaw(void)
{
	uint8_t Buf[14];
	uint8_t i;

	if(MpuReady == 0)
	{
		return 1;
	}

	if(MPU_ReadRegs(MPU6050_REG_ACCEL_XOUT,Buf,14) != 0)
	{
		return 1;
	}

	/* 寄存器顺序：AX AH AL AY AH AL AZ AH AL TEMP AH AL GX AH AL GY AH AL GZ AH AL */
	for(i=0;i<3;i++)
	{
		AccRaw[i]  = (int16_t)((Buf[i*2] << 8) | Buf[i*2+1]);
		GyroRaw[i] = (int16_t)((Buf[8+i*2] << 8) | Buf[8+i*2+1]);
	}

	return 0;
}

/**
  * @brief  零偏校准：模块保持静止，采多个样本求平均作为零偏
  * @param  无
  * @retval 无
  * 备注：校准时如果手还在动，角度就会一直漂 —— 一定要放稳
  */
void MPU6050_Calibrate(void)
{
	uint16_t i,Count=0; uint8_t Axis;
	int32_t Sum[3]={0,0,0}; float Squares[3]={0,0,0};
	MpuCalibrated=0; MpuHaveSample=0; MpuAngleInitialized=0;
	if(!MpuReady) return;
	for(i=0;i<MPU6050_CALI_TIMES;i++)
	{
		if(MPU6050_ReadRaw()==0)
		{
			Count++;
			for(Axis=0;Axis<3;Axis++)
			{Sum[Axis]+=GyroRaw[Axis]; Squares[Axis]+=(float)GyroRaw[Axis]*(float)GyroRaw[Axis];}
		}
		Delay_ms(2);
		if(MpuStartupHook)MpuStartupHook(); /* 当前样本已读完，无IIC事务交叠。 */
	}
	if(Count<MPU6050_CALI_TIMES*3u/4u) return;
	for(Axis=0;Axis<3;Axis++)
	{
		float Mean=(float)Sum[Axis]/(float)Count;
		float Variance=Squares[Axis]/(float)Count-Mean*Mean;
		/* 校准中明显晃动则不将错误零偏用于闭环；稳态仍需用户放稳。 */
		if(Variance>36.0f*MPU6050_GYRO_LSB_PER_DPS*MPU6050_GYRO_LSB_PER_DPS) return;
	}
	for(Axis=0;Axis<3;Axis++) GyroBias[Axis]=(int16_t)(Sum[Axis]/Count);
	AngleRoll=0.0f; AnglePitch=0.0f; AngleYaw=0.0f;
	MpuCalibrated=1;
}


/**
  * @brief  姿态解算任务：主循环每 20ms 调一次（互补滤波）
  * @param  无
  * @retval 无
  * 备注：加速度计只能算出 Roll/Pitch（没有绝对的偏航基准），
  *       Yaw 只能靠陀螺仪积分，会缓慢漂移，这是 MPU6050 的固有限制
  */
/* 机体轴角速度转换为321欧拉角变化率。侧装时Gy不等于Pitch变化率。
 * |Pitch|接近90度时欧拉角奇异；限制分母防止数值发散，不能消除奇异性。
 * 本机工作区约Pitch=-9度，定角行程限制为启动输出附近25度。
 */
static void MPU_GetEulerRates(float *RollRate,float *PitchRate,float *YawRate)
{
 float R=AngleRoll*0.01745329252f, P=AnglePitch*0.01745329252f;
 float Sr=sinf(R),Cr=cosf(R),Cp=cosf(P),Coupled;
 float Gx=(float)(GyroRaw[0]-GyroBias[0])/MPU6050_GYRO_LSB_PER_DPS;
 float Gy=(float)(GyroRaw[1]-GyroBias[1])/MPU6050_GYRO_LSB_PER_DPS;
 float Gz=(float)(GyroRaw[2]-GyroBias[2])/MPU6050_GYRO_LSB_PER_DPS;
 if(fabsf(Cp)<0.1f) Cp=Cp<0.0f ? -0.1f : 0.1f;
 Coupled=Sr*Gy+Cr*Gz;
 *RollRate=Gx+Coupled*sinf(P)/Cp;
 *PitchRate=Cr*Gy-Sr*Gz;
 *YawRate=Coupled/Cp;
}

void MPU6050_Update(void)
{
	float Gx,Gy,Gz,Ax,Ay,Az,Norm2,Roll,Pitch,Dt,Alpha; uint32_t Now,Elapsed;
	if(!MpuReady || !MpuCalibrated || MPU6050_ReadRaw()!=0) return;
	Now=Timer_GetTick(); Elapsed=Now-MpuLastSampleTick;
	Dt=MpuHaveSample ? (float)Elapsed*0.001f : 0.02f;
	if(MpuHaveSample && Elapsed==0) return;
	if(Dt>0.1f) {MpuAngleInitialized=0; Dt=0.02f;}
	Ax=(float)AccRaw[0]/MPU6050_ACCEL_LSB_PER_G;
	Ay=(float)AccRaw[1]/MPU6050_ACCEL_LSB_PER_G;
	Az=(float)AccRaw[2]/MPU6050_ACCEL_LSB_PER_G;
	Norm2=Ax*Ax+Ay*Ay+Az*Az;
	Roll=atan2f(Ay,Az)*57.29578f;
	Pitch=atan2f(-Ax,sqrtf(Ay*Ay+Az*Az))*57.29578f;
	if(!MpuAngleInitialized)
	{
		if(Norm2<0.49f || Norm2>1.69f) return;
		AngleRoll=Roll; AnglePitch=Pitch; MpuAngleInitialized=1;
		MPU_GetEulerRates(&Gx,&Gy,&Gz);
	}
	else
	{
		MPU_GetEulerRates(&Gx,&Gy,&Gz);
		/* 实际采样间隔；先转换侧装机体角速度，再预测姿态。 */
		float RollPrediction=AngleRoll+Gx*Dt;
		float Correction=Roll-RollPrediction;
		while(Correction>180.0f) Correction-=360.0f;
		while(Correction< -180.0f) Correction+=360.0f;
		Alpha=(Norm2>=0.49f && Norm2<=1.69f) ? 0.5f/(0.5f+Dt) : 1.0f;
		AngleRoll=RollPrediction+(1.0f-Alpha)*Correction;
		while(AngleRoll>180.0f) AngleRoll-=360.0f;
		while(AngleRoll< -180.0f) AngleRoll+=360.0f;
		AnglePitch=Alpha*(AnglePitch+Gy*Dt)+(1.0f-Alpha)*Pitch;
	}
	AngleYaw+=Gz*Dt;
	while(AngleYaw>180.0f) AngleYaw-=360.0f;
	while(AngleYaw< -180.0f) AngleYaw+=360.0f;
	MpuLastSampleTick=Now; MpuHaveSample=1; MpuSampleCount++;
}


/**
  * @brief  器件是否可用
  * @param  无
  * @retval 1=可用   0=没检测到
  */
uint8_t MPU6050_IsReady(void)
{
	return MpuReady;
}

/**
  * @brief  取三轴角速度，单位 0.1°/s
  * @param  X 输出：X 轴角速度（×10 的整数）
  * @param  Y 输出：Y 轴角速度（×10 的整数）
  * @param  Z 输出：Z 轴角速度（×10 的整数）
  * @retval 无
  * 备注：这是"实时角速度"，静止时应接近 0，转动时对应轴明显变化
  */
void MPU6050_GetGyroDps10(int16_t *X, int16_t *Y, int16_t *Z)
{
	float Temp;

	if(MpuReady == 0)
	{
		if(X) *X = 0;
		if(Y) *Y = 0;
		if(Z) *Z = 0;
		return;
	}

	Temp = (float)(GyroRaw[0] - GyroBias[0]) * 10.0f / MPU6050_GYRO_LSB_PER_DPS;
	if(X) *X = (int16_t)Temp;

	Temp = (float)(GyroRaw[1] - GyroBias[1]) * 10.0f / MPU6050_GYRO_LSB_PER_DPS;
	if(Y) *Y = (int16_t)Temp;

	Temp = (float)(GyroRaw[2] - GyroBias[2]) * 10.0f / MPU6050_GYRO_LSB_PER_DPS;
	if(Z) *Z = (int16_t)Temp;
}

/**
  * @brief  取姿态角，单位 0.1°
  * @param  Roll  输出：横滚角（×10 的整数）
  * @param  Pitch 输出：俯仰角（×10 的整数）
  * @param  Yaw   输出：偏航积分角（×10 的整数）
  * @retval 无
  * 备注：串口命令 "IMU ?" 返回的就是这三个值，与 OLED 同源
  */
void MPU6050_GetAngle10(int16_t *Roll, int16_t *Pitch, int16_t *Yaw)
{
	if(Roll)  *Roll  = (int16_t)(AngleRoll  * 10.0f);
	if(Pitch) *Pitch = (int16_t)(AnglePitch * 10.0f);
	if(Yaw)   *Yaw   = (int16_t)(AngleYaw   * 10.0f);
}

uint8_t MPU6050_HasFreshSample(void)
{
	return MpuReady && MpuCalibrated && MpuHaveSample && Timer_GetTick()-MpuLastSampleTick<=100u;
}
uint8_t MPU6050_IsCalibrated(void) {return MpuCalibrated;}
uint32_t MPU6050_GetSampleCount(void) {return MpuSampleCount;}

/* 与姿态解算使用同一坐标变换，原始GX/GY/GZ显示接口保持为机体轴角速度。 */
void MPU6050_GetTiltRate10(int16_t *RollRate,int16_t *PitchRate)
{
 float R=0.0f,P=0.0f,Y;
 if(MpuReady && MpuCalibrated && MpuAngleInitialized) MPU_GetEulerRates(&R,&P,&Y);
 if(R>3276.7f) R=3276.7f;
 if(R< -3276.7f) R=-3276.7f;
 if(P>3276.7f) P=3276.7f;
 if(P< -3276.7f) P=-3276.7f;
 if(RollRate) *RollRate=(int16_t)(R*10.0f);
 if(PitchRate) *PitchRate=(int16_t)(P*10.0f);
}
