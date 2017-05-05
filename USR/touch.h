#ifndef __TOUCH_H
#define __TOUCH_H

#include "apollo.h"

#define TP_PRES_DOWN 0x80  //触屏被按下	  
#define TP_CATH_PRES 0x40  //有按键按下了 
#define CT_MAX_TOUCH  5    //电容屏支持的点数,固定为5点

static u8 CMD_RDX=0XD0;
static u8 CMD_RDY=0X90;
#define READ_TIMES 5 	//读取次数
#define LOST_VAL 1	  	//丢弃值

#define ERR_RANGE 50 //误差范围 

#define FT_TP1_REG 				0X03	  	//第一个触摸点数据地址
#define FT_TP2_REG 				0X09		//第二个触摸点数据地址
#define FT_TP3_REG 				0X0F		//第三个触摸点数据地址
#define FT_TP4_REG 				0X15		//第四个触摸点数据地址
#define FT_TP5_REG 				0X1B		//第五个触摸点数据地址  

//I2C读写命令	
#define FT_CMD_WR 				0X70    	//写命令
#define FT_CMD_RD 				0X71		//读命令
  
//FT5206 部分寄存器定义 
#define FT_DEVIDE_MODE 			0x00   		//FT5206模式控制寄存器
#define FT_REG_NUM_FINGER       0x02		//触摸状态寄存器

#define	FT_ID_G_LIB_VERSION		0xA1		//版本		
#define FT_ID_G_MODE 			0xA4   		//FT5206中断模式控制寄存器
#define FT_ID_G_THGROUP			0x80   		//触摸有效值设置寄存器
#define FT_ID_G_PERIODACTIVE	0x88   		//激活状态周期设置寄存器

//电阻屏芯片连接引脚	   
#define PEN         HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_7) //T_PEN
#define DOUT        HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3) //T_MISO

//IO操作函数	  
#define FT_RST(n)  (n?HAL_GPIO_WritePin(GPIOI,GPIO_PIN_8,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOI,GPIO_PIN_8,GPIO_PIN_RESET))//FT5206复位引脚
#define FT_INT      HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_7)  //FT5206中断引脚	
//IO方向设置
#define CT_SDA_IN()  {GPIOI->MODER&=~(3<<(3*2));GPIOI->MODER|=0<<3*2;}	//PI3输入模式
#define CT_SDA_OUT() {GPIOI->MODER&=~(3<<(3*2));GPIOI->MODER|=1<<3*2;} 	//PI3输出模式
//IO操作函数	 
#define CT_IIC_SCL(n) (n?HAL_GPIO_WritePin(GPIOH,GPIO_PIN_6,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOH,GPIO_PIN_6,GPIO_PIN_RESET))//SCL
#define CT_IIC_SDA(n) (n?HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3,GPIO_PIN_RESET))//SDA	 
#define CT_READ_SDA   HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_3)//输入SDA 



typedef struct
{
	u8 (*init)(void);			//初始化触摸屏控制器
	u8 (*scan)(void);				//扫描触摸屏.0,屏幕扫描;1,物理坐标;	
        u8 sta;
	u16 x[CT_MAX_TOUCH]; 		//当前坐标
	u16 y[CT_MAX_TOUCH];		//电容屏有最多5组坐标
}_touch_dev;


u8 FT5206_Init(void);
void CT_Delay(void);
void CT_IIC_Init(void);
void CT_IIC_Start(void);
u8 CT_IIC_Wait_Ack(void);
void CT_IIC_Stop(void);
u8 CT_IIC_Read_Byte(unsigned char ack);
void CT_IIC_Send_Byte(u8 txd);
u8 FT5206_WR_Reg(u16 reg,u8 *buf,u8 len);
void FT5206_RD_Reg(u16 reg,u8 *buf,u8 len);
u8 FT5206_Scan(void);
void CT_IIC_Ack(void);
void CT_IIC_NAck(void);
void delay_ms(uint32_t times);
void delay_us(uint32_t times);

#endif