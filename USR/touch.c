#include "touch.h"

const u16 FT5206_TPX_TBL[5]={FT_TP1_REG,FT_TP2_REG,FT_TP3_REG,FT_TP4_REG,FT_TP5_REG};
_touch_dev tp_dev={
    FT5206_Init,
    FT5206_Scan,
    0,
    0,
    0,
};

inline void delay_ms(uint32_t times)
{
  HAL_Delay(times);
}

void delay_us(uint32_t times)
{
  uint32_t count,local_time;
  for(local_time=0;local_time<times;local_time++)
  for(count=0;count<0xffff;count++);
}

//控制I2C速度的延时
void CT_Delay(void)
{
    delay_us(2);
} 

//电容触摸芯片IIC接口初始化
void CT_IIC_Init(void)
{					     
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOH_CLK_ENABLE();			//开启GPIOH时钟
    __HAL_RCC_GPIOI_CLK_ENABLE();			//开启GPIOI时钟
    
    GPIO_Initure.Pin=GPIO_PIN_6;            //PH6 SCL
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
	
    GPIO_Initure.Pin=GPIO_PIN_3;            //PI3 SDA
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);     //初始化
}

void CT_IIC_Start(void)
{
	CT_SDA_OUT();     //sda线输出
	CT_IIC_SDA(1);	  	  
	CT_IIC_SCL(1);
	delay_us(30);
 	CT_IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	CT_Delay();
	CT_IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 CT_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	CT_SDA_IN();      //SDA设置为输入  
	CT_IIC_SDA(1);	   
	CT_IIC_SCL(1);
	CT_Delay();
	while(CT_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			CT_IIC_Stop();
			return 1;
		} 
		CT_Delay();
	}
	CT_IIC_SCL(0);//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void CT_IIC_Ack(void)
{
	CT_IIC_SCL(0);
	CT_SDA_OUT();
	CT_Delay();
	CT_IIC_SDA(0);
	CT_Delay();
	CT_IIC_SCL(1);
	CT_Delay();
	CT_IIC_SCL(0);
}
//不产生ACK应答		    
void CT_IIC_NAck(void)
{
	CT_IIC_SCL(0);
	CT_SDA_OUT();
	CT_Delay();
	CT_IIC_SDA(1);
	CT_Delay();
	CT_IIC_SCL(1);
	CT_Delay();
	CT_IIC_SCL(0);
}	

//产生IIC停止信号
void CT_IIC_Stop(void)
{
	CT_SDA_OUT();//sda线输出
	CT_IIC_SCL(1);
	delay_us(30);
	CT_IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
	CT_Delay();
	CT_IIC_SDA(1);//发送I2C总线结束信号  
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 CT_IIC_Read_Byte(unsigned char ack)
{
	u8 i,receive=0;
 	CT_SDA_IN();//SDA设置为输入
	delay_us(30);
	for(i=0;i<8;i++ )
	{ 
		CT_IIC_SCL(0); 	    	   
		CT_Delay();
		CT_IIC_SCL(1);	 
		receive<<=1;
		if(CT_READ_SDA)receive++;   
	}	  				 
	if (!ack)CT_IIC_NAck();//发送nACK
	else CT_IIC_Ack(); //发送ACK   
 	return receive;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void CT_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	CT_SDA_OUT(); 	    
    CT_IIC_SCL(0);//拉低时钟开始数据传输
	CT_Delay();
	for(t=0;t<8;t++)
    {              
        CT_IIC_SDA((txd&0x80)>>7);
        txd<<=1; 	      
		CT_IIC_SCL(1); 
		CT_Delay();
		CT_IIC_SCL(0);	
		CT_Delay();
    }	 
} 

//向FT5206写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
u8 FT5206_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	 
	CT_IIC_Send_Byte(FT_CMD_WR);	//发送写命令 	 
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	CT_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}

//从FT5206读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void FT5206_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(FT_CMD_WR);   	//发送写命令 	 
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(FT_CMD_RD);   	//发送读命令		   
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    CT_IIC_Stop();//产生一个停止条件     
} 


u8 FT5206_Init(void)
{
    u8 temp[2]; 
    GPIO_InitTypeDef GPIO_Initure;
 
    __HAL_RCC_GPIOH_CLK_ENABLE();			//开启GPIOH时钟
    __HAL_RCC_GPIOI_CLK_ENABLE();			//开启GPIOI时钟
                
    //PH7  INT
    GPIO_Initure.Pin=GPIO_PIN_7;            //PH7
    //GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    //GPIO_Initure.Mode=GPIO_MODE_IT_RISING;
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //初始化
            
    //PI8   CS
    GPIO_Initure.Pin=GPIO_PIN_8;            //PI8
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);     //初始化
        
	CT_IIC_Init();      	//初始化电容屏的I2C总线  
	FT_RST(0);				//复位
	delay_ms(20);
 	FT_RST(1);				//释放复位		    
	delay_ms(50);  	
	temp[0]=0;
	FT5206_WR_Reg(FT_DEVIDE_MODE,temp,1);	//进入正常操作模式 
	FT5206_WR_Reg(FT_ID_G_MODE,temp,1);		//查询模式 
	temp[0]=22;								//触摸有效值，22，越小越灵敏	
	FT5206_WR_Reg(FT_ID_G_THGROUP,temp,1);	//设置触摸有效值
	temp[0]=12;								//激活周期，不能小于12，最大14
	FT5206_WR_Reg(FT_ID_G_PERIODACTIVE,temp,1); 
	//读取版本号，参考值：0x3003
	FT5206_RD_Reg(FT_ID_G_LIB_VERSION,&temp[0],2);  
	if((temp[0]==0X30&&temp[1]==0X03)||temp[1]==0X01||temp[1]==0X02)//版本:0X3003/0X0001/0X0002
	{
		printf("CTP ID:%x\r\n",((u16)temp[0]<<8)+temp[1]);
		return 0;
	} 
	return 1;
}

u8 FT5206_Scan(void)
{
  u8 mode,temp,i;
  u8 buf[4];
  FT5206_RD_Reg(FT_REG_NUM_FINGER,&mode,1);//读取触摸点的状态  
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);//将点的个数转换为1的位数,匹配tp_dev.sta定义,最多为5个1 0x1f 
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES;
                        temp=mode&0XF;
			for(i=0;i<temp;i++)
			{
				//if(tp_dev.sta&(1<<i))		//触摸有效?
				//{
					FT5206_RD_Reg(FT5206_TPX_TBL[i],buf,4);	//读取XY坐标值
					tp_dev.y[i]=((u16)(buf[0]&0X0F)<<8)+buf[1];
					tp_dev.x[i]=((u16)(buf[2]&0X0F)<<8)+buf[3];
					//printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);
				//}			
			} 
		}
  return 0;
}
