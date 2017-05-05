#include "rgb.h"


LTDC_HandleTypeDef  LTDC_Handler;	    //LTDC���
DMA2D_HandleTypeDef DMA2D_Handler; 	    //DMA2D���

__root uint16_t ltdc_lcd_framebuf[1280][800] @ 0xc0000000;

uint32_t *ltdc_framebuf[2];					//LTDC LCD֡��������ָ��,����ָ���Ӧ��С���ڴ�����
_ltdc_dev lcdltdc;
_lcd_dev lcddev;
uint8_t ret;
uint32_t POINT_COLOR=0xFF000000;		//������ɫ
uint32_t BACK_COLOR =0xFFFFFFFF;  	//����ɫ 
void LCD_Init(void)
{ 	  
    lcddev.id=LTDC_PanelID_Read();	//����Ƿ���RGB������
	if(lcddev.id!=0)
	{
		LTDC_Init();			    //ID����,˵����RGB������.
	}
}

void LTDC_Init(void)
{   
	uint16_t lcdid=0;
	
	lcdid=LTDC_PanelID_Read();			//��ȡLCD���ID	
	if(lcdid==0X4342)
	{
		lcdltdc.pwidth=480;			    //�����,��λ:����
		lcdltdc.pheight=272;		    //���߶�,��λ:����
		lcdltdc.hsw=1;				    //ˮƽͬ�����
		lcdltdc.vsw=1;				    //��ֱͬ�����
		lcdltdc.hbp=40;				    //ˮƽ����
		lcdltdc.vbp=8;				    //��ֱ����
		lcdltdc.hfp=5;				    //ˮƽǰ��
		lcdltdc.vfp=8;				    //��ֱǰ��
         LTDC_Clk_Set(288,4,RCC_PLLSAIDIVR_8);   //��������ʱ�� 9Mhz 
		//������������.
	}else if(lcdid==0X7084)
	{
		lcdltdc.pwidth=800;			    //�����,��λ:����
		lcdltdc.pheight=480;		    //���߶�,��λ:����
		lcdltdc.hsw=1;				    //ˮƽͬ�����
		lcdltdc.vsw=1;				    //��ֱͬ�����
		lcdltdc.hbp=46;				    //ˮƽ����
		lcdltdc.vbp=23;				    //��ֱ����
		lcdltdc.hfp=210;			    //ˮƽǰ��
		lcdltdc.vfp=22;				    //��ֱǰ��
        ret=LTDC_Clk_Set(396,3,RCC_PLLSAIDIVR_4); //��������ʱ�� 33M(�����˫��,��Ҫ����DCLK��:18.75Mhz  300/4/4,�Ż�ȽϺ�)
	}else if(lcdid==0X7016)		
	{
		lcdltdc.pwidth=1024;			//�����,��λ:����
		lcdltdc.pheight=600;			//���߶�,��λ:����
        lcdltdc.hsw=20;				    //ˮƽͬ�����
		lcdltdc.vsw=3;				    //��ֱͬ�����
		lcdltdc.hbp=140;			    //ˮƽ����
		lcdltdc.vbp=20;				    //��ֱ����
		lcdltdc.hfp=160;			    //ˮƽǰ��
		lcdltdc.vfp=12;				    //��ֱǰ��
		LTDC_Clk_Set(360,2,RCC_PLLSAIDIVR_4);//��������ʱ��  45Mhz 
		//������������.
	}else if(lcdid==0X7018)		
	{
		lcdltdc.pwidth=1280;			//�����,��λ:����
		lcdltdc.pheight=800;			//���߶�,��λ:����
		//������������.
	}else if(lcdid==0X8017)		
	{
		lcdltdc.pwidth=1024;			//�����,��λ:����
		lcdltdc.pheight=768;			//���߶�,��λ:����
		//������������.
    }

	lcddev.width=lcdltdc.pwidth;
	lcddev.height=lcdltdc.pheight;
  
    lcdltdc.pixsize=2;				//ÿ������ռ2���ֽ�
    ltdc_framebuf[0]=(uint32_t*)&ltdc_lcd_framebuf;	
    
    //LTDC����
    LTDC_Handler.Instance=LTDC;
    LTDC_Handler.Init.HSPolarity=LTDC_HSPOLARITY_AL;         //ˮƽͬ������
    LTDC_Handler.Init.VSPolarity=LTDC_VSPOLARITY_AL;         //��ֱͬ������
    LTDC_Handler.Init.DEPolarity=LTDC_DEPOLARITY_AL;         //����ʹ�ܼ���
    LTDC_Handler.Init.PCPolarity=LTDC_PCPOLARITY_IPC;        //����ʱ�Ӽ���
    LTDC_Handler.Init.HorizontalSync=lcdltdc.hsw-1;          //ˮƽͬ�����
    LTDC_Handler.Init.VerticalSync=lcdltdc.vsw-1;            //��ֱͬ�����
    LTDC_Handler.Init.AccumulatedHBP=lcdltdc.hsw+lcdltdc.hbp-1; //ˮƽͬ�����ؿ��
    LTDC_Handler.Init.AccumulatedVBP=lcdltdc.vsw+lcdltdc.vbp-1; //��ֱͬ�����ظ߶�
    LTDC_Handler.Init.AccumulatedActiveW=lcdltdc.hsw+lcdltdc.hbp+lcdltdc.pwidth-1;//��Ч���
    LTDC_Handler.Init.AccumulatedActiveH=lcdltdc.vsw+lcdltdc.vbp+lcdltdc.pheight-1;//��Ч�߶�
    LTDC_Handler.Init.TotalWidth=lcdltdc.hsw+lcdltdc.hbp+lcdltdc.pwidth+lcdltdc.hfp-1;   //�ܿ��
    LTDC_Handler.Init.TotalHeigh=lcdltdc.vsw+lcdltdc.vbp+lcdltdc.pheight+lcdltdc.vfp-1;  //�ܸ߶�
    LTDC_Handler.Init.Backcolor.Red=0;           //��Ļ�������ɫ����
    LTDC_Handler.Init.Backcolor.Green=0;         //��Ļ��������ɫ����
    LTDC_Handler.Init.Backcolor.Blue=0;          //��Ļ����ɫ��ɫ����
    HAL_LTDC_Init(&LTDC_Handler);
 	
	//������
	LTDC_Layer_Parameter_Config(0,(uint32_t)ltdc_framebuf[0],LCD_PIXFORMAT,255,0,6,7,0X000000);//���������
	LTDC_Layer_Window_Config(0,0,0,lcdltdc.pwidth,lcdltdc.pheight);	//�㴰������,��LCD�������ϵΪ��׼,��Ҫ����޸�!	
	 	
 	LTDC_Display_Dir(RGB_DIR);			//Ĭ������
	LTDC_Select_Layer(0); 			//ѡ���1��
    LCD_LED(1);         		    //��������
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    LTDC_Clear(0XFFFFFFFF);			//����
    //LTDC_Clear(BLUE);
}

//��ȡ������
//PG6=R7(M0);PI2=G7(M1);PI7=B7(M2);
//M2:M1:M0
//0 :0 :0	//4.3��480*272 RGB��,ID=0X4342
//0 :0 :1	//7��800*480 RGB��,ID=0X7084
//0 :1 :0	//7��1024*600 RGB��,ID=0X7016
//0 :1 :1	//7��1280*800 RGB��,ID=0X7018
//1 :0 :0	//8��1024*768 RGB��,ID=0X8017 
//����ֵ:LCD ID:0,�Ƿ�;����ֵ,ID;
uint16_t LTDC_PanelID_Read(void)
{
    uint8_t idx=0;
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOG_CLK_ENABLE();       //ʹ��GPIOGʱ��
    __HAL_RCC_GPIOI_CLK_ENABLE();       //ʹ��GPIOIʱ��
    
    GPIO_Initure.Pin=GPIO_PIN_6;        //PG6
    GPIO_Initure.Mode=GPIO_MODE_INPUT;  //����
    GPIO_Initure.Pull=GPIO_PULLUP;      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH; //����
    HAL_GPIO_Init(GPIOG,&GPIO_Initure); //��ʼ��
    
    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_7; //PI2,7
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);     //��ʼ��
    
    idx=(uint8_t)HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6); //��ȡM0
    idx|=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_2)<<1;//��ȡM1
    idx|=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7)<<2;//��ȡM2
	if(idx==0)return 0X4342;		//4.3����,480*272�ֱ���
	else if(idx==1)return 0X7084;	//7����,800*480�ֱ���
	else if(idx==2)return 0X7016;	//7����,1024*600�ֱ���
	else if(idx==3)return 0X7018;	//7����,1280*800�ֱ���
	else if(idx==4)return 0X8017; 	//8����,1024*768�ֱ���
	else return 0;
}

//LTDC,������������.
//ע��:�˺���,������LTDC_Layer_Window_Config֮ǰ����.
//layerx:��ֵ,0/1.
//bufaddr:����ɫ֡������ʼ��ַ
//pixformat:��ɫ��ʽ.0,ARGB8888;1,RGB888;2,RGB565;3,ARGB1555;4,ARGB4444;5,L8;6;AL44;7;AL88
//alpha:����ɫAlphaֵ,0,ȫ͸��;255,��͸��
//alpha0:Ĭ����ɫAlphaֵ,0,ȫ͸��;255,��͸��
//bfac1:���ϵ��1,4(100),�㶨��Alpha;6(101),����Alpha*�㶨Alpha
//bfac2:���ϵ��2,5(101),�㶨��Alpha;7(111),����Alpha*�㶨Alpha
//bkcolor:��Ĭ����ɫ,32λ,��24λ��Ч,RGB888��ʽ
//����ֵ:��
void LTDC_Layer_Parameter_Config(uint8_t layerx,uint32_t bufaddr,uint8_t pixformat,uint8_t alpha,uint8_t alpha0,uint8_t bfac1,uint8_t bfac2,uint32_t bkcolor)
{
	LTDC_LayerCfgTypeDef pLayerCfg;
	
	pLayerCfg.WindowX0=0;                       //������ʼX����
	pLayerCfg.WindowY0=0;                       //������ʼY����
	pLayerCfg.WindowX1=lcdltdc.pwidth;          //������ֹX����
	pLayerCfg.WindowY1=lcdltdc.pheight;         //������ֹY����
	pLayerCfg.PixelFormat=pixformat;		    //���ظ�ʽ
	pLayerCfg.Alpha=alpha;				        //Alphaֵ���ã�0~255,255Ϊ��ȫ��͸��
	pLayerCfg.Alpha0=alpha0;			        //Ĭ��Alphaֵ
	pLayerCfg.BlendingFactor1=(uint32_t)bfac1<<8;    //���ò���ϵ��
	pLayerCfg.BlendingFactor2=(uint32_t)bfac2<<8;	//���ò���ϵ��
	pLayerCfg.FBStartAdress=bufaddr;	        //���ò���ɫ֡������ʼ��ַ
	pLayerCfg.ImageWidth=lcdltdc.pwidth;        //������ɫ֡�������Ŀ��    
	pLayerCfg.ImageHeight=lcdltdc.pheight;      //������ɫ֡�������ĸ߶�
	pLayerCfg.Backcolor.Red=(uint8_t)(bkcolor&0X00FF0000)>>16;   //������ɫ��ɫ����
	pLayerCfg.Backcolor.Green=(uint8_t)(bkcolor&0X0000FF00)>>8;  //������ɫ��ɫ����
	pLayerCfg.Backcolor.Blue=(uint8_t)bkcolor&0X000000FF;        //������ɫ��ɫ����
	HAL_LTDC_ConfigLayer(&LTDC_Handler,&pLayerCfg,layerx);   //������ѡ�еĲ�
} 

//LTDC,���մ�������,������LCD�������ϵΪ��׼
//ע��:�˺���������LTDC_Layer_Parameter_Config֮��������.
//layerx:��ֵ,0/1.
//sx,sy:��ʼ����
//width,height:��Ⱥ͸߶�
void LTDC_Layer_Window_Config(uint8_t layerx,uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
    HAL_LTDC_SetWindowPosition(&LTDC_Handler,sx,sy,layerx);  //���ô��ڵ�λ��
    HAL_LTDC_SetWindowSize(&LTDC_Handler,width,height,layerx);//���ô��ڴ�С    
}

//����LCD��ʾ����
//dir:0,������1,����
void LTDC_Display_Dir(uint8_t dir)
{
    lcdltdc.dir=dir; 	//��ʾ����
	if(dir==0)			//����
	{
		lcdltdc.width=lcdltdc.pheight;
		lcdltdc.height=lcdltdc.pwidth;	
	}else if(dir==1)	//����
	{
		lcdltdc.width=lcdltdc.pwidth;
		lcdltdc.height=lcdltdc.pheight;
	}
}

//ѡ���
//layerx:���;0,��һ��;1,�ڶ���;
void LTDC_Select_Layer(uint8_t layerx)
{
	lcdltdc.activelayer=layerx;
}

//LCD����
//color:��ɫֵ
void LTDC_Clear(uint32_t color)
{
	LTDC_Fill(0,0,lcdltdc.width-1,lcdltdc.height-1,color);
}

//LTDC������,DMA2D���
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//ע��:sx,ex,���ܴ���lcddev.width-1;sy,ey,���ܴ���lcddev.height-1!!!
//color:Ҫ������ɫ
void LTDC_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint32_t color)
{ 
	uint32_t psx,psy,pex,pey;	//��LCD���Ϊ��׼������ϵ,����������仯���仯
	static uint32_t timeout=0; 
	uint16_t offline;
	uint32_t addr; 
	//����ϵת��
	if(lcdltdc.dir)	//����
	{
		psx=sx;psy=sy;
		pex=ex;pey=ey;
	}else			//����
	{
		psx=sy;psy=lcdltdc.pheight-ex-1;
		pex=ey;pey=lcdltdc.pheight-sx-1;
	} 
	offline=lcdltdc.pwidth-(pex-psx+1);
	addr=((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*psy+psx));
	RCC->AHB1ENR|=1<<23;			//ʹ��DM2Dʱ��
	DMA2D->CR=3<<16;				//�Ĵ������洢��ģʽ
	DMA2D->OPFCCR=LCD_PIXFORMAT;	//������ɫ��ʽ
	DMA2D->OOR=offline;				//������ƫ�� 
	DMA2D->CR&=~(1<<0);				//��ֹͣDMA2D
	DMA2D->OMAR=addr;				//����洢����ַ
	DMA2D->NLR=(pey-psy+1)|((pex-psx+1)<<16);	//�趨�����Ĵ���
	DMA2D->OCOLR=color;				//�趨�����ɫ�Ĵ��� 
	DMA2D->CR|=1<<0;				//����DMA2D
	while((DMA2D->ISR&(1<<1))==0)	//�ȴ��������
	{
		timeout++;
		if(timeout>0X1FFFFF)break;	//��ʱ�˳�
	}  
	DMA2D->IFCR|=1<<1;				//���������ɱ�־ 	
}
//Fvco:VCOƵ��
//Fin:����ʱ��Ƶ��һ��Ϊ1Mhz(����ϵͳʱ��PLLM��Ƶ���ʱ��,��ʱ����ͼ)
//pllsain:SAIʱ�ӱ�Ƶϵ��N,ȡֵ��Χ:50~432.  
//pllsair:SAIʱ�ӵķ�Ƶϵ��R,ȡֵ��Χ:2~7
//pllsaidivr:LCDʱ�ӷ�Ƶϵ��,ȡֵ��Χ:0~3,��Ӧ��Ƶ2~16 
//����:�ⲿ����Ϊ25M,pllm=25��ʱ��,Fin=1Mhz.
//����:Ҫ�õ�20M��LTDCʱ��,���������:pllsain=400,pllsair=5,pllsaidivr=1
//Fdclk=1*396/3/2*2^1=396/12=33Mhz
//����ֵ:0,�ɹ�;1,ʧ�ܡ�
uint8_t LTDC_Clk_Set(uint32_t pllsain,uint32_t pllsair,uint32_t pllsaidivr)
{
	RCC_PeriphCLKInitTypeDef PeriphClkIniture;
	
	//LTDC�������ʱ�ӣ���Ҫ�����Լ���ʹ�õ�LCD�����ֲ������ã�
        PeriphClkIniture.PeriphClockSelection=RCC_PERIPHCLK_LTDC;	//LTDCʱ�� 	
	PeriphClkIniture.PLLSAI.PLLSAIN=pllsain;    
	PeriphClkIniture.PLLSAI.PLLSAIR=pllsair;  
	PeriphClkIniture.PLLSAIDivR=pllsaidivr;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkIniture)==HAL_OK) //��������ʱ�ӣ���������Ϊʱ��Ϊ18.75MHZ
    {
        return 0;   //�ɹ�
    }
    else return 1;  //ʧ��    
}

void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p)
{         
	uint8_t x0=x;
	width+=x;
	height+=y;
        //LTDC_Fill(x,y,width,height,BACK_COLOR);
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//�˳�
        LCD_ShowChar(x,y,*p,size,0);
        //LCD_ShowChar(x,y,*p,size,1);
        x+=size/2;
        p++;
    }  
}

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{  							  
    uint8_t temp,t1,t;
	uint16_t y0=y;
	uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���	
 	num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[num][t];	//����1608����
		else if(size==24)temp=asc2_2412[num][t];	//����2412����
		else if(size==32)temp=asc2_3216[num][t];	//����3216����
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LTDC_Draw_Point(x,y,POINT_COLOR);
			else if(mode==0)LTDC_Draw_Point(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}

//���㺯��
//x,y:����
//color:��ɫֵ
void LTDC_Draw_Point(uint16_t x,uint16_t y,uint32_t color)
{ 
#if LCD_PIXFORMAT==LCD_PIXFORMAT_ARGB8888||LCD_PIXFORMAT==LCD_PIXFORMAT_RGB888
	if(lcdltdc.dir)	//����
	{
        *(uint32_t*)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*y+x))=color;
	}else 			//����
	{
        *(uint32_t*)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*(lcdltdc.pheight-x-1)+y))=color; 
	}
#else
	if(lcdltdc.dir)	//����
	{
        *(uint16_t*)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*y+x))=color;
	}else 			//����
	{
        *(uint16_t*)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*(lcdltdc.pheight-x-1)+y))=color; 
	}
#endif
}

//LTDC�ײ�IO��ʼ����ʱ��ʹ��
//�˺����ᱻHAL_LTDC_Init()����
//hltdc:LTDC���
void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_LTDC_CLK_ENABLE();                //ʹ��LTDCʱ��
    __HAL_RCC_DMA2D_CLK_ENABLE();               //ʹ��DMA2Dʱ��
    __HAL_RCC_GPIOB_CLK_ENABLE();               //ʹ��GPIOBʱ��
    __HAL_RCC_GPIOF_CLK_ENABLE();               //ʹ��GPIOFʱ��
    __HAL_RCC_GPIOG_CLK_ENABLE();               //ʹ��GPIOGʱ��
    __HAL_RCC_GPIOH_CLK_ENABLE();               //ʹ��GPIOHʱ��
    __HAL_RCC_GPIOI_CLK_ENABLE();               //ʹ��GPIOIʱ��
    
    //��ʼ��PB5����������
    GPIO_Initure.Pin=GPIO_PIN_5;                //PB5������������Ʊ���
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //�������
    GPIO_Initure.Pull=GPIO_PULLUP;              //����        
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    //��ʼ��PF10
    GPIO_Initure.Pin=GPIO_PIN_10; 
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //����
    GPIO_Initure.Pull=GPIO_NOPULL;              
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
    GPIO_Initure.Alternate=GPIO_AF14_LTDC;      //����ΪLTDC
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
    
    //��ʼ��PG6,7,11
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11;
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
    
    //��ʼ��PH9,10,11,12,13,14,15
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|\
                     GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    
    //��ʼ��PI0,1,2,4,5,6,7,9,10
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|\
                     GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
    HAL_GPIO_Init(GPIOI,&GPIO_Initure); 
}

/*
*added by iysheng@163.com 
*posx,posy:ͼ�����ʼ������
*uint8_t* ͼ�������յ�ַ
*/
void APPOLO_RGB(uint16_t posx,uint16_t posy,const unsigned char *pic)
{
 uint32_t width,high,piex,uitemp[2];
 if(pic[0]&0x01)
 {
   width=(pic[2]<<8)|pic[3];
   high=(pic[4]<<8)|pic[5];
   for(uitemp[0]=0;uitemp[0]<width;uitemp[0]++)
    {
      for(uitemp[1]=0;uitemp[1]<high*2;)
      {
      piex=((pic[(high<<1)*uitemp[0]+uitemp[1]])<<8)|(pic[(high<<1)*uitemp[0]+uitemp[1]+1]);
      LTDC_Draw_Point(uitemp[0]+posx,posy+uitemp[1]/2,(uint32_t)(piex));
      uitemp[1]+=2;
      }
    }
 }
   
 
}