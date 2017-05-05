#include "apollo.h"
#include "rgb.h"
#include "touch.h"

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define GETCHAR_PROTOTYPE int __io_getchar(int ch)
#else
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */


UART_HandleTypeDef IUART;
TIM_HandleTypeDef ITIM2,ITIM3,ITIM5;
TIM_OC_InitTypeDef IConfig;
ADC_HandleTypeDef ICEKONG;
DMA_HandleTypeDef IDMA_ADC;
SDRAM_HandleTypeDef ISDRAM;
FMC_SDRAM_TimingTypeDef ISDRAM_Timing;
char rstr[RSTR_SIZE];
uint32_t led_flag;
uint32_t ic_value;//捕获的计数值
uint8_t ic_state;//捕获的状态值
uint8_t lcd_led_flag=1;
uint32_t key_qd[3];
uint16_t dma_adc_flag;
uint64_t hole_ic_value;
uint8_t  scanf_ch;


void LED_init(void)
{
  GPIO_InitTypeDef  GPIO_Initure;
  __HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOB时钟  	
  GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5; //PB0,1,PB5RGB背光
  GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
  GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
  GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
  HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //初始化GPIOB.0和GPIOB.1
}

void LED_on(void)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0置1,熄灭
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1置0,点亮
}

void LED_off(void)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置1,熄灭
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置0,点亮
}

void KEY_init(void)
{
  GPIO_InitTypeDef GPIO_Initure;
  __HAL_RCC_GPIOH_CLK_ENABLE();	
  GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3;
  GPIO_Initure.Mode=GPIO_MODE_IT_RISING;
  GPIO_Initure.Pull=GPIO_PULLUP;
  GPIO_Initure.Speed=GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOH,&GPIO_Initure);
}

void UART_init(UART_HandleTypeDef *UARTX)
{
  UARTX->Instance=USART2;
  UARTX->Init.BaudRate=9600;
  UARTX->Init.WordLength=UART_WORDLENGTH_8B;
  UARTX->Init.StopBits=UART_STOPBITS_1;
  UARTX->Init.Parity=UART_PARITY_NONE;
  UARTX->Init.Mode=UART_MODE_TX_RX;
  UARTX->Init.HwFlowCtl=UART_HWCONTROL_NONE;
  HAL_UART_Init(UARTX);
}

/**
* @brief  CPU L1-Cache enable.
* @param  None
* @retval None
*/
void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();
  /* Enable D-Cache */
  SCB_EnableDCache();
  //SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题
}

#if 0
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }

  /* Activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    while(1) {};
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    while(1) {};
  }
}
#endif
void SystemClock_Config(void)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_OscInitTypeDef RCC_OscInitStructure; 
    RCC_ClkInitTypeDef RCC_ClkInitStructure;
	
    __HAL_RCC_PWR_CLK_ENABLE(); //使能PWR时钟
 
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//设置调压器输出电压级别，以便在器件未以最大频率工作
      
    RCC_OscInitStructure.OscillatorType=RCC_OSCILLATORTYPE_HSE;    //时钟源为HSE
    RCC_OscInitStructure.HSEState=RCC_HSE_ON;                      //打开HSE
    RCC_OscInitStructure.PLL.PLLState=RCC_PLL_ON;				   //打开PLL
    RCC_OscInitStructure.PLL.PLLSource=RCC_PLLSOURCE_HSE;          //PLL时钟源选择HSE
    RCC_OscInitStructure.PLL.PLLM=25;	//主PLL和音频PLL分频系数(PLL之前的分频)
    RCC_OscInitStructure.PLL.PLLN=432; //主PLL倍频系数(PLL倍频)
    RCC_OscInitStructure.PLL.PLLP=2; //系统时钟的主PLL分频系数(PLL之后的分频)
    RCC_OscInitStructure.PLL.PLLQ=9; //USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频)
    ret=HAL_RCC_OscConfig(&RCC_OscInitStructure);//初始化
    if(ret!=HAL_OK) while(1);
    
    ret=HAL_PWREx_EnableOverDrive(); //开启Over-Driver功能
    if(ret!=HAL_OK) while(1);
    
    //选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2
    RCC_ClkInitStructure.ClockType=(RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStructure.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;//设置系统时钟时钟源为PLL
    RCC_ClkInitStructure.AHBCLKDivider=RCC_SYSCLK_DIV1;//AHB分频系数为1
    RCC_ClkInitStructure.APB1CLKDivider=RCC_HCLK_DIV4;//APB1分频系数为4
    RCC_ClkInitStructure.APB2CLKDivider=RCC_HCLK_DIV2;//APB2分频系数为2
    
    ret=HAL_RCC_ClockConfig(&RCC_ClkInitStructure,FLASH_LATENCY_7);//同时设置FLASH延时周期为7WS，也就是8个CPU周期。
    if(ret!=HAL_OK) while(1);
}


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&IUART, (uint8_t *)&ch, 1, 0xFFFF); 
  return ch;
}

GETCHAR_PROTOTYPE
{
  HAL_UART_Receive(&IUART,(uint8_t *)&scanf_ch, 1, 0xFFFF);
  return  scanf_ch;
}

void MPU_set_protection(uint32_t addr,uint8_t size,uint8_t num,uint8_t ap)
{
  MPU_Region_InitTypeDef MPU_Initure;
  HAL_MPU_Disable();
  MPU_Initure.BaseAddress=addr;
  MPU_Initure.Enable=MPU_REGION_ENABLE;
  MPU_Initure.Size=size;
  MPU_Initure.SubRegionDisable=0x00;
  MPU_Initure.TypeExtField=MPU_TEX_LEVEL0;
  MPU_Initure.AccessPermission=ap;
  MPU_Initure.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_Initure.IsShareable=MPU_ACCESS_NOT_SHAREABLE;
  MPU_Initure.IsCacheable=MPU_ACCESS_NOT_CACHEABLE;
  MPU_Initure.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_Initure);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);  
}

void MPU_init(void)
{
  MPU_set_protection(0X20000000,MPU_REGION_SIZE_512KB,MPU_REGION_NUMBER0,MPU_REGION_FULL_ACCESS);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef  GPIO_Initure;
  if(hadc->Instance==ADC1)
  {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();//使能DMA2时钟
  GPIO_Initure.Mode=GPIO_MODE_ANALOG;
  GPIO_Initure.Pull=GPIO_NOPULL;
  GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;
  GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA,&GPIO_Initure);//PA4\PA5\PA6

  IDMA_ADC.Instance=DMA2_Stream0;
  IDMA_ADC.Init.Channel=DMA_CHANNEL_0;
  IDMA_ADC.Init.Direction=DMA_PERIPH_TO_MEMORY;
  IDMA_ADC.Init.PeriphInc=DMA_PINC_DISABLE;
  IDMA_ADC.Init.MemInc=DMA_MINC_ENABLE;  
  IDMA_ADC.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;
  IDMA_ADC.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;
  IDMA_ADC.Init.Mode=DMA_CIRCULAR;
  IDMA_ADC.Init.Priority=DMA_PRIORITY_LOW;
  IDMA_ADC.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&IDMA_ADC);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);//开启DMA2中断
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0x0, 2); 
  
  __HAL_LINKDMA(&ICEKONG,DMA_Handle,IDMA_ADC);  
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    dma_adc_flag|=0x01;
  }
}

void DMA2_Stream0_IRQHandler(void)
{
   HAL_DMA_IRQHandler(&IDMA_ADC);
}

void CEKONG_init(void)
{
  ADC_ChannelConfTypeDef Iconfig;
  __HAL_RCC_ADC1_CLK_ENABLE();
  ICEKONG.Instance = ADC1;
  ICEKONG.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;//APB2=108MHz
  ICEKONG.Init.Resolution = ADC_RESOLUTION_12B;//12bit转换结果
  ICEKONG.Init.ScanConvMode = ENABLE;
  ICEKONG.Init.ContinuousConvMode = DISABLE;
  ICEKONG.Init.DiscontinuousConvMode = DISABLE;
  ICEKONG.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ICEKONG.Init.NbrOfConversion = IDAC_COUNT;
  ICEKONG.Init.DMAContinuousRequests = ENABLE;
  ICEKONG.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  ICEKONG.Init.ExternalTrigConv=ADC_EXTERNALTRIGCONV_T2_TRGO;//定时器2驱动ADC周期转换
  ICEKONG.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;
  HAL_ADC_Init(&ICEKONG);
  
  Iconfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  Iconfig.Rank = 1;
  Iconfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  
  HAL_ADC_ConfigChannel(&ICEKONG,&Iconfig);
  
  Iconfig.Channel = ADC_CHANNEL_4;//PA4
  Iconfig.Rank = 2;
  Iconfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  
  HAL_ADC_ConfigChannel(&ICEKONG,&Iconfig);
  
  Iconfig.Channel = ADC_CHANNEL_5;//PA5
  Iconfig.Rank = 3;
  Iconfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&ICEKONG,&Iconfig);
  
  Iconfig.Channel = ADC_CHANNEL_6;//PA6
  Iconfig.Rank = 4;
  Iconfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&ICEKONG,&Iconfig);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_Initure;
  if(huart->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();//USART2时钟使能
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_2;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;
    GPIO_Initure.Pull=GPIO_NOPULL;
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;
    GPIO_Initure.Alternate=GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA,&GPIO_Initure); 
    GPIO_Initure.Pin=GPIO_PIN_3;
    GPIO_Initure.Alternate=GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);   
  }
  else if(huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();//USART2时钟使能
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_10;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;
    GPIO_Initure.Pull=GPIO_NOPULL;
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;
    GPIO_Initure.Alternate=GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    GPIO_Initure.Pin=GPIO_PIN_9; 
    GPIO_Initure.Alternate=GPIO_AF7_USART1;
  }
}

/*
void TIM_init(TIM_HandleTypeDef *ITIMX)
{
  ITIMX->Instance=TIM3;
  ITIMX->Init.Period=999;//周期为1000ms
  ITIMX->Init.Prescaler=53999;//采样时钟频率为1ms
  ITIMX->Init.CounterMode=TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(ITIMX); 
  HAL_TIM_Base_Start_IT(ITIMX);
}*/

void TIM2_init(void)
{
  TIM_MasterConfigTypeDef iMasterConfig;
  __HAL_RCC_TIM2_CLK_ENABLE();
  memset(&iMasterConfig,0,sizeof(iMasterConfig));
  ITIM2.Instance=TIM2;
  ITIM2.Init.Period=1999;//周期为1000ms,频率1Hz
  ITIM2.Init.Prescaler=53999;//APB1=108MHz,采样时钟间隔为500us，频率为2khz
  ITIM2.Init.CounterMode=TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&ITIM2); 
  iMasterConfig.MasterOutputTrigger=TIM_TRGO_UPDATE;
  iMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&ITIM2,&iMasterConfig);
  HAL_TIM_Base_Start(&ITIM2);
}

void TIM3_init(void)
{
  TIM_OC_InitTypeDef iConfig;
  memset(&iConfig,0,sizeof(iConfig));
  __HAL_RCC_TIM3_CLK_ENABLE();
  ITIM3.Instance=TIM3;
  ITIM3.Init.Period=999;//周期为9999--5000ms，频率为0.2hz
  ITIM3.Init.Prescaler=53999;//采样时钟间隔为500us，频率为2khz
  ITIM3.Init.CounterMode=TIM_COUNTERMODE_UP;
  HAL_TIM_PWM_Init(&ITIM3);
  iConfig.OCMode=TIM_OCMODE_PWM1;
  iConfig.Pulse=499;
  iConfig.OCPolarity=TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&ITIM3,&iConfig,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&ITIM3,TIM_CHANNEL_4);
  //HAL_TIM_PWM_ConfigChannel(&ITIM3,&iConfig,TIM_CHANNEL_3); //PB0
  //HAL_TIM_PWM_Start(&ITIM3,TIM_CHANNEL_3);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef  GPIO_Initure;
  if(htim->Instance==TIM3)
  {
  __HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOB时钟  	
  GPIO_Initure.Pin=GPIO_PIN_1; //PB1--CH4 PB0--CH3
  GPIO_Initure.Mode=GPIO_MODE_AF_PP;  //复用推挽输出
  GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
  GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
  GPIO_Initure.Alternate=GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //GPIOB1、TIM3的4通道，LED0
  }
}

void TIM5_init(void)
{
  TIM_IC_InitTypeDef IC_Config;
  memset(&IC_Config,0,sizeof(IC_Config));
  ITIM5.Instance=TIM5;
  ITIM5.Init.Prescaler=107;//107--精度达到1us
  ITIM5.Init.CounterMode=TIM_COUNTERMODE_UP;
  ITIM5.Init.Period=0xffffffff;
  ITIM5.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&ITIM5);
  IC_Config.ICPolarity=TIM_ICPOLARITY_RISING;
  IC_Config.ICPrescaler=TIM_ICPSC_DIV1;
  IC_Config.ICFilter=0;
  IC_Config.ICSelection=TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_IC_ConfigChannel(&ITIM5,&IC_Config,TIM_CHANNEL_1);//TIM_CHANNEL_1
  HAL_TIM_IC_Start_IT(&ITIM5,TIM_CHANNEL_1);//开启中断
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef  GPIO_Initure;
  if(htim->Instance==TIM5)
  {
    __HAL_RCC_TIM5_CLK_ENABLE();
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_NVIC_SetPriority(TIM5_IRQn, 0x0, 2);  //开启TIM5中断
    __HAL_RCC_GPIOA_CLK_ENABLE();		//开启GPIOA时钟  	
    GPIO_Initure.Pin=GPIO_PIN_0; //PA0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //GPIOA0、TIM5的1通道
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  LED_init();
  if(htim->Instance==TIM2){
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0x0, 2);
  }else if(htim->Instance==TIM3){
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0x0, 3);
  } 
}

void SDRAM_init(void)
{
  ISDRAM.Instance=FMC_SDRAM_DEVICE;
  ISDRAM.Init.SDBank=FMC_SDRAM_BANK1;
  ISDRAM.Init.ColumnBitsNumber=FMC_SDRAM_COLUMN_BITS_NUM_9;
  ISDRAM.Init.RowBitsNumber=FMC_SDRAM_ROW_BITS_NUM_13;
  ISDRAM.Init.MemoryDataWidth=FMC_SDRAM_MEM_BUS_WIDTH_16;
  ISDRAM.Init.InternalBankNumber=FMC_SDRAM_INTERN_BANKS_NUM_4;
  ISDRAM.Init.CASLatency=FMC_SDRAM_CAS_LATENCY_3;
  ISDRAM.Init.WriteProtection=FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  ISDRAM.Init.SDClockPeriod=FMC_SDRAM_CLOCK_PERIOD_2;
  ISDRAM.Init.ReadBurst=FMC_SDRAM_RBURST_ENABLE;
  ISDRAM.Init.ReadPipeDelay=FMC_SDRAM_RPIPE_DELAY_1;
  ISDRAM_Timing.LoadToActiveDelay=2;
  ISDRAM_Timing.ExitSelfRefreshDelay=8;
  ISDRAM_Timing.SelfRefreshTime=6;
  ISDRAM_Timing.RowCycleDelay=6;
  ISDRAM_Timing.WriteRecoveryTime=2;
  ISDRAM_Timing.RPDelay=2;
  ISDRAM_Timing.RCDDelay=2;
  HAL_SDRAM_Init(&ISDRAM,&ISDRAM_Timing);  
  SDRAM_Initialization_Sequence(&ISDRAM);
}

void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram)
{
  GPIO_InitTypeDef GPIO_Initure;
  __HAL_RCC_FMC_CLK_ENABLE();                 //使能FMC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();               //使能GPIOC时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();               //使能GPIOD时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();               //使能GPIOE时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();               //使能GPIOF时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();               //使能GPIOG时钟
    
    //初始化PC0,2,3
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;  
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    GPIO_Initure.Alternate=GPIO_AF12_FMC;       //复用为FMC    
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);         //初始化
    
    
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOD,&GPIO_Initure); //初始化PD0,1,8,9,10,14,15
    
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOE,&GPIO_Initure); //初始化PE0,1,7,8,9,10,11,12,13,14,15
    
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOF,&GPIO_Initure); //初始化PF0,1,2,3,4,5,11,12,13,14,15
    
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_15;              
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);	//初始化PG0,1,2,4,5,8,15 
  
}

//发送SDRAM初始化序列
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram)
{
    uint32_t temp=0;
    //SDRAM控制器初始化完成以后还需要按照如下顺序初始化SDRAM
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_CLK_ENABLE,1,0); //时钟配置使能
    HAL_Delay(1);                                  //至少延时1MS
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_PALL,1,0);       //对所有存储区预充电
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_AUTOREFRESH_MODE,8,0);//设置自刷新次数 
    //配置模式寄存器,SDRAM的bit0~bit2为指定突发访问的长度，
	//bit3为指定突发访问的类型，bit4~bit6为CAS值，bit7和bit8为运行模式
	//bit9为指定的写突发模式，bit10和bit11位保留位
	temp=(uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |	//设置突发长度:1(可以是1/2/4/8)
              SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |	//设置突发类型:连续(可以是连续/交错)
              SDRAM_MODEREG_CAS_LATENCY_3           |	//设置CAS值:3(可以是2/3)
              SDRAM_MODEREG_OPERATING_MODE_STANDARD |   //设置操作模式:0,标准模式
              SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;     //设置突发写模式:1,单点访问
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_LOAD_MODE,1,temp);   //设置SDRAM的模式寄存器
    
    //刷新频率计数器(以SDCLK频率计数),计算方法:
	//COUNT=SDRAM刷新周期/行数-20=SDRAM刷新周期(us)*SDCLK频率(Mhz)/行数
    //我们使用的SDRAM刷新周期为64ms,SDCLK=216/2=108Mhz,行数为8192(2^13).
	//所以,COUNT=64*1000*108/8192-20=823
	HAL_SDRAM_ProgramRefreshRate(&ISDRAM,823);	

}	

uint8_t SDRAM_Send_Cmd(uint8_t bankx,uint8_t cmd,uint8_t refresh,uint16_t regval)
{
    uint32_t target_bank=0;
    FMC_SDRAM_CommandTypeDef Command;
    
    if(bankx==0) target_bank=FMC_SDRAM_CMD_TARGET_BANK1;       
    else if(bankx==1) target_bank=FMC_SDRAM_CMD_TARGET_BANK2;   
    Command.CommandMode=cmd;                //命令
    Command.CommandTarget=target_bank;      //目标SDRAM存储区域
    Command.AutoRefreshNumber=refresh;      //自刷新次数
    Command.ModeRegisterDefinition=regval;  //要写入模式寄存器的值
    if(HAL_SDRAM_SendCommand(&ISDRAM,&Command,0X1000)==HAL_OK) //向SDRAM发送命令
    {
        return 0;  
    }
    else return 1;    
}

//在指定地址(WriteAddr+Bank5_SDRAM_ADDR)开始,连续写入n个字节.
//pBuffer:字节指针
//WriteAddr:要写入的地址
//n:要写入的字节数
void FMC_SDRAM_WriteBuffer(uint8_t *pBuffer,uint32_t WriteAddr,uint32_t n)
{
	for(;n!=0;n--)
	{
		*(uint8_t*)(Bank5_SDRAM_ADDR+WriteAddr)=*pBuffer;
		WriteAddr++;
		pBuffer++;
	}
}

//在指定地址((WriteAddr+Bank5_SDRAM_ADDR))开始,连续读出n个字节.
//pBuffer:字节指针
//ReadAddr:要读出的起始地址
//n:要写入的字节数
void FMC_SDRAM_ReadBuffer(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t n)
{
	for(;n!=0;n--)
	{
		*pBuffer++=*(uint8_t*)(Bank5_SDRAM_ADDR+ReadAddr);
		ReadAddr++;
	}
}

/*定时器溢出中断处理函数*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM5)
  {
    if((ic_state&0x40)==0x40)
    {
      ic_state++;//溢出次数加 1
    }        
  }
  else if(htim->Instance==TIM3)
  {
    //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//DS1
  }
}

void TIM3_IRQHandler(void){
  HAL_TIM_IRQHandler(&ITIM3);
}

void TIM5_IRQHandler(void){
  HAL_TIM_IRQHandler(&ITIM5);
}

/*输入捕获中断的回调函数*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM5)
  {
    switch(ic_state)
    {
    case 0x00:ic_state=0x40;ic_value=0x00;__HAL_TIM_DISABLE(htim);__HAL_TIM_SET_COUNTER(htim,0);__HAL_TIM_ENABLE(htim);break;//捕获第一个中断
    case 0x40:ic_state|=0x80;ic_value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);break;//捕获第二个中断，完成一次周期捕获
    default:break;
    }
  }
}

/*KEY1*/
void EXTI2_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//DS1
}

/*KEY0*/
void EXTI3_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  /*感觉去抖效果不大*/
  /*
  key_qd[key_qd[2]]=HAL_GetTick();
  key_qd[2]++;
  if(key_qd[2]==2)
  { if((key_qd[1]-key_qd[0])>30)
    {
      LCD_LED(++lcd_led_flag%2);
    }
   key_qd[2]=0;
  }*/
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);//LCD_LED
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//DS1
}
/*PH7 上升沿 触摸屏*/
void EXTI9_5_IRQHandler(void) {
  FT5206_Scan();
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
}

void USART2_IRQHandler(void) {
  HAL_UART_IRQHandler(&IUART);
}

void MemManage_Handler(void)
{
  printf("MPU opened!\r\n");
  NVIC_SystemReset();
}
