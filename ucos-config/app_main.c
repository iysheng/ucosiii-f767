/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                          STM32746G-EVAL2
*                                         Evaluation Board
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <stdarg.h>
#include  <stdio.h>
#include  <math.h>
#include  <stm32f7xx_hal.h>

#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>

#include  <app_cfg.h>
#include  <bsp.h>
#include  <bsp_led.h>
#include  <bsp_clock.h>

#include "apollo.h"
#include "rgb.h"
#include "xiong.h"
#include "touch.h"

/*
*
*added from gpio pro
*/
extern UART_HandleTypeDef IUART;
extern TIM_HandleTypeDef ITIM3;
extern TIM_HandleTypeDef ITIM5;
extern TIM_OC_InitTypeDef IConfig;
extern ADC_HandleTypeDef ICEKONG;
extern DMA_HandleTypeDef IDMA_ADC;
extern uint32_t POINT_COLOR;		//������ɫ
extern uint32_t BACK_COLOR;             //������ɫ
extern char rstr[RSTR_SIZE];
extern uint16_t dma_adc_flag;
extern uint64_t hole_ic_value;//����ļ���ֵ
extern uint32_t ic_value;
extern uint8_t ic_state;//�����״ֵ̬

extern _touch_dev tp_dev;

uint16_t raw_icekong[IDAC_COUNT];

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  APP_TASK_EQ_0_ITERATION_NBR              16u
#define  APP_TASK_EQ_1_ITERATION_NBR              18u


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

                                                                /* --------------- APPLICATION GLOBALS ---------------- */
static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

                                                                /* --------------- SEMAPHORE TASK TEST --------------- */
static  OS_TCB       AppTaskObj0TCB;
static  CPU_STK      AppTaskObj0Stk[APP_CFG_TASK_OBJ_STK_SIZE];

static  OS_TCB       AppTaskObj1TCB;
static  CPU_STK      AppTaskObj1Stk[APP_CFG_TASK_OBJ_STK_SIZE];   

static  OS_TCB       AppTaskDisplayTCB;
static  CPU_STK      AppTaskDisplayStk[APP_CFG_TASK_OBJ_STK_SIZE];   /*RGB display task*/

#if (OS_CFG_SEM_EN > 0u)
//static  OS_SEM       AppTaskObjSem;
#endif

#if (OS_CFG_MUTEX_EN > 0u)
//static  OS_MUTEX     AppTaskObjMutex;
#endif

#if (OS_CFG_Q_EN > 0u)
//static  OS_Q         AppTaskObjQ;
#endif

#if (OS_CFG_FLAG_EN > 0u)
//static  OS_FLAG_GRP  AppTaskObjFlag;
#endif

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg);
static  void  AppTaskCreate(void);

static  void  AppTaskObj0  (void  *p_arg);
static  void  AppTaskObj1  (void  *p_arg);
static  void  AppTaskDisplay  (void  *p_arg);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*
* Notes       : 1) HAL library initialization:
*                      a) Configures the Flash prefetch, intruction and data caches.
*                      b) Configures the Systick to generate an interrupt. However, the function ,
*                         HAL_InitTick(), that initializes the Systick has been overwritten since Micrium's
*                         RTOS has its own Systick initialization and it is recommended to initialize the
*                         Systick after multitasking has started.
*
*********************************************************************************************************
*/

int main(void)
{
    OS_ERR   err;

    HAL_Init();                                                 /* See Note 1.                                          */

    BSP_SystemClkCfg();                                         /* Initialize CPU clock frequency to 216Mhz             */

    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    Mem_Init();                                                 /* Initialize Memory Managment Module                   */
    Math_Init();                                                /* Initialize Mathematical Module                       */

    CPU_IntDis();                                               /* Disable all Interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    App_OS_SetAllHooks();

    OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
                  "App Task Start",
                  AppTaskStart,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &err);
    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    while (DEF_ON) {                                            /* Should Never Get Here.                               */
    
    }
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    OS_ERR  err;


   (void)p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

   //APP_TRACE_DBG(("Creating Application kernel objects\n\r"));
   //AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

   APP_TRACE_DBG(("Creating Application Tasks\n\r"));
   AppTaskCreate();                                            /* Create Application tasks                             */
   OSTaskDel((OS_TCB *)0, &err);//ɾ���Լ�
}

/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create Application Tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
    OS_ERR  os_err;
    
    HAL_ADC_Start_DMA(&ICEKONG,(uint32_t *)raw_icekong,IDAC_COUNT);
    
                                                                /* ---------- CREATE KERNEL OBJECTS TEST TASK --------- */
    OSTaskCreate(&AppTaskObj0TCB,
                 "Kernel Objects Task 0",
                  AppTaskObj0,
                  0,
                  APP_CFG_TASK_OBJ_PRIO,
                 &AppTaskObj0Stk[0],
                  AppTaskObj0Stk[APP_CFG_TASK_OBJ_STK_SIZE / 10u],
                  APP_CFG_TASK_OBJ_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    OSTaskCreate(&AppTaskObj1TCB,
                 "Kernel Objects Task 1",
                  AppTaskObj1,
                  0,
                  APP_CFG_TASK_OBJ1_PRIO,
                 &AppTaskObj1Stk[0],
                  AppTaskObj0Stk[APP_CFG_TASK_OBJ_STK_SIZE / 10u],
                  APP_CFG_TASK_OBJ_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    OSTaskCreate(&AppTaskDisplayTCB,
                 "RGB Display Task",
                  AppTaskDisplay,
                  0,
                  APP_CFG_TASK_DISPLAY_PRIO,
                 &AppTaskDisplayStk[0],
                  AppTaskDisplayStk[APP_CFG_TASK_OBJ_STK_SIZE / 10u],
                  APP_CFG_TASK_OBJ_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
}

/*
*********************************************************************************************************
*                                          AppTaskObj()
*
* Description : Test uC/OS-III objects.
*
* Argument(s) : p_arg is the argument passed to 'AppTaskObj' by 'OSTaskCreate()'.
*
* Return(s)   : none
*
* Caller(s)   : This is a task
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskObj0(void  *p_arg)
{
  OS_ERR  os_err;
  (void)p_arg;
  while (DEF_TRUE) {
           BSP_LED_Off(0u);
           APP_TRACE_DBG(("%s\r\n",AppTaskObj0TCB.NamePtr));
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
  }
}

static  void  AppTaskObj1(void  *p_arg)
{
  OS_ERR  os_err;
  (void)p_arg;
  while (DEF_TRUE) {
           BSP_LED_On(0u);
           APP_TRACE_DBG(("%s\r\n",AppTaskObj1TCB.NamePtr));
               OSTimeDlyHMSM(0u, 0u, 2u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
        
  }
}

static  void  AppTaskDisplay(void  *p_arg)
{ 
  OS_ERR  os_err;
  static uint8_t uline,upoint;
  static float ftemp;
  static uint32_t uitemp;
  
  (void)p_arg;
  BACK_COLOR=WHITE;
  LTDC_Clear(WHITE);
  POINT_COLOR=RED;
  APPOLO_RGB(0,0,gImage_xiong);
  while (DEF_TRUE) {
    if(dma_adc_flag<<7){
    dma_adc_flag=0X00;
    while(uline<4){
    switch(uline)
    {
    case 0:{sprintf((char *)rstr,"PA4:%4dKg",raw_icekong[1]);break;}
    case 1:{sprintf((char *)rstr,"PA5:%4dN.m",raw_icekong[2]);break;}
    case 2:{sprintf((char *)rstr,"PA6:%4dcm",raw_icekong[3]);break;}
    case 3:{uitemp=raw_icekong[0];ftemp=((float)uitemp)/4095*3300;ftemp=((ftemp-760.0)/2.5)+25;
            sprintf((char *)rstr,"%0.3f",ftemp);break;}
    default:break;
    }
    LCD_ShowString(120,130+uline*80,strlen(rstr)*16,32,32,(uint8_t *)rstr);
    printf("%s\r\n",rstr);
    uline++;
    }
    uline=0;
    }
    if((ic_state&0x80)==0x80)
    {
      ic_state&=0x3f;
      hole_ic_value=ic_state*(0xffffffff);
      hole_ic_value+=ic_value;
      ic_value=hole_ic_value/1000;
      sprintf((char *)rstr,"PWM:%6dms...%9lldus",ic_value,hole_ic_value);
      LCD_ShowString(120,50,strlen(rstr)*16,32,32,(uint8_t *)rstr);
      printf("%s\r\n",rstr);
      ic_state=0x00;
    }
    if(tp_dev.sta!=0)
    {
      tp_dev.sta&=0x1f;
      while(tp_dev.sta&0x01){
        upoint++;
        tp_dev.sta>>=1;
      }
      for(tp_dev.sta=0;tp_dev.sta<upoint;tp_dev.sta++)
      {
      sprintf((char *)rstr,"%dtouchpoint,x=%3d,y=%3d",tp_dev.sta,(uint16_t)tp_dev.x[tp_dev.sta],(uint16_t)tp_dev.y[tp_dev.sta]);
      printf("%s\r\n",rstr);}
      LCD_ShowString(300,50+3*80,32*16,32,32,(uint8_t *)rstr);
      upoint=0;
      tp_dev.sta=0;
    }   
    OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &os_err);    
  }
}