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
*                                    MICRIUM BOARD SUPPORT PACKAGE
*
* Filename      : bsp_os.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDES
*********************************************************************************************************
*/

#include  "os.h"
#include  "bsp_clock.h"

#include  "stm32f7xx_hal.h"


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                    INITIALIZE OS TICK INTERRUPT
*
* Description : Initialize the tick interrupt.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none
*********************************************************************************************************
*/

void  BSP_OSTickInit (void)
{
    CPU_INT32U  cpu_clk_freq;


    cpu_clk_freq = BSP_ClkFreqGet(BSP_CLK_ID_HCLK);             /* Determine SysTick reference freq.                    */

    OS_CPU_SysTickInitFreq(cpu_clk_freq);                       /* Init uC/OS periodic time src (SysTick).              */
}


/*
*********************************************************************************************************
*                                            HAL_InitTick()
*
* Description : This function has been overwritten from the STM32F7xx HAL libraries because Micrium's RTOS
*               has its own Systick initialization and because it is recomended to initialize the tick after
*               multi-tasking has started.
*
* Argument(s) : TickPriority          Tick interrupt priority.
*
* Return(s)   : HAL_OK.
*
* Caller(s)   : HAL_InitTick ()) is called automatically at the beginning of the program after reset by
*               HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
*
* Note(s)     : none.
*********************************************************************************************************
*/

HAL_StatusTypeDef  HAL_InitTick (uint32_t  TickPriority)
{
    HAL_NVIC_SetPriorityGrouping(0);

    return (HAL_OK);
}


/*
*********************************************************************************************************
*                                            HAL_GetTick()
*
* Description : This function has been overwritten from the STM32F7xx HAL libraries because Micrium's OS's
*               has their own Tick counter values.
*
* Argument(s) : None.
*
* Return(s)   : Tick current value.
*
* Caller(s)   : STM32F7xx HAL driver files.
*
* Note(s)     : (1) Please ensure that the Tick Task has a higher priority than the App Start Task.
*********************************************************************************************************
*/

uint32_t  HAL_GetTick(void)
{
    CPU_INT32U  os_tick_ctr;
#if (OS_VERSION >= 30000u)
    OS_ERR      os_err;


    os_tick_ctr = OSTimeGet(&os_err);
#else
    os_tick_ctr = OSTimeGet();
#endif

    return os_tick_ctr;
}
