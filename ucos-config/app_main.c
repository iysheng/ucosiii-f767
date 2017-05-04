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
                                                                /* ------------ FLOATING POINT TEST TASK -------------- */
static  OS_TCB       AppTaskEq0FpTCB;
static  CPU_STK      AppTaskEq0FpStk[APP_CFG_TASK_EQ_STK_SIZE];

static  OS_TCB       AppTaskEq1FpTCB;
static  CPU_STK      AppTaskEq1FpStk[APP_CFG_TASK_EQ_STK_SIZE];

#if (OS_CFG_SEM_EN > 0u)
static  OS_SEM       AppTaskObjSem;
#endif

#if (OS_CFG_MUTEX_EN > 0u)
static  OS_MUTEX     AppTaskObjMutex;
#endif

#if (OS_CFG_Q_EN > 0u)
static  OS_Q         AppTaskObjQ;
#endif

#if (OS_CFG_FLAG_EN > 0u)
static  OS_FLAG_GRP  AppTaskObjFlag;
#endif

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg);
static  void  AppTaskCreate(void);
static  void  AppObjCreate (void);

static  void  AppTaskEq0Fp (void  *p_arg);                      /* Floating Point Equation 0 task.                      */
static  void  AppTaskEq1Fp (void  *p_arg);                      /* Floating Point Equation 1 task.                      */
static  void  AppTaskObj0  (void  *p_arg);
static  void  AppTaskObj1  (void  *p_arg);

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
        ;
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

    APP_TRACE_DBG(("Creating Application kernel objects\n\r"));
    AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

    //APP_TRACE_DBG(("Creating Application Tasks\n\r"));
    //AppTaskCreate();                                            /* Create Application tasks                             */

    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
        BSP_LED_Toggle(0u);
        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
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
                 "Kernel Objects Task 0",
                  AppTaskObj1,
                  0,
                  APP_CFG_TASK_OBJ_PRIO,
                 &AppTaskObj1Stk[0],
                  AppTaskObj1Stk[APP_CFG_TASK_OBJ_STK_SIZE / 10u],
                  APP_CFG_TASK_OBJ_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);

                                                                /* ------------- CREATE FLOATING POINT TASK ----------- */
    OSTaskCreate(&AppTaskEq0FpTCB,
                 "FP  Equation 1",
                  AppTaskEq0Fp,
                  0,
                  APP_CFG_TASK_EQ_PRIO,
                 &AppTaskEq0FpStk[0],
                  AppTaskEq0FpStk[APP_CFG_TASK_EQ_STK_SIZE / 10u],
                  APP_CFG_TASK_EQ_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &os_err);

    OSTaskCreate(&AppTaskEq1FpTCB,
                 "FP  Equation 1",
                  AppTaskEq1Fp,
                  0,
                  APP_CFG_TASK_EQ_PRIO,
                 &AppTaskEq1FpStk[0],
                  AppTaskEq1FpStk[APP_CFG_TASK_EQ_STK_SIZE / 10u],
                  APP_CFG_TASK_EQ_STK_SIZE,
                  0u,
                  0u,
                  0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &os_err);
}


/*
*********************************************************************************************************
*                                          AppObjCreate()
*
* Description : Create Application Kernel Objects.
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
static  void  AppObjCreate (void)
{
    OS_ERR  os_err;


#if (OS_CFG_SEM_EN > 0u)
    OSSemCreate(&AppTaskObjSem,
                "Sem Test",
                 0u,
                &os_err);
#endif

#if (OS_CFG_MUTEX_EN > 0u)
    OSMutexCreate(&AppTaskObjMutex,
                  "Mutex Test",
                  &os_err);
#endif

#if (OS_CFG_Q_EN > 0u)
    OSQCreate(&AppTaskObjQ,
              "Queue Test",
               1,
              &os_err);
#endif

#if (OS_CFG_FLAG_EN > 0u)
    OSFlagCreate(&AppTaskObjFlag,
                 "Flag Test",
                  DEF_BIT_NONE,
                 &os_err);
#endif
}


/*
*********************************************************************************************************
*                                          AppTaskObj0()
*
* Description : Test uC/OS-III objects.
*
* Argument(s) : p_arg is the argument passed to 'AppTaskObj0' by 'OSTaskCreate()'.
*
* Return(s)   : none
*
* Caller(s)   : This is a task
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskObj0 (void  *p_arg)
{
    OS_ERR  os_err;

    (void)p_arg;


    while (DEF_TRUE) {
#if (OS_CFG_SEM_EN > 0u)
        OSSemPost(&AppTaskObjSem,
                   OS_OPT_POST_1,
                  &os_err);
#endif

#if (OS_CFG_MUTEX_EN > 0u)
        OSMutexPend(&AppTaskObjMutex,
                     0,
                     OS_OPT_PEND_BLOCKING,
                     0,
                    &os_err);

        OSTimeDlyHMSM( 0u, 0u, 0u, 100u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);

        OSMutexPost(&AppTaskObjMutex,
                     OS_OPT_POST_NONE,
                    &os_err);
#endif

#if (OS_CFG_Q_EN > 0u)
        OSQPost(        &AppTaskObjQ,
                (void *) 1u,
                         1u,
                         OS_OPT_POST_FIFO,
                        &os_err);

#endif

#if (OS_CFG_FLAG_EN > 0u)
        OSFlagPost(&AppTaskObjFlag,
                    DEF_BIT_00,
                    OS_OPT_POST_FLAG_SET,
                   &os_err);
#endif
        OSTimeDlyHMSM( 0u, 0u, 0u, 10u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
        APP_TRACE_INFO(("Object test task 0 running ....\n"));
    }
}


/*
*********************************************************************************************************
*                                          AppTaskObj1()
*
* Description : Test uC/OS-III objects.
*
* Argument(s) : p_arg is the argument passed to 'AppTaskObj1' by 'OSTaskCreate()'.
*
* Return(s)   : none
*
* Caller(s)   : This is a task
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskObj1 (void  *p_arg)
{
    OS_ERR       os_err;
    OS_MSG_SIZE  msg_size;


    (void)p_arg;

    while (DEF_TRUE) {

#if (OS_CFG_SEM_EN > 0u)
        OSSemPend(&AppTaskObjSem,
                   0,
                   OS_OPT_PEND_BLOCKING,
                   0,
                  &os_err);
#endif

#if (OS_CFG_MUTEX_EN > 0u)
        OSMutexPend(&AppTaskObjMutex,
                     0,
                     OS_OPT_PEND_BLOCKING,
                     0,
                    &os_err);

        OSMutexPost(&AppTaskObjMutex,
                     OS_OPT_POST_NONE,
                    &os_err);

#endif

#if (OS_CFG_Q_EN > 0u)
        OSQPend(&AppTaskObjQ,
                 0,
                 OS_OPT_PEND_BLOCKING,
                &msg_size,
                 0,
                &os_err);
#endif

#if (OS_CFG_FLAG_EN > 0u)
        OSFlagPend(&AppTaskObjFlag,
                    DEF_BIT_00,
                    0,
                    OS_OPT_PEND_FLAG_SET_ALL + OS_OPT_PEND_FLAG_CONSUME + OS_OPT_PEND_BLOCKING,
                    0,
                   &os_err);
#endif
        OSTimeDlyHMSM( 0u, 0u, 0u, 10u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
        APP_TRACE_INFO(("Object test task 1 running ....\n"));
    }
}


/*
*********************************************************************************************************
*                                             AppTaskEq0Fp()
*
* Description : This task finds the root of the following equation.
*               f(x) =  e^-x(3.2 sin(x) - 0.5 cos(x)) using the bisection mehtod
*
* Argument(s) : p_arg   is the argument passed to 'AppTaskEq0Fp' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Note(s)     : 1) Atollic TrueSTUDIO needs special settings to output floating point format; however,
*                  IAR and KeilMDK should work properly.
*********************************************************************************************************
*/

void  AppTaskEq0Fp (void  *p_arg)
{
    CPU_FP32    a;
    CPU_FP32    b;
    CPU_FP32    c;
    CPU_FP32    eps;
    CPU_FP32    f_a;
    CPU_FP32    f_c;
    CPU_FP32    delta;
    CPU_INT08U  iteration;
    RAND_NBR    wait_cycles;
    OS_ERR      os_err;


    while (DEF_TRUE) {
        eps       = 0.00001;
        a         = 3.0;
        b         = 4.0;
        delta     = a - b;
        iteration = 0u;
        if (delta < 0) {
            delta = delta * -1.0;
        }

        while (((2.00 * eps) < delta) ||
               (iteration    > 20u  )) {
            c   = (a + b) / 2.00;
            f_a = (exp((-1.0) * a) * (3.2 * sin(a) - 0.5 * cos(a)));
            f_c = (exp((-1.0) * c) * (3.2 * sin(c) - 0.5 * cos(c)));

            if (((f_a > 0.0) && (f_c < 0.0)) ||
                ((f_a < 0.0) && (f_c > 0.0))) {
                b = c;
            } else if (((f_a > 0.0) && (f_c > 0.0)) ||
                       ((f_a < 0.0) && (f_c < 0.0))) {
                a = c;
            } else {
                break;
            }

            delta = a - b;
            if (delta < 0) {
               delta = delta * -1.0;
            }
            iteration++;

            wait_cycles = Math_Rand();
            wait_cycles = wait_cycles % 1000;

            while (wait_cycles > 0u) {
                wait_cycles--;
            }

            if (iteration > APP_TASK_EQ_0_ITERATION_NBR) {
                APP_TRACE_INFO(("AppTaskEq0Fp() max # iteration reached\n"));
                break;
            }
        }

        APP_TRACE_INFO(("Eq0 Task Running ....\n"));

        if (iteration == APP_TASK_EQ_0_ITERATION_NBR) {
#if 0                                                           /* See Note 1.                                         */
            APP_TRACE_INFO(("Root = %f; f(c) = %f; #iterations : %d\n", c, f_c, iteration));
#else
            APP_TRACE_INFO(("Eq0 task; #iterations : %d\n", iteration));
#endif
        }

        OSTimeDlyHMSM( 0u, 0u, 0u, 10u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
    }
}



/*
*********************************************************************************************************
*                                             AppTaskEq1Fp()
*
* Description : This task finds the root of the following equation.
*               f(x) = x^2 -3 using the bisection mehtod
*
* Argument(s) : p_arg   is the argument passed to 'AppTaskEq()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Note(s)     : 1) Atollic TrueSTUDIO needs special settings to properly output floating point format;
*                  however, IAR and KeilMDK should work properly.
*********************************************************************************************************
*/

void  AppTaskEq1Fp (void  *p_arg)
{
    CPU_FP32    a;
    CPU_FP32    b;
    CPU_FP32    c;
    CPU_FP32    eps;
    CPU_FP32    f_a;
    CPU_FP32    f_c;
    CPU_FP32    delta;
    CPU_INT08U  iteration;
    RAND_NBR    wait_cycles;
    OS_ERR      os_err;


    while (DEF_TRUE) {
        eps       = 0.00001;
        a         = 1.0;
        b         = 4.0;
        delta     = a - b;
        iteration = 0u;

        if (delta < 0) {
            delta = delta * -1.0;
        }

        while ((2.00 * eps) < delta) {
            c   = (a + b) / 2.0;
            f_a = a * a - 3.0;
            f_c = c * c - 3.0;

            if (((f_a > 0.0) && (f_c < 0.0)) ||
                ((f_a < 0.0) && (f_c > 0.0))) {
                b = c;
            } else if (((f_a > 0.0) && (f_c > 0.0)) ||
                       ((f_a < 0.0) && (f_c < 0.0))) {
                a = c;
            } else {
                break;
            }

            delta = a - b;
            if (delta < 0) {
               delta = delta * -1.0;
            }
            iteration++;

            wait_cycles = Math_Rand();
            wait_cycles = wait_cycles % 1000;

            while (wait_cycles > 0u) {
                wait_cycles--;
            }

            if (iteration > APP_TASK_EQ_1_ITERATION_NBR) {
                APP_TRACE_INFO(("AppTaskEq1Fp() max # iteration reached\n"));
                break;
            }
        }

        APP_TRACE_INFO(("Eq1 Task Running ....\n"));

        if (iteration == APP_TASK_EQ_1_ITERATION_NBR) {
#if 0                                                           /* See Note 1.                                         */
            APP_TRACE_INFO(("Root = %f; f(c) = %f; #iterations : %d\n", c, f_c, iteration));
#else
            APP_TRACE_INFO(("Eq1 task; #iterations : %d\n", iteration));
#endif
        }

        OSTimeDlyHMSM( 0u, 0u, 0u, 10u,
                       OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
    }
}
