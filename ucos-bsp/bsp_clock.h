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
*                                         STM32746G-EVAL2
*                                         Evaluation Board
*
* Filename      : bsp_clock.h
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_CLOCK_H_
#define  BSP_CLOCK_H_


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <cpu_core.h>


/*
*********************************************************************************************************
*                                     EXTERNAL C LANGUAGE LINKAGE
*
* Note(s) : (1) C++ compilers MUST 'extern'ally declare ALL C function prototypes & variable/object
*               declarations for correct C language linkage.
*********************************************************************************************************
*/

#ifdef __cplusplus
extern  "C" {                                  /* See Note #1.                                         */
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              DATA TYPES
*********************************************************************************************************
*/
                                                                /* System clock IDs.                                    */
typedef  enum  bsp_clk_id {
    BSP_CLK_ID_SYSCLK,
    BSP_CLK_ID_HCLK,
    BSP_CLK_ID_PCLK1,
    BSP_CLK_ID_PCLK2,
} BSP_CLK_ID;


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void        BSP_SystemClkCfg(void);
CPU_INT32U  BSP_ClkFreqGet  (BSP_CLK_ID  clk_id);


/*
*********************************************************************************************************
*                                   EXTERNAL C LANGUAGE LINKAGE END
*********************************************************************************************************
*/

#ifdef __cplusplus
}                                              /* End of 'extern'al C lang linkage.                    */
#endif


/*
*********************************************************************************************************
*                                          CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of module include.                               */
