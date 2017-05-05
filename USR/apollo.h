#ifndef __APOLLO_H
#define __APOLLO_H

#include "stm32f7xx.h"
#include "stm32f767xx.h"
#include "core_cm7.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

/*接收缓冲区的大小*/
#define RSTR_SIZE 128
#define IDAC_COUNT 4

#define Bank5_SDRAM_ADDR    ((uint32_t)(0XC0000000)) //SDRAM开始地址

//SDRAM配置参数
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

void CPU_CACHE_Enable(void);
void SystemClock_Config(void);
void LED_init(void);
void LED_on(void);
void LED_off(void);
void KEY_init(void);
void TEMP_init(void);
void CEKONG_init(void);
void UART_init(UART_HandleTypeDef *UARTX);
//void TIM_init(TIM_HandleTypeDef *ITIMX);
void TIM2_init(void);
void TIM3_init(void);
void TIM5_init(void);
void MPU_set_protection(uint32_t addr,uint8_t size,uint8_t num,uint8_t ap);
void MPU_init(void);
void SDRAM_init(void);
uint8_t SDRAM_Send_Cmd(uint8_t bankx,uint8_t cmd,uint8_t refresh,uint16_t regval);
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram);
void FMC_SDRAM_WriteBuffer(uint8_t *pBuffer,uint32_t WriteAddr,uint32_t n);
void FMC_SDRAM_ReadBuffer(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t n);

#endif