#include "main.h"

int main(void)
{
    HAL_Init();
    
    GPIO_InitTypeDef  GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOB时钟  	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5; //PB0,1,PB5RGB背光
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //初始化GPIOB.0和GPIOB.1

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0置1,熄灭
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB0置1,熄灭

    return 0;
}
