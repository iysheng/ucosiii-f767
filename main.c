#include "main.h"

int main(void)
{
    HAL_Init();
    
    GPIO_InitTypeDef  GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();		//����GPIOBʱ��  	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5; //PB0,1,PB5RGB����
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //��ʼ��GPIOB.0��GPIOB.1

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0��1,Ϩ��
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB0��1,Ϩ��

    return 0;
}
