/**
 * led.c
 */
#include "led.h"

/**
*/
void InitLed(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = LED_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_PORT, &GPIO_InitStruct);
    GPIO_SetBits(LED_PORT, LED_PIN);
}

/**
*/
void LedOn(void)
{
    LED_PORT->BSRRH = LED_PIN;
}

/**
*/
void LedOff(void)
{
    LED_PORT->BSRRL = LED_PIN;
}

void LedToggle(void)
{
    LED_PORT->ODR ^= LED_PIN;
}
