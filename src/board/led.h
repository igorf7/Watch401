/**
 * led.h
 */
#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

#define LED_PORT	GPIOC
#define LED_PIN		GPIO_Pin_13

/* API */
void InitLed(void);
void LedOn(void);
void LedOff(void);
void LedToggle(void);
#endif // __LED_H
