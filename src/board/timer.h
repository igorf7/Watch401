/**
 * timer.h
 */
#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"

//void InitSysTick(void);
//void delay_micro_seconds(uint32_t us);
void InitTim10(void);
void Wait_us(uint16_t uSec);
void Wait_ms(uint16_t mSec);
#endif // __TIMER_H
