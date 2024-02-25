/*
 * rtc.h
*/
#ifndef __RTC_H
#define __RTC_H

#include "stm32f4xx.h"
#include "stm32f4xx_rtc.h"

/* RTC events callback functions */
typedef struct{
    void(*secondEvent)(void);
}RtcEvents_t;

/* API */
void InitRTC(void);
void RtcSetDateTime(RTC_DateTypeDef *date, RTC_TimeTypeDef *time);
void RtcSetFromUtc(uint32_t utc);
//void RTC_Alarm_IRQHandler(void);
#endif // __RTC_H
