#include "rtc.h"
#include "unixtime.h"

static RTC_TimeTypeDef TimeNow;
static RTC_DateTypeDef DateNow;

void InitRTC(void)
{
    RTC_InitTypeDef rtc;
    
    // If the clock has already started, get out
    if (RTC->ISR & RTC_ISR_INITS) return;

    // Otherwise - initial initialization
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    PWR_BackupAccessCmd(ENABLE);
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);
    
    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }
    
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();
    
    rtc.RTC_AsynchPrediv = 0x7F;
    rtc.RTC_SynchPrediv  = 0xFF;
    rtc.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&rtc); 
    
    /* Indicator for the RTC configuration */
    RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
}

void RtcSetDateTime(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
{
    RTC_SetTime(RTC_Format_BIN, time);
    RTC_SetDate(RTC_Format_BIN, date);
}

void RtcSetFromUtc(uint32_t utc)
{
    UnixStruct_t calendar;
    
    utc_to_calendar(utc, &calendar);
    DateNow.RTC_Date = calendar.mday;
    DateNow.RTC_Month = calendar.month;
    DateNow.RTC_WeekDay = calendar.wday;
    DateNow.RTC_Year = calendar.year - 2000;
    //TimeNow.RTC_H12 = RTC_H12_AM;
    TimeNow.RTC_Hours = calendar.hour;
    TimeNow.RTC_Minutes = calendar.min;
    TimeNow.RTC_Seconds = calendar.sec;
    RTC_SetTime(RTC_Format_BIN, &TimeNow);
    RTC_SetDate(RTC_Format_BIN, &DateNow);
}

//void RTC_Alarm_IRQHandler(void)
//{
//    if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
//    {
//        RTC_ClearITPendingBit(RTC_IT_ALRA);
//        EXTI_ClearITPendingBit(EXTI_Line17);
//    }
//}
