/**
 *  UTC convertion library (found this on the internet)
 */
#include "stm32f4xx.h"

typedef struct
{
	uint16_t year;
	uint8_t month;
	uint8_t mday;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t wday;
} UnixStruct_t;

void utc_to_calendar(uint64_t timer, UnixStruct_t* unixtime);
uint64_t calendar_to_utc(UnixStruct_t* unixtime);
