#include "unixtime.h"

void utc_to_calendar(uint64_t timer, UnixStruct_t* unixtime)
{
	uint64_t a, time;
	int16_t b, c, d;

	time = timer % 86400;
	a = ((timer + 43200) / (86400 >> 1)) + (2440587 << 1) + 1;
	a >>= 1;
	unixtime->wday = a % 7;
	a += 32044;
	b = (4 * a + 3) / 146097;
	a = a - (146097 * b) / 4;
	c = (4 * a + 3) / 1461;
	a = a - (1461 * c) / 4;
	d = (5 * a + 2) / 153;
	unixtime->mday = a - (153 * d + 2) / 5 + 1;
	unixtime->month = d + 3 - 12 * (d / 10);
	unixtime->year = 100 * b + c - 4800 + (d / 10);
	unixtime->hour = time / 3600;
	unixtime->min = (time % 3600) / 60;
	unixtime->sec = (time % 3600) % 60;
}

uint64_t calendar_to_utc(UnixStruct_t* unixtime)
{
    uint64_t Uday, time;
    uint32_t y;
	int16_t a, m;
	
	a = ((14 - unixtime->month) / 12);
	y = unixtime->year + 4800 - a;
	m = unixtime->month + (12 * a) - 3;
	Uday = (unixtime->mday + ((153 * m + 2) / 5) + 365 * y + (y / 4) - (y / 100) + (y / 400) - 32045) - 2440588;
	time = Uday * 86400;
	time += unixtime->sec + unixtime->min * 60 + unixtime->hour * 3600;
	return time;
}
