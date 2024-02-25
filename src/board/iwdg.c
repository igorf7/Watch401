#include "iwdg.h"

void InitIWDG(void)
{
#ifndef DEBUG
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_16); // 4, 8, 16 ... 256
	IWDG_SetReload(0x0FFF); //This parameter must be a number between 0 and 0x0FFF.
	IWDG_ReloadCounter();
	IWDG_Enable();
#endif
}

void ReloadIWDG(void)
{
#ifndef DEBUG
    IWDG_ReloadCounter();
#endif
}
