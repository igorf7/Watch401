/**
 * timer.c
 */
#include "timer.h"

#define SYS_TICK_PERIOD_IN_MS 10

//bool isDelayRun = false;

//void InitSysTick(void)
//{
//    /* SysTick is defined in core_cm3.h */
//    SysTick->LOAD = (SystemCoreClock/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS) - 1;
//    SysTick->VAL = 0;
//    SysTick->CTRL = 7; /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
//}

///* Generic ARM delay procedure, based on the system timer (SysTick) */

///**
//  Delay by the provided number of system ticks.
//  The delay must be smaller than the RELOAD value.
//  This delay has an imprecision of about +/- 20 system ticks.   
// */
//static void _delay_system_ticks_sub(uint32_t sys_ticks)
//{
//    uint32_t start_val, end_val, curr_val;
//    uint32_t load;
//  
//    start_val = SysTick->VAL;
//    start_val &= 0x0ffffffUL;
//    end_val = start_val;
//  
//    if( end_val < sys_ticks ){
//        /* check, if the operation after this if clause would lead to a negative result */
//        /* if this would be the case, then add the reload value first */
//        load = SysTick->LOAD;
//        load &= 0x0ffffffUL;
//        end_val += load;
//    }
//    /* counter goes towards zero, so end_val is below start value */
//    end_val -= sys_ticks;        
//  
//    /* wait until interval is left */
//    if( start_val >= end_val ){
//        for(;;){
//            curr_val = SysTick->VAL;
//            curr_val &= 0x0ffffffUL;
//        if( curr_val <= end_val )
//            break;
//        if( curr_val > start_val )
//            break;
//        }
//    }
//    else{
//        for(;;){
//            curr_val = SysTick->VAL;
//            curr_val &= 0x0ffffffUL;
//            if( curr_val <= end_val && curr_val > start_val )
//                break;
//        }
//    }
//}

///**
//  Delay by the provided number of system ticks.
//  Any values between 0 and 0x0ffffffff are allowed.
// */
//void delay_system_ticks(uint32_t sys_ticks)
//{
//    uint32_t load4;
//    load4 = SysTick->LOAD;
//    load4 &= 0x0ffffffUL;
//    load4 >>= 2;
//  
//    while( sys_ticks > load4 )
//    {
//        sys_ticks -= load4;
//        _delay_system_ticks_sub(load4);
//    }
//    _delay_system_ticks_sub(sys_ticks);
//}

///**
//  Delay by the provided number of micro seconds.
//  Limitation: "us" * System-Freq in MHz must now overflow in 32 bit.
//  Values between 0 and 1.000.000 (1 second) are ok.
// */
//void delay_micro_seconds(uint32_t us)
//{
//    uint32_t sys_ticks;

//    //isDelayRun = true;
//    sys_ticks = SystemCoreClock;
//    sys_ticks /=1000000UL;
//    sys_ticks *= us;
//    delay_system_ticks(sys_ticks);
//    //isDelayRun = false;
//}

void InitTim10(void)
{
    RCC_ClocksTypeDef RCC_ClockFreq;
    RCC_GetClocksFreq(&RCC_ClockFreq);
    
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    TIM10->PSC = (uint16_t)(RCC_ClockFreq.HCLK_Frequency / 1000000) - 1;
    TIM10->CR1 = TIM_CR1_OPM;
}

void Wait_us(uint16_t us)
{
    TIM10->CNT = 0;
    TIM10->ARR = us;
    TIM10->CR1 |= TIM_CR1_CEN;
    while(!(TIM10->SR & TIM_SR_UIF))
    {
    }
    TIM10->SR &= ~TIM_SR_UIF;
}

void Wait_ms(uint16_t ms)
{
    uint16_t i;
    
    for(i = 0; i < ms; i++){
        Wait_us(1000);
    }
}
