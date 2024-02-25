/**
 * u8g_arm.c
 */
#include "u8g_arm.h"
#include "timer.h"
#include "stdbool.h"

//#define SYS_TICK_PERIOD_IN_MS 10

uint16_t odr_mask;
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

//    isDelayRun = true;
//    sys_ticks = SystemCoreClock;
//    sys_ticks /=1000000UL;
//    sys_ticks *= us;
//    delay_system_ticks(sys_ticks);
//    isDelayRun = false;
//}

/**
  The following delay procedures must be implemented for u8glib
  void u8g_Delay(uint16_t val)  Delay by "val" milliseconds
 */

void u8g_Delay(uint16_t ms)
{
    ///delay_micro_seconds(1000UL*(uint32_t)ms);
    ///Wait_us(1000UL*(uint32_t)ms);
    Wait_ms(ms);
}

/* For parallel 8080 interface */
void initParallelPort(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(DISP_CTRL_PORT_Periph, ENABLE);
    GPIO_InitStruct.GPIO_Pin = DISP_CTRL_PINS;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(DISP_CTRL_PORT, &GPIO_InitStruct);
    
    GPIO_SetBits(DISP_CTRL_PORT, DISP_CTRL_PINS);
    
    RCC_AHB1PeriphClockCmd(DISP_DATA_PORT_Periph, ENABLE);
    GPIO_InitStruct.GPIO_Pin = DISP_DATA_PINS;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(DISP_DATA_PORT, &GPIO_InitStruct);
}

uint8_t u8g_com_hw_8080_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
    switch (msg)
    {
        case U8G_COM_MSG_STOP:
            break;
    
        case U8G_COM_MSG_INIT:
            initParallelPort();
            break;
    
        case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
            if (arg_val != 0) DISP_CTRL_PORT->BSRRL = DISP_DC_PIN;
            else DISP_CTRL_PORT->BSRRH = DISP_DC_PIN;
            break;

        case U8G_COM_MSG_CHIP_SELECT:
            if (arg_val == 0) {
                DISP_CTRL_PORT->BSRRL = DISP_CS_PIN;
            }
            else {
                DISP_CTRL_PORT->BSRRH = DISP_CS_PIN;
            }
            break;
      
        case U8G_COM_MSG_RESET:
            if (arg_val != 0) DISP_CTRL_PORT->BSRRL = DISP_RES_PIN;
            else DISP_CTRL_PORT->BSRRH = DISP_RES_PIN;
            ///delay_micro_seconds(10);
            Wait_us(10);
            break;
      
        case U8G_COM_MSG_WRITE_BYTE:
            odr_mask = (DISP_DATA_PORT->ODR & 0xFF00);
            odr_mask |= arg_val;
            DISP_CTRL_PORT->BSRRH = DISP_WR_PIN;
            DISP_DATA_PORT->ODR = odr_mask;
            //DISP_DATA_PORT->ODR = arg_val;
            DISP_CTRL_PORT->BSRRL = DISP_WR_PIN;
            break;
    
        case U8G_COM_MSG_WRITE_SEQ:
        case U8G_COM_MSG_WRITE_SEQ_P:
        {
            register uint8_t *ptr = arg_ptr;
            while (arg_val > 0)
            {
                DISP_CTRL_PORT->BSRRH = DISP_WR_PIN;
                DISP_DATA_PORT->ODR = *ptr++;
                DISP_CTRL_PORT->BSRRL = DISP_WR_PIN;
                arg_val--;
            }
        }
        break;
    }
    return 1;
}
//eof
