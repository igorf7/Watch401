/**
 * u8g_arm.h
 */
#ifndef __U8G_ARM_H
#define __U8G_ARM_H

#include "u8g.h"
#include "stm32f4xx.h"

/* Defining ports/pins for the 8080 parallel interface */
// Control port
#define DISP_CTRL_PORT_Periph   RCC_AHB1Periph_GPIOA
#define DISP_CTRL_PORT          GPIOA
#define DISP_DC_PIN             GPIO_Pin_0      // 14
#define DISP_RES_PIN            GPIO_Pin_1      // 15
#define DISP_CS_PIN             GPIO_Pin_2      // 16
#define DISP_RD_PIN             GPIO_Pin_8      // 12
#define DISP_WR_PIN             GPIO_Pin_15     // 13
#define DISP_CTRL_PINS          (DISP_DC_PIN|DISP_CS_PIN|DISP_RES_PIN|DISP_RD_PIN|DISP_WR_PIN)
// Data port
#define DISP_DATA_PORT_Periph   RCC_AHB1Periph_GPIOB
#define DISP_DATA_PORT          GPIOB
#define DISP_DB0                GPIO_Pin_0      // 4
#define DISP_DB1                GPIO_Pin_1      // 5
#define DISP_DB2                GPIO_Pin_2      // 6
#define DISP_DB3                GPIO_Pin_3      // 7
#define DISP_DB4                GPIO_Pin_4      // 8
#define DISP_DB5                GPIO_Pin_5      // 9
#define DISP_DB6                GPIO_Pin_6      // 10
#define DISP_DB7                GPIO_Pin_7      // 11
#define DISP_DATA_PINS          (DISP_DB0|DISP_DB1|DISP_DB2|DISP_DB3|DISP_DB4|DISP_DB5|DISP_DB6|DISP_DB7)
    
/* U8G driver API */
//void InitSysTick(void);
//void delay_micro_seconds(uint32_t us);
uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
uint8_t u8g_com_hw_8080_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
#endif // __U8G_ARM_H
