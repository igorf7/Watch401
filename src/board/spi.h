/*
 * spi.h
*/
#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx.h"

/* API */
void InitSpi(SPI_TypeDef *SPIx, uint16_t br_prescaler);
uint8_t SpiWriteRead(SPI_TypeDef *SPIx, uint8_t tx_data);
#endif
