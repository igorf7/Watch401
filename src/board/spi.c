#include "spi.h"

void InitSpi(SPI_TypeDef *SPIx, uint16_t br_prescaler)
{
    SPI_InitTypeDef SPI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_TypeDef *spi_port;
    uint16_t spi_sck, spi_miso, spi_mosi;
    
    if (SPIx == SPI1) {
        spi_port = GPIOA;
        spi_sck = GPIO_Pin_5;
        spi_miso = GPIO_Pin_6;
        spi_mosi = GPIO_Pin_7;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
        
    }
    else if (SPIx == SPI2) {
        spi_port = GPIOB;
        spi_sck = GPIO_Pin_13;
        spi_miso = GPIO_Pin_14;
        spi_mosi = GPIO_Pin_15;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
        
    }
    else {
        return;
    }
    
    // Port settings
    GPIO_InitStruct.GPIO_Pin = spi_sck | spi_miso | spi_mosi;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(spi_port, &GPIO_InitStruct);
    
    // SPI Settings
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = br_prescaler;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStruct);
    SPI_Cmd(SPIx, ENABLE);
}

/**
 * Send/Receive byte via SPI2
*/
uint8_t SpiWriteRead(SPI_TypeDef *SPIx, uint8_t tx_data)
{
    uint8_t rx_data;
    
    while ((SPIx->SR & SPI_I2S_FLAG_TXE) != SPI_I2S_FLAG_TXE)
    {
    }
    
    SPIx->DR = tx_data;
    
    while ((SPIx->SR & SPI_I2S_FLAG_RXNE) != SPI_I2S_FLAG_RXNE)
    {
    }
    
    rx_data = SPIx->DR;
    
    return rx_data;
}
