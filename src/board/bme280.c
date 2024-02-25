#include "bme280.h"
#include "spi.h"

#define BME280_SPI  SPI1

BME280_CalibData_t CalibData;

int32_t temperInt = 0;

/**
 *
 */
uint8_t BME280_ReadStatus(void)
{
  uint8_t res = BME280_ReadReg(BME280_REGISTER_STATUS)&0x09;
  return res;
}

/**
 *
 */
void BME280_ReadCoefficients(void)
{
    CalibData.dig_T1 = BME280_ReadReg_U16(BME280_REGISTER_DIG_T1, LITTLE_ENDIAN);
    CalibData.dig_T2 = BME280_ReadReg_S16(BME280_REGISTER_DIG_T2, LITTLE_ENDIAN);
    CalibData.dig_T3 = BME280_ReadReg_S16(BME280_REGISTER_DIG_T3, LITTLE_ENDIAN);
    CalibData.dig_P1 = BME280_ReadReg_U16(BME280_REGISTER_DIG_P1, LITTLE_ENDIAN);
    CalibData.dig_P2 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P2, LITTLE_ENDIAN);
    CalibData.dig_P3 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P3, LITTLE_ENDIAN);
    CalibData.dig_P4 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P4, LITTLE_ENDIAN);
    CalibData.dig_P5 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P5, LITTLE_ENDIAN);
    CalibData.dig_P6 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P6, LITTLE_ENDIAN);
    CalibData.dig_P7 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P7, LITTLE_ENDIAN);
    CalibData.dig_P8 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P8, LITTLE_ENDIAN);
    CalibData.dig_P9 = BME280_ReadReg_S16(BME280_REGISTER_DIG_P9, LITTLE_ENDIAN);
    CalibData.dig_H1 = BME280_ReadReg(BME280_REGISTER_DIG_H1);
    CalibData.dig_H2 = BME280_ReadReg_S16(BME280_REGISTER_DIG_H2, LITTLE_ENDIAN);
    CalibData.dig_H3 = BME280_ReadReg(BME280_REGISTER_DIG_H3);
    CalibData.dig_H4 = (BME280_ReadReg(BME280_REGISTER_DIG_H4) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H4+1) & 0x0F);
    CalibData.dig_H5 = (BME280_ReadReg(BME280_REGISTER_DIG_H5+1) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H5) >> 4);
    CalibData.dig_H6 = (int8_t)BME280_ReadReg(BME280_REGISTER_DIG_H6);
}

/**
 *
 */
void BME280_SetStandby(uint8_t tsb)
{
    uint8_t reg;
    
    reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;
    reg |= tsb & BME280_STBY_MSK;
    BME280_WriteReg(BME280_REG_CONFIG,reg);
}

/**
 *
 */
void BME280_SetFilter(uint8_t filter)
{
    uint8_t reg;
    
    reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
    reg |= filter & BME280_FILTER_MSK;
    BME280_WriteReg(BME280_REG_CONFIG,reg);
}

/**
 *
 */
void BME280_SetOversamplingTemper(uint8_t osrs)
{
    uint8_t reg;
    
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
    reg |= osrs & BME280_OSRS_T_MSK;
    BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);
}

/**
 *
 */
void BME280_SetOversamplingPressure(uint8_t osrs)
{
    uint8_t reg;
    
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
    reg |= osrs & BME280_OSRS_P_MSK;
    BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);
}

/**
 *
 */
void BME280_SetOversamplingHum(uint8_t osrs)
{
    uint8_t reg;
    
    reg = BME280_ReadReg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
    reg |= osrs & BME280_OSRS_H_MSK;
    BME280_WriteReg(BME280_REG_CTRL_HUM, reg);
    //The 'ctrl_hum' register needs to be written
    //after changing 'ctrl_hum' for the changes to become effwctive.
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS);
    BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

/**
 *
 */
void BME280_SetMode(uint8_t mode)
{
    uint8_t reg;
    
    reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
    reg |= mode & BME280_MODE_MSK;
    BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

static void init_bme280_cs_port()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    RCC_AHB1PeriphClockCmd(BME280_CS_CLOCK, ENABLE);
    GPIO_InitStruct.GPIO_Pin = BME280_CS_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BME280_CS_PORT, &GPIO_InitStruct);
    
    BME280_CS_PORT->BSRRL = BME280_CS_PIN;
}

static void set_bme280_cs_low()
{
    BME280_CS_PORT->BSRRH = BME280_CS_PIN;
}

static void set_bme280_cs_high()
{
    BME280_CS_PORT->BSRRL = BME280_CS_PIN;
}

/**
 *
 */
void BME280_Init(void)
{
    init_bme280_cs_port();
    InitSpi(BME280_SPI, SPI_BaudRatePrescaler_16);
    
    BME280_WriteReg(BME280_REG_SOFTRESET, BME280_SOFTRESET_VALUE);
    while (BME280_ReadStatus() & BME280_STATUS_IM_UPDATE) ;
    BME280_ReadCoefficients();
    BME280_SetStandby(BME280_STBY_1000);
    BME280_SetFilter(BME280_FILTER_4);
    BME280_SetOversamplingTemper(BME280_OSRS_T_x4);
    BME280_SetOversamplingPressure(BME280_OSRS_P_x2);
    BME280_SetOversamplingHum(BME280_OSRS_H_x1);
}

/**
 *
 */
void BME280_WriteReg(uint8_t Reg, uint8_t value)
{
    set_bme280_cs_low();        // sets CS low
    SpiWriteRead(BME280_SPI, (Reg & 0x7F)); // BME280 reg address
    SpiWriteRead(BME280_SPI, value);        // data
    set_bme280_cs_high();       // sets CS high
}

/**
 *
 */
uint8_t BME280_ReadReg(uint8_t Reg)
{
    uint8_t data;
    
    set_bme280_cs_low();        // sets CS low
    SpiWriteRead(BME280_SPI, (Reg | 0x80)); // BME280 reg address
    data = SpiWriteRead(BME280_SPI, 0xAA);  // read from Reg
    set_bme280_cs_high();       // sets CS high
    
    return data;
}

/**
 *
 */
uint16_t BME280_ReadReg_U16(uint8_t Reg, endian_t endian)
{
    uint16_t buff = 0;
    
    set_bme280_cs_low();    // sets CS low
    SpiWriteRead(BME280_SPI, (Reg | 0x80)); // BME280 reg address
    if( endian == LITTLE_ENDIAN ){
        buff = SpiWriteRead(BME280_SPI, 0xAA);          // read lsb from Reg
        buff |= SpiWriteRead(BME280_SPI, 0xAA) << 8;    // read msb from Reg
    }
    else{
        buff = SpiWriteRead(BME280_SPI, 0xAA) << 8;     // read lsb from Reg
        buff |= SpiWriteRead(BME280_SPI, 0xAA);         // read msb from Reg
    }
    set_bme280_cs_high();   // sets CS high
    
    return buff;
}

/**
 *
 */
int16_t BME280_ReadReg_S16(uint8_t Reg, endian_t endian)
{
    uint16_t buff = 0;
    
    set_bme280_cs_low();    // sets CS low
    SpiWriteRead(BME280_SPI, (Reg | 0x80));     // BME280 reg address
    if( endian == LITTLE_ENDIAN ){
        buff = SpiWriteRead(BME280_SPI, 0xAA);  // read lsb from Reg
        buff |= SpiWriteRead(BME280_SPI, 0xAA) << 8;    // read msb from Reg
    }
    else{
        buff = SpiWriteRead(BME280_SPI, 0xAA) << 8; // read lsb from Reg
        buff |= SpiWriteRead(BME280_SPI, 0xAA);     // read msb from Reg
    }
    set_bme280_cs_high();   // sets CS high
    
    return buff;
}

/**
 *
 */
uint32_t BME280_ReadReg_U24(uint8_t Reg)
{
    uint32_t buff = 0;
    
    set_bme280_cs_low();    // sets CS low
    SpiWriteRead(BME280_SPI, (Reg | 0x80)); // BME280 reg address
    buff = SpiWriteRead(BME280_SPI, 0xAA);  // read msb from Reg
    buff <<= 8;
    buff |= SpiWriteRead(BME280_SPI, 0xAA);
    buff <<= 8;
    buff |= SpiWriteRead(BME280_SPI, 0xAA);
    buff &= 0x00FFFFFF;
    set_bme280_cs_high();   // sets CS high
    
    return buff;
}

/**
 *
 */
float BME280_ReadTemperature(void)
{
    float temper_float = 0.0f;
    int32_t val1, val2,
            temper_raw;
    
    temper_raw = (int32_t)BME280_ReadReg_U24(BME280_REGISTER_TEMPDATA);
    temper_raw >>= 4;
    
    val1 = ((((temper_raw>>3) - ((int32_t)CalibData.dig_T1 <<1))) * ((int32_t)CalibData.dig_T2)) >> 11;
    val2 = (((((temper_raw>>4) - ((int32_t)CalibData.dig_T1)) * 
    ((temper_raw>>4) - ((int32_t)CalibData.dig_T1))) >> 12) * ((int32_t)CalibData.dig_T3)) >> 14;
    temperInt = val1 + val2;
    temper_float = ((temperInt * 5 + 128) >> 8);
    temper_float /= 100.0f;
    
    return temper_float;
}

/**
 *
 */
float BME280_ReadPressure(void)
{
    double press_real = 0.0f;
    uint32_t press_raw, pres_int;
    int64_t val1, val2, p;
    
    BME280_ReadTemperature(); // must be done first to get t_fine
    press_raw = BME280_ReadReg_U24(BME280_REGISTER_PRESSUREDATA);
    press_raw >>= 4;
    val1 = ((int64_t) temperInt) - 128000;
    val2 = val1 * val1 * (int64_t)CalibData.dig_P6;
    val2 = val2 + ((val1 * (int64_t)CalibData.dig_P5) << 17);
    val2 = val2 + ((int64_t)CalibData.dig_P4 << 35);
    val1 = ((val1 * val1 * (int64_t)CalibData.dig_P3) >> 8) + ((val1 * (int64_t)CalibData.dig_P2) << 12);
    val1 = (((((int64_t)1) << 47) + val1)) * ((int64_t)CalibData.dig_P1) >> 33;
    if (val1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - press_raw;
    p = (((p << 31) - val2) * 3125) / val1;
    val1 = (((int64_t)CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    val2 = (((int64_t)CalibData.dig_P8) * p) >> 19;
    p = ((p + val1 + val2) >> 8) + ((int64_t)CalibData.dig_P7 << 4);
    pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
    press_real = pres_int / 100.0f;
    
    return (float)(press_real * 0.000750061683); // mmHg
}

/**
 *
 */
float BME280_ReadHumidity(void)
{
    float hum_float = 0.0f;
    int16_t hum_raw;
    int32_t hum_raw_sign, v_x1_u32r;

    BME280_ReadTemperature(); // must be done first to get t_fine
    hum_raw = BME280_ReadReg_S16(BME280_REGISTER_HUMIDDATA, BIG_ENDIAN);
    hum_raw_sign = ((int32_t)hum_raw)&0x0000FFFF;
    v_x1_u32r = (temperInt - ((int32_t)76800));
    v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t)CalibData.dig_H4) << 20) -
    (((int32_t)CalibData.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
    (((((((v_x1_u32r * ((int32_t)CalibData.dig_H6)) >> 10) *
    (((v_x1_u32r * ((int32_t)CalibData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
    ((int32_t)2097152)) * ((int32_t)CalibData.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((int32_t)CalibData.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    hum_float = (v_x1_u32r>>12);
    hum_float /= 1024.0f;
    
    return hum_float;
}
//eof
