#include "timer.h"
#include "spi.h"
#include "SX1211Driver.h"

#define RF_SPI  SPI2

static RadioEvents_t *RadioEvents;
static uint8_t PreMode = RF_STANDBY; // Previous chip operating mode

uint16_t RegistersCfg[] = // SX1211 configuration registers values
{ 
	DEF_MCPARAM1 | RF_MC1_STANDBY | RF_MC1_BAND_868 | RF_MC1_VCO_TRIM_00 | RF_MC1_RPS_SELECT_1,
	DEF_MCPARAM2 | RF_MC2_MODULATION_FSK | RF_MC2_DATA_MODE_PACKET | RF_MC2_OOK_THRESH_TYPE_PEAK | RF_MC2_GAIN_IF_01,
	DEF_FDEV | RF_FDEV_100,
	DEF_BITRATE | RF_BITRATE_50000,
	DEF_OOKFLOORTHRESH | RF_OOKFLOORTHRESH_VALUE,
	DEF_MCPARAM6 | RF_MC6_FIFO_SIZE_64 | RF_MC6_FIFO_THRESH_VALUE,
	DEF_R1 | RF_R1_VALUE,
	DEF_P1 | RF_P1_VALUE,
	DEF_S1 | RF_S1_VALUE,
	DEF_R2 | RF_R2_VALUE,
	DEF_P2 | RF_P2_VALUE,
	DEF_S2 | RF_S2_VALUE,
	DEF_PARAMP | RF_PARAMP_11,
	
	DEF_IRQPARAM1 | RF_IRQ0_RX_STDBY_PAYLOADREADY | RF_IRQ1_RX_STDBY_CRCOK | RF_IRQ1_TX_TXDONE,
	DEF_IRQPARAM2 | RF_IRQ0_TX_FIFOEMPTY_START_FIFONOTEMPTY | RF_IRQ2_PLL_LOCK_PIN_ON,
	DEF_RSSIIRQTHRESH | RF_RSSIIRQTHRESH_VALUE,
	
	DEF_RXPARAM1 | RF_RX1_PASSIVEFILT_321 | RF_RX1_FC_FOPLUS100,
	DEF_RXPARAM2 | RF_RX2_FO_100, 
	DEF_RXPARAM3 | RF_RX3_POLYPFILT_OFF | RF_RX3_SYNC_SIZE_32 | RF_RX3_SYNC_TOL_0,
	DEF_RES19,
	//RSSI Value (Read only)
	DEF_RXPARAM6 | RF_RX6_OOK_THRESH_DECSTEP_000 | RF_RX6_OOK_THRESH_DECPERIOD_000 | RF_RX6_OOK_THRESH_AVERAGING_00,
	
	DEF_SYNCBYTE1 | 0xC1, // 1st byte of Sync word,
	DEF_SYNCBYTE2 | 0x94, // 2nd byte of Sync word,
	DEF_SYNCBYTE3 | 0xC1, // 3rd byte of Sync word,
	DEF_SYNCBYTE4 | 0x94, // 4th byte of Sync word,
	
	DEF_TXPARAM | RF_TX_FC_200 | RF_TX_POWER_PLUS4,
	
	DEF_OSCPARAM | RF_OSC_CLKOUT_OFF | RF_OSC_CLKOUT_427,
	
	DEF_PKTPARAM1 | RF_PKT1_MANCHESTER_OFF | 64,                  
	DEF_NODEADRS  | RF_NODEADRS_VALUE,                 
	DEF_PKTPARAM3 | RF_PKT3_FORMAT_VARIABLE | RF_PKT3_PREAMBLE_SIZE_24 | RF_PKT3_WHITENING_ON | RF_PKT3_CRC_ON | RF_PKT3_ADRSFILT_00,                    
	DEF_PKTPARAM4 | RF_PKT4_AUTOCLEAR_ON | RF_PKT4_FIFO_STANDBY_WRITE
};

static void SetupRfPorts(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    
    // NSS port
    RCC_AHB1PeriphClockCmd(NSS_PORT_CLOCK, ENABLE);
    GPIO_InitStruct.GPIO_Pin = RF_NSS_CONF_PIN | RF_NSS_DATA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RF_NSS_PORT, &GPIO_InitStruct);
    
    // IRQ port
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
	EXTI_InitStruct.EXTI_Line = EXTI_Line3;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
    
    NVIC_SetPriority(EXTI3_IRQn, 1);
    NVIC_EnableIRQ(EXTI3_IRQn);
}

/* Writing a value to the SX1211 register at a given address */
static void WriteRegister(uint8_t address, uint8_t value)
{    
	set_nss_conf_low();
	address = ((address << 1) & 0x3E);
	SpiWriteRead(RF_SPI, address);
	SpiWriteRead(RF_SPI, value);
	set_nss_conf_high();
}

/* Reading the SX1211 register at a given address */
static uint8_t ReadRegister(uint8_t address)
{
    volatile uint8_t reg;
    
	set_nss_conf_low();
	address = ((address << 1) & 0x7E) | 0x40;
	reg = SpiWriteRead(RF_SPI, address);
	reg = SpiWriteRead(RF_SPI, 0);
	set_nss_conf_high();
	return reg;
}

/* Sending a byte of data */
static void SendByte(uint8_t data)
{	
	set_nss_data_low();
	SpiWriteRead(RF_SPI, data);
	set_nss_data_high();
}

/* Receive a data byte */
static uint8_t ReceiveByte(void)
{    
    uint8_t rcv_byte = 0;
	
	set_nss_data_low();
	rcv_byte = SpiWriteRead(RF_SPI, 0x55);
	set_nss_data_high();
	return rcv_byte;
}

/* Initializing SX1211 */
void InitRFChip(RadioEvents_t *events)
{	
    uint16_t i;
    
    RadioEvents = events;
    
    SetupRfPorts();
    set_nss_conf_high();
    set_nss_data_high();
    InitSpi(RF_SPI, SPI_BaudRatePrescaler_64);
    
    for (i = 0; (i + 1) <= REG_PKTPARAM4; i++) {
        if (i < REG_RSSIVALUE) {
            WriteRegister(i, RegistersCfg[i]);
        }
        else {
            WriteRegister(i + 1, RegistersCfg[i]);
        }
    }
    SetRFMode(RF_SLEEP);
}

/* Switching operating modes of the SX1211 transceiver */
void SetRFMode(uint8_t mode)
{	
    NVIC_DisableIRQ(RF_IRQ_1); // Desable interrupt IRQ_1
    if (mode != PreMode) {
        if (mode == RF_TRANSMITTER) {
            if (PreMode == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);        		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS);         		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
                Wait_us(TS_TR);
            }
            else if (PreMode == RF_STANDBY) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS);         		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
                Wait_us(TS_TR);
            }
            else if (PreMode == RF_SYNTHESIZER) {
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
        		   Wait_us(TS_TR);
        		}
        		else if (PreMode == RF_RECEIVER) {
                    WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
                    Wait_us(TS_TR);
        		}		        
                PreMode = RF_TRANSMITTER;
        }
        else if (mode == RF_RECEIVER) {
            if (PreMode == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);        		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
                Wait_us(TS_RE);
            }
            else if (PreMode == RF_STANDBY) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
                Wait_us(TS_RE);
            }
            else if (PreMode == RF_SYNTHESIZER) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait_us(TS_RE);     		
        		}

        		else if (PreMode == RF_TRANSMITTER) {	
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait_us(TS_RE);
        		}
                EXTI_ClearFlag(RF_IRQ_1_LINE);	// Clear flag IRQ_1
                NVIC_EnableIRQ(RF_IRQ_1); // Enable interrupt IRQ_1 (CRCOK)
                PreMode = RF_RECEIVER;
        }
        else if (mode == RF_SYNTHESIZER) {
            if (PreMode == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);        		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
            }
            else if (PreMode == RF_STANDBY) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
            }
            else {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
            }
            PreMode = RF_SYNTHESIZER;
        }
        else if (mode == RF_STANDBY) {
            if (PreMode == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);
            }
            else {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);       		
            }
            PreMode = RF_STANDBY;
        }
        else {// mode == RF_SLEEP
            WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SLEEP);
            PreMode = RF_SLEEP;
        }
    }
}

/* Reading RSSI value */
uint16_t ReadRssi(void)
{	
	return ReadRegister(REG_RSSIVALUE);
}

/* Transmitting a data packet via radio */
void SendRfPacket(uint8_t *buffer, uint8_t size)
{
    uint16_t timeout = 0;
    
    if ((size+1) > RF_BUFFER_SIZE) {
        if (RadioEvents->TxError != NULL) {
			RadioEvents->TxError(); // notify application about TxError event
		}
        return;
    }
    
    SetRFMode(RF_STANDBY);
	
    WriteRegister(REG_PKTPARAM4, (RegistersCfg[REG_PKTPARAM4-1] & 0xBF) | RF_PKT4_FIFO_STANDBY_WRITE);
    
    SendByte(size);
    
    while (size--) {
        SendByte(*buffer++);
    }
    
    SetRFMode(RF_TRANSMITTER); // Cf RF_IRQ0_TX_FIFOEMPTY_START_FIFONOTEMPTY 
    
    do {
        if (++timeout > 3000) break;
    }
    while (!(TEST_PORT_PINS(RF_IRQ_PORT, RF_IRQ_1_PIN))); // Wait for TX done
    
    EXTI_ClearFlag(RF_IRQ_1_LINE);	// Clear flag IRQ_1
    Wait_us(500); // Wait for last bit to be sent (worst case bitrate)
    
    SetRFMode(RF_STANDBY);
	
	if (RadioEvents->TxDone != NULL) {
		RadioEvents->TxDone(); // Notify application about TxDone event
	}
}

/* Receive data packet */
void ReadRfPacket(uint8_t *buffer, uint16_t *rx_size, int16_t *rx_rssi)
{
    int16_t rssi;
    uint16_t size;
	
	if (PreMode == RF_RECEIVER) {
		rssi = ReadRegister(REG_RSSIVALUE);
		SetRFMode(RF_STANDBY);
        
		WriteRegister(REG_PKTPARAM4, (RegistersCfg[REG_PKTPARAM4-1] & 0xBF) | RF_PKT4_FIFO_STANDBY_READ);
		
		*rx_rssi = size = ReceiveByte(); // reading the packet size
		
		if ((size == 0) || (size > RF_BUFFER_SIZE)) {
		    if (RadioEvents->RxError != NULL) {
				RadioEvents->RxError(); // notify application about RxError event
			}
            SetRFMode(RF_RECEIVER);
			return;
		}
		// reading data
        while (size--) {
            *buffer++ = ReceiveByte();
        }
		*rx_rssi = (int16_t)((rssi - 40) * 0.6f - 100.0f); // RSSI calculation
	}
}

/**
 */
uint8_t GetRfMode(void)
{
	return PreMode;
}

/**
 * IRQ_1 -> CRC_OK
 */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        NVIC_DisableIRQ(EXTI3_IRQn);
        EXTI_ClearFlag(EXTI_Line3);
        if (RadioEvents->RxDone != NULL) {
			RadioEvents->RxDone(); // notify application about RxDone event
		}
    }
}
/* eof */
