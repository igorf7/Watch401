#include "main.h"
#include "u8g_arm.h"
#include "rtc.h"
#include "led.h"
#include "iwdg.h"
#include "SX1211Driver.h"
#include "bme280.h"
#include "timer.h"
#include "scheduler.h"

u8g_t u8g;

static RTC_TimeTypeDef TimeNow;
static RTC_DateTypeDef DateNow;

static RF_Packet_t *rxPacket = NULL;

static DisplayPage_t pageNow = CLOCK_PAGE;

static PrmList_t prmList[SENSORS_QUNTITY];

//static PortEvents_t portEvents;
static RadioEvents_t RadioEvents;

static uint32_t
    i, rxUtcValue;

static int16_t rxRssi;

static uint16_t
    rxSize,
    ownAddress = (METEO_WATCH << 8) | OWN_ID;

static uint8_t
    strX = 0,
    strY = 61,
    timeX = TIME_X_POS,
    timeY = 43,
    char_index = 0,
    str_index = 0;

//static char *colon = ":";

//volatile uint8_t
//    selMode = 0;

bool timeStrRunning;

static char
    strYear[5],
    strMonth[3],
    strMday[3],
    strHour[3],
    strMin[3],
    strSec[3],
    strDate[16],
    runningLine[STR_BUFF_SIZE];

static char lineBuffer[STR_BUFF_SIZE];

static char
    strTemper[SENSORS_QUNTITY][8],
    strHumidity[SENSORS_QUNTITY][8],
    strPressure[SENSORS_QUNTITY][8];

static const char strWeekDay[7][16] = {
    {"Monday"},{"Tuesday"},{"Wednesday"},
    {"Thursday"},{"Friday"},{"Saturday"},{"Sunday"}
};

static uint8_t rfRxBuffer[RF_BUFFER_SIZE];

static void DrawClockPage(uint8_t x, uint8_t y)
{
    // draw clock string
    u8g_SetFont(&u8g, u8g_font_osr35n);
    u8g_DrawStr(&u8g, x, y, strHour);
    u8g_DrawStr(&u8g, x + 65, y - 5, ":");
    u8g_DrawStr(&u8g, x + 88, y, strMin);
    // draw sec str
    u8g_SetFont(&u8g, u8g_font_ncenR14r);
    u8g_DrawStr(&u8g, x + 160, y - 19, strSec);
    // draw running line
    u8g_SetFont(&u8g, rus9x15_changed);
    u8g_DrawStr(&u8g, strX, strY, runningLine);
}

static void DrawParamPage(uint8_t x, uint8_t y)
{
    u8g_SetFont(&u8g, rus9x15_changed);
    
//    u8g_DrawStr(&u8g, 0, 12, strTmp[0]);
//    u8g_DrawStr(&u8g, 0, 28, strHmd[0]);
//    u8g_DrawStr(&u8g, 0, 44, strPrs[0]);
//    u8g_DrawStr(&u8g, 198, 12, strTmp[1]);
//    u8g_DrawStr(&u8g, 198, 28, strHmd[1]);
//    u8g_DrawStr(&u8g, 198, 44, strPrs[1]);
}

static void DrawPage(DisplayPage_t page)
{
    switch (page)
    {
        case CLOCK_PAGE:
            DrawClockPage(timeX, timeY);
            break;
        case PARAM_PAGE:
            DrawParamPage(5, 15);
            break;
        default:
            break;
    }
}

static void LetLineRun()
{
    static uint8_t change_simb_pos = 0;
    
    if (timeStrRunning) {
        timeX++;
        if (timeX == TIME_X_POS) {
            timeStrRunning = false;
        }
    }
    
    switch (str_index)
    {
        case 0:
            strcpy(lineBuffer, "Today: ");
            strcat(lineBuffer, strWeekDay[DateNow.RTC_WeekDay]);
            strcat(lineBuffer, " ");
            strcat(lineBuffer, strDate);
            break;
        case 1:
            strcpy(lineBuffer, "Temperature: ");
            sprintf(strTemper[0], "%d", iRound(prmList[0].prm[0]));
            sprintf(strTemper[1], "%d", iRound(prmList[1].prm[0]));
            strcat(lineBuffer, strTemper[0]);
            strcat(lineBuffer, " / ");
            strcat(lineBuffer, strTemper[1]);
            strcat(lineBuffer, " *C");
            break;
        case 2:
            strcpy(lineBuffer, "Pressure: ");
            sprintf(strPressure[0], "%d", iRound(prmList[0].prm[1]));
            sprintf(strPressure[1], "%d", iRound(prmList[1].prm[1]));
            strcat(lineBuffer, strPressure[0]);
            strcat(lineBuffer, " / ");
            strcat(lineBuffer, strPressure[1]);
            strcat(lineBuffer, " mmHg");
            break;
        case 3:
            strcpy(lineBuffer, "Air humidity: ");
            sprintf(strHumidity[0], "%d", iRound(prmList[0].prm[2]));
            sprintf(strHumidity[1], "%d", iRound(prmList[1].prm[2]));
            strcat(lineBuffer, strHumidity[0]);
            strcat(lineBuffer, " / ");
            strcat(lineBuffer, strHumidity[1]);
            strcat(lineBuffer, " %");
            break;
    }
    
    while (strlen(lineBuffer) < STR_BUFF_SIZE) {
        strcat(lineBuffer, " ");
    }
    
    strX--;
    if (change_simb_pos < SIMBOL_WIDTH)
        change_simb_pos = CHANGE_SIMB_POS;
    if (strX == change_simb_pos) {
        runningLine[char_index] = lineBuffer[char_index];
        if (++char_index > MAX_MSGSTR_SIZE) {
            char_index = 0;
            if (++str_index == 4) str_index = 0;
        }
        change_simb_pos -= SIMBOL_WIDTH;
    }
}

/*!
 \brief Entry point
 */
int main(void)
{
    /* Initialize the task queue */
    InitTaskQueue(&BackgroundTask);
    
    /* Initialize SysTick timer */
    ///InitSysTick();
    InitTim10();
    
    /* RTC configuration  */
    InitRTC();
    
    ///
    RtcSetFromUtc(1707644580);
    ///
    
    /* Setup u8glib */
    u8g_InitComFn(&u8g, &u8g_dev_ssd1322_nhd31oled_gr_parallel, u8g_com_hw_8080_fn);
    u8g_SetDefaultForegroundColor(&u8g);
    u8g_SetFontRefHeightExtendedText(&u8g);
    u8g_SetDefaultMidColor(&u8g);
    //u8g_SetDefaultForegroundColor(&u8g);
    
    /* Setup Radio */
	RadioEvents.RxDone = OnRadioRxDone;   // data reception completed
	RadioEvents.TxDone = OnRadioTxDone;   // data transfer is complete
	RadioEvents.RxError = OnRadioRxError; // data reception error
	RadioEvents.TxError = OnRadioTxError; // data transfer error
	InitRFChip(&RadioEvents);
    
    /* Setup BME_280 */
    BME280_Init();
    BME280_SetMode(BME280_MODE_NORMAL);
    
    ///
    prmList[0].prm[0] = -37.0f;
    prmList[0].prm[1] = 754.0f;
    prmList[0].prm[2] = 43.0f;
    prmList[0].presence = true;
    ///
    
    InitLed();
    
    /* Initialize Watchdog timer */
    InitIWDG();
    
    ///
    //SetRFMode(RF_RECEIVER);
    //EXTI_GenerateSWInterrupt(EXTI_Line3);/////////////////////
    ///
    
    /* Enable global interrups */
    __enable_irq();
    
    
    
    while (1)
    {
        ReloadIWDG();
        RunTaskSheduler(); // task management
    }
}

/*!
 \brief Background task callback
 */
void BackgroundTask(void)
{
    for (i = 0; i < SENSORS_QUNTITY; i++) {
        if (prmList[i].timeout > 65) {
            prmList[i].presence = false;
        }
    }
    
    RTC_GetTime(RTC_Format_BIN, &TimeNow);
    RTC_GetDate(RTC_Format_BIN, &DateNow);
    
    sprintf(strHour, "%.2d", TimeNow.RTC_Hours);
    sprintf(strMin, "%.2d", TimeNow.RTC_Minutes);
    sprintf(strSec, "%.2d", TimeNow.RTC_Seconds);
    
    strcpy(strDate, strMday);
    strcat(strDate, ".");
    strcat(strDate, strMonth);
    strcat(strDate, ".");
    strcat(strDate, strYear);
    
    if (TimeNow.RTC_Seconds == 58) timeStrRunning = true;
    
    // Load calendar values to status string
    sprintf(strMday, "%.2d", DateNow.RTC_Date);
    sprintf(strMonth, "%.2d", DateNow.RTC_Month);
    sprintf(strYear, "%.2d", DateNow.RTC_Year);
    
    if (TimeNow.RTC_Seconds % 2) {
        BME280_SetMode(BME280_MODE_FORCED);
        prmList[1].prm[0] = BME280_ReadTemperature();
        prmList[1].prm[1] = BME280_ReadPressure();
        prmList[1].prm[2] = BME280_ReadHumidity();
        prmList[1].presence = true;
    }
    
    /* Picture loop */
    u8g_FirstPage(&u8g);
    
    do {
        DrawPage(pageNow);
    }
    while(u8g_NextPage(&u8g));
    
    /* Running line */
    if (pageNow == CLOCK_PAGE) {
        LetLineRun();
    }
}

/*!
 \brief On Radio Receive Done callback function
 */
void OnRadioRxDone(void)
{
    PutEvent(ParseRfPacketTask, NULL);
}

/*!
 \brief On Radio Receive Error callback function
 */
void OnRadioRxError(void)
{
    if (GetRfMode() != RF_RECEIVER) {
        SetRFMode(RF_RECEIVER);
    }
}

/*!
 \brief On Radio Transmit Done callback function
 */
void OnRadioTxDone(void)
{
    if (GetRfMode() != RF_RECEIVER) {
        SetRFMode(RF_RECEIVER);
    }
}

/*!
 \brief On Radio Transmit Error callback function
 */
void OnRadioTxError(void)
{
    if (GetRfMode() != RF_RECEIVER) {
        SetRFMode(RF_RECEIVER);
    }
}

/*!
 \brief Parses the received data according to the protocol
 \param [IN] prm - pointer to handler parameter
 */
void ParseRfPacketTask(void *prm)
{
    static uint16_t id;
    
    ReadRfPacket(rfRxBuffer, &rxSize, &rxRssi); // Read received data from SX1211
    
    rxPacket = (RF_Packet_t*)rfRxBuffer;
    id = rxPacket->header.snd_addr & 0x00FF; // get device id
	
	if (rxPacket->header.dst_addr == ownAddress)
	{
        switch (rxPacket->header.pack_type)  // get packet type
        {
        case BME280_PACKET:
            switch (rxPacket->header.snd_addr >> 8) // get device type
            {
            case RF_BME280_SENSOR:
                if (id <= SENSORS_QUNTITY) {
                    memcpy(&prmList[id-1], rxPacket->data, sizeof(prmList[id-1].prm));
                    prmList[id-1].timeout = 0;
                    prmList[id-1].presence = true;
                }
                break;
            default:
                break;
            }
            break;
        
        case CMD_PACKET:
            switch (rxPacket->header.snd_addr >> 8) // get device type
            {
            case RF_BME280_SENSOR:
                break;
            case RF_REMOTE_CONTROL:
                switch (rxPacket->data[0])    // CMD code
                {
                case DISPLAY_CMD:
                    break;
                case SET_MSG:
                    break;
                case GET_MSG:
                    break;
                case SET_DATETIME:
                    rxUtcValue = *((uint32_t*)(rxPacket->data+1)); // read UTC value
                    RtcSetFromUtc(rxUtcValue);
                    break;
                }
                break;
            }
            break;
        default:
            break;
        }
	}
    SetRFMode(RF_RECEIVER);
}
