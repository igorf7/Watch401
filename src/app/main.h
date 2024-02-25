/*
 * main.h
*/
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define RUNNING_STR_MAX         (uint8_t)32
#define MAX_MSGSTR_SIZE         (uint8_t)26     // up to 26 symbols/line
#define TIME_X_POS              (uint8_t)53
#define SIMBOL_WIDTH            (uint8_t)9      // simbol width in rus9x15 font
#define CHANGE_SIMB_POS         (uint8_t)(255 - SIMBOL_WIDTH)
#define PARAMS_QUNTITY          (uint8_t)3
#define SENSORS_QUNTITY         (uint8_t)2      // including internal sensor
#define MAX_PACKET_SIZE         (uint8_t)64     // Including Size Byte
#define MAX_DATA_SIZE           (uint8_t)58     // MAX_PACKET_SIZE - Header Size - Size Byte
#define OWN_ID                  (uint8_t)1

//typedef char(*pt2)[];

/* Display's page to show */
typedef enum {
    CLOCK_PAGE = (uint8_t)0,
    PARAM_PAGE,
    MESSAGE_PAGE
} DisplayPage_t;

/* Device types */
typedef enum {
    METEO_WATCH = (uint8_t)9,
    RF_BME280_SENSOR,
    RF_REMOTE_CONTROL
} DevType_t;

/* Packet types */
typedef enum { /* Packet types */
    BME280_PACKET = (uint8_t)1,
    CMD_PACKET,
    ACK_PACKET,
    DIA_PACKET,
    CONNECT
} PacketType_t;

/* Cmd code */
typedef enum {
    DISPLAY_CMD = (uint8_t)7,
    SET_MSG,
    GET_MSG,
    SET_DATETIME,
    GET_DATETIME,
    DUMMY_CMD,
    GET_ADDRESS,
    SET_RECEIVE,
    SET_SLEEP,
    RESET_CMD
} CmdCode_t;

/* Diagnostic data structure */
typedef struct {
    float voltage_tx, // Battery voltage in rf transmit mode
          voltage_sl; // Battery voltage in rf sleep mode
    uint32_t uptime;
} Diagnostic_t;

/* Sensor data structure */
typedef struct {
    float prm[PARAMS_QUNTITY];
    uint32_t timeout;
    bool presence;
    Diagnostic_t diagnostic;
} PrmList_t;

/* Packet header structure */
#pragma pack(push, 1)
typedef struct {
	uint16_t dst_addr,
             snd_addr;
    uint8_t pack_type;
} RF_Header_t;
#pragma pack(pop)

/* Packet structure */
#pragma pack(push, 1)
typedef struct {
	RF_Header_t header;
	uint8_t data[MAX_DATA_SIZE];
} RF_Packet_t;
#pragma pack(pop)

__STATIC_INLINE int32_t iRound(float value)
{
    return (value < 0.0f) ? (value -= 0.5f) : (value += 0.5f);
}

/* API */
void OnRadioRxDone(void);
void OnRadioTxDone(void);
void OnRadioRxError(void);
void OnRadioTxError(void);
void BackgroundTask(void);
void ParseRfPacketTask(void *prm);
#endif // __MAIN_H
