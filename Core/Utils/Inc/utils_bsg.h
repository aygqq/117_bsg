/*
 * utils_bsg.h
 *
 *  Created on: Apr 20, 2020
 *      Author: bls
 */

#ifndef INC_UTILS_BSG_H_
#define INC_UTILS_BSG_H_

#include "../Tasks/Inc/task_get_train_data.h"
#include "../Utils/Inc/circularBuffer.h"
#include "../Utils/Inc/utils_crc.h"
#include "../xDrvrs/Inc/simcom.h"
#include "../xDrvrs/Inc/spiflash.h"
#include "main.h"
#include "time.h"
#include "usart.h"

#define BSG_THRESHOLD_CNT_PAGES 2
#define BSG_ID_DEV_BSG          0x12
#define BSG_ADDR_ID_MCU         0x1FFF7A10

#define SZ_PART_NEW_SOFTWARE 1360
#define SZ_MAX_TX_DATA       4096

#define BSG_MSG_NO_GPS (char*)"0000.000000,N,00000.000000,E,+0000,000,000"

#define BSG_ID_FIRMWARE         1
#define BSG_ID_BOOT             2
#define BSG_ID_TRAINCAR         0
#define BSG_ID_TRAIN            1706
#define BSG_VER_BETA_FIRMWARE   (char)'B'
#define BSG_VER_STABLE_FIRMWARE (char)'S'

#define BSG_CNT_INVALID_GNSS_DATA 5

#define BSG_SZ_BUF_PCKG_GNSS 512

#define LEN_TIMESTAMP 13

#define BSG_BIG_DIF_RTC_SERVTIME 600

#define UPD_TARGET_BSG   0
#define UPD_TARGET_TABLO 1

typedef union {
    struct {
        u8 cntr   : 3;
        u8 flagOn : 1;
    };
    u8 regSleepTimer;
} SleepTimer;

typedef __packed struct {
    u32 sec;
    u16 fst;
} Coord;

typedef __packed struct {
    Coord latitude;
    Coord longitude;
    s32   altitude;
    u16   course;
    u16   speed;
    u16   sattelites;
    u16   hdop;
} gps_coords_t;

typedef __packed struct {
    u8           valid;
    u32          stopTime;
    gps_coords_t coords;
} gps_state_t;

typedef __packed struct {
    u32          unixTimeStamp;
    gps_coords_t coords;
    DateTime     dateTime;
} PckgGnss;

typedef struct {
    u32 gpsInvaligCount;
    u32 gpsParseFailCount;

    u32 tabloErrCnt;

    u32 pageWrCount;
    u32 pageRdCount;
    u32 pageBadCount;
    u32 pageFromIUCount;

    u32 simSendCnt;
    u32 simErrCnt;
    u32 simResetCnt;
    u32 simOpenCnt;
    u32 simBadCsqCnt;
    u32 simLowCsqCnt;
    u32 simGoodCsqCnt;
    u32 simHighCsqCnt;

    u32 queueErrCount;
} statistics_t;

typedef struct {
    u32 tcp_open_time;
    u32 tcp_close_time;
    u32 tcp_send_time;
    u32 tcp_all_time;
} time_stat_t;

typedef struct {
    u64 imei;
    u8  niacIdent[40];
    u32 idMCU[3];
    u16 idTrain;
    u8  idTrainCar;
    u16 idReceiver;
    u8  idDev;
    u8  idFirmware;
    u8  idNewFirmware;
    u8  szNewFirmware;
    u8  idBoot;
    u8  isSentData;
    u8  isTCPOpen;
    u8  updTarget;
    u8  tcpErrCnt;
    u8  csq;
    u8  isSpiFlashReady;
    u8  server;

    FIRMWARE_INFO info;
    time_stat_t   timers;
    statistics_t  stat;
    TabloInfo     tablo;
    gps_state_t   cur_gps;
} BSG;

typedef struct {
    u32 unixTimeStamp;
    u32 data;
    u8  group;
    u8  code;
} PckgTelemetry;

typedef struct {
    u32 unixTimeStamp;
    s8  temp[4];
} PckgTemp;

typedef struct {
    u32 fromByte;
    u32 toByte;
} PckgUpdFirmware;

typedef __packed struct {
    u32 unixTimeStamp;
    u8  idx;
    u32 enAct;
    u32 enReact;
} PckgEnergy_127;

typedef __packed struct {
    u32 unixTimeStamp;
    u8  idx;
    s16 amper;
    u16 volt;
} PckgVoltAmper_127;

typedef __packed struct {
    u32 unixTimeStamp;
    u8  from;
    u8  to;
    u8  perc_perfect;
    u8  perc_good;
    u8  perc_normal;
    u8  perc_bad;
    u8  perc_no_rssi;
    u32 sum;
} PckgPercRSSI_127;

typedef struct {
    u32 unixTimeStamp;
    u8  number;
    u8  state;
} PckgDoors;

typedef enum {
    CMD_DATA_VOLTAMPER = 1,
    CMD_DATA_ENERGY,
    CMD_DATA_TEMP,
    CMD_DATA_GRMC,
    CMD_DATA_TELEMETRY = 5,
    CMD_DATA_VOLTAMPER_127,
    CMD_DATA_ENERGY_127,
    CMD_DATA_PERCRSSI_127,
    CMD_DATA_DOORS,
    CMD_DATA_GEO_PLUS
} CMD_DATA_TYPE;

typedef enum {
    TEL_GR_GENINF = 1,
    TEL_GR_HARDWARE_STATUS,
    TEL_GR_PROJECT_127,
    TEL_GR_PROJECT_127_STAT,
    TEL_GR_PROJECT_127_MEM,
    TEL_GR_SIMCOM,
    TEL_GR_PROJECT_127_RADIO_STAT,
    TEL_GR_127_COMMON = 0x10
} TELEMETRY_GROUP;

typedef enum {
    TEL_CD_GENINF_NUM_FIRMWARE = 1,
    TEL_CD_GENINF_NUM_BOOT,
    TEL_CD_GENINF_PHONE_NUM1,
    TEL_CD_GENINF_PHONE_NUM2,
    TEL_CD_GENINF_SIMCARD_MONEY,
    TEL_CD_GENINF_NUM_PCB,
    TEL_CD_GENINF_GPS_INV_CNT,
    TEL_CD_GENINF_GPS_PARSE_ER_CNT
} TELEMETRY_CODE_GEN_INF;

typedef enum {
    TEL_CD_HW_BSG = 1,
    TEL_CD_HW_SD,
    TEL_CD_HW_DS2482,
    TEL_CD_HW_SPI_FLASH,
    TEL_CD_HW_LORA,
    TEL_CD_HW_BATTERY,
    TEL_CD_HW_BSG_ALIVE,
    TEL_CD_HW_WIRELESS_SENS_RSSI,
    TEL_CD_HW_UPDATED,
    TEL_CD_HW_UPDATE_LEN,
    TEL_CD_HW_UPDATE_ERR
} TELEMETRY_CODE_STATES;

typedef enum {
    TEL_CD_127_BKT_FW = 1,
    TEL_CD_127_PPRU_FW,
    TEL_CD_127_DOORS,
    TEL_CD_127_PPRU_RSSI,
    TEL_CD_127_PPRU_PWR,
    TEL_CD_127_TABLO_FW,
    TEL_CD_127_TABLO_BOOT_FW,
    TEL_CD_127_TABLO_BOOT_ERR,
    TEL_CD_127_PPRU_STATE_TIME
} TELEMETRY_CODE_127;

typedef enum {
    TEL_CD_127_BSG_ERR = 1,
    TEL_CD_127_MU_ERR,
    TEL_CD_127_MU_STRANGE,
    TEL_CD_127_MU_SEND = 4,
    TEL_CD_127_MU_ERR_ANSW,
    TEL_CD_127_MU_ERR_REQ,
    TEL_CD_127_MU_MISS_DOOR,
    TEL_CD_127_MU_EMPTY = 8,
    TEL_CD_127_MU_PARSE_ERR,
    TEL_CD_127_MU_RSSI_STAT,
    TEL_CD_127_IU_ERR,
    TEL_CD_127_MU_RECV,
    TEL_CD_127_CNT_RSSI_ALL,
    TEL_CD_127_CNT_RSSI_FILTER,
    TEL_CD_127_SAME_TELEMETRY
} TELEMETRY_CODE_127_STAT;

typedef enum {
    TEL_CD_127_IU_PAGE_WR = 1,
    TEL_CD_127_IU_PAGE_RD,
    TEL_CD_127_IU_PAGE_BAD,
    TEL_CD_127_BSG_PAGE_WR = 4,
    TEL_CD_127_BSG_PAGE_RD,
    TEL_CD_127_BSG_PAGE_BAD,
    TEL_CD_127_BSG_PAGE_FROM_IU
} TELEMETRY_CODE_127_MEM;

typedef enum {
    TEL_CD_127_SIM_SEND = 1,
    TEL_CD_127_SIM_ERR,
    TEL_CD_127_SIM_RESET,
    TEL_CD_127_SIM_OPEN,
    TEL_CD_127_SIM_BAD_CSQ,
    TEL_CD_127_SIM_LOW_CSQ,
    TEL_CD_127_SIM_GOOD_CSQ,
    TEL_CD_127_SIM_HIGH_CSQ,
    TEL_CD_SIM_TIME_OPEN,
    TEL_CD_SIM_TIME_CLOSE,
    TEL_CD_SIM_TIME_SEND,
    TEL_CD_SIM_TIME_ALL
} TELEMETRY_CODE_SIMCOM;

typedef enum {
    TEL_CD_BSG_FW = 1,
    TEL_CD_IU_HEAD_FW,
    TEL_CD_IU_TAIL_FW,
    TEL_CD_PPRU_FW,
    TEL_CD_BKTE_FW,
    TEL_CD_CSQ,
    TEL_CD_START_STOP,
    TEL_CD_POWER,
    TEL_CD_KEEP_ALIVE,
    TEL_CD_SERVICE
} TELEMETRY_CODE_COMMON;

typedef enum {
    CMD_REQUEST_SERVER_TIME = 0x11,
    CMD_REQUEST_NUM_FIRMWARE,
    CMD_REQUEST_SZ_FIRMWARE,
    CMD_REQUEST_PART_FIRMWARE
} CMD_REQUEST;

typedef enum { MSG_TEMP = 0xF000,
               MSG_TELEMETRY = 0x0000 } TYPE_MSG;

extern BSG      bsg;
extern DateTime dateTimeGprmc;

void bsgInit();
void updateCurCoords(PckgGnss* pckgGnss);

u32  getUnixTimeStamp();
void getServerTime();
void getBsgNumFw();
void getTabloNumFw();

u32 getFlashData(u32 ADDR);
u8  waitGoodCsq(u32 timeout);

u8 isCrcOk(char* pData, int len);
u8 isDataFromFlashOk(char* pData, u8 len);

void updSpiFlash(CircularBuffer* cbuf);
void saveData(u8* data, u8 sz, u8 cmdData, CircularBuffer* cbuf);
void copyTelemetry(u8* buf, PckgTelemetry* pckgTel);
void saveTelemetry(PckgTelemetry* pckg, CircularBuffer* cbuf);

#endif /* INC_UTILS_BSG_H_ */
