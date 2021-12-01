#include "../Utils/Inc/utils_bsg.h"
#include "../Utils/Inc/utils_gps.h"
#include "../Utils/Inc/utils_pckgs_manager.h"
#include "cmsis_os.h"
#include "rtc.h"

extern BSG            bsg;
extern osMutexId      mutexGPSBufHandle;
extern osMutexId      mutexRTCHandle;
extern osMutexId      mutexWebHandle;
extern osThreadId     getNewBinHandle;
extern osSemaphoreId  semCreateWebPckgHandle;
extern CircularBuffer circBufTTLVtoFlash;
extern CircularBuffer circBufAllPckgs;

static RTC_TimeTypeDef tmpTime;
static RTC_DateTypeDef tmpDate;

void bsgInit() {
    for (u8 i = 0; i < 3; i++)
        bsg.idMCU[i] = getFlashData(BSG_ADDR_ID_MCU + (i * 4));
    LOG(LEVEL_MAIN, "%08x%08x%08x\r\n", (uint)bsg.idMCU[0], (uint)bsg.idMCU[1],
        (uint)bsg.idMCU[2]);

    bsg.idTrain = BSG_ID_TRAIN;
    bsg.idTrainCar = BSG_ID_TRAINCAR;
    bsg.idReceiver = 1;
    bsg.idDev = BSG_ID_DEV_BSG;
    bsg.idFirmware = BSG_ID_FIRMWARE;
    bsg.idBoot = BSG_ID_BOOT;
    bsg.server = SERVER_NIAC;
    u8 id[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
    memcpy(bsg.niacIdent, id, 40);
    memset(&bsg.stat, 0, sizeof(statistics_t));
    memset(&bsg.cur_gps, 0, sizeof(gps_state_t));
    memset(&bsg.timers, 0, sizeof(time_stat_t));
    bsg.cur_gps.stopTime = 100;
}

void updateCurCoords(PckgGnss* pckgGnss) {
    bsg.cur_gps.valid = 1;
    memcpy(&bsg.cur_gps.coords, &pckgGnss->coords, sizeof(gps_coords_t));
}

u32 getUnixTimeStamp() {
    time_t           t;
    static struct tm curTime;

    osMutexWait(mutexRTCHandle, osWaitForever);
    HAL_RTC_GetTime(&hrtc, &tmpTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &tmpDate, RTC_FORMAT_BIN);
    curTime.tm_year = tmpDate.Year + 100;
    curTime.tm_mday = tmpDate.Date;
    curTime.tm_mon = tmpDate.Month - 1;

    curTime.tm_hour = tmpTime.Hours;
    curTime.tm_min = tmpTime.Minutes;
    curTime.tm_sec = tmpTime.Seconds;
    osMutexRelease(mutexRTCHandle);
    curTime.tm_isdst = 0;

    t = mktime(&curTime);
    return (u32)t;
}

void getServerTime() {
    u8 bufTime[4];
    if (generateWebPckgReq(CMD_REQUEST_SERVER_TIME, NULL, 0, SZ_REQUEST_GET_SERVER_TIME, bufTime, 4, &bsg.idMCU) == ERROR) {
        LOG_WEB(LEVEL_ERROR, "ERROR: bad server time\r\n");
    } else {
        time_t t =
            bufTime[0] << 24 | bufTime[1] << 16 | bufTime[2] << 8 | bufTime[3];
        struct tm* pTm;
        pTm = gmtime(&t);
        if (pTm != NULL) {
            tmpTime.Hours = pTm->tm_hour;
            tmpTime.Minutes = pTm->tm_min;
            tmpTime.Seconds = pTm->tm_sec;

            tmpDate.Date = pTm->tm_mday;
            tmpDate.Month = pTm->tm_mon + 1;
            tmpDate.Year = pTm->tm_year - 100;

            osMutexWait(mutexRTCHandle, osWaitForever);
            if (tmpDate.Year < 30 &&
                tmpDate.Year > 19) {  // sometimes timestamp is wrong and has
                                      // value like 2066 year
                HAL_RTC_SetTime(&hrtc, &tmpTime, RTC_FORMAT_BIN);
                HAL_RTC_SetDate(&hrtc, &tmpDate, RTC_FORMAT_BIN);
            }
            osMutexRelease(mutexRTCHandle);
        }
    }
}

void getBsgNumFw() {
    u8 bufFirmware[4];
    if (generateWebPckgReq(CMD_REQUEST_NUM_FIRMWARE, NULL, 0, SZ_REQUEST_GET_NUM_FIRMWARE, bufFirmware, 4, &bsg.idMCU) == ERROR) {
        LOG_WEB(LEVEL_ERROR, "ERROR: getBsgNumFw()\r\n");
    } else {
        u32 numFirmware = bufFirmware[0] << 24 | bufFirmware[1] << 16 | bufFirmware[2] << 8 | bufFirmware[3];
        LOG_WEB(LEVEL_INFO, "BSG FIRMWARE v.:%d\r\n", (int)numFirmware);
        if (numFirmware != BSG_ID_FIRMWARE && numFirmware > 0 && numFirmware < 10) {
            LOG_WEB(LEVEL_MAIN, "New FIRMWARE v.:%d\r\n", (int)numFirmware);
            bsg.updTarget = UPD_TARGET_BSG;
            bsg.idNewFirmware = (u8)numFirmware;
            vTaskResume(getNewBinHandle);
        }
    }
}

void getTabloNumFw() {
    u8 bufFirmware[4];
    if ((bsg.tablo.info.idMCU[0] == 0 && bsg.tablo.info.idMCU[1] == 0 && bsg.tablo.info.idMCU[2] == 0) || bsg.tablo.info.idFirmware == 0) {
        LOG_WEB(LEVEL_ERROR, "ERROR: null tablo id mcu or firmware\r\n");
        return;
    }
    if (generateWebPckgReq(CMD_REQUEST_NUM_FIRMWARE, NULL, 0, SZ_REQUEST_GET_NUM_FIRMWARE, bufFirmware, 4, &bsg.tablo.info.idMCU) == ERROR) {
        LOG_WEB(LEVEL_ERROR, "ERROR: getTabloNumFw()\r\n");
    } else {
        u32 numFirmware = bufFirmware[0] << 24 | bufFirmware[1] << 16 | bufFirmware[2] << 8 | bufFirmware[3];
        LOG_WEB(LEVEL_INFO, "Tablo FIRMWARE v.:%d\r\n", (int)numFirmware);
        if (numFirmware != bsg.tablo.info.idFirmware && numFirmware > 10 && numFirmware < 20 && bsg.tablo.info.idFirmware > 0) {
            LOG_WEB(LEVEL_MAIN, "New FIRMWARE v.:%d\r\n", (int)numFirmware);
            bsg.updTarget = UPD_TARGET_TABLO;
            bsg.idNewFirmware = (u8)numFirmware;
            vTaskResume(getNewBinHandle);
        }
    }
}

u32 getFlashData(u32 ADDR) { return (*(__IO u32*)ADDR); }

u8 isCrcOk(char* pData, int len) {
    u32 crcCalc = crc32_byte(pData, len);
    u32 crcRecv = pData[len] << 24 | pData[len + 1] << 16 | pData[len + 2] << 8 | pData[len + 3];
    if (crcCalc != crcRecv) {
        LOG(LEVEL_ERROR, "isCrcOk bad crc \r\n");
    }
    for (u8 i = 0; i < sizeof(u32); i++) {
        pData[len + i] = 0xFF;  //! clear crc32
    }

    return crcCalc == crcRecv;
}

void updSpiFlash(CircularBuffer* cbuf) {
    u16 bufEnd[2] = {0, BSG_PREAMBLE};

    osMutexWait(mutexGPSBufHandle, osWaitForever);

    bufEnd[0] = calcCrc16(cbuf->buf, cbuf->readAvailable);
    cBufWriteToBuf(cbuf, (u8*)bufEnd, 4);
    spiFlashWriteNextPg(cbuf->buf, cbuf->readAvailable, 0);
    cBufReset(cbuf);

    osMutexRelease(mutexGPSBufHandle);

    LOG_FLASH(LEVEL_INFO, "updSpiFlash()\r\n");
}

u8 waitGoodCsq(u32 timeout) {
    u8  csq = 0;
    u16 cntNOCsq = 0;
    u16 cntNOCsqMax = timeout / 3;

    while ((csq = simCheckCSQ()) < 5 || csq > 99) {
        osDelay(3000);
        cntNOCsq++;
        if (cntNOCsq == cntNOCsqMax) {
            cntNOCsq = 0;
            return 0;
        }
        LOG_SIM(LEVEL_DEBUG, "ER: CSQ %d\r\n", csq);
    }
    LOG_SIM(LEVEL_DEBUG, "OK: CSQ %d\r\n", csq);
    return 1;
}

void saveData(u8* data, u8 sz, u8 cmdData, CircularBuffer* cbuf) {
    u16 bufEnd[2] = {0, BSG_PREAMBLE};

    osMutexWait(mutexGPSBufHandle, osWaitForever);

    if (cbuf->writeAvailable < sz + 2 + 4) {
        bufEnd[0] = calcCrc16(cbuf->buf, cbuf->readAvailable);
        cBufWriteToBuf(cbuf, (u8*)bufEnd, 4);
        spiFlashWriteNextPg(cbuf->buf, cbuf->readAvailable, 0);
        cBufReset(cbuf);
    } else {
        cBufWriteToBuf(cbuf, &cmdData, 1);
        cBufWriteToBuf(cbuf, data, sz);
    }
    osMutexRelease(mutexGPSBufHandle);
}

u8 isDataFromFlashOk(char* pData, u8 len) {
    u16 crc;
    for (u8 i = len - 1; i; --i) {
        if (pData[i] == BSG_PREAMBLE_LSB && pData[i - 1] == BSG_PREAMBLE_MSB) {
            crc = pData[i - 3] | pData[i - 2] << 8;
            if (calcCrc16(pData, i - 3) == crc) {
                len = (i + 1) - 4;
                return len;
            }
        }
    }
    return 0;
}

void copyTelemetry(u8* buf, PckgTelemetry* pckgTel) {
    memcpy(buf, &pckgTel->unixTimeStamp, 4);
    memcpy(buf + 4, &pckgTel->group, 1);
    memcpy(buf + 5, &pckgTel->code, 1);
    memcpy(buf + 6, &pckgTel->data, 4);
}

void saveTelemetry(PckgTelemetry* pckg, CircularBuffer* cbuf) {
    u8 buf[10];
    pckg->unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(buf, pckg);
    saveData(buf, SZ_CMD_TELEMETRY, CMD_DATA_TELEMETRY, cbuf);
}