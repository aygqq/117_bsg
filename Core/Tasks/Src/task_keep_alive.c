#include "../Tasks/Inc/task_keep_alive.h"

#include "../Tasks/Inc/task_iwdg.h"
#include "../Utils/Inc/utils_bsg.h"
#include "../Utils/Inc/utils_pckgs_manager.h"

extern u16 iwdgTaskReg;

extern osThreadId    webExchangeHandle;
extern osThreadId    getGPSHandle;
extern osThreadId    keepAliveHandle;
extern osThreadId    getNewBinHandle;
extern osThreadId    createWebPckgHandle;
extern osMutexId     mutexWebHandle;
extern osMessageQId  queueWebPckgHandle;
extern osSemaphoreId semCreateWebPckgHandle;

extern u8 isRxNewFirmware;

extern CircularBuffer circBufAllPckgs;

u8 bufTxData[256];

void taskKeepAlive(void const* argument) {
    u16 timeout = 1;
    vTaskSuspend(keepAliveHandle);

    for (;;) {
        // if ((timeout % 360 == 100) && !isRxNewFirmware) {
        //     LOG(LEVEL_MAIN, "getBsgNumFw\r\n\r\n");
        //     getBsgNumFw();
        // }
        // if ((timeout % 360 == 200) && !isRxNewFirmware) {
        //     LOG(LEVEL_MAIN, "getTabloNumFw\r\n\r\n");
        //     getTabloNumFw();
        // }
        if (!(timeout % 180) && !isRxNewFirmware) {
            LOG(LEVEL_MAIN, "sendMsgStatistics\r\n");
            sendMsgStatistics();
        }
        // if (!(timeout % 600) && !isRxNewFirmware) {
        //     LOG(LEVEL_MAIN, "\r\ngenerateMsgKeepAlive\r\n\r\n");
        //     generateMsgKeepAlive();
        // }
        // if (!(timeout % 5400) && !isRxNewFirmware) {
        //     LOG(LEVEL_MAIN, "updRTC\r\n\r\n");
        //     updRTC();
        // }

        timeout++;
        iwdgTaskReg |= IWDG_TASK_REG_ALIVE;
        osDelay(2000);
    }
}

void updRTC() {
    getServerTime();
}

void generateMsgKeepAlive() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BSG_ALIVE;
    pckgTel.data = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

void generateMsgFWUpdated() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bsg.idNewFirmware;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

void generateMsgDevOff() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BSG;
    pckgTel.data = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

void generateMsgTabloFW() {
    PckgTelemetry pckgTel;

    pckgTel.group = TEL_GR_PROJECT_127;
    pckgTel.code = TEL_CD_127_TABLO_FW;
    pckgTel.data = bsg.tablo.info.idFirmware;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_TABLO_BOOT_FW;
    pckgTel.data = bsg.tablo.info.idBoot;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_TABLO_BOOT_ERR;
    pckgTel.data = bsg.tablo.info.bootErr;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

ErrorStatus sendMsgTabloFW() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    u8            ptr = 0;
    u8*           idMCU;

    // idMCU = (u8*)&bsg.tablo.info.idMCU;
    idMCU = (u8*)&bsg.idMCU;

    LOG(LEVEL_MAIN, "sendMsgTabloFW\r\n");

    memset(bufTxData, 0, 128);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bsg.idNewFirmware;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_BSG;
    pckgTel.data = 0;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.group = TEL_GR_PROJECT_127;
    pckgTel.code = TEL_CD_127_TABLO_FW;
    pckgTel.data = bsg.tablo.info.idFirmware;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * ptr, ptr, idMCU);

    return ret;
}

ErrorStatus sendMsgFWUpdated() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    u8*           idMCU;

    // if (bsg.updTarget == UPD_TARGET_TABLO) {
    //     idMCU = (u8*)&bsg.tablo.info.idMCU;
    // } else if (bsg.updTarget == UPD_TARGET_BSG) {
    //     idMCU = (u8*)&bsg.idMCU;
    // } else {
    //     return ERROR;
    // }
    idMCU = (u8*)&bsg.idMCU;

    LOG(LEVEL_MAIN, "sendMsgFWUpdated\r\n");

    memset(bufTxData, 0, 20);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bsg.idNewFirmware;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(bufTxData, &pckgTel);

    pckgTel.code = TEL_CD_HW_BSG;
    pckgTel.data = 0;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * 2, 2, idMCU);

    return ret;
}

ErrorStatus sendMsgFWUpdateBegin() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    u8            ptr = 0;
    u8*           idMCU;

    // if (bsg.updTarget == UPD_TARGET_TABLO) {
    //     idMCU = (u8*)&bsg.tablo.info.idMCU;
    // } else if (bsg.updTarget == UPD_TARGET_BSG) {
    //     idMCU = (u8*)&bsg.idMCU;
    // } else {
    //     return ERROR;
    // }
    idMCU = (u8*)&bsg.idMCU;

    LOG(LEVEL_MAIN, "sendMsgFWUpdated\r\n");

    memset(bufTxData, 0, 64);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bsg.idNewFirmware;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_HW_UPDATE_LEN;
    pckgTel.data = bsg.szNewFirmware;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * ptr, ptr, idMCU);

    return ret;
}

ErrorStatus sendMsgDevOff() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;

    memset(bufTxData, 0, 20);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BSG;
    pckgTel.data = 0;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(bufTxData, &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY, 1, &bsg.idMCU);

    return ret;
}

ErrorStatus sendMsgStatistics() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;

    memset(bufTxData, 0, 256);
    pckgTel.group = TEL_GR_GENINF;
    pckgTel.code = TEL_CD_GENINF_GPS_INV_CNT;
    pckgTel.data = bsg.stat.gpsInvaligCount;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_GENINF_GPS_PARSE_ER_CNT;
    pckgTel.data = bsg.stat.gpsParseFailCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.group = TEL_GR_PROJECT_127_STAT;
    pckgTel.code = TEL_CD_127_IU_ERR;
    pckgTel.data = bsg.stat.tabloErrCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.group = TEL_GR_PROJECT_127_MEM;
    pckgTel.code = TEL_CD_127_BSG_PAGE_WR;
    pckgTel.data = bsg.stat.pageWrCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_BSG_PAGE_RD;
    pckgTel.data = bsg.stat.pageRdCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_BSG_PAGE_BAD;
    pckgTel.data = bsg.stat.pageBadCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_BSG_PAGE_FROM_IU;
    pckgTel.data = bsg.stat.pageFromIUCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.group = TEL_GR_SIMCOM;
    pckgTel.code = TEL_CD_127_SIM_SEND;
    pckgTel.data = bsg.stat.simSendCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_ERR;
    pckgTel.data = bsg.stat.simErrCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_RESET;
    pckgTel.data = bsg.stat.simResetCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_OPEN;
    pckgTel.data = bsg.stat.simOpenCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_BAD_CSQ;
    pckgTel.data = bsg.stat.simBadCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_LOW_CSQ;
    pckgTel.data = bsg.stat.simLowCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_GOOD_CSQ;
    pckgTel.data = bsg.stat.simGoodCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_127_SIM_HIGH_CSQ;
    pckgTel.data = bsg.stat.simHighCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_OPEN;
    pckgTel.data = bsg.timers.tcp_open_time / 1000;
    bsg.timers.tcp_open_time = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_CLOSE;
    pckgTel.data = bsg.timers.tcp_close_time / 1000;
    bsg.timers.tcp_close_time = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_SEND;
    pckgTel.data = bsg.timers.tcp_send_time / 1000;
    bsg.timers.tcp_send_time = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_ALL;
    bsg.timers.tcp_all_time = HAL_GetTick() - bsg.timers.tcp_all_time;
    pckgTel.data = bsg.timers.tcp_all_time / 1000;
    bsg.timers.tcp_all_time = HAL_GetTick();
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    return ret;
}