#include "../Tasks/Inc/task_get_new_bin.h"

#include "../Tasks/Inc/task_get_train_data.h"
#include "../Tasks/Inc/task_iwdg.h"
#include "../Tasks/Inc/task_keep_alive.h"
#include "../Utils/Inc/utils_pckgs_manager.h"

extern u16 iwdgTaskReg;

extern osThreadId webExchangeHandle;
extern osThreadId getGPSHandle;
extern osThreadId getNewBinHandle;
extern osThreadId keepAliveHandle;
extern osThreadId createWebPckgHandle;
extern osThreadId getTrainDataHandle;

extern osMutexId mutexGPSBufHandle;
extern osMutexId mutexWebHandle;
extern osMutexId mutexRTCHandle;
extern osMutexId mutexSpiFlashHandle;

extern CircularBuffer circBufAllPckgs;

u8                     isRxNewFirmware = 0;
static u8              bufNumBytesFirmware[8];
static PckgUpdFirmware pckgInfoFirmware;
static u8              partFirmware[SZ_PART_FIRMW + 1];
static u32             flashAddrFirmware = FLASH_ADDR_BUF_NEW_FIRMWARE;
static u32             szSoft = 0;

void big_update_func();

void taskGetNewBin(void const* argument) {
    FLASH_Erase_Sector(FLASH_SECTOR_1, VOLTAGE_RANGE_3);

    vTaskSuspend(getNewBinHandle);
    D(printf("taskGetNewBin\r\n"));

    lockAllTasks();
    isRxNewFirmware = 1;

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_NEW_BIN;
        big_update_func();
        osDelay(5000);
    }

    /* USER CODE END taskGetNewBin */
}

void big_update_func() {
    u32 curSzSoft = 0;
    u32 szPartSoft;
    u8  cntFailTCPReq = 0;
    u8  cntFailTablo = 0;
    u8  res;

    bsg.isTCPOpen = 0;
    while (openTcp() != TCP_OK) {}

    while (!(szSoft = getSzFirmware())) {}

    flashClearPage(FLASH_SECTOR_6);
    flashClearPage(FLASH_SECTOR_7);
    clearAllWebPckgs();

    if (bsg.updTarget == UPD_TARGET_TABLO) {
        res = 0;
        cntFailTablo = 0;
        while (!res) {
            res = tablo_send_request(CMD_NEW_FW, NULL, 0);
            cntFailTablo++;
            if (cntFailTablo > 5) {
                return;
            }
        }

        res = 0;
        cntFailTablo = 0;
        while (!res || szSoft != bsg.tablo.fwSize) {
            res = tablo_send_request(CMD_FW_LEN, (u8*)&szSoft, sizeof(u32));
            cntFailTablo++;
            if (cntFailTablo > 5) {
                return;
            }
        }
    }

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_NEW_BIN;
        if (szSoft != curSzSoft) {
            if (szSoft - curSzSoft > SZ_PART_FIRMW) {
                szPartSoft = SZ_PART_FIRMW;
            } else {
                szPartSoft = szSoft - curSzSoft;
            }
            pckgInfoFirmware.fromByte = curSzSoft;
            pckgInfoFirmware.toByte = szPartSoft + curSzSoft;
            memcpy(bufNumBytesFirmware, &pckgInfoFirmware.fromByte, 4);
            memcpy(bufNumBytesFirmware + 4, &pckgInfoFirmware.toByte, 4);
            memset(partFirmware, 0xFF, SZ_PART_FIRMW + 1);

            if (!bsg.isTCPOpen) {
                while (openTcp() != TCP_OK) {}
                cntFailTCPReq = 0;
            }

            osDelay(100);
            if (getPartFirmware(bufNumBytesFirmware, partFirmware, szPartSoft + 4, 8) == SUCCESS &&
                isCrcOk(partFirmware, szPartSoft)) {
                // HAL_GPIO_TogglePin(LED4G_GPIO_Port, LED4G_Pin);
                cntFailTCPReq = 0;

                if (bsg.updTarget == UPD_TARGET_TABLO) {
                    res = 0;
                    cntFailTablo = 0;
                    while (!res || (curSzSoft + szPartSoft) != bsg.tablo.fwCurSize) {
                        res = tablo_send_fw_part(curSzSoft, partFirmware, szPartSoft);
                        cntFailTablo++;
                        if (cntFailTablo > 5) {
                            return;
                        }
                    }
                } else if (bsg.updTarget == UPD_TARGET_BSG) {
                    flashWrite(partFirmware, szPartSoft, &flashAddrFirmware);
                }

                curSzSoft += szPartSoft;
                D(printf("OK: DOWNLOAD %d BYTES\r\n", (int)curSzSoft));
            } else {
                D(printf("ERROR: httpPost() DOWNLOAD\r\n"));
                cntFailTCPReq++;
                if (cntFailTCPReq > 10) {
                    cntFailTCPReq = 0;
                    simReset();
                }
            }
        } else {
            if (!bsg.isTCPOpen) {
                while (openTcp() != TCP_OK) {}
            }
            if (sendMsgFWUpdated() != SUCCESS) {
                D(printf("ERROR: Send FW UPDATED\r\n"));
            }
            D(printf("DOWNLOAD COMPLETE\r\n"));

            if (bsg.updTarget == UPD_TARGET_TABLO) {
                res = 0;
                cntFailTablo = 0;
                while (!res) {
                    res = tablo_send_request(CMD_FW_END, NULL, 0);
                    cntFailTablo++;
                    if (cntFailTablo > 5) {
                        return;
                    }
                }
            } else if (bsg.updTarget == UPD_TARGET_BSG) {
                updBootInfo();
            }

            osDelay(1000);
            NVIC_SystemReset();
        }
    }
}

void updBootInfo() {
    szSoft = szSoft % 4 == 0 ? szSoft : ((szSoft / 4) + 1) * 4;
    while (HAL_FLASH_Unlock() != HAL_OK) D(printf("ERROR: HAL_FLASH_Unlock()\r\n"));
    FLASH_Erase_Sector(FLASH_SECTOR_1, VOLTAGE_RANGE_3);
    // D(printf("FLASH_Erase_Sector\r\n"));
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_ID_BOOT, BSG_ID_BOOT))
        D(printf("ERROR: HAL_FLASH_Program(BOOT_ADDR_ID_LOADER)\r\n"));
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_IS_NEW_FIRWARE, (u32)1))
        D(printf("ERROR: HAL_FLASH_Program(BOOT_ADDR_IS_NEW_FIRWARE)\r\n"));
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_SZ_NEW_FIRMWARE, (u32)(szSoft)))
        D(printf("ERROR: HAL_FLASH_Program(FLASH_ADDR_SZ_NEW_FIRMWARE)\r\n"));
    while (HAL_FLASH_Lock() != HAL_OK) D(printf("ERROR: HAL_FLASH_Lock()\r\n"));
    D(printf("BOOT_ID: %d\r\n", (int)getFlashData(FLASH_ADDR_ID_BOOT)));
    D(printf("IS_NEW_FIRMARE: %d\r\n", (int)getFlashData(FLASH_ADDR_IS_NEW_FIRWARE)));
}

void lockAllTasks() {
    osMutexWait(mutexGPSBufHandle, osWaitForever);
    osMutexWait(mutexRTCHandle, osWaitForever);
    osMutexWait(mutexSpiFlashHandle, osWaitForever);
    osMutexWait(mutexWebHandle, osWaitForever);

    vTaskSuspend(webExchangeHandle);
    vTaskSuspend(getGPSHandle);
    vTaskSuspend(keepAliveHandle);
    vTaskSuspend(createWebPckgHandle);
    vTaskSuspend(getTrainDataHandle);

    osMutexRelease(mutexGPSBufHandle);
    osMutexRelease(mutexRTCHandle);
    osMutexRelease(mutexSpiFlashHandle);
    osMutexRelease(mutexWebHandle);
}

u32 getSzFirmware() {
    u8  bufSzFirmware[4];
    u8* idMCU;
    if (bsg.updTarget == UPD_TARGET_TABLO) {
        idMCU = (u8*)&bsg.tablo.info.idMCU;
    } else if (bsg.updTarget == UPD_TARGET_BSG) {
        idMCU = (u8*)&bsg.idMCU;
    } else {
        return ERROR;
    }

    if (generateWebPckgReq(CMD_REQUEST_SZ_FIRMWARE, NULL, 0, SZ_REQUEST_GET_SZ_FIRMWARE, bufSzFirmware, 4, idMCU) == ERROR) {
        D(printf("ERROR: sz firmware\r\n"));
        return 0;
    } else {
        u32 numFirmware = bufSzFirmware[0] << 24 | bufSzFirmware[1] << 16 | bufSzFirmware[2] << 8 | bufSzFirmware[3];
        D(printf("OK: sz firmware %d\r\n", numFirmware));
        return numFirmware;
    }
}

ErrorStatus getPartFirmware(u8* reqData, u8* answBuf, u16 szAnsw, u8 szReq) {
    WebPckg*    curPckg;
    ErrorStatus ret = SUCCESS;

    u8* idMCU;
    if (bsg.updTarget == UPD_TARGET_TABLO) {
        idMCU = (u8*)&bsg.tablo.info.idMCU;
    } else if (bsg.updTarget == UPD_TARGET_BSG) {
        idMCU = (u8*)&bsg.idMCU;
    } else {
        return ERROR;
    }

    curPckg = createWebPckgReq(CMD_REQUEST_PART_FIRMWARE, reqData, szReq, SZ_REQUEST_GET_PART_FIRMWARE, idMCU);
    osMutexWait(mutexWebHandle, osWaitForever);
    if (sendTcp(curPckg->buf, curPckg->shift) != TCP_OK) {
        D(printf("ERROR: part Firmware\r\n"));
        ret = ERROR;
    } else {
        waitIdleCnt("wait IDLE part firmware", &(uInfoSim.irqFlags), szAnsw / SZ_TCP_PCKG, 100, 10000);
        osDelay(100);
        memcpy(answBuf, &uInfoSim.pRxBuf[11], szAnsw);
    }
    osMutexRelease(mutexWebHandle);
    clearWebPckg(curPckg);
    return ret;
}
