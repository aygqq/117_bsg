#include "../Utils/Inc/utils_pckgs_manager.h"

#include "../Utils/Inc/utils_bsg.h"

WebPckg             webPckgs[CNT_WEBPCKGS + 1];
static u16          webPream = BSG_PREAMBLE;
static u8           endBytes[] = {0x0D, 0x0A};  // reverse order
extern osMessageQId queueWebPckgHandle;
extern osMutexId    mutexWebHandle;

void addInfo(WebPckg* pPckg, u8* src, u16 sz) {
    memcpy(&pPckg->buf[pPckg->shift], src, sz);
    pPckg->shift += sz;
}

void clearWebPckg(WebPckg* pPckg) {
    memset(pPckg->buf, '\0', SZ_WEB_PCKG);
    pPckg->shift = 0;
    pPckg->isFull = 0;
}

void clearAllWebPckgs() {
    for (u8 i = 0; i < CNT_WEBPCKGS; i++) {
        clearWebPckg(&webPckgs[i]);
    }
}

void initWebPckg(WebPckg* pPckg, u16 len, u8 isReq, u8* idMCU) {
    static u32 num = 0;
    num++;
    memset(pPckg->buf, '\0', SZ_WEB_PCKG);
    pPckg->shift = 0;
    pPckg->isRequest = isReq;
    addInfo(pPckg, (u8*)&webPream, 2);
    addInfo(pPckg, (u8*)&num, 4);
    addInfo(pPckg, (u8*)idMCU, 12);
    addInfo(pPckg, (u8*)&len, 2);
}

void addInfoToWebPckg(WebPckg* pPckg, u8* src, u16 sz, u8 cnt, u8 cmdData) {
    addInfo(pPckg, &cmdData, 1);
    addInfo(pPckg, &cnt, 1);
    addInfo(pPckg, src, sz);
}

void showWebPckg(WebPckg* pPckg) {
    // for (u16 i = 0; i < pPckg->shift; i++) {
    //     printf("%02x", pPckg->buf[i]);
    // }
    // printf("\r\n");
}

void closeWebPckg(WebPckg* pPckg) {
    addInfo(pPckg, (u8*)endBytes, 2);
    pPckg->isFull = 1;
}

void freeWebPckg(WebPckg* pckg) {
    pckg->isFull = 0;
}

WebPckg* getFreePckg() {
    for (u8 i = 0; i < CNT_WEBPCKGS; i++) {
        if (!webPckgs[i].isFull) {
            webPckgs[i].isFull = 1;
            return &webPckgs[i];
        }
    }
    // LOG_WEB(LEVEL_INFO, "ER: NO FREEPCKG\r\n"));
    return NULL;
}

WebPckg* getFreePckgReq() {
    if (!webPckgs[CNT_WEBPCKGS].isFull) {
        webPckgs[CNT_WEBPCKGS].isFull = 1;
        return &webPckgs[CNT_WEBPCKGS];
    }

    // LOG_WEB(LEVEL_INFO, "ER: NO FREEPCKG\r\n"));
    return NULL;
}

u8 getCntFreePckg() {
    u8 ret = 0;
    for (u8 i = 0; i < CNT_WEBPCKGS; i++) {
        if (!webPckgs[i].isFull) ret++;
    }
    return ret;
}

void waitAnswServer(u8 req) {
    switch (req) {
        case CMD_REQUEST_SERVER_TIME:
        case CMD_REQUEST_NUM_FIRMWARE:
            osDelay(2000);
            break;
        case CMD_REQUEST_SZ_FIRMWARE:
            osDelay(2000);
            break;
        case CMD_REQUEST_PART_FIRMWARE:
            osDelay(2000);
            break;
    }
}

WebPckg* createWebPckgReq(u8 CMD_REQ, u8* data, u8 sz, u8 szReq, u8* idMCU) {
    u8       req[10];
    WebPckg* curPckg;
    req[0] = CMD_REQ;
    req[1] = 1;
    curPckg = getFreePckg();
    initWebPckg(curPckg, szReq, 1, idMCU);
    if (sz) {
        memcpy(req + 2, data, sz);
    }
    addInfo(curPckg, req, szReq);
    closeWebPckg(curPckg);
    // showWebPckg(curPckg);
    return curPckg;
}

ErrorStatus sendWebPckgData(u8 CMD_DATA, u8* data, u8 sz, u8 szReq, u8* idMCU) {
    WebPckg*    curPckg;
    u8          req[64];
    ErrorStatus ret = SUCCESS;

    req[0] = CMD_DATA;
    req[1] = szReq;
    curPckg = getFreePckg();
    initWebPckg(curPckg, sz + 2, 0, &bsg.idMCU);
    if (sz) {
        memcpy(req + 2, data, sz);
    }
    addInfo(curPckg, req, sz + 2);
    closeWebPckg(curPckg);
    // showWebPckg(curPckg);

    osMutexWait(mutexWebHandle, osWaitForever);
    if (sendTcp(curPckg->buf, curPckg->shift) != TCP_OK) {
        LOG_WEB(LEVEL_ERROR, "send data failed\r\n");
        ret = ERROR;
    }
    osMutexRelease(mutexWebHandle);
    clearWebPckg(curPckg);
    return ret;
}

ErrorStatus generateWebPckgReq(u8 CMD_REQ, u8* data, u8 sz, u8 szReq, u8* answ, u16 szAnsw, u8* idMCU) {
    ErrorStatus ret = SUCCESS;
    u8          statSend;
    u8          req[10];
    WebPckg*    curPckg;

    req[0] = CMD_REQ;
    req[1] = 1;
    if ((curPckg = getFreePckgReq()) != NULL) {
        initWebPckg(curPckg, szReq, 1, idMCU);
        if (sz) {
            memcpy(req + 2, data, sz);
        }
        addInfo(curPckg, req, szReq);
        closeWebPckg(curPckg);
        // showWebPckg(curPckg);

        osMutexWait(mutexWebHandle, osWaitForever);
        bsg.isTCPOpen = 0;
        statSend = sendTcp(curPckg->buf, curPckg->shift);
        if (statSend != TCP_OK)
            ret = ERROR;
        else if (uInfoSim.pRxBuf[11] == '\0' && uInfoSim.pRxBuf[12] == '\0' &&
                 uInfoSim.pRxBuf[13] == '\0' && uInfoSim.pRxBuf[14] == '\0') {
            uInfoSim.irqFlags.isIrqIdle = 0;
            if (waitIdle("wait req answ", &(uInfoSim.irqFlags), 100, 10000)) {
                memcpy(answ, &uInfoSim.pRxBuf[11], szAnsw);
            } else {
                LOG_WEB(LEVEL_ERROR, "NO ANSW REQ\r\n");
                ret = ERROR;
                bsg.isTCPOpen = 0;
            }
        } else {
            osDelay(10);
            memcpy(answ, &uInfoSim.pRxBuf[11], szAnsw);
        }
        clearWebPckg(curPckg);
        closeTcp();
        osMutexRelease(mutexWebHandle);
    } else {
        ret = ERROR;
        LOG_WEB(LEVEL_ERROR, "NO FREE PCKG\r\n");
    }

    return ret;
}