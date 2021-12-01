#include "../Tasks/Inc/task_get_train_data.h"

#include "../Tasks/Inc/task_get_gps.h"
#include "../Tasks/Inc/task_iwdg.h"
#include "../Tasks/Inc/task_keep_alive.h"
#include "../Utils/Inc/circularBuffer.h"
#include "../Utils/Inc/utils_bsg.h"
#include "../Utils/Inc/utils_crc.h"
#include "cmsis_os.h"

extern u16 iwdgTaskReg;

extern osThreadId    getTrainDataHandle;
extern osSemaphoreId uart6RXSemHandle;

extern UartInfo uInfoTablo;

extern CircularBuffer circBufAllPckgs;

u8 tmpBufPage[SZ_PAGE];

void tablo_send_pack(UartInfo* pUInf, u16 len);
u16  tablo_create_request(u8* request, u8 cmd, u8* data, u8 len);
u8   tablo_parse_responce(UartInfo* pUInf);

void taskGetTrainData(void const* argument) {
    // u16        iter = 0;
    // u8         res;
    // u32        timeStamp;
    // gps_body_t gps_body;
    osSemaphoreWait(uart6RXSemHandle, osWaitForever);

    vTaskSuspend(getTrainDataHandle);
    bsg.tablo.initStep = IU_INIT_NONE;

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_TABLO;
        osDelay(60000);
        generateTestPackage();

        // switch (bsg.tablo.initStep) {
        //     case IU_INIT_NONE:
        //         LOG(LEVEL_MAIN, "IU_INIT_NONE\r\n");
        //         gps_body.valid = bsg.cur_gps.valid;
        //         gps_body.speed = bsg.cur_gps.coords.speed;
        //         gps_body.stopTime = bsg.cur_gps.stopTime;

        //         if (tablo_send_request(CMD_GNSS_SHORT, (u8*)&gps_body, sizeof(gps_body_t))) {
        //             bsg.tablo.initStep = IU_INIT_TIMESYNC;
        //         } else {
        //             osDelay(1000);
        //         }
        //         bsg.tablo.initStep = IU_INIT_TIMESYNC;
        //         break;
        //     case IU_INIT_TIMESYNC:
        //         LOG(LEVEL_MAIN, "IU_INIT_TIMESYNC\r\n");
        //         timeStamp = getUnixTimeStamp();
        //         if (tablo_send_request(CMD_SYNC, (u8*)&timeStamp, sizeof(timeStamp))) {
        //             bsg.tablo.initStep = IU_INIT_GET_INFO;
        //         } else {
        //             osDelay(1000);
        //         }
        //         break;
        //     case IU_INIT_GET_INFO:
        //         LOG(LEVEL_MAIN, "IU_INIT_GET_INFO\r\n");
        //         bsg.tablo.info.idFirmware = 0;
        //         res = tablo_send_request(CMD_GET_INFO, NULL, 0);
        //         if (res && bsg.tablo.info.idFirmware != 0 && bsg.tablo.info.idMCU[0] != 0) {
        //             bsg.tablo.initStep = IU_INIT_COMPLETE;
        //         }

        //         generateMsgTabloFW();
        //         break;
        //     case IU_INIT_COMPLETE:
        //         if (iter % 10000 == 7000) {
        //             timeStamp = getUnixTimeStamp();
        //             tablo_send_request(CMD_SYNC, (u8*)&timeStamp, sizeof(timeStamp));
        //         } else if (iter % 50 == 40) {
        //             gps_body.valid = bsg.cur_gps.valid;
        //             gps_body.speed = bsg.cur_gps.coords.speed;
        //             gps_body.stopTime = bsg.cur_gps.stopTime;
        //             tablo_send_request(CMD_GNSS_SHORT, (u8*)&gps_body, sizeof(gps_body_t));
        //         } else if (iter % 5000 == 3013) {
        //             tablo_send_request(CMD_GET_INFO, NULL, 0);
        //         }
        //         osDelay(100);
        //         tablo_send_request(CMD_DATA, (u8*)&uInfoTablo.szRxBuf, sizeof(uInfoTablo.szRxBuf));
        //         iter++;
        //         break;
        //     default:
        //         bsg.tablo.initStep = IU_INIT_NONE;
        //         break;
        // }
        // osDelay(100);
    }
}

u8 tablo_send_request(u8 cmd, u8* data, u8 len) {
    u16 req_len;
    u8  res = 0;
    req_len = tablo_create_request(uInfoTablo.pTxBuf, cmd, data, len);

    osSemaphoreWait(uart6RXSemHandle, 0);
    tablo_send_pack(&uInfoTablo, req_len);
    if (osSemaphoreWait(uart6RXSemHandle, 5000) == osOK) {
        __HAL_DMA_DISABLE(uInfoTablo.pHuart->hdmarx);
        res = tablo_parse_responce(&uInfoTablo);
        __HAL_DMA_SET_COUNTER(uInfoTablo.pHuart->hdmarx, uInfoTablo.szRxBuf);
        __HAL_DMA_DISABLE(uInfoTablo.pHuart->hdmarx);
    } else {
        bsg.stat.tabloErrCnt++;
    }
    return res;
}

u16 tablo_create_request(u8* request, u8 cmd, u8* data, u8 len) {
    u16          ptr = 0;
    cmd_header_t hdr;

    // LOG(LEVEL_DEBUG, "tablo_create_request %d\r\n", cmd);

    hdr.cmd_type = cmd;
    hdr.length = len;
    memcpy(&request[ptr], &hdr, CMD_HDR_SIZE);
    ptr += CMD_HDR_SIZE;

    memcpy(&request[ptr], data, len);
    ptr += len;

    return ptr;
}

void tablo_send_pack(UartInfo* pUInf, u16 len) {
    uint16_t crc16;

    crc16 = calcCrc16(pUInf->pTxBuf, len);
    pUInf->pTxBuf[len] = crc16 & 0x00ff;
    pUInf->pTxBuf[len + 1] = (crc16 & 0xff00) >> 8;

    uart6Tx(pUInf->pTxBuf, len + CMD_CRC_SIZE, pUInf);
}

u8 tablo_send_fw_part(u32 write_from, u8* data, u16 len) {
    u16          ptr = 0;
    cmd_header_t hdr;
    u32          write_to;
    u8           res = 0;

    LOG(LEVEL_MAIN, "tablo send FW part %d %d\r\n", write_from, len);

    hdr.cmd_type = CMD_FW_PART;
    hdr.length = len + 4 + 4;
    memcpy(&uInfoTablo.pTxBuf[ptr], &hdr, CMD_HDR_SIZE);
    ptr += CMD_HDR_SIZE;

    memcpy(&uInfoTablo.pTxBuf[ptr], &write_from, sizeof(u32));
    ptr += sizeof(u32);
    write_to = write_from + len;
    memcpy(&uInfoTablo.pTxBuf[ptr], &write_to, sizeof(u32));
    ptr += sizeof(u32);
    memcpy(&uInfoTablo.pTxBuf[ptr], data, len);
    ptr += len;

    tablo_send_pack(&uInfoTablo, ptr);
    if (osSemaphoreWait(uart6RXSemHandle, 5000) == osOK) {
        __HAL_DMA_DISABLE(uInfoTablo.pHuart->hdmarx);
        res = tablo_parse_responce(&uInfoTablo);
        __HAL_DMA_SET_COUNTER(uInfoTablo.pHuart->hdmarx, uInfoTablo.szRxBuf);
        __HAL_DMA_DISABLE(uInfoTablo.pHuart->hdmarx);
    } else {
        bsg.stat.tabloErrCnt++;
    }
    return res;
}

void parse_tablo_data(u8* data_all, u16 len_all) {
    u16 ptr = 0;
    u16 len_pg;
    u8* data;
    u16 bufEnd[2] = {0, BSG_PREAMBLE};

    while (ptr < len_all) {
        len_pg = (data_all[ptr] + (data_all[ptr + 1] << 8));
        ptr += 2;
        data = (u8*)(&data_all[ptr]);

        if (len_pg > SZ_PAGE) {
            bsg.stat.tabloErrCnt++;
            LOG(LEVEL_ERROR, "Returned big page (len %d)\r\n", len_pg);
            return;
        }

        bsg.stat.pageFromIUCount++;
        memset(tmpBufPage, 0, SZ_PAGE);
        memcpy(tmpBufPage, data, len_pg);
        bufEnd[0] = calcCrc16(data, len_pg);
        memcpy(&tmpBufPage[len_pg], bufEnd, 4);
        spiFlashWriteNextPg(tmpBufPage, len_pg + 4, 0);

        LOG(LEVEL_DEBUG, "Returned page (len %d)\r\n", len_pg);

        ptr += len_pg;
    }
}

void parse_tablo_sync(u8* data_all, u16 len_all) {
    LOG(LEVEL_INFO, "Time syncronized\r\n");
}

void parse_tablo_gnss(u8* data_all, u16 len_all) {
    LOG(LEVEL_INFO, "Coordinates sended\r\n");
}

void parse_tablo_error(u8* data_all, u16 len_all) {
    u8 error_type = data_all[0];

    if (error_type == ERROR_CAPACITY) {
        bsg.stat.tabloErrCnt++;
        LOG(LEVEL_ERROR, "Returned error type %d\r\n", error_type);
    } else if (error_type == ERROR_NO_DATA) {
        LOG(LEVEL_DEBUG, "Returned error type %d\r\n", error_type);
    } else if (error_type == ERROR_NO_SYNC) {
        bsg.tablo.initStep = IU_INIT_TIMESYNC;
    }
}

void parse_tablo_iu_info(u8* data_all, u16 len_all) {
    iu_info_t* info = (iu_info_t*)data_all;

    memcpy(&bsg.tablo.info, data_all, sizeof(iu_info_t));
    LOG(LEVEL_INFO, "Returned Tablo UID: %08x%08x%08x\r\n", (uint)info->idMCU[0], (uint)info->idMCU[1], (uint)info->idMCU[2]);
    LOG(LEVEL_INFO, "Returned Tablo FW Num: %d, boot %d, err %d\r\n", info->idFirmware, info->idBoot, info->bootErr);
}

void parse_tablo_gnss_short(u8* data_all, u16 len_all) {
    // LOG(LEVEL_INFO, "Short GEO sended\r\n");
}

void parse_tablo_new_fw(u8* data_all, u16 len_all) {
    LOG(LEVEL_INFO, "FW download sterted\r\n");
}

void parse_tablo_fw_num(u8* data_all, u16 len_all) {
}

void parse_tablo_fw_len(u8* data_all, u16 len_all) {
    bsg.tablo.fwSize = *(u32*)data_all;
    bsg.tablo.fwCurSize = 0;
    bsg.tablo.updStep = UPD_DOWNLOAD;
    LOG(LEVEL_INFO, "Returned Tablo FW Len %d\r\n", bsg.tablo.fwSize);
}

void parse_tablo_fw_part(u8* data_all, u16 len_all) {
    u32 write_from, write_to;

    write_from = *(u32*)data_all;
    write_to = *(u32*)(data_all + 4);
    bsg.tablo.fwCurSize = *(u32*)(data_all + 8);
    bsg.tablo.updStep = UPD_DOWNLOAD;

    LOG(LEVEL_INFO, "Returned Tablo FW Part %d %d %d\r\n", write_from, write_to, bsg.tablo.fwCurSize);
}

void parse_tablo_fw_end(u8* data_all, u16 len_all) {
    bsg.tablo.updStep = UPD_FINISH;
    LOG(LEVEL_INFO, "Returned Tablo FW Len %d\r\n", bsg.tablo.fwSize);
}

u8 tablo_parse_responce(UartInfo* pUInf) {
    cmd_header_t* hdr = (cmd_header_t*)pUInf->pRxBuf;
    u8*           data;
    u16           crc_calc;

    u16 pkt_len = uInfoTablo.szRxBuf - pUInf->pHuart->hdmarx->Instance->NDTR;

    if (pkt_len < CMD_HDR_SIZE + CMD_CRC_SIZE) {
        bsg.stat.tabloErrCnt++;
        LOG(LEVEL_ERROR, "Short answ, len %d\r\n", pkt_len);
        return 0;
    }

    crc_calc = calcCrc16(pUInf->pRxBuf, (hdr->length + CMD_HDR_SIZE + CMD_CRC_SIZE));
    if (crc_calc != 0) {
        bsg.stat.tabloErrCnt++;
        LOG(LEVEL_ERROR, "Bad crc, len %d\r\n", pkt_len);
        return 0;
    }

    data = pUInf->pRxBuf + CMD_HDR_SIZE;
    data[hdr->length] = 0;

    switch (hdr->cmd_type) {
        case CMD_DATA:
            parse_tablo_data(data, hdr->length);
            break;
        case CMD_TELEMETRY:
            break;
        case CMD_CFG:
            break;
        case CMD_SYNC:
            parse_tablo_sync(data, hdr->length);
            break;
        case CMD_GNSS:
            parse_tablo_gnss(data, hdr->length);
            break;
        case CMD_ERROR:
            parse_tablo_error(data, hdr->length);
            return 0;
            break;
        case CMD_GNSS_SHORT:
            parse_tablo_gnss_short(data, hdr->length);
            break;
        case CMD_NEW_FW:
            parse_tablo_new_fw(data, hdr->length);
            break;
        case CMD_GET_INFO:
            parse_tablo_iu_info(data, hdr->length);
            break;
        case CMD_FW_LEN:
            parse_tablo_fw_len(data, hdr->length);
            break;
        case CMD_FW_PART:
            parse_tablo_fw_part(data, hdr->length);
            break;
        case CMD_FW_END:
            parse_tablo_fw_end(data, hdr->length);
            break;
        default:
            bsg.stat.tabloErrCnt++;
            return 0;
            break;
    }
    return 1;
}
