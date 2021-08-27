#ifndef __TASK_GET_NEW_BIN_H
#define __TASK_GET_NEW_BIN_H

#include "../Utils/Inc/utils_bsg.h"
#include "../Utils/Inc/utils_flash.h"
#include "cmsis_os.h"
#include "crc.h"
#include "main.h"

#define SZ_PART_FIRMW 1400
#define SZ_TCP_PCKG   1400

void updBootInfo();
void lockAllTasks();
u32  getSzFirmware();

ErrorStatus getPartFirmware(u8* reqData, u8* answBuf, u16 szAnsw, u8 szReq);

#endif