/*
 * FlashSaver.h
 *
 *  Created on: 1 дек. 2021 г.
 *      Author: Alex-VID
 */

#ifndef FLASH_WORKER_H_
#define FLASH_WORKER_H_

#include "FlashWorkerConf.h"

void FlashWorker_save(void* s,  uint16_t sizeByte);
uint8_t FlashWorker_load(void *s, uint16_t sizeByte);

int16_t FlashWorker_getAvailableRecords(uint16_t sizeByte);

#endif /* FLASHSAVER_H_ */
