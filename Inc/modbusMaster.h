#ifndef __MODBUS_MASTER_
#define __MODBUS_MASTER_

#include "stm32f1xx_hal.h"
#include "main.h"


void modbusMasterScan(void);
void sendDataMaster03(uint8_t slaveAdd, uint16_t start, uint16_t num);
void sendDataMaster04(uint8_t slaveAdd, uint16_t start, uint16_t num);
void sendDataMaster16(uint8_t slaveAdd, uint16_t scrArray[], uint16_t start, uint16_t num);
void dataProcessing();

#endif // __MODBUS_MASTER_
