#ifndef I2C2LIB_H_
#define I2C2LIB_H_

#include "stm32l1xx.h"

void I2C2_Init(void);

uint8_t I2C2_getDeviceAddress(void);

void I2C2_clearDeviceAddress(void);

uint8_t I2C2_getReadRegister(void);

void I2C2_clearReadRegister(void);

uint32_t I2C2_getRxBuffer(int index);

void I2C2_BytesWrite(uint8_t slaveAddr, uint8_t pBuffer[], uint8_t length,uint8_t writeAddr);

void I2C2_DMA_Read(uint8_t slaveAddr, uint8_t readAddr,uint8_t numberBytesReceive);

#endif /* I2CLIB_H_ */
