#ifndef I2C2LIB_H_
#define I2C2LIB_H_

#include "stm32l1xx.h"
//------------------------------------------------------------------
#define I2C_READ_TIMEOUT 10000
#define DMA_TIMEOUT 200
#define I2C_TIMEOUT 2000
//------------------------------------------------------------------

void I2C2_Init(void);

void I2C2_clearReadRegister();

void I2C2_BytesWrite(uint8_t slaveAddr, uint8_t pBuffer[], uint8_t length,uint8_t writeAddr);

void I2C2_WriteRegisterValue(uint8_t slaveAddr,uint8_t regAddr,uint8_t val);

void I2C2_DMA_Read(uint8_t slaveAddr, uint8_t readAddr,uint8_t numberBytesReceive);

uint8_t I2C2_ReadRegister8(uint8_t slaveAddr, uint8_t regAddr);

uint16_t I2C2_ReadRegister16(uint8_t slaveAddr, uint8_t regAddr);

uint8_t I2C2_ReadRegisterBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t pin);

#endif /* I2CLIB_H_ */
