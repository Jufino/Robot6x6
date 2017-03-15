#include <I2CMasterLib/I2CMasterLib.h>

volatile static uint8_t readReg = 0;
volatile static uint32_t I2C_Rx_Buffer[5];

void I2C2_initDMA(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel5); //reset DMA1 channe1 to default values;

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) 0x40005810; //=0x40005810 : address of data reading register of I2C2
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) I2C_Rx_Buffer; //variable to store data
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for peripheral to memory transfer
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //setting normal mode (non circular)
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	//medium priority
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_BufferSize = 2;	//number of data to be transfered
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//automatic memory increment enable for memory
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//source peripheral data size = 8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//destination memory data size = 8bit
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn; //I2C2 connect to channel 5 of DMA1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void I2C2_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	//GPIO port, PIN, AF Function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;

	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	I2C2_initDMA();
}

void I2C2_BytesWrite(uint8_t slaveAddr, uint8_t pBuffer[], uint8_t length,
		uint8_t writeAddr) {
	uint16_t timeout_var = 0;

	I2C_GenerateSTART(I2C2, ENABLE);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			return;
		}
	}
	timeout_var = 0;

	I2C_Send7bitAddress(I2C2, slaveAddr, I2C_Direction_Transmitter);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			return;
		}
	}
	timeout_var = 0;

	I2C_SendData(I2C2, writeAddr);
	for (int i = 0; i < length; i++) {
		I2C_SendData(I2C2, pBuffer[i]);
		for (int x = 0; x < 1000; x++)
			;
	}
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			return;
		}
	}

	I2C_GenerateSTOP(I2C2, ENABLE);

}

void I2C2_DMA_Read(uint8_t slaveAddr, uint8_t readAddr,
		uint8_t numberBytesReceive) {
	uint16_t timeout_var = 0;
	while (readReg != 0 && timeout_var++ < DMA_TIMEOUT)
		;
	timeout_var = 0;
	readReg = readAddr;

	/* Disable DMA channel*/
	DMA_Cmd(DMA1_Channel5, DISABLE);

	DMA_SetCurrDataCounter(DMA1_Channel5, numberBytesReceive);

	/* While the bus is busy */
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			readReg = 0;
			return;
		}
	}
	timeout_var = 0;

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE); //Note this one, very important

	/* Send START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			readReg = 0;
			return;
		}
	}
	timeout_var = 0;

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C2, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C2,
	I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			readReg = 0;
			return;
		}
	}

	timeout_var = 0;

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C2, ENABLE);

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C2, readAddr);

	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			readReg = 0;
			return;
		}
	}
	timeout_var = 0;

	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			readReg = 0;
			return;
		}
	}
	timeout_var = 0;

	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C2, slaveAddr, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if (I2C_TIMEOUT < timeout_var++) {
			I2C2_Init();
			readReg = 0;
			return;
		}
	}
	timeout_var = 0;

	/* Start DMA to receive data from I2C */
	DMA_Cmd(DMA1_Channel5, ENABLE);
	I2C_DMACmd(I2C2, ENABLE);
}

void I2C2_DMA_Stop(){
	DMA_ClearFlag(DMA1_FLAG_TC5);
	I2C_DMACmd(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);
	I2C2_clearReadRegister();
}

void I2C2_WriteRegisterValue(uint8_t slaveAddr, uint8_t regAddr, uint8_t val) {
	uint8_t tmp[1];
	tmp[0] = val;
	I2C2_BytesWrite(slaveAddr, tmp, 1, regAddr);
}

void I2C2_WriteRegister(uint8_t slaveAddr, uint8_t regAddr) {
	uint8_t tmp[1];
	I2C2_BytesWrite(slaveAddr, tmp, 0, regAddr);
}

void I2C2_clearReadRegister(){
	readReg = 0;
}

uint8_t I2C2_ReadRegister8(uint8_t slaveAddr, uint8_t regAddr) {
	I2C2_DMA_Read(slaveAddr, regAddr, 1);
	uint16_t timeout = 0;
	uint8_t wait = readReg;
	while (wait != 0){
		for(int i=0;i<10;i++);
		wait = readReg;
		if(timeout++ > DMA_TIMEOUT){
			I2C2_DMA_Stop();
			return 0;
		}
	}
	return I2C_Rx_Buffer[0];
}

uint16_t I2C2_ReadRegister16(uint8_t slaveAddr, uint8_t regAddr) {
	I2C2_DMA_Read(slaveAddr, regAddr, 2);
	uint16_t timeout = 0;
	uint8_t wait = readReg;
	while (wait != 0){
		for(int i=0;i<10;i++);
		wait = readReg;
		if(timeout++ > DMA_TIMEOUT){
			I2C2_DMA_Stop();
			return 0;
		}
	}
	return I2C_Rx_Buffer[0];
}

uint8_t I2C2_ReadRegisterBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t pin) {
	return (I2C2_ReadRegister8(slaveAddr, regAddr) & (1 << pin)) && (1 << pin);
}
