#include <Leds/Leds.h>

void Leds_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15
			| GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	setLed(1, OFF, OFF);
	setLed(2, OFF, OFF);
	setLed(3, OFF, OFF);
}

void setLed(char index, uint8_t green, uint8_t red) {
	switch (index) {
	case 0:
		if (green == ON)
			GPIOB->BSRRL = GPIO_Pin_14;
		else
			GPIOB->BSRRH = GPIO_Pin_14;
		if (red == ON)
			GPIOB->BSRRL = GPIO_Pin_13;
		else
			GPIOB->BSRRH = GPIO_Pin_13;
		break;
	case 1:
		if (green == ON)
			GPIOB->BSRRL = GPIO_Pin_15;
		else
			GPIOB->BSRRH = GPIO_Pin_15;
		if (red == ON)
			GPIOB->BSRRL = GPIO_Pin_1;
		else
			GPIOB->BSRRH = GPIO_Pin_1;
		break;
	case 2:
		if (green == ON)
			GPIOB->BSRRL = GPIO_Pin_2;
		else
			GPIOB->BSRRH = GPIO_Pin_2;
		if (red == ON)
			GPIOB->BSRRL = GPIO_Pin_12;
		else
			GPIOB->BSRRH = GPIO_Pin_12;
		break;
	}
}
