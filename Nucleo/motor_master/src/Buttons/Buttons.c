#include <Buttons/Buttons.h>
void Buttons_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	buttons[0] = !(GPIOC->IDR & GPIO_Pin_6) && GPIO_Pin_6;
	buttons[1] = !(GPIOC->IDR & GPIO_Pin_8) && GPIO_Pin_8;
	buttons[2] = !(GPIOC->IDR & GPIO_Pin_9) && GPIO_Pin_9;
}
void Buttons_Interrupt(void){
	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		buttons[0] = !(GPIOC->IDR & GPIO_Pin_6) && GPIO_Pin_6;
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
		buttons[1] = !(GPIOC->IDR & GPIO_Pin_8) && GPIO_Pin_8;
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
		buttons[2] = !(GPIOC->IDR & GPIO_Pin_9) && GPIO_Pin_9;
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
}
