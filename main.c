#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_i2c.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_syscfg.h>
#include <stm32f0xx_exti.h>

//HMC5883L I2C Pins
#define I_SDA 			GPIO_Pin_7
#define I_SCL 			GPIO_Pin_6

#define I_SDAPS			GPIO_PinSource7
#define I_SCLPS			GPIO_PinSource6

#define I_GPIO			GPIOB
#define I_GPIOAF		GPIO_AF_1

//HMC5883L Bytes to receive
#define HMC_BYTESTOREC	6

//HMC5883L DRDY Pin
#define HMC_DRDY		GPIO_Pin_0
#define HMC_DRDYPOS		EXTI_PortSourceGPIOA
#define HMC_DRDYPS		EXTI_PinSource0
#define HD_GPIO			GPIOA

#define I_I2C			I2C1
#define I_HMCADDR		(0x1E<<1)

volatile int16_t x, y, z;
volatile uint8_t done = 0;

//This function can be used to read a register in a blocking fashion - i.e not using interrupts
uint8_t H_BlockingReadReg(uint8_t Reg){
	uint8_t Dat;

	I2C_ITConfig(I_I2C, I2C_IT_TXIS, DISABLE);
	I2C_ITConfig(I_I2C, I2C_IT_TC, DISABLE);
	I2C_ITConfig(I_I2C, I2C_IT_RXNE, DISABLE);
	I2C_ITConfig(I_I2C, I2C_IT_STOPF, DISABLE);

	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I_I2C, Reg);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TC) == RESET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_RXNE) == RESET);

	Dat = I2C_ReceiveData(I_I2C);

	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I_I2C, I2C_FLAG_STOPF);

	I2C_ITConfig(I_I2C, I2C_IT_TXIS, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_TC, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_RXNE, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_STOPF, ENABLE);

	return Dat;
}

//This function can be used to write to a register in a blocking fashion - i.e not using interrupts
//This is mainly used for initialization.
void H_BlockingWriteReg(uint8_t Reg, uint8_t Dat){
	I2C_ITConfig(I_I2C, I2C_IT_TXIS, DISABLE);
	I2C_ITConfig(I_I2C, I2C_IT_TC, DISABLE);
	I2C_ITConfig(I_I2C, I2C_IT_RXNE, DISABLE);
	I2C_ITConfig(I_I2C, I2C_IT_STOPF, DISABLE);

	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I_I2C, Reg);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TCR) == RESET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I_I2C, Dat);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I_I2C, I2C_FLAG_STOPF);

	I2C_ITConfig(I_I2C, I2C_IT_TXIS, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_TC, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_RXNE, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_STOPF, ENABLE);
}

//This interrupt handler controls the I2C module for each subsequent step in transfering 6 bytes
void I2C1_IRQHandler(void){
	static uint8_t dcnt = 0;
	static uint8_t data[HMC_BYTESTOREC] = {0};

	//TXIS is the first interrupt to occur once the slave has ACK'd the address
	if(I2C_GetITStatus(I_I2C, I2C_IT_TXIS)){
		I2C_ClearITPendingBit(I_I2C, I2C_IT_TXIS);

		//After TXIS has been cleared, send the byte "3" as this is the start of the result registers
		I2C_SendData(I_I2C, 3);
	}
	else if(I2C_GetITStatus(I_I2C, I2C_IT_TC)){
		I2C_ClearITPendingBit(I_I2C, I2C_IT_TC);

		//Once the register location transfer has completed, TC will then go high.
		//Upon clearing the TC bit, start a new transfer receiving "HMC_BYTESTOREC" (6) bytes
		I2C_TransferHandling(I_I2C, I_HMCADDR, HMC_BYTESTOREC, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

		//Reset the data counter
		dcnt = 0;
	}
	else if(I2C_GetITStatus(I_I2C, I2C_IT_RXNE)){
		I2C_ClearITPendingBit(I_I2C, I2C_IT_RXNE);
		//For every received byte, an RXNE interrupt will be generated.
		//Upon clearing the RXNE flag, receive the byte, store it in "data" and increment dcnt
		data[dcnt++] = I2C_ReceiveData(I_I2C);
	}
	else if(I2C_GetITStatus(I_I2C, I2C_IT_STOPF)){
		I2C_ClearITPendingBit(I_I2C, I2C_IT_STOPF);

		//Final step of the I2C transfer process is upon a successful stop flag generation.
		//After clearing the stop flag flag (confusing!), shift and or the data into the correct
		//variables. Data is stored in an X, Z, Y fashion according to the HMC5883L datasheet.
		x = (data[0]<<8)|data[1];
		z = (data[2]<<8)|data[3];
		y = (data[4]<<8)|data[5];

		//Upon completion, set "done" flag
		done = 1;
	}
}

//This handler will be called upon a falling edge of the HMC_DRDY pin
void EXTI0_1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line0)){
		EXTI_ClearITPendingBit(EXTI_Line0);

		//Upon the pins falling edge, clear the done flag and start an I2C transaction
		done = 0;
		I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	}
}

GPIO_InitTypeDef G;
I2C_InitTypeDef I;
NVIC_InitTypeDef N;
EXTI_InitTypeDef E;

int main(void)
{
	//Set HSI as system clock
	RCC_HSICmd(ENABLE);
	while(!RCC_GetFlagStatus(RCC_FLAG_HSIRDY));
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

	//Enable required clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//Initialize I2C pins
	G.GPIO_Pin = I_SDA | I_SCL;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_OD;
	G.GPIO_PuPd = GPIO_PuPd_UP;
	G.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I_GPIO, &G);

	//Initialize DRDY input pin
	G.GPIO_Pin = HMC_DRDY;
	G.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(HD_GPIO, &G);

	GPIO_PinAFConfig(I_GPIO, I_SCLPS, I_GPIOAF);
	GPIO_PinAFConfig(I_GPIO, I_SDAPS, I_GPIOAF);

	//Initialize I2C module
	I.I2C_Ack = I2C_Ack_Enable;
	I.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
	I.I2C_Mode = I2C_Mode_I2C;
	I.I2C_OwnAddress1 = 0x01;
	I.I2C_Timing = 0x20000A0D;
	I2C_Init(I_I2C, &I);
	I2C_Cmd(I_I2C, ENABLE);

	I2C_ClearITPendingBit(I_I2C, I2C_IT_TXIS);
	I2C_ClearITPendingBit(I_I2C, I2C_IT_TC);
	I2C_ClearITPendingBit(I_I2C, I2C_IT_RXNE);
	I2C_ClearITPendingBit(I_I2C, I2C_IT_STOPF);

	//Enable required I2C interrupts
	I2C_ITConfig(I_I2C, I2C_IT_TXIS, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_TC, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_RXNE, ENABLE);
	I2C_ITConfig(I_I2C, I2C_IT_STOPF, ENABLE);

	N.NVIC_IRQChannel = I2C1_IRQn;
	N.NVIC_IRQChannelPriority = 1;
	N.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&N);

	//Configure the EXTI controller
	SYSCFG_EXTILineConfig(HMC_DRDYPOS, HMC_DRDYPS);
	E.EXTI_Line = EXTI_Line0;
	E.EXTI_Mode = EXTI_Mode_Interrupt;
	E.EXTI_Trigger = EXTI_Trigger_Falling;
	E.EXTI_LineCmd = ENABLE;
	EXTI_Init(&E);

	N.NVIC_IRQChannel = EXTI0_1_IRQn;
	N.NVIC_IRQChannelPriority = 1;
	N.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&N);

	//Write initialization registers
	H_BlockingWriteReg(0, 0x18);
	H_BlockingWriteReg(1, 0x80);
	H_BlockingWriteReg(2, 0x00);

	while(1)
	{
		//Wait for data to be ready
		while(!done);

		//Print data through semihosting (slow!)
		printf("X: %i, Y: %i, Z: %i\n", x, y, z);
	}
}
