#include "stm32f10x.h"                  // Device header
#include "Configure.h"
#include "OLED.h"

#define packet_head 0xFF
#define packet_tail 0xFE
#define receiving_packet_head 0
#define receiving_data        1
#define receiving_packet_tail 2
#define packet_length 4

uint8_t Serial_RxPacket[packet_length];
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = Common_USART_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = Common_USART_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = Common_USART_Baud_rate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(Common_USART, &USART_InitStructure);
	
	USART_ITConfig(Common_USART, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//STM32F10X_HD
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(Common_USART, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(Common_USART, Byte);
	while (USART_GetFlagStatus(Common_USART, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}



uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

void USART1_IRQHandler(void)
{
	static uint8_t RxState = receiving_packet_head;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(Common_USART, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(Common_USART);
		
		switch (RxState)
		{
			case receiving_packet_head :
				if(RxData == packet_head)
				{
					RxState = receiving_data;
					pRxPacket = 0;
				}
				break;
			case receiving_data :
				Serial_RxPacket[pRxPacket] = RxData;				
				pRxPacket ++;
				if(pRxPacket >= packet_length)
				{
					RxState = receiving_packet_tail;
				}
				break;
			case receiving_packet_tail :
				if(RxData == packet_tail)
				{
					RxState = receiving_packet_head;
					/*这里写发送给电源柜电流数据的流程*/
					OLED_ShowHexNum(1, 1, Serial_RxPacket[0], 2);
					OLED_ShowHexNum(1, 4, Serial_RxPacket[1], 2);
					OLED_ShowHexNum(1, 7, Serial_RxPacket[2], 2);
					OLED_ShowHexNum(1, 10, Serial_RxPacket[3], 2);				
				}
				break;					
		}
		
		USART_ClearITPendingBit(Common_USART, USART_IT_RXNE);
	}
}



