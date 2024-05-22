#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Configure.h"
#include "RS485.h"

uint16_t RxData;
float voltage = 48.6f;
float current = 4.561f;

int main(void)
{
	OLED_Init();
	Serial_Init();
	RS485_Init();
	
	//RS485_SendReadFrame(0x01, cmd_read_multiple_readonly_register, 0x03E8, 0x0001);
	//RS485_Write_Address(0x01, 0x02);
	//RS485_Set_Vlotage(0x01, voltage);
	//RS485_Set_Current(0x01, current);
	//RS485_Launch(0x01);
	
	while (1)
	{
		
	}
}

