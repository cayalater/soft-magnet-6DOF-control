#ifndef __CONFIGURE_H__
#define __CONFIGURE_H__

#define Common_USART USART1         //USART1作为普通串口接收上位机发送的电流数据
#define RS485_USART USART2          //USART2作为与电源柜通信的串口

#define Common_USART_Baud_rate 115200     //普通串口波特率：115200
#define RS485_USART_Baud_rate 115200    //RS485串口波特率：115200
/*定义stm32引脚*/
#define Common_USART_TX GPIO_Pin_9      //GPIOA
#define Common_USART_RX GPIO_Pin_10     //GPIOA
#define RS485_USART_TX GPIO_Pin_2       //GPIOA
#define RS485_USART_RX GPIO_Pin_3       //GPIOA

#endif

