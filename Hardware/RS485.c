#include "stm32f10x.h" // Device header
#include "Configure.h"
#include "RS485.h"
#include <stdio.h>


/****************************************************************底层通讯相关函数*********************************************************************/
void RS485_Init(void)
{
    // 开启总线外设时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 初始化GPIO引脚
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Pin = RS485_USART_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_InitStructure.GPIO_Pin = RS485_USART_RX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 初始化串口
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = RS485_USART_Baud_rate;                     // 波特率：115200
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 不开启硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // 串口模式：收发
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无校验
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 停止位：1位
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 数据长度：8位
    USART_Init(RS485_USART, &USART_InitStructure);

    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE); // 开启串口接收中断

    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 选择NVIC优先级分组2

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //STM32F10X_HD
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // RS485抢占优先级和响应优先级均低于普通串口
    NVIC_Init(&NVIC_InitStructure);

    // 开启串口
    USART_Cmd(RS485_USART, ENABLE);
}

void RS485_SendByte(uint8_t Byte)
{
    USART_SendData(RS485_USART, Byte);
    while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET)
        ; // 等待发送结束
}

uint16_t CRC16(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i = 0;
    uint8_t j = 0;
    for (j = 0; j < len; j++)
    {
        crc = crc ^ *buf++;
        for (i = 0; i < 8; i++)
        {
            if ((crc & 0x0001) > 0)
            {
                crc = crc >> 1;
                crc = crc ^ 0xa001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

void RS485_SendReadFrame(uint8_t device_address, uint8_t cmd, uint16_t register_address, uint16_t register_number)
{
    //计算CRC校验码
    uint8_t crc_origin_data[6] = {device_address, cmd, (uint8_t)((register_address >> 8) & 0xff), (uint8_t)(register_address & 0xff), (uint8_t)((register_number >> 8) & 0xff), (uint8_t)(register_number & 0xff)};
    uint8_t crc_upper_byte = (CRC16(crc_origin_data, 6)) >> 8 & 0xff;
    uint8_t crc_lower_byte = CRC16(crc_origin_data, 6) & 0xff;
    //发送数据
    RS485_SendByte(device_address);
    RS485_SendByte(cmd);
    RS485_SendByte((uint8_t)((register_address >> 8) & 0xff));      //寄存器地址高八位
    RS485_SendByte((uint8_t)(register_address & 0xff));             //寄存器地址低八位
    RS485_SendByte((uint8_t)((register_number >> 8) & 0xff));       //寄存器数量高八位
    RS485_SendByte((uint8_t)(register_number & 0xff));              //寄存器数量低八位
    RS485_SendByte(crc_lower_byte);                                 //CRC校验码低八位
    RS485_SendByte(crc_upper_byte);                                 //CRC校验码高八位(modbus crc校验码低位先行)
}

void RS485_SendWriteFrame(uint8_t device_address, uint8_t cmd, uint16_t register_address, uint16_t register_number, uint8_t data_length, uint8_t *data)
{
    //计算CRC校验码
    uint8_t crc_origin_data[11] = {device_address, cmd, (uint8_t)((register_address >> 8) & 0xff), (uint8_t)(register_address & 0xff), (uint8_t)((register_number >> 8) & 0xff), (uint8_t)(register_number & 0xff), data_length, 0, 0, 0, 0};
    for(uint8_t i = 0; i < data_length; i++)
    {
        crc_origin_data[7+i] = data[i];
    }
    uint8_t crc_upper_byte = (CRC16(crc_origin_data, 7 + data_length)) >> 8 & 0xff;
    uint8_t crc_lower_byte = CRC16(crc_origin_data, 7 + data_length) & 0xff;

    //发送数据
    RS485_SendByte(device_address);
    RS485_SendByte(cmd);
    RS485_SendByte((uint8_t)((register_address >> 8) & 0xff));      //寄存器地址高八位
    RS485_SendByte((uint8_t)(register_address & 0xff));             //寄存器地址低八位
    RS485_SendByte((uint8_t)((register_number >> 8) & 0xff));       //寄存器数量高八位
    RS485_SendByte((uint8_t)(register_number & 0xff));              //寄存器数量低八位
    RS485_SendByte(data_length);
    for(uint8_t i = 0; i < data_length; i++)
    {
        RS485_SendByte(data[i]);
    }
    RS485_SendByte(crc_lower_byte);                                 //CRC校验码低八位
    RS485_SendByte(crc_upper_byte);                                 //CRC校验码高八位(modbus crc校验码低位先行)
}

/*****************************************************************功能实现函数******************************************************************/

/***********************************************************************************************************************************************
 * 函数名称：void RS485_Set_Vlotage(uint8_t coil_address, uint16_t Vlotage)
 * 功    能：设置电源地址
 * 说    明：
 * 入口参数：电源当前地址；电源设定地址
***********************************************************************************************************************************************/
void RS485_Write_Address(uint8_t current_address, uint8_t set_address)
{
    uint8_t address[2] = {0, set_address};
    RS485_SendWriteFrame(current_address, cmd_write_multiple_register, 0x07D0, 1, 2, address);      //2000寄存器：读写设备地址
}

/***********************************************************************************************************************************************
 * 函数名称：void RS485_Set_Vlotage(uint8_t coil_address, uint16_t Vlotage)
 * 功    能：设置输出电压（预置电压）
 * 说    明：此函数规定输出电压额定值均<= 600V且>= 60V
 * 入口参数：线圈电源地址；电压值，单位10mV，原始数据带2位小数
***********************************************************************************************************************************************/
void RS485_Set_Vlotage(uint8_t coil_address, float vlotage)
{
    uint8_t vlotage_int[2] = {(uint8_t)((((uint16_t) (vlotage * 100)) >> 8) & 0xff), (uint8_t)(((uint16_t) (vlotage * 100)) & 0xff)};
    RS485_SendWriteFrame(coil_address, cmd_write_multiple_register, 0x07D1, 1, 2, vlotage_int);         //2001寄存器：读写预置电压，16进制07D1
}

/***********************************************************************************************************************************************
 * 函数名称：void RS485_Set_Current(uint8_t coil_address, uint16_t current)
 * 功    能：设置输出电流（预置电流）
 * 说    明：此函数规定输出电流额定值均<= 60A
 * 入口参数：线圈电源地址；电流值，单位mA，原始数据带3位小数
***********************************************************************************************************************************************/
void RS485_Set_Current(uint8_t coil_address, float current)
{
    uint8_t current_int[2] = {(uint8_t)((((uint16_t) (current * 1000)) >> 8) & 0xff), (uint8_t)(((uint16_t) (current * 1000)) & 0xff)};
    RS485_SendWriteFrame(coil_address, cmd_write_multiple_register, 0x07D2, 1, 2, current_int);         //2002寄存器：读写预置电流，16进制07D2
}

/***********************************************************************************************************************************************
 * 函数名称：void RS485_Set_Vlotage(uint8_t coil_address, uint16_t Vlotage)
 * 功    能：开启电源输出
 * 说    明：
 * 入口参数：线圈电源地址
***********************************************************************************************************************************************/
void RS485_Launch(uint8_t coil_address)
{
    uint8_t output[2] = {0, 1};
    RS485_SendWriteFrame(coil_address, cmd_write_multiple_register, 0x07D9, 1, 2, output);                   //2009寄存器：输出控制，0是停止输出，非0值启动输出
}


