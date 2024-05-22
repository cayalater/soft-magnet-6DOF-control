#include "stm32f10x.h"

/**************************************************************************************************
 * 定时器说明：stm32f10x系列中，TIM1和TIM8是高级定时器，TIM2-5是通用定时器，TIM6和TIM7是基本定时器
 * TIM1和TIM8挂载在APB2总线上，TIM2-7挂载在APB1总线上
 * stm32f103c8t6和stm32f103c6t6等小容量设备没有TIM5-8,需要更大容量的设备
**************************************************************************************************/
#define Serial_Timer                    TIM1
#define PID_Timer1                      TIM2
#define PID_Timer2                      TIM3
#define Alternative_Magnet_Timer1       TIM4
#define Alternative_Magnet_Timer2       TIM5
#define RS485_Timer                     TIM6

void Timer_Init(void)
{
    //开启总线外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    //时钟输入：内部时钟,频率72Mhz
    TIM_InternalClockConfig(Serial_Timer);
    TIM_InternalClockConfig(PID_Timer1);
    TIM_InternalClockConfig(PID_Timer2);
    TIM_InternalClockConfig(Alternative_Magnet_Timer1);
    TIM_InternalClockConfig(Alternative_Magnet_Timer2);
    TIM_InternalClockConfig(RS485_Timer);

    //定时器时基单元设置，使用内部时钟时，定时时间：T = （ARR + 1） * (PSC + 1) / 720000
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;             //滤波器预分频系数，如果不使用外部时钟可以随意设置
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;         //计数模式，通常使用向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;                       //ARR自动重装，取值0~65535
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;                     //PSC预分频系数，取值0~65535
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                    //重复计数次数，一般不设置
	TIM_TimeBaseInit(Serial_Timer, &TIM_TimeBaseInitStructure);

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;         
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;                       
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;                     
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                    
	TIM_TimeBaseInit(PID_Timer1, &TIM_TimeBaseInitStructure);                 
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;         
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;                       
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;                     
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                    
	TIM_TimeBaseInit(PID_Timer2, &TIM_TimeBaseInitStructure);
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;         
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;                       
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;                     
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                    
	TIM_TimeBaseInit(Alternative_Magnet_Timer1, &TIM_TimeBaseInitStructure);
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;         
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;                       
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;                     
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                    
	TIM_TimeBaseInit(Alternative_Magnet_Timer2, &TIM_TimeBaseInitStructure);

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;         
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;                       
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;                     
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                    
	TIM_TimeBaseInit(RS485_Timer, &TIM_TimeBaseInitStructure);

    //开启定时器更新中断控制（注：开启后，系统一上电便会产生一次中断，手动清除中断标志位可以避免这一现象）
    TIM_ClearFlag(Serial_Timer, TIM_FLAG_Update);                                   //手动清除标志位示例

    TIM_ITConfig(Serial_Timer, TIM_IT_Update, ENABLE);
    TIM_ITConfig(PID_Timer1, TIM_IT_Update, ENABLE);
    TIM_ITConfig(PID_Timer2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(Alternative_Magnet_Timer1, TIM_IT_Update, ENABLE);
    TIM_ITConfig(Alternative_Magnet_Timer2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(RS485_Timer, TIM_IT_Update, ENABLE);
    //若需要触发中断，第二个参数需要修改

    //配置NVIC嵌套向量中断控制器
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);             //选择中断优先级分组

    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;     //高级定时器TIM1通讯触发中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

    //开启定时器
    TIM_Cmd(Serial_Timer, ENABLE);
    TIM_Cmd(PID_Timer1, ENABLE);
    TIM_Cmd(PID_Timer2, ENABLE);
    TIM_Cmd(Alternative_Magnet_Timer1, ENABLE);
    TIM_Cmd(Alternative_Magnet_Timer2, ENABLE);
    TIM_Cmd(RS485_Timer, ENABLE);
}

/****************************************定时器中断函数********************************************/

/**********Serial_Timer中断**********/
void TIM1_UP_IRQHanlder()
{

}

/**********PID_Timer1中断**********/
void TIM2_IRQHanlder()
{

}

/**********PID_Timer2中断**********/
void TIM3_IRQHanlder()
{
    
}

/**********Alternative_Magnet_Timer1中断**********/
void TIM4_IRQHanlder()
{
    
}

/**********Alternative_Magnet_Timer2中断**********/
void TIM5_IRQHanlder()
{
    
}

/**********RS485_Timer中断**********/
void TIM6_IRQHanlder()
{
    
}
