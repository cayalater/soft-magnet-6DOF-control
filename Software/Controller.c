#include "Controller.h"

/**********************************************************
 函数名称：PID_Init(PID_TypeDef *p)
 功    能：初始化PID结构体的参数
 说    明：
 入口参数：无
**********************************************************/
void PID_Init(PID_TypeDef *p)
{
    for(int i = 0; i < 3; i++)
    {
        p->kp[i] = kp_x;
        p->ki[i] = ki_x;
        p->kd[i] = kd_x;

        p->u_current[i] = 0;
        p->u_last[i] = 0;
        p->error_current[i] = 0;
        p->error_last[i] = 0;
        p->error_previous[i] = 0;
        p->adjust[i] = 0;
        p->limit[i] = 0;

        p->position_current[i] = 0;
        p->position_target[i] = 0;
    }
}

/*****************************************************************************
 函数名称：Set_Current_Position(PID_TypeDef *p, float current_position[])
 功    能：设置当前位置
 说    明：
 入口参数：PID参数结构体地址；当前位置向量
******************************************************************************/
void Set_Current_Position(PID_TypeDef *p, float current_position[])
{
    for(int i = 0; i < 3; i++)
    {
        p->position_current[i] = current_position[i];
    }
}

/*****************************************************************************
 函数名称：Set_Target_Position(PID_TypeDef *p, float target_position[])
 功    能：设置目标位置
 说    明：
 入口参数：PID参数结构体地址；目标位置向量
******************************************************************************/
void Set_Target_Position(PID_TypeDef *p, float target_position[])
{
    for(int i = 0; i < 3; i++)
    {
        p->position_target[i] = target_position[i];
    }
}

/*****************************************************************************
 函数名称：Set_Limit(PID_TypeDef *p, float limit[])
 功    能：设置限幅值
 说    明：
 入口参数：PID参数结构体地址；限幅值向量
******************************************************************************/
void Set_Limit(PID_TypeDef *p, float limit[])
{
    for(int i = 0; i < 3; i++)
    {
        p->limit[i] = limit[i];
    }
}

/**********************************************************************************************
 函数名称：PID(PID_TypeDef *p)
 功    能：增量式pid控制器实现
 说    明：核心公式：delta_u(k) = Kp*[e(k) - e(k-1)] + Ki*e(k) + Kd*[e(k) - 2*e(k-1) + e(k-2)]
 入口参数：PID参数结构体地址
**********************************************************************************************/
void PID(PID_TypeDef *p)
{
    for(int i = 0; i < 3; i++)
    {
        //计算误差：误差=目标位置-实际位置
        p->error_current[i] = p->position_target[i] - p->position_current[i];

        //计算输出增量
        p->adjust[i] = p->kp[i] * (p->error_current[i] - p->error_last[i]) + p->ki[i] * p->error_current[i] + p->kd[i] * (p->error_current[i] - 2 * p->error_last[i] + p->error_previous[i]);

        //计算实际输出
        p->u_current[i] = p->u_last[i] + p->adjust[i];

        //输出限幅
        if(p->u_current[i] > p->limit[i])
            p->u_current[i] = p->limit[i];
        if(p->u_current[i] < -(p->limit[i]))
            p->u_current[i] = -(p->limit[i]);        

        //更新历史参数
        p->u_last[i] = p->u_current[i];
        p->error_previous[i] = p->error_last[i];
        p->error_last[i] = p->error_current[i];
    }
}
