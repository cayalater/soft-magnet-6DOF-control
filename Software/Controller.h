#ifndef __PID_H__
#define __PID_H__

#define kp_x 0      //x轴pid比例系数
#define ki_x 0      //x轴pid积分系数
#define kd_x 0      //x轴pid微分系数
#define limit_x 0   //x轴输出限幅值

#define kp_y 0      //y轴pid比例系数
#define ki_y 0      //y轴pid积分系数
#define kd_y 0      //y轴pid微分系数
#define limit_y 0   //y轴输出限幅值

#define kp_z 0      //z轴pid比例系数
#define ki_z 0      //z轴pid积分系数
#define kd_z 0      //z轴pid微分系数
#define limit_z 0   //z轴输出限幅值

typedef struct
{
    float kp[3];     //比例
    float ki[3];     //积分
    float kd[3];     //微分

    float u_current[3];         //增量式pid控制器本次实际输出量，u(k)
    float u_last[3];            //增量式pid控制器上次实际输出量，u(k-1)
    float error_current[3];     //本次误差，e(k)
    float error_last[3];        //上次误差，e(k-1)
    float error_previous[3];    //上上次误差，e(k-2)
    float adjust[3];            //输出增量，delta_u(k)
    float limit[3];             //输出限幅值

    float position_current[3];  //当前位置
    float position_target[3];   //目标位置
} PID_TypeDef;

/*********************************外接函数*************************************/
//初始化PID结构体参数
void PID_Init(PID_TypeDef *p);
//设置当前位置
void Set_Current_Position(PID_TypeDef *p, float current_position[]);
//设置目标位置
void Set_Target_Position(PID_TypeDef *p, float target_position[]);
//设置限幅值
void Set_Limit(PID_TypeDef *p, float limit[]);
//增量式pid控制器
void PID(PID_TypeDef *p);

#endif
