#ifndef __STRUCT_ENCAPSULATION_H
#define __STRUCT_ENCAPSULATION_H

#include <stdio.h>
#include "cmsis_os.h"

#define finish					        1
#define Incomplete				     	0

typedef enum {
    enable=1,
    unable=0
}ABLE_T;

typedef enum{
	Relative_Position=0,//相对位置模式
	Absolute_Position=1//绝对位置模式
}MODE_POSITION;

typedef enum{
	YUANPANJI_ACTION=0,//准备识别圆盘机的动作组
	CIRCLE_ACTION=1,   //准备定位圆盘机的动作组
    CATCH_YUANPANJI_ACTION=2,
    POSITONING_CIRCLE_ACTION=3,
    ACITON_INIT=4
}ACTION_TOTAL_SET;

typedef struct wasehouse_single_t {
   uint8_t color;
   int angle;
}WAREHOUSE_SINGLE_T;

typedef struct WASEHOUSE_T{
    WAREHOUSE_SINGLE_T firse;  // 第一个仓库
    WAREHOUSE_SINGLE_T second; // 第二个仓库
    WAREHOUSE_SINGLE_T thrid;  // 第三个仓库
}WASEHOUSE_T;


typedef struct ACTION_FLAG_T{
	uint8_t   finish_flag;
    ACTION_TOTAL_SET set_action; //当前动作组
}ACTION_FLAG_T;

typedef struct CHECK_FLAG_t
{
    uint8_t flag_ready; //任务是否就绪的标志位
	uint8_t flag_finish;//任务是否完成的标志位
}CHECK_FLAG_t;

typedef struct MOTOR_SPEED_t
{
    float x_setpeed;  // X轴速度
    float y_setpeed;  // Y轴速度
    float w_setpeed;  // 角速度
}MOTOR_SPEED_t;

typedef struct CARDATA_T {
    float target_y;  // 目标里程计Y
    float target_x;  // 目标里程计X
    float target_w;  // 目标角度W
    float actual_y;  // 实际里程计Y
    float actual_x;  // 实际里程计X
    float actual_w;  // 实际角度W
    volatile ABLE_T imu_modeable;// 模式（是否使能陀螺仪控制）
    volatile ABLE_T Odometer_able;//是否使用里程计
    volatile ACTION_FLAG_T action_set;//动作组
} CARDATA_T;


struct P_pid_obj {
    float output;      // 控制器的输出值，通常是调整的控制量，用于驱动执行器（如电机、阀门等）
    float bias;        // 偏差值，通常是目标值与测量值之间的差值 (target - measure)
    float measure;     // 测量值，表示系统当前的状态或反馈值（例如，传感器读数）
    float last_bias;   // 上一个偏差值，用于计算微分项（D项），即偏差变化速率
    float integral;    // 积分项，用于累积偏差值，解决稳态误差的问题 (积累过去的偏差)
    float target;      // 目标值，系统需要达到的理想状态值
};
struct PID {
	float kp;
	float ki;
	float kd;
	float integralMax;
	float outputMin;
	float outputMax;
};
struct PIDstruct{
    float kp;            // 比例增益
    float ki;            // 积分增益
    float kd;            // 微分增益
    float integralMax;   // 积分限幅
    float outputMin;     // 输出最小值
    float outputMax;     // 输出最大值
    float previousError; // 上次误差
    float integral;      // 当前积分值
};

typedef struct PIDstructIntegralSeparation
{
	struct PIDstruct PID_basic;//全量式PID的结构体
	float  IntegralSeparationThreshold;//积分分离阈值
}PIDstructIntegralSeparation;

struct User_parameter_t{
		int need_color;
		double lift_angle;
		double change_angle;
		int length;
};



#endif
