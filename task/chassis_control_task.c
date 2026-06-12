#include "chassis_control_task.h"
#include "struct_encapsulation.h"
#include "pid.h"
#include "task.h"
#include "fast_math.h"

#define CHASSIS_CONTROL_TASK_H_STACK 512//任务堆栈
#define CHASSIS_CONTROL_TASK_H_PRIORITY 6//优先级

#define POSITION_THRESHOLD 15.0f     // 位置误差阈值
#define ORIENTATION_THRESHOLD 0.3f  // 角度误差阈值
#define LiMITED_SPEED 3.0f // 最小速度限制
#define LIMITED_LOW_POSITION  0
#define LIMITED_LOW_POSITION_SPEED 0

#define MAX_DELTA 20.0f     //每周期最大速度变化量
#define MAX_W_DELTA 20.0f   //每周期最大航向变化量

static inline float clamp(float val, float low, float high){
    if (val < low)  return low;
    if (val > high) return high;
    return val;
}

volatile CARDATA_T   car;
struct   PIDstruct   chassis_pid_y;
struct   PIDstruct   chassis_pid_x;
volatile struct  CHECK_FLAG_t  motor_check;

volatile float imu_w_c_output = 0.0f; // 角速度输出

static float x_c_output,y_c_output,w_c_output;

TaskHandle_t Chassis_Control_Task_Handle;

void Get_Chassis_c_output(float *x_output, float *y_output, float *w_output)
{
	*x_output = x_c_output;
	*y_output = y_c_output;
	*w_output = w_c_output;
}

void chassis_control_init(void)
{
	PID_Init(&chassis_pid_y, 0.280f, 0.005f, 0, 350.0f, -350.0f);
	PID_Init(&chassis_pid_x, 0.280f, 0.005f, 0, 350.0f, -350.0f);
	car.target_y = 0.0f;
	car.target_x = 0.0f;
	car.target_w = 0.0f;
	car.actual_y = 0.0f;
	car.actual_x = 0.0f;
	car.actual_w = 0.0f;
	car.imu_modeable = enable; // 陀螺仪控制使能
	car.Odometer_able = enable; // 里程计使能
}

void Obimeter_SetZero(void)
{
    car.actual_y = 0.0f;
    car.actual_x = 0.0f;
    car.actual_w = 0.0f;
}

void  Set_chassis_able(ABLE_T Odometer)
{
	taskENTER_CRITICAL(); // 进入临界区（关闭中断）
    if (Odometer == enable) {
        vTaskResume(Chassis_Control_Task_Handle);
    }
    else if (Odometer == unable) {
        vTaskSuspend(Chassis_Control_Task_Handle);
    }
    taskEXIT_CRITICAL(); // 退出临界区
}

void check(float tar,float actual)
{
    static uint16_t _check=0;
    if(tar>0)
    {
        if(actual>=(tar+100))
        {
            _check++;
        }   
    }
    else if (tar<0)
    {
        if(actual<=(tar-100))
        {
            _check++;
        }   
    }
    //vofa_printf("check:%d\n",_check);
}

static float last_y = 0, last_x = 0, last_w = 0;

void chassis_control(void)
{
    // 读取电机坐标并计算实际位置
    motor_read_coordination_all();
    if(motor_check.flag_finish<0x0F)
    {
        Motor_setspeed(0, 0, 0);
		Delay_ms(6);
        return;
    }
    Motor_Action_Calculate_actual(&car.actual_y, &car.actual_x, &car.actual_w);
    //更新朝向角（IMU模式或编码器积分模式）
    if(car.imu_modeable == enable)
    {
        car.actual_w = normalize_angle(imu.yaw);
    }
    // // 获取控制输出
    w_c_output = Direction_Calibration_turn(car.target_w); 
    y_c_output = PID_Compute(&chassis_pid_y, car.target_y, car.actual_y);
    x_c_output = PID_Compute(&chassis_pid_x, car.target_x, car.actual_x);

    //pid输出限幅，梯形加减速
    float delta_y = y_c_output - last_y;
    float delta_x = x_c_output - last_x;
    float delta_w = w_c_output - last_w;
    delta_y = clamp(delta_y, -MAX_DELTA, MAX_DELTA);
    delta_x = clamp(delta_x, -MAX_DELTA, MAX_DELTA);
    delta_w = clamp(delta_w, -MAX_DELTA, MAX_DELTA);
    y_c_output = last_y + delta_y;
    x_c_output = last_x + delta_x;
    w_c_output = last_w + delta_w;

    // 计算位置和角度误差
    float err_y = __fabs(car.target_y - car.actual_y);
    float err_x = __fabs(car.target_x - car.actual_x);
    float err_w = __fabs(car.target_w - car.actual_w);
    // the death dispose
    //y
    if(__fabs(err_y)<LIMITED_LOW_POSITION && (__fabs(err_y) > POSITION_THRESHOLD))
    {
        if(y_c_output > 0)
        {
            y_c_output = LIMITED_LOW_POSITION_SPEED; // 如果输出小于0.5，则设置为5
        }
        else if (y_c_output < 0)
        {	
            y_c_output = -LIMITED_LOW_POSITION_SPEED; // 如果输出小于0.5，则设置为0
        }
    }
    else  if((__fabs(err_y) <= POSITION_THRESHOLD))
    {
        y_c_output=0;
    }
    //x
    if(__fabs(err_x)<LIMITED_LOW_POSITION && (__fabs(err_x) > POSITION_THRESHOLD))
    {
        if(x_c_output > 0)
        {
            x_c_output = LIMITED_LOW_POSITION_SPEED; // 如果输出小于8，则设置为8
        }
        else if (x_c_output < 0)
        {	
            x_c_output = -LIMITED_LOW_POSITION_SPEED; // 如果输出小于8，则设置为8
        }
    }
    else if((__fabs(err_x) <= POSITION_THRESHOLD))
    {
        x_c_output=0;
    }

    if((__fabs(err_w) <= ORIENTATION_THRESHOLD))
    {
        w_c_output=0;
    }
    // 判断是否到达目标位置
    if ((__fabs(err_y) <= POSITION_THRESHOLD) && 
        (__fabs(err_x) <= POSITION_THRESHOLD) && 
        MOTOR_ACTIONFALG == Incomplete && 
        (__fabs(err_w) <= ORIENTATION_THRESHOLD))
    {
        MOTOR_ACTIONFALG = finish;
    }
    //动作完成后结束底盘控制输出
    if(MOTOR_ACTIONFALG == finish){
        last_y = 0;
        last_x = 0;
        last_w = 0;
        Motor_setspeed(0, 0, 0);
        return;
    }
    check(car.target_x,car.actual_x);
    //发送调试信息
    last_y = y_c_output;
    last_x = x_c_output;
    Motor_setspeed(y_c_output, x_c_output, w_c_output);
    Delay_ms(5);
    //motor_check.flag_ready=Incomplete;
    // 发送速度到电机
    motor_check.flag_finish=0;
}

void Chassis_Control_Task(void*pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(30);

	while(1)
	{   
        //固定周期唤醒
        vTaskDelayUntil(&last_wake, period); 
        chassis_control();
	}
}

void Chassis_Control_Task_Create(void)
{
	xTaskCreate(Chassis_Control_Task,"Chassis_Control_Task",CHASSIS_CONTROL_TASK_H_STACK,NULL,CHASSIS_CONTROL_TASK_H_PRIORITY,&Chassis_Control_Task_Handle);
}

