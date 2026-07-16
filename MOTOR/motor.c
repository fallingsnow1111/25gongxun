#include "motor.h"
#include "pid.h"
#include "usart.h"
#include <string.h>
#include "dma.h"
#include "stm32f7xx_hal.h"
#include "delay.h"
#include "tjc_usart_hmi.h"
#include "IMU.h"
#include <math.h>

volatile char MOTOR_ACTIONFALG=Incomplete;

extern DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t LB_send[15];
static uint8_t LF_send[15];
static uint8_t RB_send[15];
static uint8_t RF_send[15];

struct DATA	GLOBAL_DATA;
struct MOTO_DATA motor1;
struct MOTO_DATA motor2;
struct MOTO_DATA motor3;
struct MOTO_DATA motor4; 
MOTOR_SPEED_t car_setspeed;
volatile CARDATA_T car;
volatile struct CHECK_FLAG_t motor_check;

static int16_t odom_last_rpm_x10[4];
static int64_t odom_segment_wheel[4];
static int64_t odom_total_wheel[4];
static uint32_t odom_last_tick;
static uint32_t odom_segment_move_time_ms;
static uint32_t odom_total_move_time_ms;
static uint8_t odom_started;
static float world_last_raw_yaw;
static uint8_t world_yaw_started;
volatile uint32_t chassis_odom_tx_fail_count = 0;
volatile int32_t motor_actual_pulse[4];
volatile uint8_t motor_speed_scale10_ready = 0;

int motor_mode = speed_mode;
int postion_bit = finish;

static uint8_t RXdat[RXdat_maxsize]={0};
char RXdat_piont=0;
uint8_t u3_Txdata;
volatile uint16_t u3_debug_size = 0;
volatile uint32_t u3_debug_rx_count = 0;
volatile uint32_t u3_debug_rx_restart_count = 0;
volatile uint32_t u3_debug_rx_restart_fail = 0;
volatile uint32_t u3_debug_error_count = 0;
volatile uint32_t u3_debug_last_error = 0;
volatile uint8_t u3_debug_buf[8];

HAL_StatusTypeDef uart3WriteBuf(uint8_t *buf, uint8_t len)
{
	uint32_t start = HAL_GetTick();

	while(huart3.gState != HAL_UART_STATE_READY)
	{
		if((HAL_GetTick() - start) > 5)
		{
			return HAL_TIMEOUT;
		}
	}

	return HAL_UART_Transmit_DMA(&huart3,buf,len);
}

static uint8_t Chassis_OdomMoving(void)
{
	return odom_last_rpm_x10[0] != 0 || odom_last_rpm_x10[1] != 0 ||
	       odom_last_rpm_x10[2] != 0 || odom_last_rpm_x10[3] != 0;
}

static void Chassis_WorldYawUpdate(void)
{
	float raw_yaw = imu.yaw;
	float delta_yaw;

	while(raw_yaw > 180.0f) raw_yaw -= 360.0f;
	while(raw_yaw < -180.0f) raw_yaw += 360.0f;

	if(world_yaw_started == 0)
	{
		world_last_raw_yaw = raw_yaw;
		car.actual_w = raw_yaw;
		world_yaw_started = 1;
		return;
	}

	delta_yaw = raw_yaw - world_last_raw_yaw;
	while(delta_yaw > 180.0f) delta_yaw -= 360.0f;
	while(delta_yaw < -180.0f) delta_yaw += 360.0f;

	car.actual_w += delta_yaw;
	world_last_raw_yaw = raw_yaw;
}

static void Chassis_OdomSettle(uint32_t now)
{
	uint32_t elapsed_ms;
	int64_t delta_wheel[4];

	Chassis_WorldYawUpdate();

	if(odom_started == 0)
	{
		odom_last_tick = now;
		odom_started = 1;
		return;
	}

	elapsed_ms = now - odom_last_tick;
	odom_last_tick = now;
	if(elapsed_ms == 0 || Chassis_OdomMoving() == 0)
		return;

	for(uint8_t i = 0; i < 4; i++)
	{
		int64_t delta = (int64_t)odom_last_rpm_x10[i] * elapsed_ms;
		delta_wheel[i] = delta;
		odom_segment_wheel[i] += delta;
		odom_total_wheel[i] += delta;
	}
	odom_segment_move_time_ms += elapsed_ms;
	odom_total_move_time_ms += elapsed_ms;
}

static void Chassis_OdomBuild(CHASSIS_ODOM_T *odom,
							  const int64_t wheel[4], uint32_t move_time_ms)
{
	if(odom == NULL)
		return;

	for(uint8_t i = 0; i < 4; i++)
		odom->wheel[i] = wheel[i] / 10;

	/* Match the public chassis convention: x left+, y forward+. */
	odom->x = (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) / 40;
	odom->y = -(wheel[0] + wheel[1] - wheel[2] - wheel[3]) / 40;
	odom->w = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) / 40;
	odom->move_time_ms = move_time_ms;
}

void Chassis_OdomResetSegment(void)
{
	Chassis_OdomSettle(HAL_GetTick());
	memset(odom_segment_wheel, 0, sizeof(odom_segment_wheel));
	odom_segment_move_time_ms = 0;
}

void Chassis_OdomGetSegment(CHASSIS_ODOM_T *odom)
{
	Chassis_OdomSettle(HAL_GetTick());
	Chassis_OdomBuild(odom, odom_segment_wheel, odom_segment_move_time_ms);
}

void Chassis_OdomGetTotal(CHASSIS_ODOM_T *odom)
{
	Chassis_OdomSettle(HAL_GetTick());
	Chassis_OdomBuild(odom, odom_total_wheel, odom_total_move_time_ms);
}

void Chassis_OdomGetSegmentSnapshot(CHASSIS_ODOM_T *odom)
{
	taskENTER_CRITICAL();
	Chassis_OdomBuild(odom, odom_segment_wheel, odom_segment_move_time_ms);
	taskEXIT_CRITICAL();
}

void Chassis_WorldPoseReset(float x_mm, float y_mm, float yaw_deg)
{
	float raw_yaw = imu.yaw;

	while(raw_yaw > 180.0f) raw_yaw -= 360.0f;
	while(raw_yaw < -180.0f) raw_yaw += 360.0f;

	taskENTER_CRITICAL();
	car.actual_x = x_mm;
	car.actual_y = y_mm;
	car.actual_w = yaw_deg;
	world_last_raw_yaw = raw_yaw;
	world_yaw_started = 1;
	taskEXIT_CRITICAL();

	Chassis_OdomResetSegment();
}

uint8_t Motor_ReadPulseSnapshot(int32_t pulse[4])
{
	if(pulse == NULL)
		return 0;

	motor_check.flag_finish = 0;
	for(uint8_t i = 1; i <= 4; i++)
	{
		motor_read_coordination(i);
		Delay_ms(3);
	}

	if(motor_check.flag_finish != 0x0F)
		return 0;

	for(uint8_t i = 0; i < 4; i++)
		pulse[i] = motor_actual_pulse[i];
	return 1;
}

void UART3_RxRestart(void)
{
    HAL_UART_AbortReceive(&huart3);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                  UART_CLEAR_PEF  | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);

    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxState = HAL_UART_STATE_READY;
    huart3.ReceptionType = HAL_UART_RECEPTION_STANDARD;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RXdat, RXdat_maxsize) == HAL_OK) {
        u3_debug_rx_restart_count++;
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
    else {
        u3_debug_rx_restart_fail++;
    }
}

void MOTOR_Init(void)// 电机初始化
{
	memset((void *)&car, 0, sizeof(car));
	memset((void *)&motor_check, 0, sizeof(motor_check));
	memset(odom_last_rpm_x10, 0, sizeof(odom_last_rpm_x10));
	memset(odom_segment_wheel, 0, sizeof(odom_segment_wheel));
	memset(odom_total_wheel, 0, sizeof(odom_total_wheel));
	memset((void *)motor_actual_pulse, 0, sizeof(motor_actual_pulse));
	odom_last_tick = HAL_GetTick();
	odom_segment_move_time_ms = 0;
	odom_total_move_time_ms = 0;
	odom_started = 1;
	world_last_raw_yaw = 0.0f;
	world_yaw_started = 0;
	car_setspeed.x_setpeed = 0.0f;
	car_setspeed.y_setpeed = 0.0f;
	car_setspeed.w_setpeed = 0.0f;
	motor1.target_angle = 0;
	motor1.actual_angle = 0;
	motor2.target_angle = 0;
	motor2.actual_angle = 0;
	motor3.target_angle = 0;
	motor3.actual_angle = 0;
	motor4.target_angle = 0;
	motor4.actual_angle = 0;
	motor_speed_scale10_ready = Motor_EnableSpeedScale10();
	UART3_RxRestart();
}

/* 上电启用并保存0.1RPM通信单位；每台配置后等待5ms。 */
uint8_t Motor_EnableSpeedScale10(void)
{
	static uint8_t command[4][6] = {
		{0x01, 0x4F, 0x71, 0x01, 0x01, 0x6B},
		{0x02, 0x4F, 0x71, 0x01, 0x01, 0x6B},
		{0x03, 0x4F, 0x71, 0x01, 0x01, 0x6B},
		{0x04, 0x4F, 0x71, 0x01, 0x01, 0x6B}
	};

	for(uint8_t i = 0; i < 4; i++)
	{
		if(uart3WriteBuf(command[i], sizeof(command[i])) != HAL_OK)
			return 0;
		Delay_ms(5);
	}

	return 1;
}

/**********************************************************************************************************
* 函数名: Motor_Send_Speed_together
* 功能:   打包四个电机速度命令(多电机同步模式)
* 输入:   LB, LF, RF, RB - 四个电机的目标速度
* 返回值: 无
**********************************************************************************************************/
void Motor_Send_Speed_together(int16_t LB,int16_t LF,int16_t RF,int16_t RB)
{
	static uint8_t* LB_speedptr = LB_send;
    static uint8_t* LF_speedptr = LF_send;
    static uint8_t* RB_speedptr = RB_send;
    static uint8_t* RF_speedptr = RF_send;
////////////////////////     1          2              3           4///////
    uint8_t* temp[4] = {LB_speedptr, LF_speedptr, RF_speedptr, RB_speedptr};
    int16_t tempspeed = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
			uint8_t var=i;
        switch(i+1)
				{
					case 1:
						tempspeed=LB;break;
					case 2:
						tempspeed=LF;break;
					case 3:
						tempspeed=RF;break;
					case 4:
						tempspeed=RB;break;
					default:
						tempspeed=0;
						break;
				}
                temp[i][0] = var+1;
				temp[i][1] = 0xF6;

				if(tempspeed>0)
				{
						temp[i][2] = 0x00;
						temp[i][3] = (tempspeed >>8) & 0xFF;
						temp[i][4] = (tempspeed & 0xFF);
				}
				else
				{
						tempspeed = -tempspeed;
						temp[i][2] = 0x01;
						temp[i][3] = (tempspeed >>8) & 0xFF;
						temp[i][4] = (tempspeed & 0x00FF);
				}
				temp[i][5] = 0x00;
				temp[i][6] = 0x01;
				temp[i][7] = 0x6B;
    }
}

HAL_StatusTypeDef Send_motor_together(void)// 多机同步触发
{
	static uint8_t data[4];
	data[0]=0x00;
	data[1]=0xFF;
	data[2]=0x66;
	data[3]=0x6B;
	return uart3WriteBuf((uint8_t*)data,4);
}

//向电机发送命令：读取电机实时位置
void motor_read_coordination(uint8_t motor_id)
{
		static uint8_t sendmotor_coordination_data[3];
		sendmotor_coordination_data[0]=motor_id;
		sendmotor_coordination_data[1]=0x36;
		sendmotor_coordination_data[2]=0x6B;
		uart3WriteBuf(sendmotor_coordination_data,3);
}

/**********************************************************************************************************
* 函数名: Motor_Send_Postion_together
* 功能:   打包四个电机位置命令(多电机同步模式)
* 输入:   LB, LF, RF, RB - 目标位置; mode - 位置模式标志
* 返回值: 无
**********************************************************************************************************/
void Motor_Send_Postion_together(int LB, int LF, int RF, int RB, char mode)
{
    static uint8_t* LB_positionptr = LB_send;
    static uint8_t* LF_positionptr = LF_send;
    static uint8_t* RB_positionptr = RB_send;
    static uint8_t* RF_positionptr = RF_send;
////////////////////////     1               2                3                4////////
    uint8_t* temp[4] = {LB_positionptr, LF_positionptr, RF_positionptr, RB_positionptr};
    int temppostion = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
				uint8_t var=i;
        switch(i+1)
				{
					case 1:
						temppostion=LB;break;
					case 2:
						temppostion=LF;break;
					case 3:
						temppostion=RF;break;
					case 4:
						temppostion=RB;break;
					default:
						temppostion=0;
						break;
				}
        temp[i][0] = var+1;
        temp[i][1] = 0xFD;
        if (temppostion > 0)
        {
            temp[i][2] = 0x00;
        }
        else
        {
            temppostion = -temppostion;
            temp[i][2] = 0x01;
        }
        temp[i][3] = 0x2E; // 速度原值0x05
        temp[i][4] = 0xE0;

        temp[i][5] = 0xAE; // 加速度原值

        temp[i][6] = (uint8_t)((temppostion * 14) >> 24);
        temp[i][7] = (uint8_t)((temppostion * 14) >> 16);
        temp[i][8] = (uint8_t)((temppostion * 14) >> 8);
        temp[i][9] = (uint8_t)(temppostion * 14);

        temp[i][10] = (uint8_t)mode;
        temp[i][11] = 0x01;
        temp[i][12] = 0x6B;
    }
}

// 多电机命令，一条命令设置四个电机速度
HAL_StatusTypeDef send_speed_data_all(void)
{
	static uint8_t all_send[37];
	all_send[0] = 0x00;
	all_send[1] = 0xAA;
	all_send[2] = 0x00;
	all_send[3] = 0x25;
	memcpy(&all_send[4], LB_send, 8);
	memcpy(&all_send[12], LF_send, 8);
	memcpy(&all_send[20],RF_send, 8);
	memcpy(&all_send[28],RB_send, 8);
	all_send[36] = 0x6B;
	return uart3WriteBuf(all_send, 37);
}

void send_speed_data_switch(void)
{
    uart3WriteBuf(LB_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
    uart3WriteBuf(LF_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
    uart3WriteBuf(RF_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
    uart3WriteBuf(RB_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
}

void send_postion_data_switch(void)
{
    uart3WriteBuf(LB_send,13);
    Delay_ms(4);
    uart3WriteBuf(LF_send,13);
    Delay_ms(4);
    uart3WriteBuf(RB_send,13);
    Delay_ms(4);
    uart3WriteBuf(RF_send,13);
	Delay_ms(4);
}
 void Motor_Action_Calculate_target(float vy,float vx,float vw)
{	
	__disable_irq();
	motor1.target_angle =  vw + vy - vx; // 1号电机
	motor2.target_angle =  vw + vy + vx; // 2号电机
	motor3.target_angle =  vw - vy + vx; // 3号电机
	motor4.target_angle =  vw - vy - vx; // 4号电机
	__enable_irq();
}

void Motor_Action_Calculate_actual(volatile float *actual_y,volatile float *actual_x,volatile float *actual_w)
{
    // 调整符号，匹配逆解算
    *actual_w = ( motor1.actual_angle + motor2.actual_angle + motor3.actual_angle + motor4.actual_angle) * 0.25f;
    *actual_y = ( motor1.actual_angle + motor2.actual_angle - motor3.actual_angle - motor4.actual_angle) * 0.25f;
    *actual_x = (-motor1.actual_angle + motor2.actual_angle + motor3.actual_angle - motor4.actual_angle) * 0.25f;
}

void Motor_setposition(float vy,float vx,float vw,char mode)
{
	Motor_Action_Calculate_target(vy,vx,vw);
	Motor_Send_Postion_together(motor1.target_angle,motor2.target_angle,motor3.target_angle,motor4.target_angle,mode);
	send_postion_data_switch();
	Send_motor_together();
}

//多机同步
static int16_t Motor_EncodeRpmX10(float rpm, uint8_t fine)
{
	float encoded;

	/* 普通跑图先取整到1RPM；精细控制保留到0.1RPM。 */
	if(fine)
		encoded = rpm * 10.0f;
	else
		encoded = (float)((int16_t)rpm) * 10.0f;

	if(encoded > 3000.0f)
		encoded = 3000.0f;
	else if(encoded < -3000.0f)
		encoded = -3000.0f;

	return (int16_t)(encoded >= 0.0f ? encoded + 0.5f : encoded - 0.5f);
}

static void Motor_setspeed_internal(float vy, float vx, float vw, uint8_t fine)
{
	int16_t next_rpm_x10[4];
	HAL_StatusTypeDef status;

	Motor_Action_Calculate_target(vy, vx, vw);
	next_rpm_x10[0] = Motor_EncodeRpmX10(motor1.target_angle, fine);
	next_rpm_x10[1] = Motor_EncodeRpmX10(motor2.target_angle, fine);
	next_rpm_x10[2] = Motor_EncodeRpmX10(motor3.target_angle, fine);
	next_rpm_x10[3] = Motor_EncodeRpmX10(motor4.target_angle, fine);
	Motor_Send_Speed_together(next_rpm_x10[0], next_rpm_x10[1],
							  next_rpm_x10[2], next_rpm_x10[3]);
	status = send_speed_data_all();
	if(status == HAL_OK)
		status = Send_motor_together();

	if(status == HAL_OK)
	{
		Chassis_OdomSettle(HAL_GetTick());
		for(uint8_t i = 0; i < 4; i++)
			odom_last_rpm_x10[i] = next_rpm_x10[i];
	}
	else
	{
		Chassis_OdomSettle(HAL_GetTick());
		chassis_odom_tx_fail_count++;
	}
}

void motor_setspeed_chassis(float vy, float vx, float vw) // 普通通道设定电机速度
{
	car_setspeed.y_setpeed = vy;
	car_setspeed.x_setpeed = vx;
	car_setspeed.w_setpeed = vw;
}

// 普通跑图接口：实际轮速保持1RPM分辨率。
void Motor_setspeed(float vy, float vx, float vw)
{
	Motor_setspeed_internal(vy, vx, vw, 0);
}

// 转向精调接口：实际轮速保留0.1RPM分辨率。
void Motor_setspeed_fine(float vy, float vx, float vw)
{
	Motor_setspeed_internal(vy, vx, vw, 1);
}

//use Motor_setspeed_in_tim() to set speed in tim for 10ms
void Motor_setspeed_in_tim(void)
{
	Motor_setspeed(car_setspeed.y_setpeed, car_setspeed.x_setpeed, car_setspeed.w_setpeed);
}

void motor_data_reset(void)
{
	//__disable_irq();
	motor1.target_angle = 0;
    motor2.target_angle = 0;
    motor3.target_angle = 0;
    motor4.target_angle = 0;
	motor1.actual_angle=0;
    motor2.actual_angle = 0;
    motor3.actual_angle = 0;
    motor4.actual_angle = 0;
	GLOBAL_DATA.W_SPEED = 0;
	GLOBAL_DATA.Y_SPEED = 0;
	GLOBAL_DATA.X_SPEED = 0;
    GLOBAL_DATA.CHANHE_SPEED = 0;
	//__enable_irq();
}


void Motor_SetZero(void)// 设置电机里程坐标，清零
{
	uint8_t TXdata[4];
	TXdata[1]=0x0A;
	TXdata[2]=0x6D;
	TXdata[3]=0x6B;
	for(uint16_t i=0x01;i<=4;i++)
	{
		TXdata[0]=i;
		uart3WriteBuf((uint8_t*)TXdata,4);
		Delay_ms(6);
	}
	//Delay_ms(10);
}

void Motor_SetZeroPiont(void)// 设置电机零位置
{
	uint8_t TXdata[5];
	TXdata[1]=0x93;
	TXdata[2]=0x88;
	TXdata[3]=0x01;
	TXdata[4]=0x6B;
	for(uint16_t i=0x01;i<=4;i++)
	{
		TXdata[0]=i;
		uart3WriteBuf((uint8_t*)TXdata,5);
		Delay_ms(5);
	}
}

void Motor_ReadZeroPiontFlag(uint8_t id)
{
	uint8_t TXdata[3];
	TXdata[0]=id;
	TXdata[1]=0x3B;
	TXdata[2]=0x6B;
	uart3WriteBuf((uint8_t*)TXdata,3);
}

void Motor_MakeZeroPiont(void)// 电机归零
{
	int count=0;
	uint8_t TXdata[5];
	TXdata[1]=0x9A;
	TXdata[2]=0x00;
	TXdata[3]=0x00;
	TXdata[4]=0x6B;
	for(uint16_t i=0x01;i<=4;i++)
	{
		TXdata[0]=i;
		uart3WriteBuf((uint8_t*)TXdata,5);
		Delay_ms(5);
	}
	Send_motor_together();
	
	while(!(RXdat[2]&0x01))
	{
		Delay_ms(5);
		Motor_ReadZeroPiontFlag(1);
		count++;
		if(count>=50)
		{
			break;
		}
	}
}

void motor_read_stateflag(uint8_t motor_id)// 读取状态标志位指令
{
	uint8_t Send_motor_state[3];
	Send_motor_state[0]=motor_id;
	Send_motor_state[1]=0x3A;
	Send_motor_state[2]=0x6B;
	uart3WriteBuf(Send_motor_state,3);
}


void Motor_Rxdata_SetSero(void)
{
	memset(RXdat,0,(RXdat_maxsize)*sizeof(uint8_t));
}

static uint8_t rxbuff1[128];
static uint8_t rxbuff2[128];
volatile uint32_t motor3_rx_probe = 0;

static void U3_process_single_frame(uint8_t* data, uint8_t len)
{
	if (data[len - 1] != 0x6B)
	{
		return;
	}

	/* [31:24]=len, [23:16]=tail, [15:8]=id, [7:0]=motor3 frame count */
	{
		uint8_t id = data[0];
		uint8_t count3 = (uint8_t)(motor3_rx_probe & 0xFF);
		if (len == 8 && id == 0x03)
		{
			count3++;
		}
		motor3_rx_probe = ((uint32_t)len << 24) | ((uint32_t)data[len - 1] << 16) | ((uint32_t)id << 8) | count3;
	}

	if (len == 8)
	{
		uint8_t index = data[0] - 1;
		uint32_t raw = ((uint32_t)data[3] << 24) |
					   ((uint32_t)data[4] << 16) |
					   ((uint32_t)data[5] << 8) | data[6];
		int32_t position;
		float angle;

		if(index >= 4)
			return;
		position = data[2] == 0x01 ? -(int32_t)raw : (int32_t)raw;
		angle = (float)position * 360.0f / 65536.0f;
		motor_actual_pulse[index] = position;
		motor_check.flag_finish |= (uint8_t)(1U << index);

		switch(index)
		{
			case 0: motor1.actual_angle = angle; break;
			case 1: motor2.actual_angle = angle; break;
			case 2: motor3.actual_angle = angle; break;
			case 3: motor4.actual_angle = angle; break;
			default: break;
		}
	}
	else if (len == 4)
	{
		if (data[0] == 1 && data[3] == 0x6B)
		{
			if (data[1] == 0xF6 && data[2] == 0x02)
			{
				motor_check.flag_ready = finish;
			}
		}
	}
}
//读取电机实际角度的函数
uint8_t U3_data_processing(uint8_t* data,uint8_t len)
{
	uint8_t index = 0;

	while (index < len)
	{
		if ((len - index) >= 8 && data[index + 7] == 0x6B)
		{
			U3_process_single_frame(&data[index], 8);
			index += 8;
			continue;
		}

		if ((len - index) >= 4 && data[index + 3] == 0x6B)
		{
			U3_process_single_frame(&data[index], 4);
			index += 4;
			continue;
		}

		index++;
	}

	return 1;
}

void MOTOR_FINISHFLAGEXAM(uint8_t *RXdat) // 接收完成标志检查
{
    uint8_t buff_len = 0;
    static uint8_t* rxbuff = NULL;
    static uint8_t rxbuff_flag = 0;
    
    // 停止DMA接收
    if(HAL_UART_DMAStop(&huart3) != HAL_OK) {
        // 错误处理
        return;
    }
    
    // 禁用中断以确保数据一致
    __disable_irq();
    
    // 正确获取DMA接收计数器
    if(huart3.hdmarx != NULL) {
        buff_len = RXdat_maxsize - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    }
    
    // 检查RXdat指针有效性和数据长度
    if(buff_len > 0 && RXdat != NULL) {
        // 双缓冲区切换，取反表示缓冲区切换
        if(rxbuff_flag == 0) {
            rxbuff = rxbuff1;
            rxbuff_flag = ~rxbuff_flag;
        } else {
            rxbuff = rxbuff2;
            rxbuff_flag = ~rxbuff_flag;
        }
        // 复制数据到缓冲区
        memcpy(rxbuff, RXdat, buff_len);
        // 恢复中断
        __enable_irq();
        // 处理接收到的数据（在中断恢复后进行，避免阻塞时间过长）
        U3_data_processing(rxbuff, buff_len);
    } else {
        // 恢复中断（如果没有数据或指针无效）
        __enable_irq();
    }
}

void UART3_RxEventHandler(uint16_t size)
{
	uint16_t rx_len = size;
	if(rx_len > RXdat_maxsize)
	{
		rx_len = RXdat_maxsize;
	}

	u3_debug_size = size;
	u3_debug_rx_count++;
	for(uint8_t i = 0; i < sizeof(u3_debug_buf); i++)
	{
		u3_debug_buf[i] = (i < rx_len) ? RXdat[i] : 0;
	}

	U3_data_processing(RXdat, (uint8_t)rx_len);
	UART3_RxRestart();
}

void UART3_ErrorHandler(void)
{
	u3_debug_error_count++;
	u3_debug_last_error = huart3.ErrorCode;
	UART3_RxRestart();
}

void MY_UART3_IRQHandler(void)
{
	UART3_RxEventHandler(RXdat_maxsize - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
}



