#include "action_control.h"
#include "action_sets.h"
#include "GO-M8010-6.h"
#include "warehouse_app.h"
#include "catch.h"
#include "postion_control.h"

ACTION_FLAG_T car_action;


//准备开始识别圆盘机物料的动作组
void Yuanpanji_PrepareDetectPose(void)
{
    // Implement the logic for Yuanpanji action
    claw_move_1(open);
    Z_SetHeight(0);
    M8010_SetAngle(PUT_AND_CATCH_ANGLE);
    Y_SetLength(YUAN_PAN_LENGHT);
    Z_SetHeight(YUAN_PAN_DETECT_HEIGHT);
}

//准备定位圆环的动作组
void Circle_PrepareDetectPose(void)
{
	claw_move_1(open);
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	M8010_SetAngle(CIRCLE_DETECT_ANGLE);
	Y_SetLength(CIRCLE_DETECT_LENGTH);
	Z_SetHeight(CIRCLE_DETECT_HEIGHT);
}

// Prepare to locate a material already placed in the ring area.
void Circle_PrepareMaterialCatchPose(void)
{
	claw_move_1(open);
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	M8010_SetAngle(CIRCLE_PLACE_ANGLE);
	Y_SetLength(CIRCLE_PLACE_LENGTH);
	Z_SetHeight(CIRCLE_MATERIAL_DETECT_HEIGHT);
}

// 从指定物料仓取料，并按给定关节角、Y轴长度和Z轴高度放置。
uint8_t Circle_PlaceFromWarehouseAtPose(uint8_t warehouse_index, uint16_t place_height,
										int16_t place_angle, uint16_t place_length,
										uint8_t need_clear, uint8_t need_cw)
{
	int wh_angle;

	if(warehouse_index > 2)
		return 0;

	claw_move_2(open);
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	wh_angle = (need_cw && warehouse_index == 2)
			 ? -394 : Get_Warehouse_Angle(warehouse_index);
	M8010_SetAngle(wh_angle);
	Y_SetLength(CIRCLE_WAREHOUSE_LENGTH);
	Z_SetHeight(CIRCLE_WAREHOUSE_HEIGHT);
	claw_move_2(close);
	vTaskDelay(pdMS_TO_TICKS(100));

	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	if (place_height == CIRCLE_SECOND_LAYER_HEIGHT)
	{
		if(need_clear)
		{
			/* 第二层特殊步：取到物料并抬升后，先伸到避障长度再旋转。 */
			Y_SetLength(CIRCLE_SECOND_LAYER_AVOID_LENGTH);
		}
		else
		{
			/* 123/213/132/312/321 默认流程：先缩回 Y，再旋转到目标放置角。 */
			Y_SetLength(0);
		}
	}
	M8010_SetAngle(place_angle);
	Y_SetLength(place_length);
	Z_SetHeight(place_height);
	vTaskDelay(pdMS_TO_TICKS(200));
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(100));
	if(place_height == CIRCLE_SECOND_LAYER_HEIGHT)
	{
		claw_move_1(open);
		Z_SetHeight(CIRCLE_MATERIAL_DETECT_HEIGHT);
	}
	else
	{
		Z_SetHeight(CIRCLE_DETECT_HEIGHT);
	}

	return 1;
}

// 通用放置姿态，供定3放3和孤立动作测试继续使用。
uint8_t Circle_PlaceFromWarehouseAtHeight(uint8_t warehouse_index, uint16_t place_height,
                                            uint8_t need_clear, uint8_t need_cw)
{
	return Circle_PlaceFromWarehouseAtPose(warehouse_index, place_height,
										 CIRCLE_PLACE_ANGLE, CIRCLE_PLACE_LENGTH,
										 need_clear, need_cw);
}

//准备定位圆盘机的动作组（决赛）；
uint8_t Positioning_Circle_action(void)
{
    // Implement the logic for Circle action
	claw_move_1(open);
    Y_SetLength(20);//must step
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Z_SetHeight(75);
    // This function should return 1 if the action is complete, otherwise 0
    return 1;
}

uint8_t Action_Init(void)
{
    taskENTER_CRITICAL();
    Y_SetLength(-70);
    YZ_SetZero(2);
    M8010_SetAngle(39);
    // vTaskDelay(pdMS_TO_TICKS(300));
    Delay_ms(300);
    M8010_Set_Zero();
    taskEXIT_CRITICAL();
    return 1;
}

uint8_t Catch_ypj_action(void)
{
    // Implement the logic for Circle action
    //Z_SetHeight(0);//先拉高
	claw_move_2(open);//张开爪子
    //Z_SetHeight(84);//下降到圆盘机的物料的高度,todo:下降为零（暂定为0）
	claw_move_2(close);
	Delay_ms(70);//等待夹爪闭合
    // This function should return 1 if the action is complete, otherwise 0
    return 1;
}

void action_set_in_user(ACTION_TOTAL_SET car_tar_action)
{
    Action_sets_task_create();
    car.action_set.set_action=car_tar_action;
    car.action_set.finish_flag=Incomplete;
}

uint8_t action_set_in_task(ACTION_TOTAL_SET car_tar_action)
{
    char flag=0;
    switch (car_tar_action)
    {
        case YUANPANJI_ACTION:
        {
            Yuanpanji_PrepareDetectPose();
            flag = 1;
        }break;
        case CIRCLE_ACTION:
        {
            Circle_PrepareDetectPose();
            flag = 1;
        }break;
        case CATCH_YUANPANJI_ACTION:
        {
            flag=Catch_ypj_action();
        }break;
        case POSITONING_CIRCLE_ACTION:
        {
            flag=Positioning_Circle_action();
        }break;
        case ACITON_INIT:
        {
            flag=Action_Init();
        }break;
        default:
        {
            return 0; // Invalid action
        }
        break;
    }
    if(flag==1)
    {
        return finish; // Action set successfully
    }
    else
    {
        return Incomplete; // Action not set
    }
}


