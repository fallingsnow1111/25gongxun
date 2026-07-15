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

// Take one material from the selected warehouse and place it at the selected layer height.
uint8_t Circle_PlaceFromWarehouseAtHeight(uint8_t warehouse_index, uint16_t place_height)
{
	if(warehouse_index > 2)
		return 0;

	claw_move_2(open);
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	M8010_SetAngle(Get_Warehouse_Angle(warehouse_index));
	Y_SetLength(CIRCLE_WAREHOUSE_LENGTH);
	Z_SetHeight(CIRCLE_WAREHOUSE_HEIGHT);
	claw_move_2(close);
	vTaskDelay(pdMS_TO_TICKS(100));

	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	if(warehouse_index < 2)
	{
		Y_SetLength(CIRCLE_ROTATE_LENGTH);
		vTaskDelay(pdMS_TO_TICKS(400));
	}
	M8010_SetAngle(CIRCLE_PLACE_ANGLE);
	Y_SetLength(CIRCLE_PLACE_LENGTH);
	Z_SetHeight(place_height);
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(100));
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);

	return 1;
}

uint8_t Circle_PlaceFromWarehouse(uint8_t warehouse_index)
{
	return Circle_PlaceFromWarehouseAtHeight(warehouse_index, CIRCLE_PLACE_HEIGHT);
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


