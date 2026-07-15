#ifndef __CATCH_H
#define __CATCH_H

#define red 1
#define green 2
#define blue 3
/* 旧动作组使用的两层 Z 轴高度。 */
#define LOW_Z_HEIGHT                 70  /* 第一圈加工区 Z 轴高度。 */
#define HIGH_Z_HEIGHT                90  /* 第二圈加工区 Z 轴高度。 */

/* 圆盘机物料识别和夹取姿态。 */
#define YUAN_PAN_HEIGHT              45  /* 从圆盘机夹取物料时的 Z 轴高度。 */
#define YUAN_PAN_DETECT_HEIGHT       10  /* 识别圆盘机物料时的 Z 轴高度。 */
#define YUAN_PAN_LENGHT              15  /* 圆盘机识别和夹取时的 Y 轴伸出长度。 */

/* 物料仓转运动作参数。 */
#define PUT_HOUSE_HEIGHT             25  /* 把物料放入物料仓时的 Z 轴高度。 */
#define Y_LENGHT_WAREHOUSE           20  /* 在物料仓位置使用的 Y 轴伸出长度。 */

/* 粗加工区色环识别、放置和回收姿态。 */
#define CIRCLE_DETECT_HEIGHT         20  /* 定位色环时的 Z 轴高度。 */
#define CIRCLE_DETECT_LENGTH          5  /* 定位色环时的 Y 轴伸出长度。 */
#define CIRCLE_SAFE_HEIGHT            0  /* 机械臂旋转前必须到达的 Z 轴安全高度。 */
#define CIRCLE_WAREHOUSE_LENGTH      20  /* 从物料仓取料时的 Y 轴伸出长度。 */
#define CIRCLE_WAREHOUSE_HEIGHT      30  /* 从物料仓取料时的 Z 轴夹取高度。 */
#define CIRCLE_ROTATE_LENGTH        110  /* 夹住物料后、机械臂旋转前的避障伸出长度。 */
#define CIRCLE_PLACE_LENGTH           5  /* 色环区放置和回收物料时的 Y 轴伸出长度。 */
#define CIRCLE_PLACE_HEIGHT         125  /* 色环区放置和回收物料时的 Z 轴工作高度。 */
#define CIRCLE_SECOND_LAYER_HEIGHT   50  /* 暂存区第二轮的二层放置高度，当前为试验值。 */
#define CIRCLE_MATERIAL_DETECT_HEIGHT 20 /* 色环区识别物料时的 Z 轴高度。 */
#define CIRCLE_DETECT_ANGLE      PUT_AND_CATCH_ANGLE /* 色环识别使用的安全关节角。 */
#define CIRCLE_PLACE_ANGLE       PUT_AND_CATCH_ANGLE /* 色环放置使用的安全关节角。 */
#define FIRST_WAREHOUSE           34 /* 1 号物料仓对应的关节角。 */
#define SECOND_WAREHOUSE           0 /* 2 号物料仓对应的关节角。 */
#define THIRD_WAREHOUSE          -32 /* 3 号物料仓对应的关节角。 */
/* 旧流程三个颜色放置区对应的关节角。 */
#define RED_PUT_AREA_ANGLE      -222 /* 红色放置区关节角。 */
#define GREEN_PUT_AREA_ANGLE    -180 /* 绿色放置区关节角。 */
#define BLUE_PUT_AREA_ANGLE     -144 /* 蓝色放置区关节角。 */
/* 旧宏名中虽然写着 HEIGHT，实际控制的是 Y 轴伸缩长度。 */
#define RED_PUT_AREA_HEIGHT     140  /* 红色放置区使用的 Y 轴伸出长度。 */
#define GREEN_PUT_AREA_HEIGHT    50  /* 绿色放置区使用的 Y 轴伸出长度。 */
#define BLUE_PUT_AREA_HEIGHT    140  /* 蓝色放置区使用的 Y 轴伸出长度。 */

#define open  1
#define close  2

#define P_round 0 //Preliminary round
#define F_round 1 //finaly round

extern struct User_parameter_t car_lift;

void claw_move(int action);
void claw_move_1(int action);
void claw_move_2(int action_2);
void catch_yuan_pan_ji(int color);
void catch_half_stage(int color);
void Put_Layer(int layer,int color,int mode);
void Catch_Material_YUAN_PAN_JI(int times);//time 1 or 2
void catch_ground(int color);
void catch_half_stage_ProMax(int color,int position);//for final round
void PUT_Material_YUAN_PAN_JI(int times);//time 1 or 2
void put_yuan_pan_ji(int color);
void Catch_TELESCOPIC_Polar_coordinates(char color,float car_angle);
void Catch_Material_YUAN_PAN_JI_with_TELESCOPIC_Polar_coordinates(int times);
void Catch_material_on_grand(char times);
#endif

