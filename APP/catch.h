#ifndef __CATCH_H
#define __CATCH_H

/* 圆盘机物料识别和夹取姿态。 */
#define YUAN_PAN_HEIGHT               45  /* 从圆盘机夹取物料时的 Z 轴高度。 */
#define YUAN_PAN_DETECT_HEIGHT        10  /* 识别圆盘机物料时的 Z 轴高度。 */
#define YUAN_PAN_LENGHT               15  /* 圆盘机识别和夹取时的 Y 轴伸出长度。 */

/* 物料仓动作参数。 */
#define PUT_HOUSE_HEIGHT              25  /* 把物料放入物料仓时的 Z 轴高度。 */
#define Y_LENGHT_WAREHOUSE            20  /* 在物料仓位置使用的 Y 轴伸出长度。 */

/* 色环区识别、放置和回收姿态。 */
#define CIRCLE_DETECT_HEIGHT          20  /* 定位色环时的 Z 轴高度。 */
#define CIRCLE_DETECT_LENGTH           5  /* 定位色环时的 Y 轴伸出长度。 */
#define CIRCLE_SAFE_HEIGHT             0  /* 机械臂旋转前必须到达的 Z 轴安全高度。 */
#define CIRCLE_WAREHOUSE_LENGTH       20  /* 从物料仓取料时的 Y 轴伸出长度。 */
#define CIRCLE_WAREHOUSE_HEIGHT       30  /* 从物料仓取料时的 Z 轴夹取高度。 */
#define CIRCLE_ROTATE_LENGTH         110  /* 夹住物料后、机械臂旋转前的避障伸出长度。 */
#define CIRCLE_PLACE_LENGTH            5  /* 色环区放置和回收物料时的 Y 轴伸出长度。 */
#define CIRCLE_PLACE_HEIGHT          125  /* 粗加工区第一层放置和回收高度。 */
#define CIRCLE_SECOND_LAYER_HEIGHT    50  /* 暂存区第二轮的二层放置高度。 */
#define CIRCLE_MATERIAL_DETECT_HEIGHT 20  /* 色环区识别物料时的 Z 轴高度。 */
#define CIRCLE_DETECT_ANGLE PUT_AND_CATCH_ANGLE /* 色环识别使用的安全关节角。 */
#define CIRCLE_PLACE_ANGLE  PUT_AND_CATCH_ANGLE /* 色环放置使用的安全关节角。 */

/* 三个物料仓对应的关节角。 */
#define FIRST_WAREHOUSE               34
#define SECOND_WAREHOUSE               0
#define THIRD_WAREHOUSE              -32

#define open  1
#define close 2

void claw_move_1(int action);
void claw_move_2(int action);
void Catch_TELESCOPIC_Polar_coordinates(char color, float car_angle);

#endif
