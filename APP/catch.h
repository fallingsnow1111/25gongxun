#ifndef __CATCH_H
#define __CATCH_H

#define red 1
#define green 2
#define blue 3
//tode:
#define LOW_Z_HEIGHT   	    168.2	 //1 round height
#define HIGH_Z_HEIGHT	 	100      //2 round height 
// #define RED_WAREHOUSE  		-35	     //the angle of the M8010 to red warehouse
// #define	GREEN_WAREHOUSE		0		 //the angle of the M8010 to green warehouse
// #define	BLUE_WAREHOUSE		32       //the angle of the M8010 to blue warehouse
#define FIRST_WAREHOUSE  		-32	     //the angle of the M8010 to 1 warehouse
#define	SECOND_WAREHOUSE		0		 //the angle of the M8010 to 2 warehouse
#define	THIRD_WAREHOUSE		    34       //the angle of the M8010 to 3 warehouse
//the angle of the M8010 to put the material
#define RED_PUT_AREA_ANGLE      141
#define GREEN_PUT_AREA_ANGLE 	180
#define BLUE_PUT_AREA_ANGLE 	217
//it's the length of telescopic for put the material
#define RED_PUT_AREA_HEIGHT     98
#define GREEN_PUT_AREA_HEIGHT   50
#define BLUE_PUT_AREA_HEIGHT    103
//it is the length of telescopic to catch the material in the wasehouse
#define Y_HEIGHT_WAREHOUSE  20

#define open  1
#define close  2

#define P_round 0 //Preliminary round
#define F_round 1 //finaly round

extern struct User_parameter_t car_lift;

void claw_move(int action);
void claw_move_1(int action_1);
void claw_move_2(int action_2);
void catch_yuan_pan_ji(int color);
void catch_half_stage(int color);
//void Put_Layer(int layer,int color);
void Put_Layer(int layer,int color,int mode);
void Catch_Material_YUAN_PAN_JI(int times);//time 1 or 2
void catch_ground(int color);
void catch_half_stage_ProMax(int color,int position);//for final round
void PUT_Material_YUAN_PAN_JI(int times);//time 1 or 2
void put_yuan_pan_ji(int color);
void Catch_TELESCOPIC_Polar_coordinates(char color,float car_angle);
void Catch_Material_YUAN_PAN_JI_with_TELESCOPIC_Polar_coordinates(int times);
void put_yuan_pan_ji(int color);
void Catch_material_on_grand(char times);
#endif

