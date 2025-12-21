#include "usmart.h"
#include "usmart_str.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ�����) 	 	
#include "sys.h"
#include "postion_control.h"

extern void led_set(u8 sta);
extern void test_fun(void(*ledset)(u8),u8 sta);	
//void Move_To_Target_Time(float vy,float vx,float w,float angle,int move_time, int mode);
//void Move_To_Target_Timei(int vy,int w,int move_time, int mode)
//{
//	Move_To_Target_Time(vy, w, move_time,, mode);
//}

void set_pid(uint32_t p, uint32_t i, uint32_t d);
//�������б���ʼ��(�û��Լ�����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�?
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����?
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",	 
    //(void*)Move_To_Target_Timei,"void Move_To_Target_Timei(int vy,int w,int move_time, int mode)",
    (void*)postion_send,"void postion_send(int X,int Y, int Z )"

		
#endif		   
								
};						  

/*
	(void*)right_out_push,"void right_out_push()",
	(void*)left_out_push,"void left_out_push(void)",
	(void*)right_in_push,"void right_in_push()",
	(void*)left_in_push,"void left_in_push()",
	(void*)all_up,"void all_up()",
	(void*)all_down,"void all_down()",

	(void*)usmart_gyro_setpid,"void usmart_gyro_setpid(int p, int i, int d)",
	(void*)usmart_gyro_start_run,"void usmart_gyro_start_run(int speed)",
	(void*)usmart_gyro_stop_run,"void usmart_gyro_stop_run()",
	

	(void*)usmart_led,"void usmart_led(uint8_t sw)",
	(void*)usmart_pid,"void usmart_pid(uint16_t val,int deno,int mode)",
	(void*)angle_write,"void angle_write(uint8_t num,int angle)",
	(void*)pca_setpwm, "void pca_setpwm(uint8_t num, uint32_t on, uint32_t off)",
	(void*)PCA_MG9XX, "void PCA_MG9XX(u8 num,int start_angle,int end_angle,u8 mode,u8 speed)",

	(void*)Use_Gray_W_PID,"void Use_Gray_W_PID()",
	(void*)usmart_gyro_setpid,"void usmart_gyro_setpid(int p, int i, int d)",
	(void*)Gamut_Filtering,"int Gamut_Filtering(void)",,
*/


///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//��������
	0,	  	//��������
	0,	 	//����ID
	1,		//������ʾ����,0,10����;1,16����
	0,		//��������.bitx:,0,����;1,�ַ���	    
	0,	  	//ÿ�������ĳ����ݴ��?,��ҪMAX_PARM��0��ʼ��
	0,		//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};   



















