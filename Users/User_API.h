#ifndef __USERAPI_H
#define __USERAPI_H
#include "sys.h"

typedef struct
{
	float Vx;			//ǰ���ٶ�
	float Vy;			//�����ٶ�
	float Chassis_Wz;			//��ת�ٶ�
	float Gimbal_Wz;
	float Gimbal_Wzlast;
	float Pitch_angle;	//pitch��Ƕ�
}ControlDATA_TypeDef;


//�������
typedef struct
{
	u16 Frequency;	//��Ƶ
	float Speed;		//����
}ShootProject_TypeDef;


extern ControlDATA_TypeDef Control_data;//��������
extern u8 Twist_Flag;				//Ť��
extern u8 Vision_Flag;			//�Ӿ�����
extern u8 Shoot_Flag;				//���
extern u8 SpinTop_Flag;     //С���ݱ�־
extern u8 Gimbal_180_flag;
extern u8 Fire_Loading_Flag;
extern u8 Fire_Loading_End_Flag;   //����������־λ
extern u8 Chassis_mode;
extern u8 Stronghold_flag;
extern u8 speed_17mm_level_Start_Flag;
extern int32_t VerticalCnt;
extern int32_t HorizontalCnt;
extern int SafeHeatflag;
extern int Chassismode_flag;
extern int Gimbalmode_flag;
extern int Shootnumber;
extern int Shootnumber_fired;
extern float speed_zoom;
extern float Ramp_K;
extern float Shootnumber_limit;
extern float speed_zoom;

void User_Api(void);
void chassis_control_acquisition(void);
void gimbal_control_acquisition(void);
static void ShooterHeat_Ctrl(void);

/* ��ֵ���ƺ����궨�� */
#define VAL_LIMIT(val, min, max)\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\


#endif


