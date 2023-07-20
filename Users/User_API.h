#ifndef __USERAPI_H
#define __USERAPI_H
#include "sys.h"

typedef struct
{
	float Vx;			//前后速度
	float Vy;			//左右速度
	float Chassis_Wz;			//旋转速度
	float Gimbal_Wz;
	float Gimbal_Wzlast;
	float Pitch_angle;	//pitch轴角度
}ControlDATA_TypeDef;


//射击方案
typedef struct
{
	u16 Frequency;	//射频
	float Speed;		//射速
}ShootProject_TypeDef;


extern ControlDATA_TypeDef Control_data;//控制数据
extern u8 Twist_Flag;				//扭腰
extern u8 Vision_Flag;			//视觉自瞄
extern u8 Shoot_Flag;				//射击
extern u8 SpinTop_Flag;     //小陀螺标志
extern u8 Gimbal_180_flag;
extern u8 Fire_Loading_Flag;
extern u8 Fire_Loading_End_Flag;   //补弹结束标志位
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

/* 极值限制函数宏定义 */
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


