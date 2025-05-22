#include "Stm32_time.h"
#include "global_status.h"
#include "math.h"
#include "pid.h"
#include "CAN_Re_Se.h"
#include "chassis_move.h"
#include "small_tools.h"
#include "gimbal.h"
#include "remote_control.h"
#include "NUC_communication.h"
#include "cap_ctl.h"
#include "string.h"

#define WHEEL_RADIUS 0.15240f // m

// car conf
#define ROLLER_DISTANCE 100 // mm  轴距
#define WHEELS_DISTANCE 100 // mm  轮距

#define FR 0
#define FL 1
#define BL 2
#define BR 3

float Plimit=1.0f;	//等比系数
// motor speed PID
PID_t motor_speed[4];
PID_t chassis_follow_flow;


// chassis struct
struct chassis_status chassis;
float x_s, y_s, r_s;
float relative_angle_Flow = 0;
float sin_beta, cos_beta;

float chassisspeedx;
float chassisspeedy;
float chassisspeedr;

int Last_Hp = 400;
int Hp_Time_Wait = 0;

/**
 * @description: 用来限制最大速度
 * @param {float} *val 被限制的???
 * @param {float} MAX  最大限???
 * @return {*}
 */
void val_limit(float *val, float MAX)
{
	if (fabs(*val) > MAX)
	{
		if (*val > 0)
			*val = MAX;
		else
			*val = -MAX;
	}
}

/**
 * @description: 初始化底盘的最大速度还有pid
 * @return {*}
 */
void chassis_move_init()
{
	chassis.speed.max_x = 7.0f; // m/s
	chassis.speed.max_y = 7.0f; // m/s
	chassis.speed.max_r = 5.0f; //
//	pid_set(&chassis_follow_flow, 30.0f, 0, 0, 100, 0);
	pid_set(&chassis_follow_flow, 13.5f, 0, 0, 100, 0);
//  pid_set_chassis(&chassis_follow_flow,0,15.0f, 0, 0, 100, 0);
	pid_set(&motor_speed[FR], 8000, 0, 250, MAX_CURRENT, 0);
	pid_set(&motor_speed[FL], 8000, 0, 250, MAX_CURRENT, 0);
	pid_set(&motor_speed[BL], 8000, 0, 250, MAX_CURRENT, 0);
	pid_set(&motor_speed[BR], 8000, 0, 250, MAX_CURRENT, 0);
}

/**
 * @description: 把全局状态里的速度数据传递给底盘
 * @return {*}
 */
void get_global_chassis_input()
{
//	relative_angle_Flow = get_motor_data(YAW_MOTOR).angle - gimbal.yaw.IMU_offset;
	relative_angle_Flow = 0;
	extern float big_yaw_angle;
	relative_angle_Flow = big_yaw_angle;

	sin_beta = sinf(degree2rad(relative_angle_Flow)); // 输入弧度，输出对应的角度sin，cos对应???
	cos_beta = cosf(degree2rad(relative_angle_Flow));

	x_s = Global.input.y * cos_beta + sin_beta * Global.input.x; // 运动分解
	y_s = -Global.input.y * sin_beta + Global.input.x * cos_beta;
	r_s = Global.input.r;	

	uint32_t mul = 0;
	mul = fabs(relative_angle_Flow) / 180;
	if (relative_angle_Flow > 180.0f)
	{
		if (mul % 2 == 1) // 处于-180度
			relative_angle_Flow -= (mul + 1) * 180;
		else // 处于180度
			relative_angle_Flow -= mul * 180;
	}
	if (relative_angle_Flow < -180.0f)
	{
		if (mul % 2 == 1) // 处于180度
			relative_angle_Flow += (mul + 1) * 180;
		else // 处于-180度
			relative_angle_Flow += mul * 180;
	}
	
	switch (Global.mode)
	{
	case FLOW:

	if(fabs(relative_angle_Flow)>1.0f)
	{
		r_s = pid_cal(&chassis_follow_flow, -degree2rad(relative_angle_Flow), 0.0f);
	}
	else
	{
		r_s = 0;
	}
		chassis.speed.x = x_s;
		chassis.speed.y = y_s;
		chassis.speed.r = r_s;
		break;
	case SPIN_L:
		// SPIN_L mode operations
		chassis.speed.x = x_s;
		chassis.speed.y = y_s;
		chassis.speed.r = 7.0f;
		break;
	case SPIN_R:
		// SPIN_R mode operations
		chassis.speed.x = x_s;
		chassis.speed.y = y_s;
		chassis.speed.r = -5.0f;
		break;
	case NAV:
		// NAV mode operations
//		chassis.speed.x =  -(-Navigation_receive_1.x_speed * sin_beta + Navigation_receive_1.y_speed * cos_beta);
//		chassis.speed.y = -(Navigation_receive_1.x_speed * cos_beta + sin_beta * Navigation_receive_1.y_speed);
//		chassis.speed.r = pid_cal(&chassis_follow_flow, degree2rad(relative_angle_Flow), 0.0f);
//行进速度传入与停止小陀螺
  if(Navigation_receive_1.x_speed != 0 && Navigation_receive_1.y_speed != 0 && Navigation_receive_1.header != 0)
		{
			chassis.speed.x =  -((Navigation_receive_1.y_speed) * sin_beta + (-Navigation_receive_1.x_speed) * cos_beta)*2.0;
			chassis.speed.y = -((Navigation_receive_1.y_speed) * cos_beta - sin_beta * (-Navigation_receive_1.x_speed))*2.0;
			chassis.speed.r = 5;
			// Navigation_receive_1.x_speed *= 0.8;
			// 	Navigation_receive_1.y_speed *= 0.8;
			// 	if(((Navigation_receive_1.x_speed)*(Navigation_receive_1.x_speed) + (Navigation_receive_1.y_speed)*(Navigation_receive_1.y_speed)) > 1)
			// {	Navigation_receive_1.x_speed *= 0.4;
			// 	Navigation_receive_1.y_speed *= 0.4;6 
			// }
			}
		else
		{
		chassis.speed.x = 0;
		chassis.speed.y = 0;
		if((REFEREE_DATA.remain_hp >= Last_Hp && Get_sys_time_s() - Hp_Time_Wait > 5) || cap .remain_vol < 16)//没受到攻击就转的慢点
		{
			chassis.speed.r = 0;
			
		}
		else//检测到掉血就高速或者变速
		{
			if(chassis.speed.r == 1.5)
			Hp_Time_Wait = Get_sys_time_s();
			chassis.speed.r = 6.0;

		}
		Last_Hp = REFEREE_DATA.remain_hp;
			Navigation_receive_1.header = 0;
			Navigation_receive_1.checksum = 0;
			Navigation_receive_1.x_speed  = 0;
			Navigation_receive_1.y_speed = 0;
			Navigation_receive_1.yaw_speed = 0;
			Navigation_receive_1.rotate = 0;
			Navigation_receive_1.running_state = 0;

		}
		break;
	case SPIN_SCAN:
		// LOCK mode operations
		chassis.speed.x = 0;
		chassis.speed.y = 0;
		chassis.speed.r = r_s;
		break;
	case LOCK:
		// LOCK mode operations
		chassis.speed.x = 0;
		chassis.speed.y = 0;
		chassis.speed.r = 0;
		break;
	default:
		// Default to LOCK mode
		chassis.speed.x = 0;
		chassis.speed.y = 0;
		chassis.speed.r = 0;
		break;
	}
}


float now_p = 0.0f;
float b = 0.001f;
float power_limit(int16_t current[4])
{
		
	
	float max_p; // = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量

	if (cap.remain_vol <= 5)
		max_p = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量
	else if (cap.remain_vol > 5)
	{
	  max_p = REFEREE_DATA.Chassis_Power_Limit +cap.remain_vol * 12.5 ;// 超电最大功率 = 超电电压 * 14A 线圈最大电流 
	}
	
	now_p = 0;	
	
	const float a = 1.23e-07;	 // k1
  const float k2 = 6.65300036e-07; // k2
	const float constant = 4.081f;
	const float toque_coefficient = (20.0f/16384.0f)*(0.3f)*(187.0f/3591.0f)/9.55f;
	for (int i = 0; i < 4; i++)
	{
		// 估算功率
		// 西交利物浦：https://github.com/MaxwellDemonLin/Motor-modeling-and-power-control/blob/master/chassis_power_control.c#L89
		now_p += fabs(current[i] * toque_coefficient * get_motor_data(i).speed_rpm +
					  k2 * get_motor_data(i).speed_rpm * get_motor_data(i).speed_rpm +
					  a * current[i] * current[i] + constant) / 0.85f;
	}

	float percentage = max_p / now_p;
	
	if (percentage > 1.0f)
		return 1.0f;
	return percentage - b;
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Send_to_Chassis_1()	//发送底盘速度数据
{
	uint8_t can_send_data [8];
	static CAN_TxHeaderTypeDef tx_message;
	uint32_t send_mail_box;

	tx_message.StdId = 0x301;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;

	float_to_bytes(chassis.speed.x, &can_send_data[0]);  // 将x的字节存入can_data[0]~can_data[3]
    float_to_bytes(chassis.speed.y, &can_send_data[4]);  // 将y的字节存入can_data[4]~can_data[7]

	HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}

void Send_to_Chassis_2()	//发送底盘速度数据
{
	uint8_t can_send_data [8];
	static CAN_TxHeaderTypeDef tx_message;
	uint32_t send_mail_box;
   // uint8_t Lock_flag=Global.mode;
	tx_message.StdId = 0x302;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	float_to_bytes(chassis.speed.r, &can_send_data[0]);  // 将r的字节存入can_data[0]~can_data[3]
	float_to_bytes(gimbal.big_yaw.set, &can_send_data[4]);	 // 将big_yaw的字节存入can_data[4]~can_data[7]
		
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}
void Send_to_Chassis_3()
{
	uint8_t can_send_data [8];
	static CAN_TxHeaderTypeDef tx_message;
	uint32_t send_mail_box;
   // uint8_t Lock_flag=Global.mode;
	tx_message.StdId = 0x304;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	float_to_bytes(Global.mode, &can_send_data[0]);  // 将r的字节存入can_data[0]~can_data[3]
		
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}
// 将float按字节拆分
void float_to_bytes(float f, uint8_t *bytes) 
	{
    uint32_t *p = (uint32_t *)&f;  // 将float指针强制转换为uint32_t指针
    uint32_t temp = *p;            // 获取float的二进制表示
    bytes[0] = (temp >> 0) & 0xFF; // 提取最低字节
    bytes[1] = (temp >> 8) & 0xFF; // 提取第二个字节
    bytes[2] = (temp >> 16) & 0xFF; // 提取第三个字节
    bytes[3] = (temp >> 24) & 0xFF; // 提取最高字节
  }