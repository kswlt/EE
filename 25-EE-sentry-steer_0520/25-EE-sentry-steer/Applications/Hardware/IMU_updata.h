/**
 * @file IMU_updata.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "struct_typedef.h"
#include "Kalman_Filter_c.h"

#define __IMU_UPDATA__
#ifdef __IMU_UPDATA__

struct IMU_t
{
	fp32 gyro[3];  // rad/s
	fp32 accel[3]; // m/s^2
	fp32 mag[3];   // ut
	fp32 temp;	   // �¶�
	
	float calibration[3];//���ٶ�У׼error
	float cali_end[3];

	Attitude_3D_t KF; //�������˲����

	struct
	{
		fp32 q[4];			   //��Ԫ��
		fp32 pitch, yaw, roll; //ŷ����
	} Mahony;

	struct
	{
		fp32 q[4];			   //��Ԫ��
		fp32 pitch, yaw, roll; //ŷ����
	} madgwick;

	struct
	{
		fp32 q[4];			   //��Ԫ��
		fp32 pitch, yaw, roll; //ŷ����
		int8_t err_code;
		fp32 last_yaw;
		fp32 yaw_rad_cnt;
		fp32 yaw_angle_cnt;
	} AHRS;

};

extern struct IMU_t IMU_data;
extern unsigned  long ulTdleCycleCount;	


// �ⲿ����
void IMU_init(void);   // IMU��ʼ��
void IMU_updata(void); // IMU���ݸ���
void MagUpdate(void);  // ���������ݸ���
void MagZero(void); // ����شż�
void Get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);//�ǵ���Lib�汾
void process_IMU_data();
float get_history_data(uint8_t yaw_or_pitch,uint32_t history_time);
float get_history_q_data(uint8_t i,uint32_t history_time);//��ȡ��ʷ���ݺ���
float rad2degree(float a);
float degree2rad(float a);
void imu_cail_program(void);
void IMU_offest(void);
#endif
// end of flie
