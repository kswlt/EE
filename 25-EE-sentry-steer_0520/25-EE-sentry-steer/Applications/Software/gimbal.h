/**
 * @file gimbal.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#define __GIMBAL_H__
#ifdef __GIMBAL_H__

#include "pid.h"
#include "stdint.h"
#include "dm_driver.h"
// yaw��5��pitch��6���¶�����
// yaw��6��pitch��5���϶�����
// ��̨�������
//#define PITCH_MOTOR CAN_1_6
//#define YAW_MOTOR CAN_1_5
#define PITCH_MOTOR CAN_1_8
#define YAW_MOTOR CAN_1_7
#define PI 3.1415926f

extern pid_t yaw_absolute_speed_pid;
extern pid_t yaw_absolute_pid;

enum gimbal_status_e
{
    LOCATION = 0,
    SPEED,
    ABSOLUTE,
    zero_force,
};

struct gimbal_status
{
    // �趨��̨����ģʽ
    enum gimbal_status_e pitch_status;
    enum gimbal_status_e yaw_status;
		enum gimbal_status_e big_yaw_status;

    struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } pitch;
    float pitch_speed;
    float set_pitch_speed;

    struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } yaw;
    float yaw_speed;
    float set_yaw_speed;
		
		struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } big_yaw;
    float big_yaw_speed;
    float set_big_yaw_speed;
    float big_yaw_current;
};

extern struct gimbal_status gimbal;

// �ⲿ����
void gimbal_init(void);    // ��ʼ����̨
void gimbal_pid_cal(void); // ��̨PID����

void gimbal_set_offset(float ab_pitch, float ab_yaw, float lo_pitch, float lo_yaw); // ��ʼ�����
void gimbal_set_pitch(float pitch, float up_angle, float down_angle);               // �趨picth�Ƕ�,�޷���

float slope_calculation(float IMU_pitch, float LOCATION_pitch); // ������������ĽǶ�

void gimbal_clear_cnt(void);
void gimbal_set_speed(float pitch, float yaw); // �趨�ٶ�
float feedforward_contorl(float target);       // ǰ��


//big_yaw
void big_yaw_init();
void big_yaw_cal();
void set_big_yaw_pos_vel(float pos, float vel,motor_t *motor);
void big_yaw_update();
void relative_angle_big_yaw_send();
void float_to_bytes(float f, uint8_t *bytes) ;
float iir_low_pass_filter(float input, float alpha);
float big_yaw_feedforward(float in);//��yawǰ��

// end of file

#endif
