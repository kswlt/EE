/**
 * @file CAN_receive&send.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2021-12-09
 *
 * @copyright Copyright (c) 2021 sethome
 *
 */

#ifndef __CAN_RECEIVE_H__
#define __CAN_RECEIVE_H__

#include "struct_typedef.h"
#include "main.h"

// ʹ��6020�����ID5-7�������������һ�㲻����ʹ��
// #define USE_CAN_1_6020
// #define USE_CAN_2_6020
#define USE_FREERTOS_DELAY
// #define USE_NOP_DELAY

/* CAN send and receive  */
typedef enum
{
  CAN_1_4_SIGN_ID = 0x200,
  CAN_ID1 = 0x201,
  CAN_ID2 = 0x202,
  CAN_ID3 = 0x203,
  CAN_ID4 = 0x204,

  CAN_5_8_SIGN_ID = 0x1FF,
  CAN_ID5 = 0x205,
  CAN_ID6 = 0x206,
  CAN_ID7 = 0x207,
  CAN_ID8 = 0x208,

  CAN_6020_SIGN_ID = 0x2FF,
  CAN_6020_ID5 = 0x209,
  CAN_6020_ID6 = 0x210,
  CAN_6020_ID7 = 0x211,
} can_msg_id_e;

typedef enum
{
  CAN_1_1 = 0,
  CAN_1_2,
  CAN_1_3,
  CAN_1_4,
  CAN_1_5,
  CAN_1_6,
  CAN_1_7,
  CAN_1_8,
  CAN_1_6020_5,
  CAN_1_6020_6,
  CAN_1_6020_7,

  CAN_2_1,
  CAN_2_2,
  CAN_2_3,
  CAN_2_4,
  CAN_2_5, // 15
  CAN_2_6,	//16
  CAN_2_7, // 17
  CAN_2_8,
  CAN_2_6020_5,
  CAN_2_6020_6,
  CAN_2_6020_7,
} can_id;

// ������궨�����ĵ��ID(������ֱ���ڵ����ļ��ж���� ����չʾ) 
// 6020�� CAN_1_5(��ӦID1) �� CAN_1_6020_7����ӦID7��
// 3508/2006�� CAN_1_1(��ӦID1) �� CAN_1_8����ӦID7��
// ���޸�can_msg_id_e
#define motor_1_1 CAN_1_1
#define motor_1_2 CAN_1_2
#define motor_1_3 CAN_1_3
#define motor_1_4 CAN_1_4
#define motor_1_5 CAN_1_5
#define motor_1_6 CAN_1_6
#define motor_1_7 CAN_1_7
#define motor_1_8 CAN_1_8
#define M6020_1_5 CAN_1_6020_5
#define M6020_1_6 CAN_1_6020_6
#define M6020_1_7 CAN_1_6020_7

#define motor_2_1 CAN_2_1
#define motor_2_2 CAN_2_2
#define motor_2_3 CAN_2_3
#define motor_2_4 CAN_2_4
#define motor_2_5 CAN_2_5
#define motor_2_6 CAN_2_6
#define motor_2_7 CAN_2_7
#define motor_2_8 CAN_2_8
#define M6020_2_5 CAN_2_6020_5
#define M6020_2_6 CAN_2_6020_6
#define M6020_2_7 CAN_2_6020_7

// �����������
#define ECD_MAX 8192.0f    // ���������ֵ
#define M3508_P 19.0f      // M3508������ٱ�
#define M2006_P 36.0f      // M2006������ٱ�
#define MAX_CURRENT 8000  // M2006+M3508������ 20A / MAX_CURRENT
#define MAX_6020_VOL 30000 // 6020����ѹ 24V / MAX_6020_VOL

// rm motor data
// notice:Ĭ��Ϊ3508���
// 3508/2006�õ��ǵ�����6020�õ��ǵ�ѹ
typedef struct
{
  int16_t set; // �趨�ĵ��� / ��ѹ

  // ԭʼ����
  uint16_t ecd;          // ��������ֵ
  int16_t speed_rpm;     // ת��  
  int16_t given_current; // ������ĵ���
  uint8_t temperate;     // �¶ȣ���ȡ������
  int16_t last_ecd;      // ��һ�α���������ֵ

  // ��������
  long long ecd_cnt;  // ������������
  double angle_cnt;   // ת�����ܽǶ� degree
  double angle_zero;  // ������0��Ƕ� degree
  double angle;       // -180~180 degree
  double round_speed; // ������ת�ٶ� degree/s
	
} motor_measure_t;

// �ⲿ����
void CAN1_send_current(void);                   // ���͵�����Ƶ���
void CAN2_send_current(void);                   // ���͵�����Ƶ���
void CAN1_send_ZERO_current(void);                   // �����͵�����Ƶ���
void CAN2_send_ZER0_current(void); // ���͵�����Ƶ���
void set_motor(int16_t val, can_id motorID);    // �趨�������/��ѹ
motor_measure_t get_motor_data(can_id motorID); // ��ȡ������Ϣ
extern motor_measure_t motor_data[22];
// ��������Ϣ���ν��루���㣩��ʹ�ò��ּ�������ǰʹ�ã����ڱ�����0�㣬�����ڸ����²�׼ȷ������
void decode_as_3508(can_id motorID);
void decode_as_2006(can_id motorID);
void decode_as_6020(can_id motorID);
void set_motor_offset(can_id motorID, float angle);
void clear_motor_cnt(can_id motorID);
void decode_as_6020_test(can_id motorID);

#endif
