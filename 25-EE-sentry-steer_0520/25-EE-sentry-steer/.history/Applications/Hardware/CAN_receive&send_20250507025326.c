/**
 * @file CAN_receive&send.c
 * @author sethome
 * @brief (CAN_motor & surper_cap) control & send_rev
 * @version 0.1
 * @date 2021-12-13
 *
 * @copyright Copyright (c) 2021 sethome
 *
 */
#include "CAN_receive&send.h"

#include "math.h"
#include "cmsis_os.h"

#include "IMU_updata.h"
#include "cap_ctl.h"
#include "chassis_move.h"
#include "dm_driver.h"

// �������
motor_measure_t motor_data[22];

// CAN�Ĵ�����������
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2; // ����ԭ����can.c�ļ�


// ������ݶ�ȡ
void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
  (ptr)->last_ecd = (ptr)->ecd;
  (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
  (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
  (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
  (ptr)->temperate = (data)[6];
}


void process_motor_data(motor_measure_t *motor_data)
{
  // ������Ȧ��
  if (motor_data->last_ecd > 5000 && motor_data->ecd < 4000)
    motor_data->ecd_cnt += ((ECD_MAX - motor_data->last_ecd) + motor_data->ecd);
  else if (motor_data->last_ecd < 5000 && motor_data->ecd > 6000)
    motor_data->ecd_cnt -= ((ECD_MAX - motor_data->ecd) + motor_data->last_ecd);
  else
    motor_data->ecd_cnt += (motor_data->ecd - motor_data->last_ecd);
}

//DM������ݽ��մ���
void DM4310_UpdateMotor(dm4310_motor_t* motor,uint8_t datas[8])
{
	motor->cur_pos=(datas[0]<<8)|datas[1];
	motor->cur_speed=((int16_t)(datas[2]<<8)|datas[3])/100.0f;
	motor->cur_current=(datas[4]<<8)|datas[5];
	motor->cur_temp=datas[6];
	motor->cur_error_states=datas[7];
	
	/* ���ʹ��λ��1�ж� */
	if(motor->cur_error_states==1)
		motor->enabled=1;
	else
		motor->enabled=0;
	
	if(	(motor->cur_pos>=0)	&&	(motor->cur_pos<=100) && (motor->last_pos>=8094)	&& (motor->last_pos<=8194)	&& motor->cur_speed>0.0f)
		motor->rounds++;
	else if(	(motor->last_pos>=0)	&&	(motor->last_pos<=100) && (motor->cur_pos>=8094)	&& (motor->cur_pos<=8194)	&& motor->cur_speed<0.0f)
		motor->rounds--;
	
	motor->last_pos=motor->cur_pos;
	
	motor->cur_pos_rad=motor->cur_pos/8194.0f*3.1415926f*2.0f;
	
	motor->total_rad=motor->rounds*3.1415926f*2.0f+motor->cur_pos_rad;
	
	
	
	motor->cur_speed_rad=motor->cur_speed*0.10472f;
	
}

// HAL���жϻص�ָ��
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header; // CAN ����ָ��
  uint8_t rx_data[8];            // ��ȡ��������

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // ȡ����Ϣ

  // ��������
  if (rx_header.StdId == 0x307)
  {
    cap_handle_message(rx_data);
    return;
  }
	
	if(rx_header.StdId == 0x08)   //0x11
	{
		dm4310_fbdata(&dm4310,rx_data);
		return;
	}
	
	if(rx_header.StdId == 0x301)
	{
		chassis_receive_from_gimbal_1(rx_data);
		return;
	}
	
	if(rx_header.StdId == 0x302)
	{
		chassis_receive_from_gimbal_2(rx_data);
		return;
	}
  
  if (rx_header.StdId == 0x303)
  {
    receive_REFEREE_DATA(rx_data);
    return;
  }
	
  // DJI���

  if (hcan == &hcan1) // CAN1/2�ж�
  {
    get_motor_measure(&motor_data[rx_header.StdId - CAN_ID1], rx_data);
    process_motor_data(&motor_data[rx_header.StdId - CAN_ID1]);
  }
  else
  {
    get_motor_measure(&motor_data[CAN_1_6020_7 + 1 + rx_header.StdId - CAN_ID1], rx_data);
    process_motor_data(&motor_data[CAN_1_6020_7 + 1 + rx_header.StdId - CAN_ID1]);
  }
}



// ����������ݣ���������ȫ��������
motor_measure_t get_motor_data(can_id motorID) // ��ȡ�������
{
  return motor_data[motorID];
}

// ����������
void set_motor(int16_t val, can_id motorID) // �趨������
{
  motor_data[motorID].set = val; // val
}

// CAN1���͵���
void CAN1_send_current() // ���͵�����Ƶ���
{
  uint8_t can_send_data[8];
  static CAN_TxHeaderTypeDef tx_message;
  uint32_t send_mail_box;

  // ����ǰ4��
  tx_message.StdId = CAN_1_4_SIGN_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;

  can_send_data[0] = (motor_data[CAN_1_1].set >> 8);
  can_send_data[1] = motor_data[CAN_1_1].set;

  can_send_data[2] = (motor_data[CAN_1_2].set >> 8);
  can_send_data[3] = motor_data[CAN_1_2].set;

  can_send_data[4] = (motor_data[CAN_1_3].set >> 8);
  can_send_data[5] = motor_data[CAN_1_3].set;

  can_send_data[6] = (motor_data[CAN_1_4].set >> 8);
  can_send_data[7] = motor_data[CAN_1_4].set;
  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);

  // ���ͺ�4��
  tx_message.StdId = CAN_5_8_SIGN_ID;

  can_send_data[0] = (motor_data[CAN_1_5].set >> 8);
  can_send_data[1] = motor_data[CAN_1_5].set;

  can_send_data[2] = (motor_data[CAN_1_6].set >> 8);
  can_send_data[3] = motor_data[CAN_1_6].set;

  can_send_data[4] = (motor_data[CAN_1_7].set >> 8);
  can_send_data[5] = motor_data[CAN_1_7].set;

  can_send_data[6] = (motor_data[CAN_1_8].set >> 8);
  can_send_data[7] = motor_data[CAN_1_8].set;
  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);

#ifdef USE_CAN_1_6020

  tx_message.StdId = CAN_6020_SIGN_ID;

  can_send_data[0] = (motor_data[CAN_1_6020_5].set >> 8);
  can_send_data[1] = motor_data[CAN_1_6020_5].set;

  can_send_data[2] = (motor_data[CAN_1_6020_6].set >> 8);
  can_send_data[3] = motor_data[CAN_1_6020_6].set;

  can_send_data[4] = (motor_data[CAN_1_6020_7].set >> 8);
  can_send_data[5] = motor_data[CAN_1_6020_7].set;

  can_send_data[6] = 0;
  can_send_data[7] = 0;

#ifdef USE_NOP_DELAY
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
  };
#endif
#ifdef USE_FREERTOS_DELAY
  osDelay(1); // ��ʱ1ms
#endif

  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
#endif
}

// CAN2 ���͵���
void CAN2_send_current() // ���͵�����Ƶ���
{
  uint8_t can_send_data[8];
  static CAN_TxHeaderTypeDef tx_message;
  uint32_t send_mail_box;

  // ����ǰ4��
  tx_message.StdId = CAN_1_4_SIGN_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;

  can_send_data[0] = (motor_data[CAN_2_1].set >> 8);
  can_send_data[1] = motor_data[CAN_2_1].set;

  can_send_data[2] = (motor_data[CAN_2_2].set >> 8);
  can_send_data[3] = motor_data[CAN_2_2].set;

  can_send_data[4] = (motor_data[CAN_2_3].set >> 8);
  can_send_data[5] = motor_data[CAN_2_3].set;

  can_send_data[6] = (motor_data[CAN_2_4].set >> 8);
  can_send_data[7] = motor_data[CAN_2_4].set;
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);

  // ���ͺ�4��
  tx_message.StdId = CAN_5_8_SIGN_ID;

  can_send_data[0] = (motor_data[CAN_2_5].set >> 8);
  can_send_data[1] = motor_data[CAN_2_5].set;

  can_send_data[2] = (motor_data[CAN_2_6].set >> 8);
  can_send_data[3] = motor_data[CAN_2_6].set;

  can_send_data[4] = (motor_data[CAN_2_7].set >> 8);
  can_send_data[5] = motor_data[CAN_2_7].set;

  can_send_data[6] = (motor_data[CAN_2_8].set >> 8);
  can_send_data[7] = motor_data[CAN_2_8].set;
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);

#ifdef USE_CAN_2_6020
  tx_message.StdId = CAN_6020_SIGN_ID;

  can_send_data[0] = (motor_data[CAN_2_6020_5].set >> 8);
  can_send_data[1] = motor_data[CAN_2_6020_5].set;

  can_send_data[2] = (motor_data[CAN_2_6020_6].set >> 8);
  can_send_data[3] = motor_data[CAN_2_6020_6].set;

  can_send_data[4] = (motor_data[CAN_2_6020_7].set >> 8);
  can_send_data[5] = motor_data[CAN_2_6020_7].set;

  can_send_data[6] = 0;
  can_send_data[7] = 0;

#ifdef USE_NOP_DELAY
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
  {
  };
    // uint32_t delay = 250 * 168 / 4; //��ʱ250ns
    // do
    // {
    //   __NOP();
    // } while (delay--);
#endif
#ifdef USE_FREERTOS_DELAY
  osDelay(1); // ��ʱ1ms
#endif
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
#endif
}

void CAN1_send_ZERO_current() // ���͵�����Ƶ���
{
  uint8_t can_send_data[8];
  static CAN_TxHeaderTypeDef tx_message;
  uint32_t send_mail_box;

  // ����ǰ4��
  tx_message.StdId = CAN_1_4_SIGN_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;

  memset(can_send_data,0,8);
  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);

  // ���ͺ�4��
  tx_message.StdId = CAN_5_8_SIGN_ID;

   memset(can_send_data,0,8);
  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);

#ifdef USE_CAN_1_6020

  tx_message.StdId = CAN_6020_SIGN_ID;

   memset(can_send_data,0,8);


#ifdef USE_NOP_DELAY
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
  };
#endif
#ifdef USE_FREERTOS_DELAY
  osDelay(1); // ��ʱ1ms
#endif

  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
#endif
}

// CAN2 ���͵���
void CAN2_send_ZER0_current() // ���͵�����Ƶ���
{
  uint8_t can_send_data[8];
  static CAN_TxHeaderTypeDef tx_message;
  uint32_t send_mail_box;

  // ����ǰ4��
  tx_message.StdId = CAN_1_4_SIGN_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;

  memset(can_send_data,0,8);
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);

  // ���ͺ�4��
  tx_message.StdId = CAN_5_8_SIGN_ID;

  memset(can_send_data,0,8);
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);

#ifdef USE_CAN_2_6020
  tx_message.StdId = CAN_6020_SIGN_ID;

    memset(can_send_data,0,8);

#ifdef USE_NOP_DELAY
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
  {
  };
    // uint32_t delay = 250 * 168 / 4; //��ʱ250ns
    // do
    // {
    //   __NOP();
    // } while (delay--);
#endif
#ifdef USE_FREERTOS_DELAY
  osDelay(1); // ��ʱ1ms
#endif
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
#endif
}
void decode_as_3508(can_id motorID)
{
  // ��������ٶ�
  motor_data[motorID].round_speed = motor_data[motorID].speed_rpm / 60.0f / M3508_P;
  // �����ܽǶ�
  motor_data[motorID].angle_cnt = motor_data[motorID].ecd_cnt / ECD_MAX / M3508_P * 360.00f;
}

void decode_as_2006(can_id motorID)
{
  // ��������ٶ�
  motor_data[motorID].round_speed = motor_data[motorID].speed_rpm / 60.0f / M2006_P;
  // �����ܽǶ�
  motor_data[motorID].angle_cnt = motor_data[motorID].ecd_cnt / ECD_MAX / M2006_P * 360.00f;
}

void decode_as_6020(can_id motorID)
{
  // ��������ٶ�
  motor_data[motorID].round_speed = motor_data[motorID].speed_rpm / ECD_MAX * 360.0f;
  // �����ܽǶ�
  motor_data[motorID].angle_cnt = motor_data[motorID].ecd_cnt/ ECD_MAX * 360.00f;

  // ������ԽǶ� -180~180 �������ȶ�ʧ �ܽǶȹ���ʱ
  float angle = motor_data[motorID].angle_cnt - motor_data[motorID].angle_zero;
  uint32_t mul = fabs(angle) / 180;
  if (angle > 180.0f)
  {
    if (mul % 2 == 1) // ����-180��
      angle -= (mul + 1) * 180;
    else // ����180��
      angle -= mul * 180;
  }
  if (angle < -180.0f)
  {
    if (mul % 2 == 1) // ����180��
      angle += (mul + 1) * 180;
    else // ����-180��
      angle += mul * 180;
  }
  motor_data[motorID].angle = angle;
}


void decode_as_6020_test(can_id motorID)
{
  // ��������ٶ�
  motor_data[motorID].round_speed = motor_data[motorID].speed_rpm / ECD_MAX * 360.0f;
  // �����ܽǶ�
  motor_data[motorID].angle_cnt = motor_data[motorID].ecd_cnt/3.0f / ECD_MAX * 360.00f;

  //����
  float angle_0 = motor_data[motorID].angle_cnt - motor_data[motorID].angle_zero;
  uint32_t mul = abs((int)angle_0) / 180;
  if (angle_0 > 180.0f)
  {
    if (mul % 2 == 1) // ����-180��
      motor_data[motorID].angle =angle_0 - (mul + 1) * 180;
    else // ����180��
      motor_data[motorID].angle =angle_0 - mul * 180;
  }
	else if (angle_0 < -180.0f)
  {
    if (mul % 2 == 1) // ����180��
      motor_data[motorID].angle =angle_0 +(mul + 1) * 180;
    else // ����-180��
      motor_data[motorID].angle =angle_0 +mul * 180;
  }
	else 
		  motor_data[motorID].angle = angle_0;

}


void set_motor_offset(can_id motorID, float angle)
{
  motor_data[motorID].angle_zero = angle;
}

void clear_motor_cnt(can_id motorID)
{
  motor_data[motorID].ecd_cnt = 0;
}


