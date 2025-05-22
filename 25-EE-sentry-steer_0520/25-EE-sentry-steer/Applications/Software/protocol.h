//���ڴ���Э�鸽¼V1.5 


#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#include <stdbool.h>


#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            5//sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))


#define REFEREE_LEN_HEADER 5  //֡ͷ����
#define REFEREE_LEN_CMDID  2  // cmd_id����
#define REFEREE_LEN_TAIL   2  //֡β����

#define REFEREE_FRAME_HEADER 0xA5  //����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5

/* ֡ͷƫ���� */
#define REFEREE_OFFSET_SOF         0  //֡ͷSOFƫ����
#define REFEREE_OFFSET_DATA_LENGTH 1  //֡ͷ���ݳ���ƫ����
#define REFEREE_OFFSET_SEQ         3  //֡ͷ�����ƫ����
#define REFEREE_OFFSET_CRC8        4  //֡ͷCRC8ƫ����

/* ֡����Ч���ݰ�ƫ���� */
#define REFEREE_OFFSET_DATA (REFEREE_LEN_HEADER + REFEREE_LEN_CMDID)

/* ����ϵͳ���ջ�������С */
#define REFEREE_RECV_BUF_SIZE 255

extern uint8_t RefereeRecvBuf[REFEREE_RECV_BUF_SIZE];


#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 =         0x0001,//����״̬���� �̶� 3Hz Ƶ�ʷ��� ��������ȫ�������
    GAME_RESULT_CMD_ID                =         0x0002,//����������� ���������������� ��������ȫ�������
    GAME_ROBOT_HP_CMD_ID              =         0x0003,//������Ѫ������ �̶� 3Hz Ƶ�ʷ��� ��������ȫ�������
    FIELD_EVENTS_CMD_ID               =         0x0101,//�����¼����ݣ��̶� 3Hz Ƶ�ʷ��� ������������ȫ�������
    SUPPLY_PROJECTILE_ACTION_CMD_ID   =         0x0102,//����վ������ʶ���� ����վ�����ͷ�ʱ�������� ������������ȫ�������
    REFEREE_WARNING_CMD_ID            =         0x0104,//���о������� �����з�/�и�ʱ�������� ����������������ȫ�������
    DART_LAUNCH_TIME_CMD_ID =                   0x0105,//�ڷ���ʱ������ �̶� 3Hz Ƶ�ʷ��� ������������ȫ�������

    ROBOT_STATE_CMD_ID                =         0x0201,//������������ϵ���� �̶� 10Hz Ƶ�ʷ��� ����ģ�����Ӧ������
    POWER_HEAT_DATA_CMD_ID            =         0x0202,//ʵʱ������������ �̶� 50Hz Ƶ�ʷ��� ����ģ�����Ӧ������
    ROBOT_POS_CMD_ID                  =         0x0203,//������λ������ �̶� 10Hz Ƶ�ʷ��� ����ģ�����Ӧ������
    BUFF_MUSK_CMD_ID                  =         0x0204,//�������������� �̶� 3Hz Ƶ�ʷ��� ����������Ӧ������
    AERIAL_ROBOT_ENERGY_CMD_ID        =         0x0205,//����֧Ԯʱ������ �̶� 10Hz Ƶ�ʷ��� ���������������л�����
    ROBOT_HURT_CMD_ID                 =         0x0206,//�˺�״̬���� �˺��������� ����ģ�����Ӧ������
    SHOOT_DATA_CMD_ID                 =         0x0207,//ʵʱ������� ���跢����� ����ģ�����Ӧ������
    BULLET_REMAINING_CMD_ID           =         0x0208,//�������� �̶� 10Hz Ƶ�ʷ��� ������������Ӣ�ۡ��������ڱ������л�����
    ROBOT_RFID_STATE_CMD_ID           =         0x0209,//������ RFID ״̬ �̶� 3Hz Ƶ�ʷ��� ������������װ�� RFID ģ��Ļ�����
    DART_PLAYER_COMMAND_CMD_ID        =         0x020A,//����ѡ�ֶ�ָ������ ����բ�����ߺ�̶� 10Hz Ƶ�ʷ��� ���������������ڻ�����
    GROUND_ROBOT_POSITION_CMD_ID      =         0x020B,//���������λ������ �̶� 1Hz Ƶ�ʷ��� �������������ڱ�������
    RADAR_MARKS_PROGRESS_CMD_ID       =         0x020C,//�״��ǽ������� �̶� 1Hz Ƶ�ʷ��� �������������״������
	SENTRY_DATA_CMD_ID				  =			0x020D,//�ձ��������ݣ�3HZ ���������ˡ�����ѡ�ֶ� 
	RADAR_DATA_CMD_ID 				  =			0x020E,//�״�������ݣ��״�����������Ϣͬ�����̶���1Hz Ƶ�ʷ��� �������������״������
	
	
    ROBOT_INTERACTION_CMD_ID          =         0x0301,//�����˽������� ���ͷ��������� Ƶ������Ϊ 10Hz
    CUSTOM_CONTROLLER_AND_BOT_INTERACTION_CMD_ID  =   0x0302,//�Զ��������������˽������� ���ͷ��������� Ƶ������Ϊ 30Hz  �Զ����������ѡ�ֶ�ͼ�����ӵĻ�����
    PLAYER_SIDE_MINIMAP_INTERACTION_CMD_ID        =   0x0303,//ѡ�ֶ�С��ͼ�������� ѡ�ֶ˴������� ѡ�ֶ˵���������������ͷ�ѡ��ļ���������
    KEYBOARD_AND_MOUSE_REMOTE_CONTROL_CMD_ID      =   0x0304,//����ң������ �̶� 30Hz Ƶ�ʷ��� �ͻ��ˡ�ѡ�ֶ�ͼ�����ӵĻ�����
    PLAYER_MINIMAP_RECEIVES_RADAR_CMD_ID          =   0x0305,//ѡ�ֶ�С��ͼ�����״����� Ƶ������Ϊ10Hz  �״������������������ѡ�ֶ�
    INTERACTION_DATA_BETWEEN_CONTROLLER_AND_PLAYER_CMD_ID = 0x0306,//�Զ����������ѡ�ֶ˽������� ���ͷ��������� Ƶ������Ϊ30Hz �Զ����������ѡ�ֶ�
    PLAYER_MINIMAP_RECEIVES_SENTRY_CMD_ID                 = 0x0307,//ѡ�ֶ�С��ͼ�����ڱ����� Ƶ������Ϊ1Hz �ڱ���������̨��ѡ�ֶ�
    EXCHANGE_DATA_CMD_ID						  =	  0x0308,//ѡ�ֶ�С��ͼ���ջ��������ݣ�Ƶ������Ϊ 3Hz���������ˡ�����ѡ�ֶ� ������·
	IDCustomData,
}referee_cmd_id_t;


typedef __packed struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;


typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;


typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;


#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
