//基于串口协议附录V1.5 


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


#define REFEREE_LEN_HEADER 5  //帧头长度
#define REFEREE_LEN_CMDID  2  // cmd_id长度
#define REFEREE_LEN_TAIL   2  //帧尾长度

#define REFEREE_FRAME_HEADER 0xA5  //数据帧起始字节，固定值为 0xA5

/* 帧头偏移量 */
#define REFEREE_OFFSET_SOF         0  //帧头SOF偏移量
#define REFEREE_OFFSET_DATA_LENGTH 1  //帧头数据长度偏移量
#define REFEREE_OFFSET_SEQ         3  //帧头包序号偏移量
#define REFEREE_OFFSET_CRC8        4  //帧头CRC8偏移量

/* 帧内有效数据包偏移量 */
#define REFEREE_OFFSET_DATA (REFEREE_LEN_HEADER + REFEREE_LEN_CMDID)

/* 裁判系统接收缓冲区大小 */
#define REFEREE_RECV_BUF_SIZE 255

extern uint8_t RefereeRecvBuf[REFEREE_RECV_BUF_SIZE];


#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 =         0x0001,//比赛状态数据 固定 3Hz 频率发送 服务器→全体机器人
    GAME_RESULT_CMD_ID                =         0x0002,//比赛结果数据 比赛结束触发发送 服务器→全体机器人
    GAME_ROBOT_HP_CMD_ID              =         0x0003,//机器人血量数据 固定 3Hz 频率发送 服务器→全体机器人
    FIELD_EVENTS_CMD_ID               =         0x0101,//场地事件数据，固定 3Hz 频率发送 服务器→己方全体机器人
    REFEREE_WARNING_CMD_ID            =         0x0104,//裁判警告数据 己方判罚/判负时触发发送 服务器→被处罚方全体机器人
    DART_LAUNCH_TIME_CMD_ID =                   0x0105,//镖发射时间数据 固定 3Hz 频率发送 服务器→己方全体机器人
    ROBOT_STATE_CMD_ID                =         0x0201,//机器人性能体系数据 固定 10Hz 频率发送 主控模块→对应机器人
    POWER_HEAT_DATA_CMD_ID            =         0x0202,//实时功率热量数据 固定 50Hz 频率发送 主控模块→对应机器人
    ROBOT_POS_CMD_ID                  =         0x0203,//机器人位置数据 固定 10Hz 频率发送 主控模块→对应机器人
    BUFF_MUSK_CMD_ID                  =         0x0204,//机器人增益数据 固定 3Hz 频率发送 服务器→对应机器人
    ROBOT_HURT_CMD_ID                 =         0x0206,//伤害状态数据 伤害发生后发送 主控模块→对应机器人
    SHOOT_DATA_CMD_ID                 =         0x0207,//实时射击数据 弹丸发射后发送 主控模块→对应机器人
    BULLET_REMAINING_CMD_ID           =         0x0208,//允许发弹量 固定 10Hz 频率发送 服务器→己方英雄、步兵、哨兵、空中机器人
    ROBOT_RFID_STATE_CMD_ID           =         0x0209,//机器人 RFID 状态 固定 3Hz 频率发送 服务器→己方装有 RFID 模块的机器人
    DART_PLAYER_COMMAND_CMD_ID        =         0x020A,//飞镖选手端指令数据 飞镖闸门上线后固定 10Hz 频率发送 服务器→己方飞镖机器人
    GROUND_ROBOT_POSITION_CMD_ID      =         0x020B,//地面机器人位置数据 固定 1Hz 频率发送 服务器→己方哨兵机器人
    RADAR_MARKS_PROGRESS_CMD_ID       =         0x020C,//雷达标记进度数据 固定 1Hz 频率发送 服务器→己方雷达机器人
    SENTRY_INFO_CMD_ID                =         0x020D,//哨兵自主决策信息同步，固定以1Hz 频率发送 服务器→己方哨兵机器人
    RADAR_INFO_CMD_ID                 =         0x020E, //雷达自主决策信息同步，固定以1Hz 频率发送 服务器→己方雷达机器人 
    ROBOT_INTERACTION_CMD_ID          =         0x0301,//机器人交互数据 发送方触发发送 频率上限为 10Hz
    CUSTOM_CONTROLLER_AND_BOT_INTERACTION_CMD_ID  =   0x0302,//自定义控制器与机器人交互数据 发送方触发发送 频率上限为 30Hz  自定义控制器→选手端图传连接的机器人
    PLAYER_SIDE_MINIMAP_INTERACTION_CMD_ID        =   0x0303,//选手端小地图交互数据 选手端触发发送 选手端点击→服务器→发送方选择的己方机器人
    KEYBOARD_AND_MOUSE_REMOTE_CONTROL_CMD_ID      =   0x0304,//键鼠遥控数据 固定 30Hz 频率发送 客户端→选手端图传连接的机器人
    PLAYER_MINIMAP_RECEIVES_RADAR_CMD_ID          =   0x0305,//选手端小地图接收雷达数据 频率上限为10Hz  雷达→服务器→己方所有选手端
    INTERACTION_DATA_BETWEEN_CONTROLLER_AND_PLAYER_CMD_ID = 0x0306,//自定义控制器与选手端交互数据 发送方触发发送 频率上限为30Hz 自定义控制器→选手端
    PLAYER_MINIMAP_RECEIVES_SENTRY_CMD_ID                 = 0x0307,//选手端小地图接收哨兵数据 频率上限为1Hz 哨兵→己方云台手选手端
    CUSTOM_INFO_CMD_ID                =         0x0308,//选手端小地图接收机器人数据， 频率上限为 3Hz 己方机器人→己方选手端
    ROBOT_CUSTOM_DATA_CMD_ID          =         0x0309,//自定义控制器接收机器人数据，频率上限为 10Hz 己方机器人→对应操作手选手端连接的自定义控制器
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
