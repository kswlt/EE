/**
 * @file global_status.h
 * @author sethome
 * @brief ȫ��״̬��
 * @version 0.1
 * @date 2022-03-25
 *
 * @copyright sethome Copyright (c) 2022
 *
 */

#include "stdint.h"

#define __GLOBAL_STATUS_H__
#ifdef __GLOBAL_STATUS_H__

// ������
enum err_e
{
    GIMBAL_ERR = 0,
    CHASSIS_ERR,
    SHOOT_ERR,
    CAP_ERR,
    REMOTE_ERR,
    PC_ERR,
};
enum ctl_e
{
    RC = 0,
    PC,
};
// �ʺ�ʮ������״̬�����
struct GlobalStatus_t
{
    uint8_t err[6]; // 9

    enum mode_e
    {
        LOCK = 0,
        FLOW,
        TANK,
        SPIN,
        LEAN,
        TUSP,

    } mode;

    enum cap_e
    {
        STOP = 0, // ��ʱ����Ӧ�ڳ��
        FULL,     // ȫ����Ӧ
    } cap;

    struct
    {
        /* data */
        float x, y, r; // �����ƶ�
        float pitch, yaw;

        uint8_t shooter_status;
        uint8_t shoot_fire;
        uint8_t shoot_num;
        uint8_t shoot_RC;

        uint8_t trigger_begin;
        uint8_t trigger_kill;
        uint8_t trigger_kill_cnt;

        uint8_t vision_status;
        uint8_t lid;
        uint8_t anti_stauts;
        uint8_t fly_status;
        uint8_t ui_init;
        uint8_t vision_online;
        enum ctl_e ctl;
    } input;
};

extern struct GlobalStatus_t Global;

void Global_set_err(enum err_e err, uint8_t status);

#endif
// end of file