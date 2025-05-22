/**
 * @file Global.c
 * @author sethome 
 * @brief 
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Global_status.h"

struct GlobalStatus_t Global;

// �����������ݲ�������
void change_cap_status(enum cap_e in)
{
    Global.cap = in;
}

enum cap_e read_cap_status()
{
    return Global.cap;
}

void Global_set_err(enum err_e err, uint8_t status)
{
    Global.err[err] = status;
}

//end of file
