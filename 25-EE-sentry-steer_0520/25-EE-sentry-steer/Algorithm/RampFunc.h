#include "stdint.h"
#include "struct_typedef.h"
#define _RAMPFUNC_H_
#ifdef  _RAMPFUNC_H_


#pragma anon_unions

// ����һ�����ڱ�ʾб�·�����״̬�Ľṹ��
 typedef struct RampGenerator
{
    float currentValue; // ��ǰֵ
    float targetValue;  // Ŀ��ֵ
    float step;         // ÿ����������Ӧ���ı����ֵ��С
    int  isBusy;        // ָʾб�·������Ƿ����ڵ�����
} RampGenerator;

// һ�������ڶ�б�·�����״̬�ĸ���
void rampIterate(RampGenerator *ramp);
// ��ʼ��б�·�����
void rampCal(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime);

#endif

// end of file
