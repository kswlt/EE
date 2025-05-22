#include "RampFunc.h"
// һ�������ڶ�б�·�����״̬�ĸ���
void rampIterate(RampGenerator *ramp)
{
	if (ramp->isBusy)
    {	
        if (ramp->currentValue < ramp->targetValue)
        {                                     // �����ǰֵС��Ŀ��ֵ
            ramp->currentValue += ramp->step; // ����ǰֵ
            if (ramp->currentValue > ramp->targetValue)
            { // ���ⳬ��
                ramp->currentValue = ramp->targetValue;
            }
        }

        else if (ramp->currentValue > ramp->targetValue)
        {   
			ramp->currentValue -= ramp->step;  //step��������ʱ
			if (ramp->currentValue < ramp->targetValue)// ���ⳬ��
				{ 
					ramp->currentValue = ramp->targetValue;
				}			
        }
        // �ж��Ƿ�ﵽĿ��
        if (ramp->currentValue == ramp->targetValue)
        {
            ramp->isBusy =0 ; // �ﵽĿ�꣬���Ϊ��æµ
        }
    }	
}
// ��ʼ��б�·�����
/**
  * @brief          б�º�������
  * @param[in]      startValue
  * @param[in]     	targetValue
  * @param[in]     	time  ÿ�ε��ӵ�ʱ��
  * @param[in]     	cycleTime   �ﵽĿ��ֵ����Ҫ��ʱ��
  * @retval         none
  */
//������������ǻ�������ķ�������4-��10�������������-4-��-10�Ǿͳ�������
void rampCal(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime)
{
	ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // ���㲽��ֵ��������Ҫע����ǣ�ȷ��б��ʱ�������ʱ�䶼��Ϊ�������������Ĵ���
    if (time != 0 && cycleTime != 0)
    {
		if(targetValue - startValue >0)
			ramp->step = (targetValue - startValue) *(cycleTime/time);
		else
			ramp->step = -(targetValue - startValue) *(cycleTime/time);//��fabs���ã�
    }
    else
    {
        ramp->step = 0; // �������������Ϊ0������Ƿ�����
    }
    ramp->isBusy = 1; // ���Ϊæµ
}
