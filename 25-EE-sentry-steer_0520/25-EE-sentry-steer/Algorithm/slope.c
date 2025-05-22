#include "slope.h"



// �����ֵ�ĺ���
float Math_Abs(float x)
{
    return ((x > 0) ? x : -x);
}




/**
 * @brief ��ʼ��б�º���
 *
 * @param slope б�º����ṹ��ָ��
 * @param Increase_Value ����ֵ����
 * @param Decrease_Value ����ֵ����
 * @param Slope_First �滮��������
 */
void Slope_set(Slope *slope, float Increase_Value, float Decrease_Value, Enum_Slope_First Slope_First) {
    slope->Out = 0.0f;
    slope->Slope_First = Slope_First;
    slope->Now_Planning = 0.0f;
    slope->Now_Real = 0.0f;
    slope->Increase_Value = Increase_Value;
    slope->Decrease_Value = Decrease_Value;
    slope->Target = 0.0f;
}


/**
 * @brief б�º�������ֵ, ��������ȡ���ڵ�����
 *
 * @param slope б�º����ṹ��ָ��
 */
float Slope_Cal(Slope *slope,float Now_Real,float Target) {
		slope->Now_Real = Now_Real;
	  slope->Target = Target;
    // �滮Ϊ��ǰ��ʵֵ���ȵĶ����߼�
    if (slope->Slope_First == Slope_First_REAL) {
        if ((slope->Target >= slope->Now_Real && slope->Now_Real >= slope->Now_Planning) ||
            (slope->Target <= slope->Now_Real && slope->Now_Real <= slope->Now_Planning)) {
            slope->Out = slope->Now_Real;
        }
    }

		if(slope->Slope_First == Slope_First_TARGET)
		{
			if (slope->Now_Planning > 0.0f)
				{
        if (slope->Target > slope->Now_Planning) 
					{
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value)
							{
                slope->Out += slope->Increase_Value;
							} 
						else
							{
								slope->Out = slope->Target;
							}
					}
				else if (slope->Target < slope->Now_Planning) 
					{
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) 
							{
                slope->Out -= slope->Decrease_Value;
              } 
						else 
							{
                slope->Out = slope->Target;
              }
          }
			  } 
		  else if (slope->Now_Planning < 0.0f) 
				{
        if (slope->Target < slope->Now_Planning)
					{
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) 
							{
                slope->Out -= slope->Increase_Value;
              } 
						else 
							{
                slope->Out = slope->Target;
              }
          } 
				else if (slope->Target > slope->Now_Planning)
					{
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) 
							{
                slope->Out += slope->Decrease_Value;
              } 
						else 
							{
                slope->Out = slope->Target;
              }
          }
       } 
			else 
				{
        if (slope->Target > slope->Now_Planning)
					{
            // 0ֵ������
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value)
							{
                slope->Out += slope->Increase_Value;
              } 
						else 
							{
                slope->Out = slope->Target;
              }
          }
				else if (slope->Target < slope->Now_Planning)
					{
            // 0ֵ������
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) 
							{
                slope->Out -= slope->Increase_Value;
              }
						else 
							{
                slope->Out = slope->Target;
              }
           }
        }
		}
    
    // �ƺ���
    slope->Now_Planning = slope->Out;
		return slope->Now_Planning;
} 
