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
 * @param __Increase_Value ����ֵ����
 * @param __Decrease_Value ����ֵ����
 * @param __Slope_First �滮��������
 */
void Slope_set(Slope *slope, float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First) {
    slope->Out = 0.0f;
    slope->Slope_First = __Slope_First;
    slope->Now_Planning = 0.0f;
    slope->Now_Real = 0.0f;
    slope->Increase_Value = __Increase_Value;
    slope->Decrease_Value = __Decrease_Value;
    slope->Target = 0.0f;
}

/**
 * @brief ��ȡ���ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @return ���ֵ
 */
float Slope_Get_Out(Slope *slope) {
    return slope->Out;
}

/**
 * @brief �趨��ǰ�滮ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Now_Real ��ǰ�滮ֵ
 */
void Slope_Set_Now_Real(Slope *slope, float __Now_Real) {
    slope->Now_Real = __Now_Real;
}

/**
 * @brief �趨����ֵ����, һ�μ������ڸı�ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Increase_Value ����ֵ����, һ�μ������ڸı�ֵ
 */
void Slope_Set_Increase_Value(Slope *slope, float __Increase_Value) {
    slope->Increase_Value = __Increase_Value;
}

/**
 * @brief �趨����ֵ����, һ�μ������ڸı�ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Decrease_Value ����ֵ����, һ�μ������ڸı�ֵ
 */
void Slope_Set_Decrease_Value(Slope *slope, float __Decrease_Value) {
    slope->Decrease_Value = __Decrease_Value;
}

/**
 * @brief �趨Ŀ��ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Target Ŀ��ֵ
 */
void Slope_Set_Target(Slope *slope, float __Target) {
    slope->Target = __Target;
}

/**
 * @brief б�º�������ֵ, ��������ȡ���ڵ�����
 *
 * @param slope б�º����ṹ��ָ��
 */
void Slope_Calculate_PeriodElapsedCallback(Slope *slope) {
    // �滮Ϊ��ǰ��ʵֵ���ȵĶ����߼�
    if (slope->Slope_First == Slope_First_REAL) {
        if ((slope->Target >= slope->Now_Real && slope->Now_Real >= slope->Now_Planning) ||
            (slope->Target <= slope->Now_Real && slope->Now_Real <= slope->Now_Planning)) {
            slope->Out = slope->Now_Real;
        }
    }

    if (slope->Now_Planning > 0.0f) {
        if (slope->Target > slope->Now_Planning) {
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out += slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target < slope->Now_Planning) {
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) {
                slope->Out -= slope->Decrease_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    } else if (slope->Now_Planning < 0.0f) {
        if (slope->Target < slope->Now_Planning) {
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out -= slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target > slope->Now_Planning) {
            // ��ֵ����
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) {
                slope->Out += slope->Decrease_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    } else {
        if (slope->Target > slope->Now_Planning) {
            // 0ֵ������
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out += slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target < slope->Now_Planning) {
            // 0ֵ������
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out -= slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    }

    // �ƺ���
    slope->Now_Planning = slope->Out;
} 
