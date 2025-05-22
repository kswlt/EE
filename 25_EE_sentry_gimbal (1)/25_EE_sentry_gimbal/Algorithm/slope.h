/**
 * @file alg_slope.h
 * @author yssickjgd (1345578933@qq.com)
 * @author davi
 * @brief б�º���, �����ٶȹ滮��
 * @date 2025.5.5
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef __SLOPE_H
#define __SLOPE_H

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief �滮��������, ��ΪĿ��ֵ���Ⱥ���ʵֵ����
 * Ŀ��ֵ����, ��Ӳ�滮
 * ��ʵֵ����, ����ǰ��ʵֵ���ڵ�ǰ�滮ֵ��Ŀ��ֵ֮��, ��ǰ�滮ֵתΪ��ǰ��ʵֵ
 *
 */
typedef enum {
    Slope_First_REAL = 0,
    Slope_First_TARGET
} Enum_Slope_First;

/**
 * @brief Reusable, б�º�������
 *
 */
typedef struct {
    // ���ֵ
    float Out;

    // �滮��������
    Enum_Slope_First Slope_First;
    // ��ǰ�滮ֵ
    float Now_Planning;
    // ��ǰ��ʵֵ
    float Now_Real;

    // ����ֵ����, һ�μ������ڸı�ֵ
    float Increase_Value;
    // ����ֵ����, һ�μ������ڸı�ֵ
    float Decrease_Value;
    // Ŀ��ֵ
    float Target;
} Slope;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief ��ʼ��б�º���
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Increase_Value ����ֵ����
 * @param __Decrease_Value ����ֵ����
 * @param __Slope_First �滮��������
 */
void Slope_set(Slope *slope, float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First);

/**
 * @brief ��ȡ���ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @return ���ֵ
 */
float Slope_Get_Out(Slope *slope);

/**
 * @brief �趨��ǰ�滮ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Now_Real ��ǰ�滮ֵ
 */
void Slope_Set_Now_Real(Slope *slope, float __Now_Real);

/**
 * @brief �趨����ֵ����, һ�μ������ڸı�ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Increase_Value ����ֵ����, һ�μ������ڸı�ֵ
 */
void Slope_Set_Increase_Value(Slope *slope, float __Increase_Value);

/**
 * @brief �趨����ֵ����, һ�μ������ڸı�ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Decrease_Value ����ֵ����, һ�μ������ڸı�ֵ
 */
void Slope_Set_Decrease_Value(Slope *slope, float __Decrease_Value);

/**
 * @brief �趨Ŀ��ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param __Target Ŀ��ֵ
 */
void Slope_Set_Target(Slope *slope, float __Target);

/**
 * @brief �������ڻص�����
 *
 * @param slope б�º����ṹ��ָ��
 */
void Slope_Calculate_PeriodElapsedCallback(Slope *slope);


float Math_Abs(float x);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/