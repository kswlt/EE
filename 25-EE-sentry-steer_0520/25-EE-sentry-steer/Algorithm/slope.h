/**
 * @file slope.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief б�º���, �����ٶȹ滮��
 * @version 0.1
 * @date 2023-08-29 0.1 23��������
 * @date 2024-06-03 1.1 �滮�������ȼ���ʽ
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef __SLOPE_H
#define __SLOPE_H

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#include "struct_typedef.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief �滮��������, ��ΪĿ��ֵ���Ⱥ���ʵֵ����
  * ��ʵֵ����, ����ǰ��ʵֵ���ڵ�ǰ�滮ֵ��Ŀ��ֵ֮��, ��ǰ�滮ֵתΪ��ǰ��ʵֵ
 * Ŀ��ֵ����, ��Ӳ�滮
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
 * @param Increase_Value ����ֵ����
 * @param Decrease_Value ����ֵ����
 * @param Slope_First �滮��������
 */
void Slope_set(Slope *slope, float Increase_Value, float Decrease_Value, Enum_Slope_First Slope_First);

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
 * @param Now_Real ��ǰ�滮ֵ
 */
void Slope_Set_Now_Real(Slope *slope, float Now_Real);

/**
 * @brief �趨����ֵ����, һ�μ������ڸı�ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param Increase_Value ����ֵ����, һ�μ������ڸı�ֵ
 */
void Slope_Set_Increase_Value(Slope *slope, float Increase_Value);

/**
 * @brief �趨����ֵ����, һ�μ������ڸı�ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param Decrease_Value ����ֵ����, һ�μ������ڸı�ֵ
 */
void Slope_Set_Decrease_Value(Slope *slope, float Decrease_Value);

/**
 * @brief �趨Ŀ��ֵ
 *
 * @param slope б�º����ṹ��ָ��
 * @param Target Ŀ��ֵ
 */
void Slope_Set_Target(Slope *slope, float Target);

/**
 * @brief �������ڻص�����
 *
 * @param slope б�º����ṹ��ָ��
 */
float Slope_Cal(Slope *slope,float Now_Real,float Target);


float Math_Abs(float x);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/