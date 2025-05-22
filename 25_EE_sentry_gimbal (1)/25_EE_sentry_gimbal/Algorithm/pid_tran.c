#include "pid_tran.h"
#include <math.h>
void Pid_Init(PIDGroup_t* pid, float Kp, float Ki, float Kd, float IntegralLimit, float LimitOut)
{
	pid -> Kp = Kp;
	pid -> Ki = Ki;
	pid -> Kd = Kd;
	pid -> IntegralLimit = IntegralLimit;
	pid -> LimitOut = LimitOut;
	pid -> Pout = 0.0f;
	pid -> Iout = 0.0f;
	pid -> Dout = 0.0f;
	pid -> TotalOut = 0.0f;

}
void Pid_Enhance(PIDGroup_t* pid,  float DeadZone, float VariableA, float VariableB, 
	                                 float Derivative_LowPassFliter_Coef, float TotalOut_LowPassFliter_Coef)
{
	pid -> DeadZone = DeadZone;
	pid -> CombinationIntegral = 0.0f;
	pid -> VariableA = VariableA;
	pid -> VariableB = VariableB;
	pid -> Derivative_LowPassFliter_Coef = Derivative_LowPassFliter_Coef;
	pid -> TotalOut_LowPassFliter_Coef = TotalOut_LowPassFliter_Coef;
	//�쳣�����ʼ��
  pid -> ERRORHandler.ERRORCount = 0;
  pid -> ERRORHandler.ERRORType = PID_ERROR_NONE;
}

//****��������pid��ʼ��������Ҫʹ��****//

/**************************************************************************************************************/

//���λ���
static void Trapezoid_Integral(PIDGroup_t* pid)
{
	pid->CombinationIntegral = pid->Ki * ((pid->Error + pid->LastError) / 2);
	//���������ʽ��������ԡ�
}

//΢������
static void First_Derivative_On_Measurement(PIDGroup_t* pid)
{
	pid->Dout = (pid->LastMeasure - pid->Measure) * pid->Kd;
	//.ԭ��ʽΪ��
	// pid->Dout = pid->Kd * (pid->Error - pid->LastError)
	// pid->Dout =��(pid->Setpoint - pid->LastSetpoint) - (pid->Measure - pid->LastMeasure)�� * pid->Kd
	//  pid->Setpoint - pid->LastSetpoint <- ����������쳣��ֵ��ʹ��Doutͻ��

}

// ΢���˲�(�ɼ�΢��ʱ,�˳���Ƶ����)--"��ͨ�˲�"�����Ͷ����˲�����'��Ƶ�ź�ͨ����˥������ֹ��Ƶ�ź�'
static void Derivative_Filter(PIDGroup_t* pid)
{
	pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LowPassFliter_Coef + pid->dt) +
	            pid->LastDout * pid->Derivative_LowPassFliter_Coef / (pid->Derivative_LowPassFliter_Coef + pid->dt);
}

//���ٻ���
static void Changing_Integral_Rate(PIDGroup_t* pid)
{
	if (pid->Error * pid->Iout > 0)
	{
		if (abs(pid->Error) <= pid->VariableB)
			return; //��������
		if (abs(pid->Error) <= (pid->VariableA + pid->VariableB))
			//ʹ�����Ժ�������
			pid->CombinationIntegral *= (pid->VariableA - abs(pid->Error) + pid->VariableB) / pid->VariableA;
		else
			pid->CombinationIntegral = 0;//ȡ�����ֻ���
	}
}

//�����޷�
static void Integral_Limit(PIDGroup_t* pid)
{
	//Ԥ��������������
	float temp_Output, temp_Iout;
	temp_Iout = pid->Iout + pid->CombinationIntegral;
	temp_Output = pid->Pout + pid->Iout + pid->Dout;
	if (abs(temp_Output) > pid->LimitOut)
	{
		if (pid->Error * pid->Iout > 0)
		{
			//������������
			pid->CombinationIntegral = 0;
		}
	}
	//���޷�
	if (temp_Iout > pid->IntegralLimit)
	{
		pid->CombinationIntegral = 0;
		pid->Iout = pid->IntegralLimit;
	}
	if (temp_Iout < -pid->IntegralLimit)
	{
		pid->CombinationIntegral = 0;
		pid->Iout = -pid->IntegralLimit;
	}
}

// ����˲�--"��ͨ�˲�"�����Ͷ����˲�����'��Ƶ�ź�ͨ����˥������ֹ��Ƶ�ź�'
static void TotalOut_Filter(PIDGroup_t* pid)
{
	pid->TotalOut= pid->TotalOut * pid->dt / (pid->TotalOut_LowPassFliter_Coef + pid->dt) +
		pid->LastTotalOut * pid->TotalOut_LowPassFliter_Coef / (pid->TotalOut_LowPassFliter_Coef + pid->dt);
}

// ����޷�
static void TotalOut_Limit(PIDGroup_t* pid)
{
	if (pid->TotalOut > pid->LimitOut)
	{
		pid->TotalOut = pid->LimitOut;
	}
	if (pid->TotalOut < -(pid->LimitOut))
	{
		pid->TotalOut = -(pid->LimitOut);
	}
}
// �����ת���
static void PID_ErrorHandle(PIDGroup_t* pid)
{
	//�ų�PID��������С�����
	if (fabsf(pid->TotalOut) < pid->LimitOut * 0.001f || fabsf(pid->Setpoint) < 0.0001f)
		return;

	if ((fabsf(pid->Setpoint - pid->Measure) / fabsf(pid->Setpoint)) > 0.95f)
	{
		//�����ת����
		pid->ERRORHandler.ERRORCount++;
	}
	else
	{
		pid->ERRORHandler.ERRORCount = 0;
	}

	if (pid->ERRORHandler.ERRORCount > 500)
	{
		//�����������һ��ʱ�����϶�Ϊ�����ת
		pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
	}
}
float Pid_Calculate(PIDGroup_t* pid, float Measure, float Setpoint)
{
	PID_ErrorHandle(pid);
	if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            //�����ת����
            pid->TotalOut = 0;
            return 0; 
        }
	pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // ��ȡ����pid�����ʱ����,���ڻ��ֺ�΢��
	// �����ϴεĲ���ֵ�����,���㵱ǰerror
	pid->Measure = Measure;
	pid->Setpoint = Setpoint;
	pid->Error = pid->Setpoint - pid->Measure;

	// �����������,�����PID
	if (fabs(pid->Error) > pid->DeadZone)
	{
		// ������pid����,ʹ��λ��ʽ
		pid->Pout = pid->Kp * pid->Error;
		pid->CombinationIntegral = pid->Ki * pid->Error * pid->dt;
		pid->Dout = pid->Kd * (pid->Error - pid->LastError) / pid->dt;

		// ���λ���
		Trapezoid_Integral(pid);
		// ���ٻ���
		Changing_Integral_Rate(pid);
		// ΢������
		First_Derivative_On_Measurement(pid);
		// ΢���˲���
		Derivative_Filter(pid);
		// �����޷�
		Integral_Limit(pid);

		pid->Iout += pid->CombinationIntegral;             // �ۼӻ���
		pid->TotalOut = pid->Pout + pid->Iout + pid->Dout; // �������

		// ����˲�
		TotalOut_Filter(pid);

		// ����޷�
		TotalOut_Limit(pid);
	}
	else // ��������, ����ջ��ֺ����
	{
		pid->TotalOut = 0;
		pid->CombinationIntegral = 0;
	}

	// ���浱ǰ����,�����´μ���
	pid->LastMeasure = pid->Measure;
	pid->LastTotalOut = pid->TotalOut;
	pid->LastDout = pid->Dout;
	pid->LastError = pid->Error;

	return pid->TotalOut;
}
