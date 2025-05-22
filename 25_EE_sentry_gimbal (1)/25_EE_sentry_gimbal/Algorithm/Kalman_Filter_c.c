/*
 * @Author: baoshan daibaoshan2018@163.com
 * @Date: 2024-12-15 21:20:13
 * @LastEditors: baoshan daibaoshan2018@163.com
 * @LastEditTime: 2024-12-15 21:26:39
 * @FilePath: /25_EE_omni_sentry/Algorithm/Kalman_Filter_c.c
 * @Description: 
 */
#include "Kalman_Filter_c.h"
#include "main.h"
#include "USB_VirCom.h"
KF_t yaw_auto_kf;
KF_t pitch_auto_kf;
/*键鼠*/
KF_t mouse_x_kf_fliter;
KF_t mouse_y_kf_fliter;



/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none,这两个给多大无所谓，只看R/Q的比例来决定预测的参数，建议某一个值给1去调试另外一个。
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)     状态方程
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q            观测方程
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)   更新卡尔曼增益
    p->X_now = p->X_mid + p->kg*(dat-p->X_mid);   //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))  修正估计值
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)           更新后验估计协方差
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}
void Kalman_Init(void)
{
    //自瞄
    KalmanCreate(&yaw_auto_kf.Angle_KF , 1,1);
    KalmanCreate(&pitch_auto_kf.Angle_KF , 1,1);
//	KalmanCreate(&mouse_x_kf_fliter.Angle_KF, 1,10);//键鼠卡尔曼滤波
//	KalmanCreate(&mouse_y_kf_fliter.Angle_KF, 1,10);	
}
float AutoAim_Algorithm(KF_t *str,float input)//yaw
{
		float res;
		str->Angle =KalmanFilter(&str->Angle_KF,input);
		str->Out = str->Angle;
		res = str->Out;//现在只对角度进行预测
	return res;
}
//float AutoAim_pitch_Algorithm(KF_t *str)//pitch
//{
//	float res;
//	str->Angle =fromNUC.pitch;
//	 /*获取2项输出*/
//	str->Out = str->Angle;
//	res = str->Out;
//	return res;
//}
