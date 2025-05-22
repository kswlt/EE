
/**
 * @file chassis_move.c
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

 #include "stdlib.h"
 #include "stdio.h"
 #include "stdint.h"
 
 #include "CAN_receive&send.h"
 #include "IMU_updata.h"
 #include "cap_ctl.h"
 
 #include "chassis_move.h"
 #include "gimbal.h"
 #include "Global_status.h"
 
 #include "Stm32_time.h"
 
 #include "math.h"
 #include "RampFunc.h" // ����б�º�����
 #include "slope.h"
 // wheel conf
 #define WHEEL_RADIUS 0.26282f // m
 #define Rotation_radius  0.37169f  //m
 #define PI 3.1415926f
 // car conf
 #define ROLLER_DISTANCE 100 // mm  ���
 #define WHEELS_DISTANCE 100 // mm  �־�
 // mm/s
 #define FR 0
 #define FL 1
 #define BL 2
 #define BR 3
 //��㶨��
 #define TURN_FR_ANGLE 7560   //7560
 #define TURN_FL_ANGLE 4097   //
 #define TURN_BL_ANGLE 5435   //122.18
 #define TURN_BR_ANGLE 6181		//6181
 //�ж����ڼ��
 float judgement = 1 ;
 float encoder_difference;
 float shortest_encoder;
 float chassis_power_limit;
 
 //����ת��ʱ�õ��ĽǶ�
 static fp32 deta[4]={45.0f,45.0f,45.0f,45.0f};
 //������ǰ�����
 fp32 X_AXIS_ECD[4]= {7560,4097,5435,6181};  //7560,4009,2766,6181
 //��ֹʱ�򱣻������������
 static fp32 still[4] = {7560,4097,5435,6181};
 //����ĽǶ�
 fp32 set_angle[4] = {0,0,0,0};
 //�Ƕ�ת���ɵı�����ֵ
 fp32 set_mangle[4] = {0,0,0,0};
 fp32 last_set_mangle[4] = {0,0,0,0};
 //��һ�εĽǶ�
 fp32 last_angle[4] = {0,0,0,0};
 //��־λ
 int fllg[4] = {0,0,0,0};
 static int flag_course=1;
 //��ʱʱ�����õļ���
 int time = 0;
 //���ת������
 float dirt[4]={-1.0f,1.0f,1.0f,-1.0f};
 
 struct chassis_status chassis;
 struct cap cap_chassis; // ������
 
 float wheel_rpm[4]; // �����ٶ�����
 /*�����������*/
 float now_p = 0.0f;
 float b = 0.015f;
 float Plimit = 1.0f; // �ȱ�ϵ��
 uint16_t Engerny_buffer;
 /*��������ٶ�*/
 float chassis_angle;
 int32_t max_curr = 0;
 float curr_a = 0.0f;
 /*����С������ر���*/
 int refresh_interval = 30; // ���´���������ʮ�θ���һ��
 int smaller_than_2_count = 0;
 float valve = 0.0f;
 /*����ģʽ*/
 extern float spin;
 float spin_forward_sen = -4.0f;
 /*ͣ������*/
 extern float slope;
 
 // �н�����ٶȻ�PID
 pid_t motor_speed_3508[4];
 // ת����˫��PID
 pid_t motor_location_6020[4];
 pid_t motor_speed_6020[4];
 //--Ŀ���ٶ�
 float target_velocity[4]={0,0,0,0};
 // ���̸���PID
 pid_t chassis_follow;
 //б�¹滮���滮�ٶ�
 float Planning_velocity[4]={0,0,0,0};
 
 //б�¹滮��
 Slope chassis_slope[4];
 
 float tmp_delta_angle[4];
 float test_angle[4];
 
 // ��ʼ������
 void chassis_move_init()
 {
	 chassis.speed.max_x = 8.0f; // m/s
	 chassis.speed.max_y = 8.0f; // m/s
	 chassis.speed.max_r = 5.0f; //
 
	 chassis.acc.max_x = 2.5f; // 1m/^2
	 chassis.acc.max_y = 2.5f; // m/^2
	 chassis.acc.max_r = 2.5f; //
 
	 /***********************************�н�������ٶȻ�����*******************************************/
	 pid_set(&motor_speed_3508[FR], 8000.0f, 0, 250, MAX_CURRENT, 0);// 8000.0f, 0, 250, MAX_CURRENT, 0
	 pid_set(&motor_speed_3508[FL], 8000.0f, 0, 250, MAX_CURRENT, 0);
	 pid_set(&motor_speed_3508[BL], 8000.0f, 0, 250, MAX_CURRENT, 0);
	 pid_set(&motor_speed_3508[BR], 8000.0f, 0, 250, MAX_CURRENT, 0);
	 /***********************************ת����˫������************************************************/
	 pid_set(&motor_location_6020[FR], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
	 pid_set(&motor_location_6020[FL], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
	 pid_set(&motor_location_6020[BL], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
	 pid_set(&motor_location_6020[BR], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
 
	 pid_set(&motor_speed_6020[FR], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);//55.0f, 0.0f, 0.0f, 10000.0f, 0.0f
	 pid_set(&motor_speed_6020[FL], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
	 pid_set(&motor_speed_6020[BL], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
	 pid_set(&motor_speed_6020[BR], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
	 
	 Slope_set(&chassis_slope[0],0.1f,0.2f,1);
	 Slope_set(&chassis_slope[1],0.1f,0.2f,1);
	 Slope_set(&chassis_slope[2],0.1f,0.2f,1);
	 Slope_set(&chassis_slope[3],0.1f,0.2f,1);
 
	 chassis.turn_FR.now = 0;
	 chassis.turn_FR.set = TURN_FR_ANGLE;
	 chassis.turn_FR.offset = 0;
	 chassis.turn_FR.stable = 0;
	 chassis.turn_FR.set_turn_FR_speed = 0;
	 
	 chassis.turn_FL.now = 0;
	 chassis.turn_FL.set = TURN_FL_ANGLE;
	 chassis.turn_FL.offset = 0;
	 chassis.turn_FL.stable = 0;
	 chassis.turn_FL.set_turn_FL_speed = 0;
	 
	 chassis.turn_BL.now = 0;
	 chassis.turn_BL.set = TURN_BL_ANGLE;
	 chassis.turn_BL.offset = 0;
	 chassis.turn_BL.stable = 0;
	 chassis.turn_BL.set_turn_BL_speed = 0;
	 
	 chassis.turn_BR.now = 0;
	 chassis.turn_BR.set = TURN_BR_ANGLE;
	 chassis.turn_BR.offset = 0;
	 chassis.turn_BR.stable = 0;
	 chassis.turn_BR.set_turn_BR_speed = 0;
	 
	 //��ת�ٶȷֽ�ĽǶ�deta
	   for(int i=0;i<4;i++)
     {
       deta[i]=deta[i]*PI/180.0f;//�Ƕ�ת����
     }
			  
	 // srand(2); // ��ʼ��һ����������ӣ�Ϊ��֮�����С����ʹ��
 }
 //����ǶȽ���
 void Chassic_course_solving(float x,float y,float w)
 {
	 static fp32 set_angle_last[4]={TURN_FR_ANGLE,TURN_FL_ANGLE,TURN_BL_ANGLE,TURN_BR_ANGLE};
	 //���ٶ�
	  w=w*Rotation_radius;
	  int16_t angle_temp[4];
 
	 //��ת�˶�
	 if(x==0&&y==0&&w==0)
	 {
				 judgement = 1 ;

         set_angle[0] = 0.0f;
				 set_angle[1] = 0.0f;
				 set_angle[2] = 0.0f;
				 set_angle[3] = 0.0f;
 
	 }
//	 set_angle[0] = 45.0f;
//				 set_angle[1] = -45.0f;
//				 set_angle[2] = 45.0f;
//				 set_angle[3] = -45.0f;
	 else
	 {
	 /********************************use*********************************************************/
		 set_angle[0] =	 atan2((y+w*0.707107f),(x+w*0.707107f))*180.0f/PI;
     set_angle[1] =	 atan2((y+w*0.707107f),(x-w*0.707107f))*180.0f/PI;
     set_angle[2] =	 atan2((y-w*0.707107f),(x-w*0.707107f))*180.0f/PI;
     set_angle[3] =   atan2((y-w*0.707107f),(x+w*0.707107f))*180.0f/PI;
	 
	 }
		 /**********************use*********************/
   
		 set_mangle[0] =	(X_AXIS_ECD[0]+set_angle[0]*8192.0/360.0)/8192*360;
		 set_mangle[1] =	(X_AXIS_ECD[1]+set_angle[1]*8192.0/360.0)/8192*360;
		 set_mangle[2] =	(X_AXIS_ECD[2]+set_angle[2]*8192.0/360.0)/8192*360;
		 set_mangle[3] =	(X_AXIS_ECD[3]+set_angle[3]*8192.0/360.0)/8192*360;
   
     last_angle[0] = set_angle[0];
		 last_angle[1] = set_angle[1];
		 last_angle[2] = set_angle[2];
		 last_angle[3] = set_angle[3];
		 /*******************************************/	
 
   //�Ƕȸ�ֵ
   //--���������õ���Ҫ�ĽǶȶ�Ӧ�ڱ������ϵ�λ��
	 chassis.turn_FR.set = (int32_t)set_mangle[0];
	 chassis.turn_FL.set = (int32_t)set_mangle[1];
	 chassis.turn_BL.set = (int32_t)set_mangle[2];
	 chassis.turn_BR.set = (int32_t)set_mangle[3];
 
	 set_angle_last[0]=set_mangle[0];
	 set_angle_last[1]=set_mangle[1];
	 set_angle_last[2]=set_mangle[2];
	 set_angle_last[3]=set_mangle[3];
 }	

 //���������ݸ���
 void Chassis_course_updata()
 {
 //--��ԭʼ���ݼ����һЩ����
	 decode_as_6020(chassis_turn_FR);
	 decode_as_6020(chassis_turn_FL);
	 decode_as_6020(chassis_turn_BL);
	 decode_as_6020(chassis_turn_BR);
 //--��������ֵ
   chassis.turn_FR.now = get_motor_data(chassis_turn_FR).angle;
   chassis.turn_FL.now = get_motor_data(chassis_turn_FL).angle;
   chassis.turn_BL.now = get_motor_data(chassis_turn_BL).angle;
   chassis.turn_BR.now = get_motor_data(chassis_turn_BR).angle;
 
 }
 //����PID����
 void Chassis_course_pid_cal()
 {
   //�ͽ�תλ
   steer_transfer_nearby();
 
   
   /******************************************************************/
   //back_up
   //FR
	 chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
   //FL
	 chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
	 //BL
		 chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
	 //BR
	 chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
   /******************************************************************/
 //--�����µ������
	 set_motor(pid_cal(&motor_speed_6020[FR], get_motor_data(chassis_turn_FR).speed_rpm, chassis.turn_FR.turn_FR_speed), chassis_turn_FR);
	 set_motor(pid_cal(&motor_speed_6020[FL], get_motor_data(chassis_turn_FL).speed_rpm, chassis.turn_FL.turn_FL_speed), chassis_turn_FL);
	 set_motor(pid_cal(&motor_speed_6020[BL], get_motor_data(chassis_turn_BL).speed_rpm, chassis.turn_BL.turn_BL_speed), chassis_turn_BL);
	 set_motor(pid_cal(&motor_speed_6020[BR], get_motor_data(chassis_turn_BR).speed_rpm, chassis.turn_BR.turn_BR_speed), chassis_turn_BR);
// 	set_motor(0, chassis_turn_FR);
// 	set_motor(0, chassis_turn_FL);
// 	set_motor(0, chassis_turn_BL);
// 	set_motor(0, chassis_turn_BR);
 }
 //�����н�����ٶȼ�����
 void Chassis_velocity_calc(float vx, float vy, float vw)
 {
   //
   //������ٶ�
	 vw=vw*Rotation_radius;
	 float  V[4]={0,0,0,0};
	 //�ֱ�����ĸ����ӵ��ٶȴ�С
	 V[3]=sqrt((vx+vw*sinf(deta[3]))*(vx+vw*sinf(deta[3]))+(vy-vw*cosf(deta[3]))*(vy-vw*cosf(deta[3])));
	 V[2]=sqrt((vx-vw*sinf(deta[2]))*(vx-vw*sinf(deta[2]))+(vy-vw*cosf(deta[2]))*(vy-vw*cosf(deta[2])));
	 V[1]=sqrt((-vx-vw*sinf(deta[1]))*(-vx-vw*sinf(deta[1]))+(-vy+vw*cosf(deta[1]))*(-vy+vw*cosf(deta[1])));
	 V[0]=sqrt((vx+vw*sinf(deta[0]))*(vx+vw*sinf(deta[0]))+(vy+vw*cosf(deta[0]))*(vy+vw*cosf(deta[0])));
 
	 //����ٶ�����
	 val_limit(&vx, chassis.speed.max_x);
	 val_limit(&vy, chassis.speed.max_y);
	 val_limit(&vw, chassis.speed.max_r);
 //	// ������ٶ�
 
 //	}
	//--������ݸ���
	 decode_as_3508(chassis_move_FR);
	 decode_as_3508(chassis_move_FL);
	 decode_as_3508(chassis_move_BL);
	 decode_as_3508(chassis_move_BR);
	//--ȷ��Ŀ���ٶȵ�����
	target_velocity[FR] =     V[0]*dirt[0];
	target_velocity[FL] =     V[1]*dirt[2];
	target_velocity[BL] =     V[2]*dirt[1];
	target_velocity[BR] =     V[3]*dirt[3];
	
 //ֱ�ӹ滮
	Planning_velocity[FR] = Slope_Cal(&chassis_slope[0],get_motor_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI,target_velocity[FR]);
	Planning_velocity[FL] = Slope_Cal(&chassis_slope[1],get_motor_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI,target_velocity[FL]);
	Planning_velocity[BL] = Slope_Cal(&chassis_slope[2],get_motor_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI,target_velocity[BL]);
	Planning_velocity[BR] = Slope_Cal(&chassis_slope[3],get_motor_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI,target_velocity[BR]);

 
	 //����������
	 chassis.wheel_current[FR] = pid_cal(&motor_speed_3508[FR], get_motor_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI, Planning_velocity[FR]);//Planning_velocity[FR]
	 chassis.wheel_current[FL] = pid_cal(&motor_speed_3508[FL], get_motor_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI, Planning_velocity[FL]);//Planning_velocity[FL]
	 chassis.wheel_current[BL] = pid_cal(&motor_speed_3508[BL], get_motor_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI, Planning_velocity[BL]);//Planning_velocity[BL]
	 chassis.wheel_current[BR] = pid_cal(&motor_speed_3508[BR], get_motor_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI, Planning_velocity[BR]);//Planning_velocity[BR]
 
	/* �й��ʿ���	3508 */ 
   Plimit = power_limit(chassis.wheel_current);
   /* �޹��ʿ��� */
   // Plimit = 1.0f;

   //���������� ��Ӧ��freeRTOS���ͣ�
  	 set_motor((Plimit*chassis.wheel_current[FR]), chassis_move_FR);
  	 set_motor((Plimit*chassis.wheel_current[FL]), chassis_move_FL);
	 set_motor((Plimit*chassis.wheel_current[BL]), chassis_move_BL);
	 set_motor((Plimit*chassis.wheel_current[BR]), chassis_move_BR);
   
	//  set_motor(chassis.wheel_current[FR], chassis_move_FR);
	//  set_motor(chassis.wheel_current[FL], chassis_move_FL);
	//  set_motor(chassis.wheel_current[BL], chassis_move_BL);
	//  set_motor(chassis.wheel_current[BR], chassis_move_BR);
// 		set_motor(0, chassis_move_FR);
// 		set_motor(0, chassis_move_FL);
// 		set_motor(0, chassis_move_BL);
// 		set_motor(0, chassis_move_BR);
 }
 /*******************************�ͽ�תλ***********************************/
 void steer_transfer_nearby()
 {

		 tmp_delta_angle[0] = obtain_modulus_normalization(chassis.turn_FR.set - chassis.turn_FR.now, 360.0f);
		 tmp_delta_angle[1] = obtain_modulus_normalization(chassis.turn_FL.set - chassis.turn_FL.now, 360.0f);
		 tmp_delta_angle[2] = obtain_modulus_normalization(chassis.turn_BL.set - chassis.turn_BL.now, 360.0f);
		 tmp_delta_angle[3] = obtain_modulus_normalization(chassis.turn_BR.set - chassis.turn_BR.now, 360.0f);
 
		 // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ FR
		 if (-90.0f <= tmp_delta_angle[0] && tmp_delta_angle[0] <= 90.0f)
		 {
			 // ��PI / 2֮�����跴��ͽ�תλ
			 chassis.turn_FR.set = tmp_delta_angle[0] + chassis.turn_FR.now;
					   dirt[0] = -1.0f;
		 }
		 else
		 {
			 // ��Ҫ��ת��Ȧ���
			 chassis.turn_FR.set = obtain_modulus_normalization(tmp_delta_angle[0] + 180.0f, 360.0f) + chassis.turn_FR.now;
			 dirt[0] = 1.0f;
		 }
		 // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ FL
		 if (-90.0f <= tmp_delta_angle[1] && tmp_delta_angle[1] <= 90.0f)
		 {
			 // ��PI / 2֮�����跴��ͽ�תλ
			 chassis.turn_FL.set = tmp_delta_angle[1] + chassis.turn_FL.now;
					   dirt[1] = 1.0f;
		 }
		 else
		 {
			 // ��Ҫ��ת��Ȧ���
			 chassis.turn_FL.set = obtain_modulus_normalization(tmp_delta_angle[1] + 180.0f, 360.0f) + chassis.turn_FL.now;
			 dirt[1] = -1.0f;
		 }
		 // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ BL
		 if (-90.0f <= tmp_delta_angle[2] && tmp_delta_angle[2] <= 90.0f)
		 {
			 // ��PI / 2֮�����跴��ͽ�תλ
			 chassis.turn_BL.set = tmp_delta_angle[2] + chassis.turn_BL.now;
			 dirt[2] = 1.0f;
		 }
		 else
		 {
			 // ��Ҫ��ת��Ȧ���
			 chassis.turn_BL.set = obtain_modulus_normalization(tmp_delta_angle[2] + 180.0f, 360.0f) + chassis.turn_BL.now;
			 dirt[2] = -1.0f;
		 }
		 // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ BR
		 if (-90.0f <= tmp_delta_angle[3] && tmp_delta_angle[3] <= 90.0f)
		 {
			 // ��PI / 2֮�����跴��ͽ�תλ
			 chassis.turn_BR.set = tmp_delta_angle[3] + chassis.turn_BR.now;
						 dirt[3] = -1.0f;
 
		 }
		 else
		 {
			 // ��Ҫ��ת��Ȧ���
			 chassis.turn_BR.set = obtain_modulus_normalization(tmp_delta_angle[3] + 180.0f, 360.0f) + chassis.turn_BR.now;
			 dirt[3] = 1.0f;
		 }
	 
 }
 /*******************************�ͽ�תλ***********************************/
 
 // �����������
 float power_limit(int32_t current[4])
 {
 
	 float max_p; // = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w����
 
	 if (cap.remain_vol <= 6)
		 max_p = chassis_power_limit - 2.0f; // 2w����
	 else if (cap.remain_vol > 6)
	 {
		 if (Global.input.fly_status == 1)
			 max_p = chassis_power_limit + cap.remain_vol * 15; // ��������� = �����ѹ * 14A ��Ȧ������
		 if (Global.input.fly_status != 1)
			 max_p = chassis_power_limit + cap.remain_vol * 14;
	 }
 
	 now_p = 0;
 
	 const float a = 1.23e-07;	// k1
	 const float k2 = 1.453e-07; // k2
	 const float constant = 4.081f;
	 // ���ٱȸ�Ϊ14�ø�ϵ�������ļ��ٱ�Ϊ19Ϊע�ͺ�ϵ��
//	 const float toque_coefficient = (20.0f / 16384.0f) * (0.22f) * (187.0f / 3591.0f) / 9.55f; // (20/16384)*(0.3)*(187/3591)/9.55=1.99688994e-6f P19
   const float toque_coefficient = (20.0f / 16384.0f) * (0.3f) * (187.0f / 3591.0f) / 9.55f; 
	 for (int i = 0; i < 4; i++)
	 {
		 // ���㹦��
		 // ���������֣�https://github.com/MaxwellDemonLin/Motor-modeling-and-power-control/blob/master/chassis_power_control.c#L89
 
		 // ��ǰ���� = ������ * ��������ת��ϵ�� * �ٶ� + K2 * �ٶȵ�ƽ�� + a * ������ƽ���� / Ч��
		 // K2 �������ٶ����ޣ����ٳ���������K2
		 // a ����������ʱ�����������𲽳���������a
		 // Ч��һ�㲻�ö���ԼΪ0.6-0.85
		 now_p += fabs(current[i] * toque_coefficient * get_motor_data(i).speed_rpm +
					   k2 * get_motor_data(i).speed_rpm * get_motor_data(i).speed_rpm +
					   a * current[i] * current[i] + constant) /
				  0.85f;
	 }
 
	 // ����Ԥ�⹦�ģ���������ı���
	 float percentage = max_p / now_p;
 
	 if (percentage > 1.0f)
		 return 1.0f;
	 return percentage - b; // b ��һ����ѧ�Ĳ��������ڲ�������ٶ�ʱ���ʳ��������
 }
 
	 void chassis_receive_from_gimbal_1(uint8_t data[8])
 {
 
		 float speed_x = bytes_to_float(&data[0]);  
	 float speed_y = bytes_to_float(&data[4]);  
		 chassis.speed_RC.x = -speed_x;
		 chassis.speed_RC.y = speed_y;
 }
 void chassis_receive_from_gimbal_2(uint8_t data[8])
 {
 
		 float speed_w = bytes_to_float(&data[0]);  
		 float speed_yaw = bytes_to_float(&data[4]);  
		 chassis.speed_RC.r = speed_w;
//		 chassis.speed_RC.big_yaw = speed_yaw*PI/180.f;		//��λ rad
   chassis.speed_RC.big_yaw = speed_yaw;		//��λ rad
 }
 void receive_REFEREE_DATA(uint8_t data[8])
 {
   chassis_power_limit = bytes_to_float(&data[0]); 
 }
 
 // ����ֵ
 inline void val_limit(float *val, float MAX)
 {
	 if (fabs(*val) > MAX)
	 {
		 if (*val > 0)
			 *val = MAX;
		 else
			 *val = -MAX;
	 }
 }
 // ���Ʊ仯��
 inline void change_limit(float last, float *now, float limit)
 {
	 float change = *now - last;
	 if (fabs(change) > limit)
	 {
		 if (change > 0)
			 *now = last + limit;
		 else
			 *now = last - limit;
	 }
 }
 
 //����С����
 //��ȡģ�黯  ת���Ƕȿ�����-PI----PI
 float obtain_modulus_normalization(float x,float modulus)
 {
   float tmp;
   tmp = fmod(x + modulus / 2.0f, modulus);
   if (tmp < 0.0f)
   {
	   tmp += modulus;
   }
   return (tmp - modulus / 2.0f);
 }

 //float Angle_Limit(float angle, float max)
 //{
 //    angle = fmodf(angle, max);
 //    // �����Ƕ�
 //    if(angle < 0)
 //        angle += max;
 //    
 //    return angle;
 //}
 // ���ֽ�����ת��Ϊfloat
 float bytes_to_float(uint8_t *bytes) 
	 {
	 uint32_t temp = 0;
	 temp |= (bytes[0] << 0);  // ����ֽ�
	 temp |= (bytes[1] << 8);  // �ڶ����ֽ�
	 temp |= (bytes[2] << 16); // �������ֽ�
	 temp |= (bytes[3] << 24); // ����ֽ�
	 return *(float *)&temp;   // ��uint32_tָ��ǿ��ת��Ϊfloatָ��
	 }
	 	 
 /*********************���º���û�б��õ��������Է��Ժ��õ�***********************/
 // ����С���� ������
 float random_anti_vision_r_s(float min, float max)
 {
	 static int run_count = 100;
	 run_count++;
	 if (run_count > refresh_interval && smaller_than_2_count < 2)
	 {
		 valve = generate_random_float(min, max); // ���޺�����
 
		 run_count = 0;
		 if (valve < 2.0f)
		 {
			 smaller_than_2_count++;
		 }
		 return valve;
	 }
	 else if (run_count > refresh_interval && smaller_than_2_count >= 2)
	 {
		 smaller_than_2_count = 0;
		 return 3.5f;
	 }
	 else
		 return valve;
 }
 
 float generate_random_float(float min, float max)
 {
	 return min + ((float)rand() / RAND_MAX) * (max - min);
 }
 
 // ��ָ������ʽȥ���������Ǹ������������acc=ae^bv-a+c,��aΪ����cΪ��ʼ���ٶ�,�����ٶ�Ϊ�Ա���
 float fly_speed_up_ex(float a, float b, float c, float v)
 {
	 return a * exp(b * v) - a + c;
 }
 // �տ�ʼʱĬ�ϼ��ٶȣ�Ȼ�����Խ��Խ��
 // ����ֵ��ÿ�����м��ٶ�����ֵ���ٶ�����ֵ
 float fly_speed_up(float *set, float acceleration_increase_rate, float limit)
 {
	 static float initial_acceleration = 0.2; // ��ʼ���ٶ���Ϊ0.2//ÿ������+���ٶ�;
	 if (set == 0)							 // Ҳ���Ǵ�ͷ��ʼ�ˣ��Ǿ����ü��ٶ�
	 {
		 initial_acceleration = 0.2;
	 }
	 if (fabs(*set) < limit)
	 {
		 *set += initial_acceleration;
		 initial_acceleration += acceleration_increase_rate;
	 }
	 else
	 {
		 *set = limit;
	 }
 }
 
 int RampInc_float(int16_t *buffer, float now, float ramp)
 {
	 if (*buffer > 0)
	 {
		 if (*buffer > ramp)
		 {
			 now += ramp;
			 *buffer -= ramp;
		 }
		 else
		 {
			 now += *buffer;
			 *buffer = 0;
		 }
	 }
	 else
	 {
		 if (*buffer < -ramp)
		 {
			 now += -ramp;
			 *buffer -= -ramp;
		 }
		 else
		 {
			 now += *buffer;
			 *buffer = 0;
		 }
	 }
 
	 return now;
 }
  