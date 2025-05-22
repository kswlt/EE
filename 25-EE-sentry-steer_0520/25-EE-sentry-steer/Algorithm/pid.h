typedef struct//PID�������ݽṹ��
{
  float p,i,d;//���������֣�΢��
  float set,err,err_last,get;//����ֵ�����ֵ���ϴε����ֵ
  float diff,lastdiff,filter;//΢��ֵ���ϴ�΢��ֵ���˲���������֪����û���ã�
  float p_out,i_out,d_out,total_out;//���������֣�΢�ֵ�����������
  float lim_i_out,lim_out;//����������ƣ����������
  
  float Kp, Ki, Kd;      // PID���� 
  float Kf;              // ǰ�����棨������
  float prev_setpoint;   // �ϴ��趨ֵ��������
}pid_t;



extern pid_t pitch_speed_pid;
extern pid_t pitch_location_pid;

extern pid_t yaw_speed_pid;
extern pid_t yaw_location_pid;

extern pid_t trigger_speed_pid;
extern pid_t trigger_location_pid;

extern pid_t shoot1_speed_pid;
extern pid_t shoot2_speed_pid;



void pid_set(pid_t *PidSet,float p_set,float i_set,float d_set,float lim_out_set,float lim_i_outset);//PID����
float pid_cal(pid_t *PidGoal,float Now,float Set);//PID����

float calculate_feedforward(pid_t* pid, float setpoint);
float pid_update(pid_t* pid, float setpoint, float measurement);