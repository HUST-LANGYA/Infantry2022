#ifndef __PID_H
#define __PID_H

typedef struct{
		float K1;			
	  float K2;
	  float Last_DeltIn;
	  float Now_DeltIn;
	  float Out;
}FeedForward_Typedef;

/*   PID����-----ZN������
		 Td/T = Kd  T/Ti = Ki   
     PID:   Ti >= 4Td    Kp = 0.6Ku   Td = 0.125Tu
     PI:    Kp = 0.45Ku  Ti = 0.83Tu
     P:     Kp = 0.5Ku
*/


typedef struct PID{
		float SetPoint;			//�趨Ŀ��ֵ
	  float SetPointLast;
		float ActualValue;

    float DeadZone;
		float P;						//��������
		float I;						//���ֳ���
		float D;						//΢�ֳ���
		
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
		float dError;
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
		float OutMax;       //�޷�
}Pid_Typedef;

float PID_Calc(Pid_Typedef * P, float ActualValue);

/*********ģ��pid����*/
typedef struct
{
		float SetPoint;			//�趨Ŀ��ֵ
	  float ActPoint;      //ʵ��ֵ
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
		float dError;
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	
	  float Kp;       
	  float Ki;
  	float Kd;
	  
}FuzzyPID;
float FuzzyPID_Calc(FuzzyPID *pid);
float FeedForward_Calc(FeedForward_Typedef *FF);
#endif
