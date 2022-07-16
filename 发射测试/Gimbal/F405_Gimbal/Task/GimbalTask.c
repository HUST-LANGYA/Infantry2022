/**********************************************************************************************************
 * @�ļ�     GimbalTask.c
 * @˵��     ��̨����
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/

int inttoshort[4];
short GimbalAct_Init_Flag=0;

short YawCurrent,PitchCurrent;
float GimbalYawPos,GimbalPitchPos;

float Recent_Angle_Buff;
float Recent_Yaw_Angle_Armor;
float Recent_Pitch_Angle_Armor;
/*-----------------------------------�ṹ��-----------------------------*/
Pid_Typedef PidPitchSpeed,PidPitchPos,PidYawSpeed,PidYawPos;
Pid_Typedef PidPitchAidPos,PidPitchAidSpeed,PidYawAidPos,PidYawAidSpeed;
FuzzyPID FuzzyPidPitchPos,FuzzyAidPidPitchPos,FuzzyAidPidYawPos;
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl; 
extern Status_t Status;
extern char PitchMotor_ReceiveFlag;
extern Gimbal_Typedef Gimbal;
extern Gyro_Typedef GyroReceive;//����������
extern RobotInit_Struct Infantry;
extern short FrictionWheel_speed;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern char Robot_ID;
extern char Budan;
extern float Buff_Yaw_Motor;
extern char q_flag;
extern FeedForward_Typedef FF_w;
/**********************************************************************************************************
*�� �� ��: Gimbal_Powerdown_Cal
*����˵��: ��̨����ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_Powerdown_Cal()
{
	if(GimbalAct_Init_Flag!=Gimbal_Powerdown_Mode)
	{
		Laser_Off();
	  GimbalAct_Init_Flag=Gimbal_Powerdown_Mode;
	}
	
	/***********************************************************************************/
	PidPitchPos.SetPoint = Infantry.pitch_min_motor;
	
  /**************************************�������ֵ**************************************/
//PITCH                 
  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);   	
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
	PitchCurrent=(short)(-inttoshort[0]);

	YawCurrent=0;
	
}

///**********************************************************************************************************
//*�� �� ��: Gimbal_Act_Cal
//*����˵��: ��̨����ģʽ
//*��    ��: rc  mouse  Pc_RecvData
//*�� �� ֵ: ��
//**********************************************************************************************************/
//float test_pitch = 0;
//float test_yaw = 0;
//int aaaaa;
//void Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
//{
//	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
//	{
//		Laser_On();
//		GimbalPitchPos = GyroReceive.PITCH;				//�Ӵ��ģʽ�лأ�����pitch����ǣ�yaw�����ǽǣ����Ҷ�
//		GimbalYawPos = Gimbal.Yaw.Gyro;
//		GimbalAct_Init_Flag=Gimbal_Act_Mode;
//	}

//	
//	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
//	{
//	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
//	  GimbalPitchPos -= (1024-rc.ch3)*0.0003f;//��������
//	}
//	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
//	{
//    GimbalPitchPos -= mouse.y*0.005f;
//    GimbalYawPos   -= mouse.x*0.005f;
//		GimbalPitchPos += mouse.z*0.001f;
//	}

//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//��λ(�õ���Ƕ�)	//��ס��С����˿���MotorTransangle��pid
////YAW/***********************************************************************************/
//	PidPitchPos.SetPoint = GimbalPitchPos;
//	PidYawPos.SetPoint = GimbalYawPos;

//	
//  /**************************************�������ֵ**************************************/
////PITCH                
////  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);   
//  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,GyroReceive.PITCH);   	
//	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
//	PitchCurrent=(short)inttoshort[0];
////YAW
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//��ҪYAW�������ǽǶ���ActualValue
//	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
//	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
//	YawCurrent=(short)inttoshort[1];
//	/* pid_test ����*/
////	test_gyro_pitch = -Gimbal.Pitch.Gyro;
////	test_gyro_yaw = -Gimbal.Yaw.Gyro;
//}
/**********************************************************************************************************
*�� �� ��: FuzzyGimbal_Act_Cal
*����˵��: ģ����̨����ģʽ(�����)
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
void FuzzyMotorGimbal_Act_Cal(Remote rc,Mouse mouse)
{
	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;				//�Ӵ��ģʽ�лأ�����pitch����ǣ�yaw�����ǽǣ����Ҷ�
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag=Gimbal_Act_Mode;
	}

	
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0015f;
	  GimbalPitchPos -= (1024-rc.ch3)*0.0003f;//��������
		
		FF_w.Now_DeltIn  = (1024-rc.ch2)*0.0008f;
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.0016f;
		GimbalPitchPos += mouse.z*0.002f;
		
		FF_w.Now_DeltIn  = -mouse.x*0.0016f;
	}

	if(Budan)
	{
	GimbalPitchPos = Infantry.pitch_min_motor;
	}
	else
	{
	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ(�õ���Ƕ�)	//��ס��С����˿���MotorTransangle��pid
	}
	/***********************************************************************************/

	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
	
  /**************************************�������ֵ**************************************/
////PITCH                 
	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);   	
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
	PitchCurrent=(short)(-inttoshort[0]);
	


//YAW
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//��ҪYAW�������ǽǶ���ActualValue
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=(short)inttoshort[1];
	/* pid_test ����*/
//	test_gyro_pitch = -Gimbal.Pitch.Gyro;
//	test_gyro_yaw = -Gimbal.Yaw.Gyro;
}
/**********************************************************************************************************
*�� �� ��: FuzzyGimbal_Act_Cal
*����˵��: ģ����̨����ģʽ(IMU��)
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/

void FuzzyGyroGimbal_Act_Cal(Remote rc,Mouse mouse)
{
	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = GyroReceive.PITCH;				
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag=Gimbal_Act_Mode;
	}

	
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
	  GimbalPitchPos += (1024-rc.ch3)*0.0003f;//��������
		FF_w.Now_DeltIn = (1024-rc.ch2)*0.0005f;
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.005f;
		GimbalPitchPos -= mouse.z*0.001f;
		FF_w.Now_DeltIn = -mouse.x*0.005f;
	}

	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro,Infantry.pitch_min_gyro);//��λ(�õ���Ƕ�)	//��ס��С����˿���MotorTransangle��pid
	/***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
	
  /**************************************�������ֵ**************************************/
//PITCH                
////PITCH                 
	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.Gyro);   	
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
	PitchCurrent=(short)(-inttoshort[0]);

//YAW
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//��ҪYAW�������ǽǶ���ActualValue
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=(short)inttoshort[1];
	/* pid_test ����*/
//	test_gyro_pitch = -Gimbal.Pitch.Gyro;
//	test_gyro_yaw = -Gimbal.Yaw.Gyro;
}

/**********************************************************************************************************
*�� �� ��: Gimbal_Armor_Cal
*����˵��: ��̨����ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
float speed_limit = 30.0f;
char Aim_Follow;
float Inte_z;
extern float aim_yaw, aim_pitch;
float last_aim_yaw;

void Gimbal_Armor_Cal(Remote rc,Mouse mouse)
{
	if(GimbalAct_Init_Flag!=Gimbal_Armor_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_Armor_Mode;
		last_aim_yaw = GimbalYawPos = Gimbal.Yaw.Gyro;
	  GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
		aim_pitch = Gimbal.Pitch.MotorTransAngle;
		aim_yaw = Gimbal.Yaw.Gyro;
		Inte_z = 0;
	}

	Recent_Pitch_Angle_Armor = aim_pitch;
	Recent_Yaw_Angle_Armor = aim_yaw;
	Inte_z += mouse.z*0.002f;
	
	if(ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - (Gimbal.Pitch.MotorTransAngle))< 60)		//����ȫ
	{
				GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z;;	//����ֵ
				GimbalYawPos = Recent_Yaw_Angle_Armor;
				FF_w.Now_DeltIn = GimbalYawPos - last_aim_yaw;
	}
	else
	{
				GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;	//����ֵ
				GimbalYawPos = Gimbal.Yaw.Gyro;
				FF_w.Now_DeltIn = GimbalYawPos - last_aim_yaw;
	}
	
	last_aim_yaw = GimbalYawPos;  
	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ(�õ���Ƕ�)	//��ס��С����˿���MotorTransangle��pid
	/***********************************************************************************/
	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	FuzzyAidPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	
  PidPitchAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchAidSpeed,GyroReceive.GY));//��������
	PitchCurrent=-(short)inttoshort[0];
		
	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	FuzzyAidPidYawPos.ActPoint = Gimbal.Yaw.Gyro;
	
	PidYawAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidYawPos); 
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}


/**********************************************************************************************************
*�� �� ��: Gimbal_Buff_Cal
*����˵��: ��̨���ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_Buff_Cal(Remote rc,Mouse mouse)
{
	if(GimbalAct_Init_Flag!=Gimbal_BigBuf_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_BigBuf_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
	  GimbalPitchPos=Gimbal.Pitch.MotorTransAngle;
		aim_pitch = Gimbal.Pitch.MotorTransAngle;
		aim_yaw = 0;
	}


	Recent_Pitch_Angle_Armor = aim_pitch;
	Recent_Yaw_Angle_Armor = aim_yaw+Buff_Yaw_Motor;
	
	if(ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.MotorTransAngle) < 70 && ABS(Recent_Pitch_Angle_Armor - Gimbal.Pitch.MotorTransAngle)< 60)		//����ȫ
	{
				GimbalPitchPos = Recent_Pitch_Angle_Armor;	//����ֵ
				GimbalYawPos = Recent_Yaw_Angle_Armor;
	}
	else
	{
				GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;	//����ֵ
				GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
	}
	
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ
	/***************************************************************************************/
	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	FuzzyAidPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	
  PidPitchAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchAidSpeed,GyroReceive.GY));//��������
	PitchCurrent=-(short)inttoshort[0];
		
	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	FuzzyAidPidYawPos.ActPoint = Gimbal.Yaw.MotorTransAngle;
	
	PidYawAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidYawPos); 
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}

///**********************************************************************************************************
//*�� �� ��: Gimbal_DropShot_Cal
//*����˵��: ��̨����ģʽ
//*��    ��: rc  mouse  Pc_RecvData
//*�� �� ֵ: ��
//**********************************************************************************************************/

//void Gimbal_DropShot_Cal(Remote rc,Mouse mouse)
//{
//	if( GimbalAct_Init_Flag!=Gimbal_DropShot_Mode)
//	{
//		Laser_On();
//		GimbalAct_Init_Flag=Gimbal_DropShot_Mode;
//		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;//����ģʽ
//		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
//	}
//	
//	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
//	{
//	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
//		GimbalPitchPos -= (1024-rc.ch3)*0.0005f;//������
//	}
//	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
//	{
//    GimbalPitchPos -= mouse.y*0.005f;
//    GimbalYawPos   -= mouse.x*0.005f;
//	}
//	
//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ(�õ���Ƕ�)
//  /***********************************************************************************/
//	PidPitchPos.SetPoint = GimbalPitchPos;
//	PidYawPos.SetPoint = GimbalYawPos;
//  /**************************************�������ֵ**************************************/
//	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);  
//	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
//	PitchCurrent=(short)inttoshort[0];
//	
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);	
//	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
//	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
//	YawCurrent=inttoshort[1];	
//}

/**********************************************************************************************************
*�� �� ��: Gimbal_Test_Cal
*����˵��: ��̨����ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_Test_Cal(Remote rc,Mouse mouse)
{
	if(GimbalAct_Init_Flag!=Gimbal_Test_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_Test_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
	  GimbalPitchPos=Gimbal.Pitch.MotorTransAngle;
	}
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.00005f;
		GimbalPitchPos -= (1024-rc.ch3)*0.00005f;//������
	}
	
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ
	/***************************************************************************************/
	PidPitchAidPos.SetPoint = GimbalPitchPos;
	PidPitchAidSpeed.SetPoint=PID_Calc(&PidPitchAidPos,Gimbal.Pitch.MotorTransAngle);  	
	inttoshort[0]=(PID_Calc(&PidPitchAidSpeed,GyroReceive.GY));//��������
	PitchCurrent=-(short)inttoshort[0];
		
	PidYawAidPos.SetPoint = GimbalYawPos;
	PidYawAidSpeed.SetPoint=PID_Calc(&PidYawAidPos,Gimbal.Yaw.MotorTransAngle); 
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}

/**********************************************************************************************************
*�� �� ��: Gimbal_SI_Cal
*����˵��: ��̨ϵͳ��ʶģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
short F_Change_flag = 0;		//�л�Ƶ�ʱ�־
float	Gimbal_direct;

int T;//����
int T_cnt = 0;//����
int T_Time_cnt = 0;//���ڴ�������
int F_cnt = 0,F_cnt_last;//ָ��F��ָ��
float F = 1;

void Gimbal_SI_Cal(float Gimbal_pitch,float Gimbal_yaw)
{
	if(GimbalAct_Init_Flag!=Gimbal_SI_Mode)
	{
		GimbalPitchPos = 0;				
		GimbalYawPos = Gimbal.Yaw.Gyro;
  	GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;   //���ڽ�Ծ��Ӧ����				
  	GimbalYawPos = Gimbal.Yaw.Gyro +30.0f ;
		F = 1;		
		GimbalAct_Init_Flag = Gimbal_SI_Mode;
	}
		
	  T_change();		
//		GimbalPitchPos = LIMIT_MAX_MIN((Gimbal_direct*Gimbal_pitch),Infantry.pitch_max_motor,Infantry.pitch_min_motor);

  /**************************************�������ֵ**************************************/	
	/***************************************************************************************/
	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ(�õ���Ƕ�)	//��ס��С����˿���MotorTransangle��pid
	/***********************************************************************************/
	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	FuzzyAidPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	
  PidPitchAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchAidSpeed,GyroReceive.GY));//��������
	PitchCurrent=-(short)inttoshort[0];
		
	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	FuzzyAidPidYawPos.ActPoint = Gimbal.Yaw.Gyro;
	
	PidYawAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidYawPos); 
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
	
///************************************** ���ڿ���ϵͳ��ʶ ******************************************/	
//  	PitchCurrent= Gimbal_direct*Gimbal_pitch;
//	  YawCurrent = Gimbal_direct*Gimbal_yaw;
}

void get_F(void)
{
	static int j = 0;
	if(F < 22)
	{
		j++;
		F += 0.5f;
	}
	else if(F == 22)
	{
		F = 24;
	}
	else if(F >= 24&&F < 40)
	{
		F = F + 2;
	}
	else if(F == 40)
	{
		F = 50;
	}
	else if(F >= 50&&F < 120)
	{
		F = F + 10;
	}
	else if(F == 120)
	{
		F = 200;
	}
	else if(F == 250)
	{
		F = 333;
	}
	else if(F == 333)
	{
		F = 500;
	}
}
/**********************************************************************************************************
*�� �� ��: T_Change
*����˵��: ���ݲ�ͬ��Ƶ�ʻ�ö�Ӧ��ͬ������sin�ź�,F����50�ظ�10���ڣ�����20����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void T_change(void)
{
	T = round(1000/F);
	if(GimbalAct_Init_Flag == Gimbal_SI_Mode)
	{
		if(F_Change_flag == 0)
		{
			Gimbal_direct = sin(2*3.14*T_cnt/T);
			if(T_cnt >= T)
			{
				T_cnt = 0;
				T_Time_cnt++;
			}
			T_cnt++;
			F_cnt_last = F_cnt;

			if(T_Time_cnt >= 10 && F <22)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			else if(T_Time_cnt >= 10 && F >= 22 && F <50)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			else if(T_Time_cnt >= 20 && F >= 50)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			if(F_cnt != F_cnt_last)		//Ƶ�ʸı�
			{
				F_Change_flag  = 1;
			}
		}
		else if(F_Change_flag == 1)
		{
			Gimbal_direct = 0;
			if((ABS(PidPitchPos.SetPoint - PidPitchPos.ActualValue) < 0.2f) || (ABS(PidYawPos.SetPoint - PidYawPos.ActualValue) < 0.2f))		//�ص���ʼλ��
			{
				get_F();
				T_cnt = 0;
				F_Change_flag = 0;
			}
		}
	}
	else if(GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		F = 1;
	}
}


/**********************************************************************************************************
*�� �� ��: Gimbal_CurrentPid_Cal
*����˵��: ���͵���ֵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_CurrentPid_Cal(void)
{
	switch (Status.GimbalMode)
	{
		case Gimbal_Powerdown_Mode:
			Gimbal_Powerdown_Cal();
			break;
		
		case Gimbal_Act_Mode:
			FuzzyMotorGimbal_Act_Cal(RC_Ctl.rc,RC_Ctl.mouse);
			break;  
		
		case Gimbal_Armor_Mode:
			Gimbal_Armor_Cal(RC_Ctl.rc,RC_Ctl.mouse);
			break;
		
		case Gimbal_BigBuf_Mode:
		case Gimbal_SmlBuf_Mode:
			Gimbal_Buff_Cal(RC_Ctl.rc,RC_Ctl.mouse);
			break;
		
//		case Gimbal_DropShot_Mode:
//			Gimbal_DropShot_Cal(RC_Ctl.rc,RC_Ctl.mouse);
//			break; 
		
		case Gimbal_SI_Mode:
			Gimbal_SI_Cal(8000.0, 0);			//pitch
//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
			break;
		
		case Gimbal_Jump_Mode:
			FuzzyGyroGimbal_Act_Cal(RC_Ctl.rc,RC_Ctl.mouse);
		  break;
		
		case Gimbal_Test_Mode:
			Gimbal_Test_Cal(RC_Ctl.rc,RC_Ctl.mouse);
			break;
		
		default:
				Gimbal_Powerdown_Cal();	
			break;
	}
	if(q_flag) F405.Gimbal_Flag = 7;     //��̨�Լ���
	else       F405.Gimbal_Flag = Status.GimbalMode;
	GimbalCan2Send(PitchCurrent,YawCurrent);

}


/**********************************************************************************************************
*�� �� ��: Pid_Yaw_MotorPos_GyroSpeed
*����˵��: Yaw�Ḩ��˫��PID
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PidGimbalMotor_Init(void)
{ 
	switch(Robot_ID)
{
/********************************************* 3�ų� ********************************************************/	
		case 3:
{
  //�ֶ�pitch˫��
  PidPitchPos.P=0.32f;       //0.32�ֶ�pitch�ǶȻ�
	PidPitchPos.I=0.0000f;
	PidPitchPos.D=0.0f;       
	PidPitchPos.IMax=10.0f;
	PidPitchPos.SetPoint=0.0f;
	PidPitchPos.OutMax=5.5f;	
	
  PidPitchSpeed.P=12000.0f;  //12000�ֶ�pitch�ٶȻ�
	PidPitchSpeed.I=12.0f;  
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//�ֶ�yaw˫��                                 // 3�ų�
  PidYawPos.P=0.22f;        //�ֶ�yaw�ǶȻ�
	PidYawPos.I=0.00f;
	PidYawPos.D=0.0f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	
	PidYawSpeed.P=22000.0f;   //�ֶ�yaw�ٶȻ�
	PidYawSpeed.I=1.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
	//����pitch
	PidPitchAidPos.P =0.20f;	  //��ͨPIDλ�û�(����)
	PidPitchAidPos.I=0.00001f; 
	PidPitchAidPos.D=0.0f;
	PidPitchAidPos.IMax=40.0f;
	PidPitchAidPos.SetPoint=0.0f;
	PidPitchAidPos.OutMax=5.5f;
	
	FuzzyAidPidPitchPos.Kp=0.2f;
	FuzzyAidPidPitchPos.Ki=0.003f;  //ģ��PIDλ�û������飩
	FuzzyAidPidPitchPos.Kd=0.0f;
	FuzzyAidPidPitchPos.IMax=40.0f;
	FuzzyAidPidPitchPos.SetPoint=0.0f;
	
  PidPitchAidSpeed.P=10000.0f;	  //�ٶȻ�PID�����飩
	PidPitchAidSpeed.I=18.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
	
	//����yaw
	PidYawAidPos.P =0.20f;	  //yaw PIDλ�û������飩
	PidYawAidPos.I=0.000f; 
	PidYawAidPos.D=0.0f;
	PidYawAidPos.IMax=40.0f;
	PidYawAidPos.SetPoint=0.0f;
	PidYawAidPos.OutMax=5.5f;
	
	                              
	FuzzyAidPidYawPos.Kp=0.26f;   //ģ��PID yaw�ǶȻ������飩
	FuzzyAidPidYawPos.Ki=0.0001f;  
	FuzzyAidPidYawPos.Kd=0.0f;
	FuzzyAidPidYawPos.IMax=40.0f;
	FuzzyAidPidYawPos.SetPoint=0.0f;	
	
	PidYawAidSpeed.P=12000.0f;			//32000  yaw�ٶȻ�PID(����)
	PidYawAidSpeed.I=3.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;

// 	FuzzyAidPidPitchPos.Kp=0.45f;
//	FuzzyAidPidPitchPos.Ki=0.01f;  //ģ��PIDλ�û������飩
//	FuzzyAidPidPitchPos.Kd=0.0f;
//	FuzzyAidPidPitchPos.IMax=40.0f;
//	FuzzyAidPidPitchPos.SetPoint=0.0f;
//	
//  PidPitchAidSpeed.P=12000.0f;	  //�ٶȻ�PID�����飩
//	PidPitchAidSpeed.I=15.0f; 
//	PidPitchAidSpeed.D=8000.0f;
//	PidPitchAidSpeed.IMax=250.0f;
//	PidPitchAidSpeed.SetPoint=0.0f;
//	PidPitchAidSpeed.OutMax=30000.0f;
}break;


/********************************************* 4�ų� ********************************************************/	
		case 4:
{
    //�ֶ�pitch˫��
  PidPitchPos.P=0.2f;       //�ֶ�pitch�ǶȻ�
	PidPitchPos.I=0.0001f;
	PidPitchPos.D=0.0f;       
	PidPitchPos.IMax=10.0f;
	PidPitchPos.SetPoint=0.0f;
	PidPitchPos.OutMax=5.5f;	
	
  PidPitchSpeed.P=8000.0f;  //�ֶ�pitch�ٶȻ�
	PidPitchSpeed.I=12.0f;  
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//�ֶ�yaw˫��                                 // 4�ų�
  PidYawPos.P=0.22f;        //�ֶ�yaw�ǶȻ�
	PidYawPos.I=0.00f;
	PidYawPos.D=0.0f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	
	PidYawSpeed.P=22000.0f;   //�ֶ�yaw�ٶȻ�
	PidYawSpeed.I=1.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
	//����pitch�ٶȻ�
  PidPitchAidSpeed.P=10000.0f;	  //10000
	PidPitchAidSpeed.I=14.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
	
	//����pitch�ǶȻ���ģ����
	FuzzyAidPidPitchPos.Kp=0.2f;
	FuzzyAidPidPitchPos.Ki=0.0001f;  //0.005f;
	FuzzyAidPidPitchPos.Kd=0.0f;
	FuzzyAidPidPitchPos.IMax=40.0f;
	FuzzyAidPidPitchPos.SetPoint=0.0f;
	
	//����yaw�ǶȻ���ģ����
	FuzzyAidPidYawPos.Kp=0.26f;
	FuzzyAidPidYawPos.Ki=0.0001f;  //0.005f;
	FuzzyAidPidYawPos.Kd=0.0f;
	FuzzyAidPidYawPos.IMax=40.0f;
	FuzzyAidPidYawPos.SetPoint=0.0f;
	
	//����yaw�ٶȻ�
	PidYawAidSpeed.P=16000.0f;			//12000
	PidYawAidSpeed.I=0.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
}break;

/********************************************* 14�ų� ********************************************************/	
		case 14:
{
  //�ֶ�pitch˫��
  PidPitchPos.P=0.35f;       //�ֶ�pitch�ǶȻ�
	PidPitchPos.I=0.0001f;
	PidPitchPos.D=0.0f;       
	PidPitchPos.IMax=10.0f;
	PidPitchPos.SetPoint=0.0f;
	PidPitchPos.OutMax=5.5f;	
	
  PidPitchSpeed.P=12000.0f;  //�ֶ�pitch�ٶȻ�
	PidPitchSpeed.I=12.0f;  
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//�ֶ�yaw˫��                                 // 5�ų�
  PidYawPos.P=0.22f;        //�ֶ�yaw�ǶȻ�
	PidYawPos.I=0.00f;
	PidYawPos.D=0.0f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	
	PidYawSpeed.P=22000.0f;   //�ֶ�yaw�ٶȻ�
	PidYawSpeed.I=1.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
//����yaw
	PidYawAidPos.P =0.20f;	  //yaw PIDλ�û������飩
	PidYawAidPos.I=0.000f; 
	PidYawAidPos.D=0.0f;
	PidYawAidPos.IMax=40.0f;
	PidYawAidPos.SetPoint=0.0f;
	PidYawAidPos.OutMax=5.5f;
	
	                              
	FuzzyAidPidYawPos.Kp=0.35f;   //ģ��PID yaw�ǶȻ������飩
	FuzzyAidPidYawPos.Ki=0.0007f;  
	FuzzyAidPidYawPos.Kd=0.0f;
	FuzzyAidPidYawPos.IMax=40.0f;
	FuzzyAidPidYawPos.SetPoint=0.0f;	
	
	PidYawAidSpeed.P=15000.0f;			//32000  yaw�ٶȻ�PID(����)
	PidYawAidSpeed.I=5.0f; 
	PidYawAidSpeed.D=10000.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
	
		//����pitch
	PidPitchAidPos.P =0.45f;	  //��ͨPIDλ�û�(����)
	PidPitchAidPos.I=0.01f; 
	PidPitchAidPos.D=0.0f;
	PidPitchAidPos.IMax=40.0f;
	PidPitchAidPos.SetPoint=0.0f;
	PidPitchAidPos.OutMax=5.5f;
	
	FuzzyAidPidPitchPos.Kp=0.45f;
	FuzzyAidPidPitchPos.Ki=0.01f;  //ģ��PIDλ�û������飩
	FuzzyAidPidPitchPos.Kd=0.0f;
	FuzzyAidPidPitchPos.IMax=40.0f;
	FuzzyAidPidPitchPos.SetPoint=0.0f;
	
  PidPitchAidSpeed.P=12000.0f;	  //�ٶȻ�PID�����飩
	PidPitchAidSpeed.I=15.0f; 
	PidPitchAidSpeed.D=8000.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
}break;


	}
}


/**********************************************************************************************************
*�� �� ��: Gimbal_task
*����˵��: ��̨������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Gimbal_high_water;
UBaseType_t GGGG;
extern TaskHandle_t RCReceiveTask_Handler; //������
extern TaskHandle_t PCReceiveTask_Handler; //������
extern uint8_t Remote_Receive_Flag;
extern uint8_t PC_ReceiveFlag;

void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
	vTaskDelay(1000);
   while (1) {
		xLastWakeTime = xTaskGetTickCount();
  
		if(Remote_Receive_Flag)  //���ݽ���
		{	
		xTaskNotifyGive(RCReceiveTask_Handler);
		}
		
		if(PC_ReceiveFlag)       //���ݽ���
		{
		xTaskNotifyGive(PCReceiveTask_Handler);		
		}
		 
		 if(PitchMotor_ReceiveFlag)  //ȷ����õ�����ݺ��ٽ��п���
	{
		ZeroCheck_cal(); 
		Gimbal_CurrentPid_Cal();
	}	 
	
		
		IWDG_Feed();//ι��
		
    vTaskDelayUntil(&xLastWakeTime,xFrequency); 

#if INCLUDE_uxTaskGetStackHighWaterMark
        Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

