/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.6
**********************************************************************************************************/
#include "main.h"
extern KalmanFilter_t pitch_Kalman, yaw_Kalman;
extern KalmanFilter_Init_t K;
unsigned volatile long run_time_check = 0;	//���������ּ��׼���������
short fric_flag = 0;//Ħ���ֵ����ʼ����־
RobotInit_Struct Infantry;
char Judge_Lost;
extern short FrictionWheel_speed;
extern short KalMan_doneflag;
/**********************************************************************************************************
*�� �� ��: main
*����˵��: ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

int main()
{

	BSP_Init();
	Robot_Init();
	startTast();
  vTaskStartScheduler();
	
	while(1)
	{
	
	}
}
	

/**********************************************************************************************************
*�� �� ��: System_Config
*����˵��: ϵͳ��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BSP_Init(void)
{
	USART1_Configuration();
	delay_ms(100);
	USART2_Configuration();
	delay_ms(100);
	MicroSwConfigration();
	delay_ms(100);
  SteeringEngine_Configuration();
	delay_ms(100);
	TIM7_Configuration();
	delay_ms(100);
	CAN1_Configuration();
	delay_ms(100);
	CAN2_Configuration();
	delay_ms(100);
	IWDG_Config(IWDG_Prescaler_128 ,625);
	delay_ms(100);
	VOFA_USART_Configuration();
	delay_ms(3000);
//	My_GPIO_Init();
	SteeringEngine_Set(Infantry.MagOpen);
}
/**********************************************************************************************************
*�� �� ��: System_Init
*����˵��: ϵͳ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Robot_Init(void)
{
	ZeroCheck_Init();
	Infantry_Init();
	Pid_ChassisPosition_Init();
  PidGimbalMotor_Init();
	Pid_BodanMotor_Init();
	Pid_Friction_Init();

}

/**********************************************************************************************************
*�� �� ��: Infantry_Init
*����˵��: ����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Infantry_Init(void)
{	

#if  Robot_ID == 3
/***************************************** 3 �ų� **************************************************************/	
	Infantry.Yaw_init=2750;                  //  3�ų� 
	Infantry.Pitch_init=7440;
	Infantry.MagOpen=830;  
	Infantry.MagClose=1900;
	Infantry.Solo_Yaw_init = 7715;
	Infantry.PitchMotorID = 0x206;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0]=0x202;
	Infantry.FricMotorID[1]=0x203;
	Infantry.BodanMotorID=0x201;
	Infantry.pitch_max_motor = 35;
	Infantry.pitch_min_motor = -13;
	Infantry.pitch_max_gyro = 31;
	Infantry.pitch_min_gyro = -13;
  Infantry.pn=-1;
	
#elif Robot_ID == 4	
/***************************************** 4 �ų� **************************************************************/	
	Infantry.Yaw_init=6865;            // 4�ų�
	Infantry.Pitch_init=4702;
	Infantry.MagOpen=1000;
	Infantry.MagClose=2170;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0]=0x202;
	Infantry.FricMotorID[1]=0x201;
	Infantry.BodanMotorID=0x203;
	Infantry.pitch_max_motor = 28;
	Infantry.pitch_min_motor = -14;
	Infantry.pitch_max_gyro = 28;
	Infantry.pitch_min_gyro = -13;
	Infantry.pn=-1;                  //�������������pitchʩ��ϵ��ʹ�䷽����ȷ

#elif Robot_ID == 14
/***************************************** 14 �ų� **************************************************************/	
	Infantry.Yaw_init=4747;            // 14�ų�
	Infantry.Pitch_init=2035;
	Infantry.MagOpen=1000;
	Infantry.MagClose=1900;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0]=0x203;
	Infantry.FricMotorID[1]=0x201;
	Infantry.BodanMotorID=0x202;
	Infantry.pitch_max_motor = 38;
	Infantry.pitch_min_motor = -15;
	Infantry.pitch_max_gyro = 36;
	Infantry.pitch_min_gyro = -16;
	Infantry.pn=1;

#endif

}

/**********************************************************************************************************
*�� �� ��: delay_ms
*����˵��: ms��ʱ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void delay_ms(unsigned long t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=10300;
 		while(a--);
	}
}
void delay_us(unsigned long t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=37;
 		while(a--);
	}
}
/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ���߼������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern Disconnect Robot_Disconnect;
void Offline_Check_task(void *pvParameters)
{
   while (1) {

		/*���\IMU���߼��*/
	if(Robot_Disconnect.YawMotor_DisConnect>10||Robot_Disconnect.PitchMotor_DisConnect>10||Robot_Disconnect.Gyro_DisConnect>20)
		{
//	    Robot_Stop();
		}
	else
		{
//			Robot_Recover();
		}
	Robot_Disconnect.Gyro_DisConnect++;
	Robot_Disconnect.PitchMotor_DisConnect++;
	Robot_Disconnect.YawMotor_DisConnect++;
	
	 /*����������� */
	 if(Robot_Disconnect.Friction_DisConnect[0]>10||Robot_Disconnect.Friction_DisConnect[1]>10||Robot_Disconnect.Pluck_DisConnect>10)
		{
		 Shoot_Stop();
		}
		else
		{
		 Shoot_Recover();
		}
	Robot_Disconnect.Friction_DisConnect[0]++;
	Robot_Disconnect.Friction_DisConnect[1]++;
	Robot_Disconnect.Pluck_DisConnect++;
		
	/*ң�������߼��*/
	if(Robot_Disconnect.RC_DisConnect>10)
		{
			RC_Rst();
		}
	Robot_Disconnect.RC_DisConnect++;
		
	/*���̰���߲���ϵͳ���߼��*/
	if(Robot_Disconnect.F105_DisConect>15||Judge_Lost==1)
		{
		F105_Rst();
		}
	Robot_Disconnect.F105_DisConect++;
		
		/* PC�ݶ� */
	if(Robot_Disconnect.PC_DisConnect>10)
		{
		}
	Robot_Disconnect.PC_DisConnect++;
	VOFA_Send();
	IWDG_Feed();	 
  vTaskDelay(5);  // 5
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



