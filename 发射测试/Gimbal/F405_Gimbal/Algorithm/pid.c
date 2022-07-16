/**********************************************************************************************************
 * @�ļ�     pid.c
 * @˵��     pid�㷨+ǰ���㷨
 * @�汾  	 V3.0
 * @����     ��־�ۡ������¡�����
 * @����     2020.1��2021.9��2022.7
**********************************************************************************************************/
#include "main.h"

/**********************************************************************************************************
*�� �� ��: PID_Calc
*����˵��: PID�����㷨
*��    ��: PID_Struct *P  PID�����ṹ��
  *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
*�� �� ֵ: PID�����������ֵ
**********************************************************************************************************/
float PID_Calc(Pid_Typedef *P, float ActualValue)
{
    P->ActualValue=ActualValue;
		P->PreError = P->SetPoint - ActualValue;
		
		if((P->PreError<(P->DeadZone))&&(P->PreError>(-P->DeadZone)))
		{
		  P->PreError = 0.0f;
			P->LastError = P->PreError;
			return 0.0f;
		}
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		P->SumError += (P->PreError+P->LastError)/2;    //���λ��ִ�����λ��֣���߾���
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax); 
}

/**********************************************************************************************************
*�� �� ��: FeedForward_Calc
*����˵��: ǰ���㷨
*��    ��: PID_Struct *P  PID�����ṹ��
  *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
*�� �� ֵ: PID�����������ֵ
**********************************************************************************************************/
float FeedForward_Calc(FeedForward_Typedef *FF)
{
	  FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
	  FF->Last_DeltIn = FF->Now_DeltIn;
    return FF->Out;
}




/*********ģ��pid����*/
/* ����Kp��Ki��Kd��һ����ֵ */

#define IS_Kp 1
#define IS_Ki 2
#define IS_Kd 3

#define stair  0.25f
#define pstair 0.015f
#define istair 0.0005f
#define dstair 0.001f
 
#define NL   -(3*stair)
#define NM	 -(2*stair)
#define NS	 -(1*stair)
#define ZE	 0
#define PS	 (1*stair)
#define PM	 (2*stair)
#define PL	 (3*stair)

#define NLp  -(3*pstair)
#define NMp	 -(2*pstair)
#define NSp	 -(1*pstair)
#define ZEp	 0
#define PSp	 (1*pstair)
#define PMp	 (2*pstair)
#define PLp	 (3*pstair)
 
#define NLi   -(3*istair)
#define NMi	 -(2*istair)
#define NSi	 -(1*istair)
#define ZEi	 0
#define PSi	 (1*istair)
#define PMi	 (2*istair)
#define PLi	 (3*istair) 

#define NLd   -(3*dstair)
#define NMd	 -(2*dstair)
#define NSd	 -(1*dstair)
#define ZEd	 0
#define PSd	 (1*dstair)
#define PMd	 (2*dstair)
#define PLd	 (3*dstair)
 
 
static const float fuzzyRuleKp[7][7]={
	PLp,	PLp,	PMp,	PMp,	PSp,	ZEp,	ZEp,
	PLp,	PLp,	PMp,	PSp,	PSp,	ZEp,	NSp,
	PMp,	PMp,	PMp,	PSp,	ZEp,	NSp,	NSp,
	PMp,	PMp,	PSp,	ZEp,	NSp,	NMp,	NMp,
	PSp,	PSp,	ZEp,	NSp,	NSp,	NMp,	NMp,
	PSp,	ZEp,	NSp,	NMp,	NMp,	NMp,	NLp,
	ZEp,	ZEp,	NMp,	NMp,	NMp,	NLp,	NLp
};
 
static const float fuzzyRuleKi[7][7]={
	NLi,	NLi,	NMi,	NMi,	NSi,	ZEi,	ZEi,
	NLi,	NLi,	NMi,	NSi,	NSi,	ZEi,	ZEi,
	NLi,	NMi,	NSi,	NSi,	ZEi,	PSi,	PSi,
	NMi,	NMi,	NSi,	ZEi,	PSi,	PMi,	PMi,
	NSi,	NSi,	ZEi,	PSi,	PSi,	PMi,	PLi,
	ZEi,	ZEi,	PSi,	PSi,	PMi,	PLi,	PLi,
	ZEi,	ZEi,	PSi,	PMi,	PMi,	PLi,	PLi
};
 
static const float fuzzyRuleKd[7][7]={
	PSd,	NSd,	NLd,	NLd,	NLd,	NMd,	PSd,
	PSd,	NSd,	NLd,	NMd,	NMd,	NSd,	ZEd,
	ZEd,	NSd,	NMd,	NMd,	NSd,	NSd,	ZEd,
	ZEd,	NSd,	NSd,	NSd,	NSd,	NSd,	ZEd,
	ZEd,	ZEd,	ZEd,	ZEd,	ZEd,	ZEd,	ZEd,
	PLd,	NSd,	PSd,	PSd,	PSd,	PSd,	PLd,
	PLd,	PMd,	PMd,	PMd,	PSd,	PSd,	PLd
};
 

 //�ؼ��㷨
void fuzzy( FuzzyPID*  fuzzy_PID,float e,float ec)
{

     float etemp,ectemp;
     float eLefttemp,ecLefttemp;    //������
     float eRighttemp ,ecRighttemp; 
 
     short eLeftIndex,ecLeftIndex;  //��ǩ
     short eRightIndex,ecRightIndex;

	  //ģ����
     if(e>=PL)
			 etemp=PL;//������Χ
		 else if(e>=PM)
			 etemp=PM;
		 else if(e>=PS)
			 etemp=PS;
		 else if(e>=ZE)
			 etemp=ZE;
		 else if(e>=NS)
			 etemp=NS;
		 else if(e>=NM)
			 etemp=NM;
		 else if(e>=NL)
			 etemp=NL;
		 else 
			 etemp=2*NL;
 
		 if( etemp == PL)
		{
		 //����E������
				eRighttemp= 0 ;    //�����
				eLefttemp= 1 ;
			
     //�����ǩ
	   eLeftIndex = 6 ;      
	   eRightIndex= 6 ;
			
		}else if( etemp == 2*NL )
    {

			//����E������
				eRighttemp = 1;    //�����
				eLefttemp = 0;
	
     //�����ǩ
	   eLeftIndex = 0 ;       
	   eRightIndex = 0 ;
			
		}	else 
    {

			//����E������
				eRighttemp=((e-etemp)/stair);  //���Ժ�����Ϊ��������
				eLefttemp=(1- eRighttemp);
			
     //�����ǩ
	   eLeftIndex =(short) ((etemp-NL)/stair);       //���� etemp=2.5��NL=-3����ô�õ������к�Ϊ5  ��0 1 2 3 4 5 6��
	   eRightIndex=(short) (eLeftIndex+1);
			
		}		
	   
		
		 if(ec>=PL)
			 ectemp=PL;
		 else if(ec>=PM)
			 ectemp=PM;
		 else if(ec>=PS)
			 ectemp=PS;
		 else if(ec>=ZE)
			 ectemp=ZE;
		 else if(ec>=NS)
			 ectemp=NS;
		 else if(ec>=NM)
			 ectemp=NM;
		 else if(ec>=NL)
			 ectemp=NL;
		 else 
			 ectemp=2*NL;
		 
	  
   if( ectemp == PL )
	 {
    //����EC������		 
		 ecRighttemp= 0 ;      //�����
		 ecLefttemp= 1 ;
			
		 ecLeftIndex = 6 ;  
	   ecRightIndex = 6 ;	 
	 
	 } else if( ectemp == 2*NL)
	 {
    //����EC������		 
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;
			
		 ecLeftIndex = 0 ;  
	   ecRightIndex = 0 ;	 	 
	 }else
	 {
    //����EC������		 
		 ecRighttemp=((ec-ectemp)/stair);
		 ecLefttemp=(1- ecRighttemp);
			
		 ecLeftIndex =(short) ((ectemp-NL)/stair);  
	   ecRightIndex= (short)(eLeftIndex+1);
	 }	

 
/*************************************��ģ��*************************************/
 
 
 
 
	fuzzy_PID->Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->Ki =   (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

 
	fuzzy_PID->Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
 
}
 
//�˴���ȫ�ֱ�����������ģ����kp��ki��kd���� 
FuzzyPID OUT;
float FuzzyPID_Calc(FuzzyPID *pid)
{
	static float LastError;
 
	pid->PreError = pid->SetPoint - pid->ActPoint;             //E :Ŀ��ֵ-ʵ��ֵ
	pid->dError = pid->PreError - LastError;                   //EC:���仯��
	pid->SumError += (pid->PreError + pid->LastError)/2.0F;    //���λ��ּ�С���
	
			
		if(pid->SumError >= pid->IMax)
			pid->SumError = pid->IMax;
		else if(pid->SumError <= -pid->IMax)
			pid->SumError = -pid->IMax;
	
	pid->LastError = pid->PreError;
	LastError=pid->PreError;
	
		pid->POut = pid->Kp * pid->PreError;
		pid->IOut = pid->Ki* pid->SumError;
		pid->DOut = pid->Kd * pid->dError;		
		
		
	 fuzzy(&OUT,pid->PreError, pid->dError);      //ģ������  kp,ki,kd   �β�1��ǰ���β�2ǰ�����Ĳ�ֵ

//	return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki+OUT.Ki)*pid->SumError;  //PID��ģ��
//  return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd)*pid->dError + (pid->Ki+OUT.Ki)*pid->SumError;              //��PIģ��
   	return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki)*pid->SumError;       //��PDģ��
//	return (pid->Kp+OUT.Kp)*pid->PreError + pid->Kd*pid->dError + pid->Ki*pid->SumError;                  //��Pģ��
}