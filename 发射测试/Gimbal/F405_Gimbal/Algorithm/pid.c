/**********************************************************************************************************
 * @文件     pid.c
 * @说明     pid算法+前馈算法
 * @版本  	 V3.0
 * @作者     黄志雄、江扬新、戴军
 * @日期     2020.1、2021.9、2022.7
**********************************************************************************************************/
#include "main.h"

/**********************************************************************************************************
*函 数 名: PID_Calc
*功能说明: PID反馈算法
*形    参: PID_Struct *P  PID参数结构体
  *        ActualValue    PID计算反馈量（当前真实检测值）
*返 回 值: PID反馈计算输出值
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
	
		P->SumError += (P->PreError+P->LastError)/2;    //梯形积分代替矩形积分，提高精度
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
*函 数 名: FeedForward_Calc
*功能说明: 前馈算法
*形    参: PID_Struct *P  PID参数结构体
  *        ActualValue    PID计算反馈量（当前真实检测值）
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
float FeedForward_Calc(FeedForward_Typedef *FF)
{
	  FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
	  FF->Last_DeltIn = FF->Now_DeltIn;
    return FF->Out;
}




/*********模糊pid部分*/
/* 其中Kp、Ki、Kd是一个初值 */

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
 

 //关键算法
void fuzzy( FuzzyPID*  fuzzy_PID,float e,float ec)
{

     float etemp,ectemp;
     float eLefttemp,ecLefttemp;    //隶属度
     float eRighttemp ,ecRighttemp; 
 
     short eLeftIndex,ecLeftIndex;  //标签
     short eRightIndex,ecRightIndex;

	  //模糊化
     if(e>=PL)
			 etemp=PL;//超出范围
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
		 //计算E隶属度
				eRighttemp= 0 ;    //右溢出
				eLefttemp= 1 ;
			
     //计算标签
	   eLeftIndex = 6 ;      
	   eRightIndex= 6 ;
			
		}else if( etemp == 2*NL )
    {

			//计算E隶属度
				eRighttemp = 1;    //左溢出
				eLefttemp = 0;
	
     //计算标签
	   eLeftIndex = 0 ;       
	   eRightIndex = 0 ;
			
		}	else 
    {

			//计算E隶属度
				eRighttemp=((e-etemp)/stair);  //线性函数作为隶属函数
				eLefttemp=(1- eRighttemp);
			
     //计算标签
	   eLeftIndex =(short) ((etemp-NL)/stair);       //例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
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
    //计算EC隶属度		 
		 ecRighttemp= 0 ;      //右溢出
		 ecLefttemp= 1 ;
			
		 ecLeftIndex = 6 ;  
	   ecRightIndex = 6 ;	 
	 
	 } else if( ectemp == 2*NL)
	 {
    //计算EC隶属度		 
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;
			
		 ecLeftIndex = 0 ;  
	   ecRightIndex = 0 ;	 	 
	 }else
	 {
    //计算EC隶属度		 
		 ecRighttemp=((ec-ectemp)/stair);
		 ecLefttemp=(1- ecRighttemp);
			
		 ecLeftIndex =(short) ((ectemp-NL)/stair);  
	   ecRightIndex= (short)(eLeftIndex+1);
	 }	

 
/*************************************反模糊*************************************/
 
 
 
 
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
 
//此处的全局变量用来保存模糊的kp，ki，kd增量 
FuzzyPID OUT;
float FuzzyPID_Calc(FuzzyPID *pid)
{
	static float LastError;
 
	pid->PreError = pid->SetPoint - pid->ActPoint;             //E :目标值-实际值
	pid->dError = pid->PreError - LastError;                   //EC:误差变化率
	pid->SumError += (pid->PreError + pid->LastError)/2.0F;    //梯形积分减小误差
	
			
		if(pid->SumError >= pid->IMax)
			pid->SumError = pid->IMax;
		else if(pid->SumError <= -pid->IMax)
			pid->SumError = -pid->IMax;
	
	pid->LastError = pid->PreError;
	LastError=pid->PreError;
	
		pid->POut = pid->Kp * pid->PreError;
		pid->IOut = pid->Ki* pid->SumError;
		pid->DOut = pid->Kd * pid->dError;		
		
		
	 fuzzy(&OUT,pid->PreError, pid->dError);      //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值

//	return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki+OUT.Ki)*pid->SumError;  //PID均模糊
//  return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd)*pid->dError + (pid->Ki+OUT.Ki)*pid->SumError;              //仅PI模糊
   	return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki)*pid->SumError;       //仅PD模糊
//	return (pid->Kp+OUT.Kp)*pid->PreError + pid->Kd*pid->dError + pid->Ki*pid->SumError;                  //仅P模糊
}