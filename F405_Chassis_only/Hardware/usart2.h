#ifndef _USART2_H_
#define _USART2_H_

/*遥控器结构体*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*鼠标结构体*/
typedef __packed 	struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*键盘结构体*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;
/*遥键鼠结构体综合*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
  char RCrecvd,RCDisconnectCnt;//RCrecvd为数据接收标志位
}RC_Ctl_t;
void USART1_Configuration(void);
#endif
