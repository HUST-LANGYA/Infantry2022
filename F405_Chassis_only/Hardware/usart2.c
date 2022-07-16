/**********************************************************************************************************
 * @�ļ�     usart2.c
 * @˵��     usart2��ʼ��
 * @�汾  	 V1.0
 * @����     ����
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
RC_Ctl_t RC_Ctl; 
volatile unsigned char sbus_rx_buffer[18];
/**********************************************************************************************************
*�� �� ��: USART1_Configuration
*����˵��: ң�������ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void USART1_Configuration(void)
{
    USART_InitTypeDef usart1;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
	
		gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA,&gpio);

		usart1.USART_BaudRate = 100000;
		usart1.USART_WordLength = USART_WordLength_8b;
		usart1.USART_StopBits = USART_StopBits_1;
		usart1.USART_Parity = USART_Parity_Even;
		usart1.USART_Mode = USART_Mode_Rx;
    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1,&usart1);

    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
		USART_Cmd(USART1,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);	

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		{
			DMA_InitTypeDef   dma;
 			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
			DMA_DeInit(DMA2_Stream5);
			dma.DMA_Channel= DMA_Channel_4;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
			dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
			dma.DMA_BufferSize = 30;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream5,&dma);
			DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
			DMA_Cmd(DMA2_Stream5,ENABLE);
		}
}



/**********************************************************************************************************
*�� �� ��: USART2_IRQHandler
*����˵��: USART2ң����������ʱ�ж�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

void RemoteReceive(volatile unsigned char rx_buffer[])
{
	RC_Ctl.rc.ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((rx_buffer[5] >> 4)& 0x0003); //!< Switch left
	RC_Ctl.rc.s2 = ((rx_buffer[5] >> 6)& 0x0003);
	RC_Ctl.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.w = rx_buffer[14]&0x01; // KeyBoard value
	RC_Ctl.key.s = (rx_buffer[14]>>1)&0x01;
	RC_Ctl.key.a = (rx_buffer[14]>>2)&0x01;
	RC_Ctl.key.d = (rx_buffer[14]>>3)&0x01;
	RC_Ctl.key.shift =(rx_buffer[14]>>4)&0x01;
	RC_Ctl.key.ctrl = (rx_buffer[14]>>5)&0x01;
	RC_Ctl.key.q = (rx_buffer[14]>>6)&0x01;
	RC_Ctl.key.e = (rx_buffer[14]>>7)&0x01;	
	RC_Ctl.key.r = (rx_buffer[15])&0x01;
	RC_Ctl.key.f = (rx_buffer[15]>>1)&0x01;
	RC_Ctl.key.g = (rx_buffer[15]>>2)&0x01; 
	RC_Ctl.key.z = (rx_buffer[15]>>3)&0x01;
	RC_Ctl.key.x = (rx_buffer[15]>>4)&0x01;
	RC_Ctl.key.c = (rx_buffer[15]>>5)&0x01;
	RC_Ctl.key.v = (rx_buffer[15]>>6)&0x01;
	RC_Ctl.key.b = (rx_buffer[15]>>7)&0x01;
	if((RC_Ctl.rc.ch0-1024<15)&&(RC_Ctl.rc.ch0-1024>-15)) RC_Ctl.rc.ch0=1024;
	if((RC_Ctl.rc.ch1-1024<15)&&(RC_Ctl.rc.ch1-1024>-15)) RC_Ctl.rc.ch1=1024;
	if((RC_Ctl.rc.ch2-1024<10)&&(RC_Ctl.rc.ch2-1024>-10)) RC_Ctl.rc.ch2=1024;
	if((RC_Ctl.rc.ch3-1024<10)&&(RC_Ctl.rc.ch3-1024>-10)) RC_Ctl.rc.ch3=1024;	


}





void USART1_IRQHandler(void)
{
	static int DATA_LENGTH=0;
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
    (void)USART1->SR;
		(void)USART1->DR;	
	  DMA_Cmd(DMA2_Stream5,DISABLE);
	  DATA_LENGTH = 30 - DMA2_Stream5->NDTR;
		 if(DATA_LENGTH==18)
		 {
   RemoteReceive(sbus_rx_buffer);
		 }
		DMA2_Stream5->NDTR = 30;	
		DMA_Cmd(DMA2_Stream5,ENABLE);
  }
}