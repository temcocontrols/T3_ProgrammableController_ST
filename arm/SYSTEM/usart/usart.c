
#include "usart.h"

#include "main.h"

//////////////////////////////////////////////////////////////////////////////////
//V1.3ÐÞ¸ÄËµÃ÷ 
//Ö§³ÖÊÊÓ¦²»Í¬ÆµÂÊÏÂµÄ´®¿Ú²¨ÌØÂÊÉèÖÃ.
//¼ÓÈëÁË¶ÔprintfµÄÖ§³Ö
//Ôö¼ÓÁË´®¿Ú½ÓÊÕÃüÁî¹¦ÄÜ.
//ÐÞÕýÁËprintfµÚÒ»¸ö×Ö·û¶ªÊ§µÄbug
//V1.4ÐÞ¸ÄËµÃ÷
//1,ÐÞ¸Ä´®¿Ú³õÊ¼»¯IOµÄbug
//2,ÐÞ¸ÄÁËUSART_RX_STA,Ê¹µÃ´®¿Ú×î´ó½ÓÊÕ×Ö½ÚÊýÎª2µÄ14´Î·½
//3,Ôö¼ÓÁËUSART_REC_LEN,ÓÃÓÚ¶¨Òå´®¿Ú×î´óÔÊÐí½ÓÊÕµÄ×Ö½ÚÊý(²»´óÓÚ2µÄ14´Î·½)
//4,ÐÞ¸ÄÁËEN_USART1_RXµÄÊ¹ÄÜ·½Ê½
//V1.5ÐÞ¸ÄËµÃ÷
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//¼ÓÈëÒÔÏÂ´úÂë,Ö§³Öprintfº¯Êý,¶ø²»ÐèÒªÑ¡Ôñuse MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//±ê×¼¿âÐèÒªµÄÖ§³Öº¯Êý                 
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;
       
//¶¨Òå_sys_exit()ÒÔ±ÜÃâÊ¹ÓÃ°ëÖ÷»úÄ£Ê½    
void _sys_exit(int x) 
{ 
	x = x; 
}

//ÖØ¶¨Òåfputcº¯Êý 
int fputc(int ch, FILE *f)
{      
//	while((USART1->SR & 0X40) == 0);//Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
//	USART1->DR = (u8)ch;
//	TXEN = SEND;
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, ch);
//	TXEN = RECEIVE;
	return ch;
}
#endif 

/*Ê¹ÓÃmicroLibµÄ·½·¨*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}

int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //Èç¹ûÊ¹ÄÜÁË½ÓÊÕ
//´®¿Ú1ÖÐ¶Ï·þÎñ³ÌÐò
//×¢Òâ,¶ÁÈ¡USARTx->SRÄÜ±ÜÃâÄªÃûÆäÃîµÄ´íÎó   	



//½ÓÊÕ×´Ì¬
//bit15£¬	½ÓÊÕÍê³É±êÖ¾
//bit14£¬	½ÓÊÕµ½0x0d
//bit13~0£¬	½ÓÊÕµ½µÄÓÐÐ§×Ö½ÚÊýÄ¿
u16 USART_RX_STA = 0;       //½ÓÊÕ×´Ì¬±ê¼Ç	  

//³õÊ¼»¯IO ´®¿Ú1 
//bound:²¨ÌØÂÊ
// SUB
void uart1_init(u32 bound)
{
    //GPIO¶Ë¿ÚÉèÖÃ
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);	//Ê¹ÄÜUSART1£¬GPIOAÊ±ÖÓ
 	USART_DeInit(USART1);  //¸´Î»´®¿Ú1
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				//PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//¸´ÓÃÍÆÍìÊä³ö
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//³õÊ¼»¯PA9
 
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				//PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//¸¡¿ÕÊäÈë
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//³õÊ¼»¯PA10

#if (ARM_MINI || ARM_CM5)
	//RS485_TXEN	PC.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				//PA.2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//ÆÕÍ¨ÍÆÍìÊä³ö
	GPIO_Init(GPIOC, &GPIO_InitStructure);					//³õÊ¼»¯PA2
//	GPIO_SetBits(GPIOC, GPIO_Pin_2);
#endif

#if ARM_TSTAT_WIFI
	//RS485_TXEN	PA.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				//PA.8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//ÆÕÍ¨ÍÆÍìÊä³ö
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//³õÊ¼»¯PA2
	//GPIO_SetBits(GPIOA, GPIO_Pin_8);
#endif
// FOR TINY
	//RS485_TXEN	PD.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				//PD.8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//ÆÕÍ¨ÍÆÍìÊä³ö
	GPIO_Init(GPIOD, &GPIO_InitStructure);					//³õÊ¼»¯PD8
//	GPIO_SetBits(GPIOC, GPIO_Pin_2);
	
	//Usart1 NVIC ÅäÖÃ
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//×ÓÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);								//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷
  
	//USART ³õÊ¼»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound;					//²¨ÌØÂÊÉèÖÃ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//Ò»¸öÍ£Ö¹Î»
	
	
	if(Modbus.uart_parity[0] == 2)
	{
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñ
	}
	else if(Modbus.uart_parity[0] == 1)
	{
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñ
	}
	else
	{
		USART_InitStructure.USART_Parity = USART_Parity_No;			//ÎÞÆæÅ¼Ð£ÑéÎ
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½»	
	}
	// stop bit
//	USART_StopBits_1        0             
// 	USART_StopBits_0_5      1          
// 	USART_StopBits_2        2           
// 	USART_StopBits_1_5  		3
	if(Modbus.uart_stopbit[0] == 1)
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_0_5;		
	}
	else if(Modbus.uart_stopbit[0] == 2)
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_2;		
	}
	else if(Modbus.uart_stopbit[0] == 3)
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_1_5;		
	}
	else
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_1;		
	}
//	
//	if((Modbus.uart_WordLen[0] == 8) && (Modbus.uart_parity[0] != 0))
//	{ 
//		USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñ
//	}
//	else 
//	{		// default 8 bit
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
//	}	
	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//ÊÕ·¢Ä£Ê½
	USART_Init(USART1, &USART_InitStructure); 					//³õÊ¼»¯´®¿Ú

	USART_ITConfig(USART1, USART_IT_RXNE/*|USART_IT_TC*/, ENABLE);				//¿ªÆôÖÐ¶Ï
	USART_Cmd(USART1, ENABLE);                    				
}

// MAIN
void uart3_init(u32 bound)
{
    //GPIO¶Ë¿ÚÉèÖÃ
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);	//Ê¹ÄÜUSART3£¬GPIOAÊ±ÖÓ
 	USART_DeInit(USART3);  //¸´Î»´®¿Ú1
	//USART3_TX   PB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				//PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//¸´ÓÃÍÆÍìÊä³ö
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//³õÊ¼»¯PB10
 
	//USART3_RX	  PB.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				//PB.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//¸¡¿ÕÊäÈë
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//³õÊ¼»¯PB11
	
	//RS485_TXEN	PE.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				//PE.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//ÆÕÍ¨ÍÆÍìÊä³ö
	GPIO_Init(GPIOE, &GPIO_InitStructure);				
//	GPIO_SetBits(GPIOE, GPIO_Pin_11);
	
	//Usart3 NVIC ÅäÖÃ
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//×ÓÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);								//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷
  
	//USART ³õÊ¼»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound;					//²¨ÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//Ò»¸öÍ£Ö¹Î»

	if(Modbus.uart_parity[2] == 2)
	{
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñÊ½
	}
	else if(Modbus.uart_parity[2] == 1)
	{
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñÊ½
	}
	else
	{
		USART_InitStructure.USART_Parity = USART_Parity_No;			//ÎÞÆæÅ¼Ð£ÑéÎ»	
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñÊ½
	}	

	// stop bit
//	USART_StopBits_1        0             
// 	USART_StopBits_0_5      1          
// 	USART_StopBits_2        2           
// 	USART_StopBits_1_5  		3
	if(Modbus.uart_stopbit[2] == 1)
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_0_5;		
	}
	else if(Modbus.uart_stopbit[2] == 2)
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_2;		
	}
	else if(Modbus.uart_stopbit[2] == 3)
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_1_5;		
	}
	else
	{
		USART_InitStructure.USART_StopBits = USART_StopBits_1;		
	}
	
//	if((Modbus.uart_WordLen[2] == 8) && (Modbus.uart_parity[2] != 0))
//	{
//		USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//×Ö³¤Îª9Î»Êý¾Ý¸ñÊ½
//	}
//	else 
//	{
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
//	}
	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//ÊÕ·¢Ä£Ê½
	USART_Init(USART3, &USART_InitStructure); 					//³õÊ¼»¯´®¿Ú

	USART_ITConfig(USART3, USART_IT_RXNE/*|USART_IT_TC*/, ENABLE);				//¿ªÆôÖÐ¶Ï
	USART_Cmd(USART3, ENABLE);                    				
}

// ZIGBEE
void uart2_init(u32 bound)
{
    //GPIO¶Ë¿ÚÉèÖÃ
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);	//Ê¹ÄÜUSART2£¬GPIOAÊ±ÖÓ
 	USART_DeInit(USART2);  //¸´Î»´®¿Ú2
	//USART1_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				//PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//¸´ÓÃÍÆÍìÊä³ö
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//³õÊ¼»¯PA9
 
	//USART1_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				//PA.3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//¸¡¿ÕÊäÈë
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//³õÊ¼»¯PA10

// SELECT RS232 or ZIGBEE by PC3	
	if((Modbus.mini_type == MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
	{
		// if T3_BB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				//PE.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//ÆÕÍ¨ÍÆÍìÊä³ö
	GPIO_Init(GPIOC, &GPIO_InitStructure);		
	}
	
	//Usart2 NVIC ÅäÖÃ
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//×ÓÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);								//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷
  
	//USART ³õÊ¼»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound;					//²¨ÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;			//ÎÞÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//ÊÕ·¢Ä£Ê½
	USART_Init(USART2, &USART_InitStructure); 					//³õÊ¼»¯´®¿Ú

	USART_ITConfig(USART2, USART_IT_RXNE/*|USART_IT_TC*/, ENABLE);				//¿ªÆôÖÐ¶Ï
	USART_Cmd(USART2, ENABLE);                    				
}



#endif
