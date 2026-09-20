#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "bitmap.h"  

//********************************************************************************
//V1.1�޸�˵��
//������STMFLASH_Write������ַƫ�Ƶ�һ��bug.
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
//�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	512//256 	 		//��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 	1              	//ʹ��FLASHд��(0��������;1��ʹ��)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH��ʼ��ַ
#define	STM32_FLASH_BASE 			0x08000000 	//STM32 FLASH����ʼ��ַ
//FLASH������ֵ
#define FLASH_KEY1					0X45670123
#define FLASH_KEY2					0XCDEF89AB
void STMFLASH_Unlock(void);					  //FLASH����
void STMFLASH_Lock(void);					  //FLASH����
u8 STMFLASH_GetStatus(void);				  //���״̬
u8 STMFLASH_WaitDone(u16 time);				  //�ȴ���������
u8 STMFLASH_ErasePage(u32 paddr);			  //����ҳ
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);//д�����
u16 STMFLASH_ReadHalfWord(u32 faddr);		  //��������  
void STMFLASH_WriteLenByte(u32 WriteAddr, u32 DataToWrite, u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr, u16 Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);
void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr, u16 WriteData);	

void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize) ;
void STMFLASH_MUL_Read(u32 ReadAddr, u8 *pBuffer, u16 NumToRead) ;  	
void iap_load_app(u32 appxaddr);
#endif

















