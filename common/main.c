/* Standard includes. */

#include "main.h"



#define Control_STACK_SIZE	((unsigned portSHORT)2048)
#define BACnet_STACK_SIZE	((unsigned portSHORT)2048)
#define TCPIP_STACK_SIZE	((unsigned portSHORT)3000)
//#define USB_STACK_SIZE	   ((unsigned portSHORT)1024)
#define COMMON_STACK_SIZE	  ((unsigned portSHORT)256)
//#define GSM_STACK_SIZE	  ((unsigned portSHORT)512)
#define SampleDISTACK_SIZE  ((unsigned portSHORT)128)
#define SampleAISTACK_SIZE  ((unsigned portSHORT)128)
#define Monitor_STACK_SIZE	((unsigned portSHORT)1000)

void init_panel(void);
void control_logic(void);
void Bacnet_Initial_Data(void);

extern U16_T PT1K_para;
extern uint8_t write_page_en[26];
U8_T Check_Ram_Err(void);
void restore_point_table_from_flash(U8_T table);

void check_task(void);
U8_T cpu_type;
U8_T current_task;
STR_Task_Test far task_test;

xTaskHandle far Handle_SampleDI;
xTaskHandle far Handle_SampleAI;
xTaskHandle far xHandleMSTP;
xTaskHandle xHandleCommon;
xTaskHandle xSoftWatchTask;
xTaskHandle xHandleTcp;
//xTaskHandle far xHandleSchedule;
xTaskHandle far xHandleBacnetControl;
xTaskHandle far xHandleUSB;
xTaskHandle far xHandleMornitor_task;
xTaskHandle far xHandleGSM;
xTaskHandle far xHandleLCD_task;
xTaskHandle far xHandleLedRefresh;

extern  uint8_t SendBuff[SENDBUFF_SIZE];  //??DMA ????
//xQueueHandle xLCDQueue;
xLCDMessage xMessage;

uint16 count_refresh_all = 0;
static uint8 backup_sub_no = 0;
static uint8 backup_current_online_ctr = 0;
static uint32 backup_ether_rx_packet = 0;
static uint32 backup_ether_tx_packet = 0;

#if (ASIX_MINI || ASIX_CM5)

#if (DEBUG_UART1)
U8_T far debug_str[200];
#endif
#endif

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
U32_T ether_rx_packet;	 
U32_T ether_tx_packet;
extern U8_T flag_output;
extern u8 IP_Change;
#endif
//void test_alarm(uint8_t test);
void check_alarm(void);

U8_T etr_reboot;
//uint16_t pdu_len = 0;  
//BACNET_ADDRESS far src;

U8_T far ChangeFlash = 0;
//U8_T far WriteFlash = 0;
U16_T count_write_Flash = 0;
U16_T count_write_E2 = 0;
U8_T flag_Updata_Clock;



U8_T flag_resume_rs485 = 0;  // 0 - intial , 1 - suspend rs485 task  2 - resume rs485 task
U8_T resume_rs485_count = 0;


//const unsigned int code SW_REV = 3401;
U16_T far default_pwm[10];
extern uint8_t prog_loading;
extern uint8_t count_prg_load;

void Bacnet_Control(void) reentrant;

/*
	put E2prom data to buffer when start-up 
*/	

void check_flash_changed(void)
{
	/* RAM ERR guess: ExtSRAM/bus shows 0xFF while flash still has real config.
	 * Check_Ram_Err() only sets bits when flash is recoverable (not 0xFF, not
	 * empty). Then restore that table from flash — do NOT sanitize-to-0.
	 * Incomplete flash writes are handled by flash_finish_pending_commit. */
	if(prog_loading == 0)
	{
		U8_T ram_err = Check_Ram_Err();
		if(ram_err)
		{
			U8_T restored = 0;
			U8_T still;

			Test[38]++; /* detect count this boot */
			if(Test[47] < 255)
				Test[47]++;
			AT24CXX_WriteOneByte(EEP_RAM_ERR, (U8_T)Test[47]); /* persist detect count */

			/* Restore only tables that are not pending host save */
			if((ram_err & 0x01) && (write_page_en[OUT] != 1))
			{
				restore_point_table_from_flash(OUT);
				restored |= 0x01;
			}
			if((ram_err & 0x02) && (write_page_en[IN] != 1))
			{
				restore_point_table_from_flash(IN);
				restored |= 0x02;
			}
			if((ram_err & 0x04) && (write_page_en[VAR] != 1))
			{
				restore_point_table_from_flash(VAR);
				restored |= 0x04;
			}

			still = Check_Ram_Err();
			/* last event: lo=restored tables, hi=detect mask, bit15=still bad */
			Test[45] = (U16_T)restored | ((U16_T)ram_err << 8);
			if(still)
				Test[45] |= 0x8000;
			else if(restored)
				Test[39]++; /* restore succeeded */
			Test[44] = ((U16_T)Rtc.Clk.hour << 8) | Rtc.Clk.min;

			/* Still bad and nothing dirty — reboot without flushing bad RAM */
			if(still
				&& write_page_en[OUT] != 1
				&& write_page_en[IN] != 1
				&& write_page_en[VAR] != 1)
			{
				SoftReset();
			}
		}
	}
	else
	{
		count_prg_load++;
		if(count_prg_load > 20)
			prog_loading = 0;
	}


	if(ChangeFlash != 0)
	{
		uint32_t write_delay;
		if(ChangeFlash == 1)	// normal write
		{
			write_delay = 10;
		}
		else if(ChangeFlash == 3) // write it now
		{
			write_delay = 1;
		}
		else //  ChangeFlash == 2 write it on time
		{// at least 1 hour
			write_delay = Modbus.refresh_flash_timer * 60;
		}
		Test[10]++;
		Test[11] = write_delay;
		Test[12] = count_write_Flash;
		if(count_write_Flash++ > write_delay ) 
		{
			U8_T i;
			U8_T any_dirty = 0;
			Test[13]++;
			Store_Pulse_Counter(1);
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
			/* Timed save (ChangeFlash==2): write only tables marked dirty.
			 * Program put_point_value marks OUT/IN/VAR when values change;
			 * host/network writes also mark dirty with ChangeFlash==1.
			 * Do not force OUT/IN/VAR every period — that caused flash wear
			 * and 0xFF corruption after erase-before-write interruptions. */
			for(i = 0; i < 26; i++)
			{
				if(write_page_en[i])
				{
					any_dirty = 1;
					break;
				}
			}
			if(any_dirty){Test[14]++;
				Flash_Write_Mass();}
#else
			/* ASIX Flash_Write_Mass always rewrites the whole image */
			write_page_en[OUT] = 1;
			write_page_en[IN] = 1;
			write_page_en[VAR] = 1;
			Flash_Write_Mass();
#endif
			
			if(Modbus.refresh_flash_timer == 0)
				ChangeFlash = 0;
			else
				ChangeFlash = 2; //write it on time

			count_write_Flash = 0;
		}
	}
	

}



void Read_ALL_Data(void)
{	
	U8_T  temp[48];
	U8_T  loop;
//	U16_T value;
	/* base infomation */  

	memset((uint8 *)(&Modbus),0,sizeof(STR_MODBUS));

#if (ASIX_MINI || ASIX_CM5)
	IntFlashReadByte(0x5fff,&Modbus.IspVer);
#endif
	
	for(loop = 0;loop < 4;loop++)	
	E2prom_Read_Byte(EEP_SERIALNUMBER_LOWORD + loop,&Modbus.serialNum[loop]);
	
	for(loop = 0;loop < 4;loop++)
	{
		E2prom_Read_Byte(EEP_INSTANCE1 + loop,&temp[loop]);
	}

	Instance = temp[0] + (U16_T)(temp[1] << 8) + ((U32_T)temp[2] << 16) + ((U32_T)temp[3] << 24);
	if(Instance == 0xffffffff || Instance > 0x3fffff)
	{
		Instance =  Modbus.serialNum[0] + (U16_T)(Modbus.serialNum[1] << 8) + ((U32_T)Modbus.serialNum[2] << 16) + ((U32_T)Modbus.serialNum[3] << 24);
	}

	E2prom_Read_Byte(EEP_HARDWARE_REV,&Modbus.hardRev);
	
	for(loop = 0;loop < 4;loop++)
		E2prom_Read_Byte(EEP_REMOTE_SERVER1 + loop,&temp[loop]);
	
#if (ASIX_MINI || ASIX_CM5)
#if REM_CONNECTION
	RM_Conns.ServerIp = temp[0] + (U16_T)(temp[1] << 8) + ((U32_T)temp[2] << 16) + ((U32_T)temp[3] << 24);
#endif
#endif
	
	E2prom_Read_Byte(EEP_ADDRESS,&Modbus.address);
	// if it has not been changed, check the flash memory
	if(( Modbus.address == 255) || ( Modbus.address == 0) )
	{
			Modbus.address = 1;
			E2prom_Write_Byte(EEP_ADDRESS,  Modbus.address);
	}
	
	Station_NUM = Modbus.address;
//	 E2prom_Read_Byte(EEP_STATION_NUM,&Station_NUM);
//	if(Station_NUM == 0 || Station_NUM == 255)
//	{
//		Station_NUM = 1;
//		E2prom_Write_Byte(EEP_STATION_NUM,Station_NUM);
//	}


	
	E2prom_Read_Byte(EEP_EN_NODE_PLUG_N_PLAY,&Modbus.external_nodes_plug_and_play);
	if(Modbus.external_nodes_plug_and_play > 1)
		Modbus.external_nodes_plug_and_play = 0;
#if !(ARM_TSTAT_WIFI)
	E2prom_Read_Byte(EEP_OUTPUT_MODE,&flag_output);
	if(flag_output > 1)
	{
		flag_output = 0; 
		E2prom_Write_Byte(EEP_OUTPUT_MODE,0);
	}
#endif
	E2prom_Read_Byte(EEP_TCP_TYPE,&Modbus.tcp_type);
	if(Modbus.tcp_type >= 2)
	{
		Modbus.tcp_type = 0;
		E2prom_Write_Byte(EEP_TCP_TYPE, 0);	
	}
#if ARM_MINI	
	E2prom_Read_Byte(EEP_CPU_TYPE,&cpu_type);
#endif
	E2prom_Read_Byte(EEP_MINI_TYPE,&Modbus.mini_type);
	if(Modbus.mini_type == 0xff && Modbus.mini_type == 0x00)
	{
		Modbus.mini_type = 1;
	}
	

	if(Modbus.mini_type > MAX_MINI_TYPE)
		Modbus.mini_type = 1;

#if ASIX_MINI
	// ?panel 25??, ? panel 7 ??????? , ????????3.4v
	if((Modbus.mini_type == MINI_BIG) && (Modbus.hardRev >= 25)
		|| (Modbus.mini_type == MINI_SMALL) && (Modbus.hardRev >= 7) )
	{
		default_pwm[0] = 120;
		default_pwm[1] = 225;
		default_pwm[2] = 330;
		default_pwm[3] = 400;
		default_pwm[4] = 465;
		default_pwm[5] = 545;
		default_pwm[6] = 630;
		default_pwm[7] = 715;
		default_pwm[8] = 810;
		default_pwm[9] = 1000;
		
	}
	else 
	{
		default_pwm[0] = 110;
		default_pwm[1] = 220;
		default_pwm[2] = 320;
		default_pwm[3] = 410;
		default_pwm[4] = 480;
		default_pwm[5] = 560;
		default_pwm[6] = 650;
		default_pwm[7] = 740;
		default_pwm[8] = 840;
		default_pwm[9] = 980;
	}
#endif

#if ARM_MINI || ARM_TSTAT_WIFI
		default_pwm[0] = 100;
		default_pwm[1] = 200;
		default_pwm[2] = 300;
		default_pwm[3] = 400;
		default_pwm[4] = 500;
		default_pwm[5] = 600;
		default_pwm[6] = 700;
		default_pwm[7] = 800;
		default_pwm[8] = 900;
		default_pwm[9] = 1000;

#endif	
	
	
#if ARM_MINI || ARM_TSTAT_WIFI

	E2prom_Read_Byte(EEP_ISP_REV,&Modbus.IspVer);
	if(Modbus.IspVer >= 49)  // ARM BOARD,change product id to 74(new arm)if it is old product id 35(old asix)
	{
		if(Modbus.mini_type == MINI_BIG)
		{
			Modbus.mini_type = MINI_BIG_ARM;
		}
		else if(Modbus.mini_type == MINI_SMALL)
		{
			Modbus.mini_type = MINI_SMALL_ARM;
		}
		else if(Modbus.mini_type == MINI_NEW_TINY)
		{
			Modbus.mini_type = MINI_TINY_ARM;
		}			
	}
		

#endif
	
#if ARM_CM5
		E2prom_Read_Byte(EEP_ISP_REV,&Modbus.IspVer);
		Modbus.mini_type = MINI_CM5;
#endif
	

	
	// get number of DO 
	if((Modbus.mini_type == MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))	 {	max_dos = BIG_MAX_DOS; max_aos = BIG_MAX_AOS; }
	else if((Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))		{	max_dos = SMALL_MAX_DOS; max_aos = SMALL_MAX_AOS; }
	else if(Modbus.mini_type == MINI_TINY)		
	{	//max_dos = TINY_MAX_DOS;	max_aos = TINY_MAX_AOS;
		max_dos = 4;
		max_aos = 2;
		if(outputs[4].digital_analog == 0)
			max_dos++;
		else
			max_aos++;
		if(outputs[5].digital_analog == 0)
			max_dos++;	
		else
			max_aos++;		
	}
	else if(Modbus.mini_type == MINI_VAV)		{		max_dos = VAV_MAX_DOS;	max_aos = VAV_MAX_AOS; }
	else if(Modbus.mini_type == MINI_CM5)		{		max_dos = CM5_MAX_DOS;	max_aos = CM5_MAX_AOS; }
	else if((Modbus.mini_type == MINI_NEW_TINY) || (Modbus.mini_type == MINI_TINY_ARM))		{		max_dos = NEW_TINY_MAX_DOS;	max_aos = NEW_TINY_MAX_AOS; }
	else if(Modbus.mini_type == MINI_TINY_11I)		{		max_dos = TINY_11I_MAX_DOS;	max_aos = TINY_11I_MAX_AOS; }
	else if(Modbus.mini_type == MINI_TSTAT10)			{		max_dos = TSTAT10_MAX_DOS;	max_aos = TSTAT10_MAX_AOS;}
	else if(Modbus.mini_type == MINI_T10P )				{		max_dos = T10P_MAX_DOS;	max_aos = T10P_MAX_AOS;}
	else if(Modbus.mini_type == MINI_T3OEM_12I)  	{		max_dos = T3OEM_12I_MAX_DOS;	max_aos = T3OEM_12I_MAX_AOS;}
	else 	{	max_aos = 0; max_dos = 0;	}
	
	if(Modbus.tcp_type != 0 && Modbus.tcp_type != 1)
		Modbus.tcp_type = 0;
	//if( Modbus.tcp_type == 0)  // static ip, read ip address fromm E2prom
	{ 
		for(loop = 0;loop < 4;loop++)
		{
#if (ASIX_MINI || ASIX_CM5)
			E2prom_Read_Byte(EEP_IP + loop,&Modbus.ip_addr[3 - loop]);
			E2prom_Read_Byte(EEP_SUBNET + loop,&Modbus.subnet[3 - loop]);
			E2prom_Read_Byte(EEP_GETWAY + loop,&Modbus.getway[3 - loop]);
#endif
			
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
			E2prom_Read_Byte(EEP_IP + loop,&Modbus.ip_addr[loop]);
			E2prom_Read_Byte(EEP_SUBNET + loop,&Modbus.subnet[loop]);
			E2prom_Read_Byte(EEP_GETWAY + loop,&Modbus.getway[loop]);
			
//			own_ip[loop] = Modbus.ip_addr[loop];

#endif
		}
		
		E2prom_Read_Byte(EEP_FLASH_MAC,&temp[0]);	
		
		if(temp[0] == 0xff || temp[0] == 0)
		{
			E2prom_Write_Byte(EEP_FLASH_MAC,0x55);
			
			// write MAC address	
			//Test[0] = RTC_GetCounter();
			srand(RTC_GetCounter());
			E2prom_Write_Byte(EEP_MAC + 3, rand());
			E2prom_Write_Byte(EEP_MAC + 4, rand());
			E2prom_Write_Byte(EEP_MAC + 5, rand());
		}

		
//printf("mini1 %u %u %u %u\r\n",Modbus.ip_addr[0],Modbus.ip_addr[1],Modbus.ip_addr[2],Modbus.ip_addr[3]);
		for(loop = 0;loop < 6;loop++)
		{
#if (ASIX_MINI || ASIX_CM5)
			E2prom_Read_Byte(EEP_MAC + loop,&Modbus.mac_addr[5 - loop]);
#endif
			
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
			E2prom_Read_Byte(EEP_MAC + loop,&Modbus.mac_addr[loop]);
#endif			
		}
		
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
			if(Modbus.mac_addr[0] != 0)
			{// inverse mac address
				char i;
				for(i = 0;i < 6;i++)
				{
					E2prom_Write_Byte(EEP_MAC, Modbus.mac_addr[5]);
					E2prom_Write_Byte(EEP_MAC + 1, Modbus.mac_addr[4]);
					E2prom_Write_Byte(EEP_MAC + 2, Modbus.mac_addr[3]);
					E2prom_Write_Byte(EEP_MAC + 3, Modbus.mac_addr[2]);
					E2prom_Write_Byte(EEP_MAC + 4, Modbus.mac_addr[1]);
					E2prom_Write_Byte(EEP_MAC + 5, Modbus.mac_addr[0]);
				}
			}
#endif
			
		if((Modbus.ip_addr[0] == 0)  && (Modbus.ip_addr[1] == 0)  && (Modbus.ip_addr[2] == 0) && (Modbus.ip_addr[3] == 0) )
		{		
			Modbus.ip_addr[0] = 192;
			Modbus.ip_addr[1] = 168;
			Modbus.ip_addr[2] = 0;
			Modbus.ip_addr[3] = 3;
		}
		
	}
	
	if(Modbus.com_config[2] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER
		|| Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[0] == BACNET_MASTER)
	{
		Send_I_Am_Flag = 1;
	}
	

	E2prom_Read_Byte(EEP_PORT_LOW,&temp[0]);
	E2prom_Read_Byte(EEP_PORT_HIGH,&temp[1]);

	Modbus.tcp_port = temp[1] * 256 + temp[0];	
	
	if(Modbus.tcp_port == 0xffff || Modbus.tcp_port == 0 )
	{
		Modbus.tcp_port = 502;
		E2prom_Write_Byte(EEP_PORT_LOW,Modbus.tcp_port);
		E2prom_Write_Byte(EEP_PORT_HIGH,Modbus.tcp_port >> 8);
	}

	E2prom_Read_Byte(EEP_BACNET_PORT_LO,&temp[0]);
	E2prom_Read_Byte(EEP_BACNET_PORT_HI,&temp[1]);
	
	Modbus.uart_parity[1] = 0;
	E2prom_Read_Byte(EEP_UART0_PARITY,&Modbus.uart_parity[0]);
	E2prom_Read_Byte(EEP_UART2_PARITY,&Modbus.uart_parity[2]);	
	if(Modbus.uart_parity[0] > 2)
	{	
		Modbus.uart_parity[0] = 0;
		E2prom_Write_Byte(EEP_UART0_PARITY,0);
	}
	if(Modbus.uart_parity[2] > 2)
	{	
		Modbus.uart_parity[2] = 0;
		E2prom_Write_Byte(EEP_UART2_PARITY,0);
	}
//	Modbus.uart_WordLen[1] = 8;
//	E2prom_Read_Byte(EEP_UART0_WORDLEN,&Modbus.uart_WordLen[0]);
//	E2prom_Read_Byte(EEP_UART2_WORDLEN,&Modbus.uart_WordLen[2]);
//	if((Modbus.uart_WordLen[0] != 8) &&  (Modbus.uart_WordLen[0] != 7))
//	{	
//		Modbus.uart_WordLen[0] = 8;
//		E2prom_Write_Byte(EEP_UART0_WORDLEN,Modbus.uart_WordLen[0]);
//	}
//	if((Modbus.uart_WordLen[2] != 8) &&  (Modbus.uart_WordLen[2] != 7))
//	{	
//		Modbus.uart_WordLen[2] = 8;
//		E2prom_Write_Byte(EEP_UART0_WORDLEN,Modbus.uart_WordLen[2]);
//	}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
	Modbus.uart_stopbit[1] = 0;
	E2prom_Read_Byte(EEP_UART0_STOPBIT,&Modbus.uart_stopbit[0]);
	E2prom_Read_Byte(EEP_UART2_STOPBIT,&Modbus.uart_stopbit[2]);
	if(Modbus.uart_stopbit[0] > 2)
	{	
		Modbus.uart_stopbit[0] = 0;
		E2prom_Write_Byte(EEP_UART0_STOPBIT,0);
	}
	if(Modbus.uart_stopbit[2] > 2)
	{	
		Modbus.uart_stopbit[2] = 0;
		E2prom_Write_Byte(EEP_UART2_STOPBIT,0);
	}
//	E2prom_Read_Byte(EEP_UART0_NETWORK,&Modbus.network_ID[0]);
//	E2prom_Read_Byte(EEP_UART1_NETWORK,&Modbus.network_ID[1]);
//	E2prom_Read_Byte(EEP_UART2_NETWORK,&Modbus.network_ID[2]);
#endif	

	Modbus.Bip_port = temp[1] * 256 + temp[0];
	if(Modbus.Bip_port == 0xffff || Modbus.Bip_port == 0 || Modbus.Bip_port == 255)
	{
		Modbus.Bip_port = 0XBAC0;
		E2prom_Write_Byte(EEP_BACNET_PORT_LO,Modbus.Bip_port);
		E2prom_Write_Byte(EEP_BACNET_PORT_HI,Modbus.Bip_port >> 8);
	}
#if OUTPUT_DEATMASTER	
	E2prom_Read_Byte(EEP_DEAD_MASTER,&Modbus.dead_master);
	if(Modbus.dead_master == 255)
	{
		Modbus.dead_master = 0;
	}
#endif
	
#if ARM_TSTAT_WIFI
	E2prom_Read_Byte(EEP_DISABLE_T10_DIS,&Modbus.disable_tstat10_display);
	if(Modbus.disable_tstat10_display == 255)
	{
		Modbus.disable_tstat10_display = 0;
	}	
	E2prom_Read_Byte(EEP_T10_ICON_CONFIG,&Modbus.icon_config);
	if(Modbus.icon_config == 255)
	{
		Modbus.icon_config = 0;
	}	
#endif
	
	E2prom_Read_Byte(EEP_COM0_CONFIG,&Modbus.com_config[0]);
	if((Modbus.com_config[0] != NOUSE) 
		&& (Modbus.com_config[0] != MODBUS_SLAVE) 
		&& (Modbus.com_config[0] != MODBUS_MASTER)
		&& (Modbus.com_config[0] != BACNET_SLAVE) 
		&& (Modbus.com_config[0] != BACNET_MASTER))
	{
		Modbus.com_config[0] = NOUSE;
		E2prom_Write_Byte(EEP_COM0_CONFIG,0);
		com_config_back[0] = Modbus.com_config[0];
	}
	
	E2prom_Read_Byte(EEP_COM1_CONFIG,&Modbus.com_config[1]);
	if((Modbus.com_config[1] != NOUSE) && (Modbus.com_config[1] != MODBUS_SLAVE) && (Modbus.com_config[1] != MODBUS_MASTER) && (Modbus.com_config[1] != RS232_METER))
	{
		Modbus.com_config[1] = NOUSE;
		E2prom_Write_Byte(EEP_COM1_CONFIG,0);
	}
	E2prom_Read_Byte(EEP_COM2_CONFIG,&Modbus.com_config[2]);
	if((Modbus.com_config[2] != NOUSE) && (Modbus.com_config[2] != MODBUS_SLAVE) && (Modbus.com_config[2] != MODBUS_MASTER) 
		&& (Modbus.com_config[2] != BACNET_SLAVE) && (Modbus.com_config[2] != BACNET_MASTER))
	{
		Modbus.com_config[2] = NOUSE;
		E2prom_Write_Byte(EEP_COM2_CONFIG,0);
		com_config_back[0] = Modbus.com_config[0];
	}

	E2prom_Read_Byte(EEP_REFRESH_FLASH,&Modbus.refresh_flash_timer);

	if(Modbus.refresh_flash_timer == 255)
	{
		Modbus.refresh_flash_timer = 0;
		E2prom_Write_Byte(EEP_REFRESH_FLASH,0);
	}
	if(Modbus.refresh_flash_timer > 0 && Modbus.refresh_flash_timer < 60)
		Modbus.refresh_flash_timer = 60;
	
	if(Modbus.refresh_flash_timer != 0)
	{
		/* Schedule periodic persist only — do not erase/rewrite OUT/IN/VAR
		 * right after boot (RAM was just loaded from the same flash). */
		ChangeFlash = 2;
		count_write_Flash = 0;
	}
	else
		ChangeFlash = 0;
		


	if(Modbus.mini_type == MINI_CM5)
	{
		//Modbus.com_config[2] = MODBUS_MASTER;
		Modbus.com_config[1] = NOUSE;
		if((Modbus.com_config[0] != NOUSE) && (Modbus.com_config[0] != MODBUS_SLAVE) && (Modbus.com_config[0] != BACNET_SLAVE))
		{
			Modbus.com_config[0] = NOUSE;
			E2prom_Write_Byte(EEP_COM0_CONFIG,0);
			com_config_back[0] = Modbus.com_config[0];
		}

		if((Modbus.com_config[2] != NOUSE) && (Modbus.com_config[2] != MODBUS_SLAVE) && (Modbus.com_config[2] != MODBUS_MASTER) 
		&& (Modbus.com_config[2] != BACNET_SLAVE) && (Modbus.com_config[2] != BACNET_MASTER))
		{
			Modbus.com_config[2] = NOUSE;
			E2prom_Write_Byte(EEP_COM2_CONFIG,0);
			com_config_back[2] = Modbus.com_config[2];
		}
		
//		Modbus.main_port = 2;
//		Modbus.sub_port = 0;

		uart0_baudrate = UART_19200;
		uart1_baudrate = 0;
		E2prom_Read_Byte(EEP_UART2_BAUDRATE,&uart2_baudrate);
		if(uart2_baudrate == 255)
		{
			uart2_baudrate = UART_19200;
			E2prom_Write_Byte(EEP_UART2_BAUDRATE,uart2_baudrate);	
		}
		
		UART_Init(0);
		UART_Init(2);
	}
	else
	{
		if(Modbus.com_config[0] == 255)
			Modbus.com_config[0] = MODBUS_SLAVE;
		if(Modbus.com_config[1] == 255)
			Modbus.com_config[1] = NOUSE;
		if(Modbus.com_config[2] == 255)
			Modbus.com_config[2] = MODBUS_MASTER;
		 
		if(Modbus.com_config[1] == MODBUS_MASTER)
			count_send_id_to_zigbee = NOUSE;

#if ARM_MINI
	if((Modbus.mini_type == MINI_BIG_ARM) || (Modbus.mini_type == MINI_SMALL_ARM))
	{
		if((Modbus.com_config[1] == MODBUS_MASTER) || (Modbus.com_config[1] == NOUSE))
			UART1_SW = 1;
		else
			UART1_SW = 0;
	}
#endif
	
#if ARM_TSTAT_WIFI
			E2prom_Read_Byte(EEP_COM0_CONFIG,&Modbus.com_config[0]);
			if(Modbus.com_config[0] == 0xff || Modbus.com_config[0] == 0)
				Modbus.com_config[0] = MODBUS_SLAVE;
			
			if((Modbus.com_config[0] != NOUSE) 
				&& (Modbus.com_config[0] != MODBUS_SLAVE) 
				&& (Modbus.com_config[0] != MODBUS_MASTER)
				&& (Modbus.com_config[0] != BACNET_SLAVE) 
				&& (Modbus.com_config[0] != BACNET_MASTER))
			{
				Modbus.com_config[0] = NOUSE;
				E2prom_Write_Byte(EEP_COM0_CONFIG,0);
				com_config_back[0] = Modbus.com_config[0];
			}
#if OLD_BAUD			
			E2prom_Read_Byte(EEP_UART0_BAUDRATE,&uart0_baudrate);  
			if(uart0_baudrate == 255)
			{
				uart0_baudrate = UART_115200;
				E2prom_Write_Byte(EEP_UART0_BAUDRATE,uart0_baudrate);	
			}
#endif
			E2prom_Read_Byte(EEP_UART0_BAUDRATE,&temp[0]);  
			E2prom_Read_Byte(EEP_UART0_BAUDRATE_NEW,&temp[1]);  
			if(temp[1] < UART_BAUDRATE_MAX)
			{			
				uart0_baudrate = temp[1];
				if(temp[0] != temp[1])
				{		
					if(temp[0] == UART_115200) // baudrate is changed by mistake in bootloader
					{// fix the wrong baud
						E2prom_Write_Byte(EEP_UART0_BAUDRATE,uart0_baudrate);
						E2prom_Write_Byte(EEP_RAM_ERR,++Test[47]);
					}
					else if(temp[0] < UART_BAUDRATE_MAX) 
					{// maybe baudrate is changed in bootloader
						uart0_baudrate = temp[0];
						E2prom_Write_Byte(EEP_UART0_BAUDRATE_NEW,uart0_baudrate);
					}
				}			
			}
			else if(temp[0] < UART_BAUDRATE_MAX)
			{// if only old uart0_baud is valid, use old baud
				uart0_baudrate = temp[0];
				E2prom_Write_Byte(EEP_UART0_BAUDRATE_NEW,uart0_baudrate);
			}
			else
			{// if all are broken, use default.
				uart0_baudrate = UART_115200;
				E2prom_Write_Byte(EEP_UART0_BAUDRATE,uart0_baudrate);
				E2prom_Write_Byte(EEP_UART0_BAUDRATE_NEW,uart0_baudrate);
			}
					
			Modbus.com_config[1] = NOUSE;
			Modbus.com_config[2] = NOUSE;	
			uart1_baudrate = 0;
			uart2_baudrate = 0;
			

#endif		
	}	
	
	E2prom_Read_Byte(EEP_FIX_COM_CONFIG,&Modbus.fix_com_config);
	if(Modbus.fix_com_config == 0xff)
	{
		Modbus.fix_com_config = 0;
		E2prom_Write_Byte(EEP_FIX_COM_CONFIG,Modbus.fix_com_config);
	}
	// for get rid of modbus slave and master
	if(Modbus.fix_com_config == 0)
	{
		if(Modbus.com_config[0] == MODBUS_MASTER)
		{
			Modbus.com_config[0] = MODBUS_SLAVE;
			E2prom_Write_Byte(EEP_COM0_CONFIG,MODBUS_SLAVE);
		}
		if(Modbus.com_config[2] == MODBUS_MASTER)
		{
			Modbus.com_config[2] = MODBUS_SLAVE;
			E2prom_Write_Byte(EEP_COM2_CONFIG,MODBUS_SLAVE);
		}
	}	
	com_config_back[0] = Modbus.com_config[0];
	com_config_back[2] = Modbus.com_config[2];
	
#if (ARM_MINI || ASIX_MINI || ARM_TSTAT_WIFI)
	Modbus.start_adc[0] = 0;
#if ADJUST_NG2_AO
	new_start_adc[0][0] = 0;
	new_start_adc[1][0] = 0;
#endif
  for(loop = 0;loop <= 9;loop++)
	{		
#if (ARM_MINI || ARM_TSTAT_WIFI)
		E2prom_Read_Byte(EEP_OUT_1V + loop * 2,&temp[loop * 2]);
		E2prom_Read_Byte(EEP_OUT_1V + loop * 2 + 1,&temp[loop * 2 + 1]);		
		Modbus.start_adc[1 + loop] = (U16_T)temp[loop * 2] * 256 + temp[loop * 2 + 1]; 		
		if((Modbus.start_adc[1 + loop] == 0xffff) || (Modbus.start_adc[1 + loop] == 0))
		{
			E2prom_Write_Byte(EEP_OUT_1V + loop * 2,default_pwm[loop] / 256);
			E2prom_Write_Byte(EEP_OUT_1V + loop * 2 + 1,default_pwm[loop] % 256);
			Modbus.start_adc[1 + loop] = default_pwm[loop];
		}
#if ADJUST_NG2_AO	
		E2prom_Read_Byte(EEP_OUT_1V_2 + loop * 2,&temp[loop * 2]);
		E2prom_Read_Byte(EEP_OUT_1V_2 + loop * 2 + 1,&temp[loop * 2 + 1]);		
		new_start_adc[0][1 + loop] = (U16_T)temp[loop * 2] * 256 + temp[loop * 2 + 1]; 		
		if((new_start_adc[0][1 + loop] == 0xffff) || (new_start_adc[0][1 + loop] == 0))
		{
			E2prom_Write_Byte(EEP_OUT_1V_2 + loop * 2,default_pwm[loop] / 256);
			E2prom_Write_Byte(EEP_OUT_1V_2 + loop * 2 + 1,default_pwm[loop] % 256);
			new_start_adc[0][1 + loop] = default_pwm[loop];
		}
		
		E2prom_Read_Byte(EEP_OUT_1V_3 + loop * 2,&temp[loop * 2]);
		E2prom_Read_Byte(EEP_OUT_1V_3 + loop * 2 + 1,&temp[loop * 2 + 1]);		
		new_start_adc[1][1 + loop] = (U16_T)temp[loop * 2] * 256 + temp[loop * 2 + 1]; 		
		if((new_start_adc[1][1 + loop] == 0xffff) || (new_start_adc[1][1 + loop] == 0))
		{
			E2prom_Write_Byte(EEP_OUT_1V_3 + loop * 2,default_pwm[loop] / 256);
			E2prom_Write_Byte(EEP_OUT_1V_3 + loop * 2 + 1,default_pwm[loop] % 256);
			new_start_adc[1][1 + loop] = default_pwm[loop];
		}
#endif	
#endif
		
#if ASIX_MINI
		E2prom_Read_Byte(EEP_OUT_1V + loop,&temp[loop]);
		Modbus.start_adc[1 + loop] = temp[loop] * 10; 
		if(temp[loop] == 255)
		{
			E2prom_Write_Byte(EEP_OUT_1V + loop,default_pwm[loop] / 10);
			Modbus.start_adc[1 + loop] = default_pwm[loop];
		}
#endif
	}

	cal_slop();
#endif

	E2prom_Read_Byte(EEP_USER_NAME,&Modbus.en_username);
	E2prom_Read_Byte(EEP_CUS_UNIT,&Modbus.cus_unit);

	Modbus.usb_mode = 0;

#if !(ARM_TSTAT_WIFI)	
	E2prom_Read_Byte(EEP_EN_DYNDNS,&Modbus.en_dyndns);
	if(Modbus.en_dyndns == 255)
	{
		Modbus.en_dyndns = 1; 
		E2prom_Write_Byte(EEP_EN_DYNDNS,1);	
	}

	E2prom_Read_Byte(EEP_DYNDNS_PROVIDER,&dyndns_provider);
	if(dyndns_provider > 3)
	{
		dyndns_provider = 3; 
		E2prom_Write_Byte(EEP_DYNDNS_PROVIDER,3);	
	}

	E2prom_Read_Byte(EEP_DYNDNS_UPDATE_LO,&temp[0]);
	E2prom_Read_Byte(EEP_DYNDNS_UPDATE_HI,&temp[1]);
	dyndns_update_time = temp[1] * 256 + temp[0];
	if(dyndns_update_time == 65535)
	{
		dyndns_update_time = 10; 
		E2prom_Write_Byte(EEP_DYNDNS_UPDATE_LO,10);	
		E2prom_Write_Byte(EEP_DYNDNS_UPDATE_HI,0);	
	}
	
	E2prom_Read_Byte(EEP_UART2_BAUDRATE,&uart2_baudrate);
	if(uart2_baudrate == 255)
	{
		uart2_baudrate = UART_19200;
		E2prom_Write_Byte(EEP_UART2_BAUDRATE,uart2_baudrate);	
	}
#if OLD_BAUD	
	E2prom_Read_Byte(EEP_UART0_BAUDRATE,&uart0_baudrate);  
	if(uart0_baudrate == 255)
	{
		uart0_baudrate = UART_19200;
		E2prom_Write_Byte(EEP_UART0_BAUDRATE,uart0_baudrate);	
	}	
#endif
		E2prom_Read_Byte(EEP_UART0_BAUDRATE,&temp[0]);  
		E2prom_Read_Byte(EEP_UART0_BAUDRATE_NEW,&temp[1]); 	
				
		if(temp[1] < UART_BAUDRATE_MAX)
		{			
			uart0_baudrate = temp[1];
			if(temp[0] != temp[1])
			{		
				if(temp[0] == UART_115200) // baudrate is changed by mistake in bootloader
				{// fix the wrong baud
					E2prom_Write_Byte(EEP_UART0_BAUDRATE,uart0_baudrate);
					E2prom_Write_Byte(EEP_RAM_ERR,++Test[47]);
				}
				else if(temp[0] < UART_BAUDRATE_MAX) 
				{// maybe baudrate is changed in bootloader
					uart0_baudrate = temp[0];
					E2prom_Write_Byte(EEP_UART0_BAUDRATE_NEW,uart0_baudrate);
				}
			}			
		}
		else if(temp[0] < UART_BAUDRATE_MAX)
		{// if only old uart0_baud is valid, use old baud
			uart0_baudrate = temp[0];
			E2prom_Write_Byte(EEP_UART0_BAUDRATE_NEW,uart0_baudrate);
		}
		else
		{// if all are broken, use default.
			uart0_baudrate = UART_115200;
			E2prom_Write_Byte(EEP_UART0_BAUDRATE,uart0_baudrate);
			E2prom_Write_Byte(EEP_UART0_BAUDRATE_NEW,uart0_baudrate);
		}
			
	E2prom_Read_Byte(EEP_UART1_BAUDRATE,&uart1_baudrate);
	if(uart1_baudrate == 255)
	{
		uart1_baudrate = UART_19200;
		E2prom_Write_Byte(EEP_UART1_BAUDRATE,uart1_baudrate);	
	} 	
	UART_Init(1);
	UART_Init(2);

	
#endif
	
#if !(ARM_UART_DEBUG)
	UART_Init(0);
#endif



	if(Modbus.mini_type == MINI_VAV)
	{
			UART_Init(1);   // control VAV board by UART1
		//Modbus.com_config[0] = 2;
		Modbus.com_config[1] = RS232_METER;
		//Modbus.com_config[2] = 2;
	}
	
#if (ASIX_MINI || ASIX_CM5)	
	PS0 = 1;
	PS1 = 1;
	PINT3 = 1;


#endif
	
//#if ARM_CM5
//	Modbus.com_config[0] = 2;
//	Modbus.com_config[1] = 0;
//	Modbus.com_config[2] = 7;
//#endif
	for(loop = 0;loop < 12;loop++)
	{
		E2prom_Read_Byte(EEP_SD_BLOCK_HI1 + loop,&temp[24 + loop]);
		if(temp[24 + loop] == 255)
		{
			E2prom_Write_Byte(EEP_SD_BLOCK_HI1 + loop,0);
			temp[24 + loop] = 0;
		}
	}
	
	for(loop = 0;loop < 12;loop++)
	{
		E2prom_Read_Byte(EEP_SD_BLOCK_A1 + loop * 2,&temp[0]);
		E2prom_Read_Byte(EEP_SD_BLOCK_A1 + loop * 2 + 1,&temp[1]);
		
		
		if(temp[0] * 256 + temp[1] == 65535)
		{
			SD_block_num[loop * 2] = 0;
			E2prom_Write_Byte(EEP_SD_BLOCK_A1 + loop * 2,0);
			E2prom_Write_Byte(EEP_SD_BLOCK_A1 + loop * 2 + 1,0);
		}
		else
			SD_block_num[loop * 2] = temp[0] * 256 + temp[1];
		
		SD_block_num[loop * 2] += 65536L * (temp[24 + loop] & 0x0f);
		
	}
	for(loop = 0;loop < 12;loop++)
	{
		E2prom_Read_Byte(EEP_SD_BLOCK_D1 + loop * 2,&temp[0]);
		E2prom_Read_Byte(EEP_SD_BLOCK_D1 + loop * 2 + 1,&temp[1]);
		SD_block_num[loop * 2 + 1] = temp[0] * 256 + temp[1];
		
		if(temp[0] * 256 + temp[1] == 65535)
		{
			SD_block_num[loop * 2 + 1] = 0;
			E2prom_Write_Byte(EEP_SD_BLOCK_D1 + loop * 2,0);
			E2prom_Write_Byte(EEP_SD_BLOCK_D1 + loop * 2 + 1,0);
		}
		else
			SD_block_num[loop * 2 + 1] = temp[0] * 256 + temp[1];
		
		SD_block_num[loop * 2 + 1] += 65536L * (temp[24 + loop] >> 4);
	}
	



	E2prom_Read_Byte(EEP_TIME_ZONE_LO,&temp[0]);
	E2prom_Read_Byte(EEP_TIME_ZONE_HI,&temp[1]);
	if(temp[0] * 256 + temp[1] == 65535)
	{		
		timezone = 800;
		E2prom_Write_Byte(EEP_TIME_ZONE_LO,timezone);
		E2prom_Write_Byte(EEP_TIME_ZONE_HI,timezone >> 8);
	}
	
	E2prom_Read_Byte(EEP_DAYLIGHT_SAVING_TIME,&Daylight_Saving_Time);
	if(Daylight_Saving_Time > 1)
	{
		Daylight_Saving_Time = 0;
		E2prom_Write_Byte(EEP_DAYLIGHT_SAVING_TIME,0);	
	}
	
 	E2prom_Read_Byte(EEP_EN_SNTP,&Modbus.en_sntp);	
	if(Modbus.en_sntp == 255)
	{
		Modbus.en_sntp = 1; 
		E2prom_Write_Byte(EEP_EN_SNTP,1);	
	}	
//	Modbus.en_sntp = 2;	
//	timezone = 800;	//202.120.2.101 24.56.178.101


	if(Modbus.en_sntp > 5)  
	{  // 5 - time server defined by customer 
			Modbus.en_sntp = 2;
	}
		
 
	
	 E2prom_Read_Byte(EEP_PANEL_NUMBER,&panel_number);
	if(panel_number == 0 || panel_number == 255)
	{
#if !(ARM_TSTAT_WIFI)	
		if(Modbus.ip_addr[3] != 0)
		{
			panel_number = Modbus.ip_addr[3];
		}
		else
#endif
			panel_number = 1;
		
		E2prom_Write_Byte(EEP_PANEL_NUMBER,panel_number);
	}
 
  E2prom_Read_Byte(EEP_NETWORK_NUMBER_LO,&temp[0]);
	E2prom_Read_Byte(EEP_NETWORK_NUMBER_HI,&temp[1]);
	Modbus.network_number = temp[1] * 256 + temp[0];
	if(Modbus.network_number == 0)
	{
		Modbus.network_number = 0xffff;
		E2prom_Write_Byte(EEP_NETWORK_NUMBER_LO,Modbus.network_number);
		E2prom_Write_Byte(EEP_NETWORK_NUMBER_HI,Modbus.network_number >> 8);
	}	
	
	E2prom_Read_Byte(EEP_MSTP_NETWORK_LO,&temp[0]);
	E2prom_Read_Byte(EEP_MSTP_NETWORK_HI,&temp[1]);
	mstp_network = temp[1] * 256 + temp[0];
	if(mstp_network == 0 || mstp_network == 0xffff)
	{
		mstp_network = 1;
		E2prom_Write_Byte(EEP_MSTP_NETWORK_LO,mstp_network);
		E2prom_Write_Byte(EEP_MSTP_NETWORK_HI,mstp_network >> 8);
	}	
	
	E2prom_Read_Byte(EEP_SYNC_UDP_SCAN_SEND_LPORT,&temp[0]);	
	if(temp[0] == 1) 
	{
		E2prom_Write_Byte(EEP_SYNC_UDP_SCAN_SEND_LPORT,1);
		udp_scan_lport = 1235/*UDP_SCAN_LPORT*/;
	}
	else
		udp_scan_lport = 1234/*UDP_SCAN_LPORT*/;
	
	
	
	E2prom_Read_Byte(EEP_BBMD_EN,&bbmd_en);
	if(bbmd_en == 255)
	{
		bbmd_en = 0;
		E2prom_Write_Byte(EEP_BBMD_EN,bbmd_en);
	}	
	

	
	E2prom_Read_Byte(EEP_LCD_TIME_OFF_DELAY,&Modbus.LCD_time_off_delay);
//	if(Modbus.LCD_time_off_delay == 255)  
//	{
//		Modbus.LCD_time_off_delay = 30;
//		E2prom_Write_Byte(EEP_LCD_TIME_OFF_DELAY,Modbus.LCD_time_off_delay);
//	}	
	
	E2prom_Read_Byte(EEP_EN_TIME_SYNC_PC,&Modbus.en_time_sync_with_pc);
	if(Modbus.en_time_sync_with_pc == 255)
	{
		Modbus.en_time_sync_with_pc = 0;
		E2prom_Write_Byte(EEP_EN_TIME_SYNC_PC,Modbus.en_time_sync_with_pc);
	}		
	
//	E2prom_Read_Byte(EEP_NETWORK_MASTER,&Modbus.network_master);
//	if(Modbus.network_master == 255)
//	{
//		Modbus.network_master = 0;
//		E2prom_Write_Byte(EEP_NETWORK_MASTER,Modbus.network_master);
//	}
	Modbus.network_master = 0;
	
	for(loop = 0;loop < 4;loop++)
	{
		E2prom_Read_Byte(EEP_SNTP_TIME1 + loop,&temp[loop]);
	}
	update_sntp_last_time = temp[0] + (U16_T)(temp[1] << 8) + ((U32_T)temp[2] << 16) + ((U32_T)temp[3] << 24);

#if ARM_MINI	
	E2prom_Read_Byte(EEP_BAC_VENDOR_ID_LO,&temp[0]);
	E2prom_Read_Byte(EEP_BAC_VENDOR_ID_HI,&temp[1]);
	Bacnet_Vendor_ID = temp[0] + temp[1] * 256;
#endif	
	
	E2prom_Read_Byte(EEP_VCC_ADC_LO,&temp[0]);
	E2prom_Read_Byte(EEP_VCC_ADC_HI,&temp[1]);
	Modbus.vcc_adc = temp[0] + (U16_T)(temp[1] << 8);
	if(Modbus.vcc_adc == 0xffff || Modbus.vcc_adc == 0)
	{		
		Modbus.vcc_adc = 1023;
		E2prom_Write_Byte(EEP_VCC_ADC_LO,Modbus.vcc_adc);
		E2prom_Write_Byte(EEP_VCC_ADC_HI,Modbus.vcc_adc >> 8);
	}
#if ARM_TSTAT_WIFI
	if(Modbus.mini_type == MINI_T3OEM_12I)
	{
		// for PLC, ONLY FOR CALIBRATE AI2, AI3, AI8
		E2prom_Read_Byte(EEP_NEW_VCC_ADC1,&temp[0]);
		E2prom_Read_Byte(EEP_NEW_VCC_ADC1 + 1,&temp[1]);
		new_vcc_adc[0] = temp[0] + (U16_T)(temp[1] << 8);
		if(new_vcc_adc[0] == 0xffff || new_vcc_adc[0] == 0)
		{		
			new_vcc_adc[0] = 1023;
			E2prom_Write_Byte(EEP_NEW_VCC_ADC1,new_vcc_adc[0]);
			E2prom_Write_Byte(EEP_NEW_VCC_ADC1 + 1,new_vcc_adc[0] >> 8);
		}
		
		E2prom_Read_Byte(EEP_NEW_VCC_ADC2,&temp[0]);
		E2prom_Read_Byte(EEP_NEW_VCC_ADC2 + 1,&temp[1]);
		new_vcc_adc[1] = temp[0] + (U16_T)(temp[1] << 8);
		if(new_vcc_adc[1] == 0xffff || new_vcc_adc[1] == 0)
		{		
			new_vcc_adc[1] = 1023;
			E2prom_Write_Byte(EEP_NEW_VCC_ADC2,new_vcc_adc[1]);
			E2prom_Write_Byte(EEP_NEW_VCC_ADC2 + 1,new_vcc_adc[1] >> 8);
		}
		
		E2prom_Read_Byte(EEP_NEW_VCC_ADC3,&temp[0]);
		E2prom_Read_Byte(EEP_NEW_VCC_ADC3 + 1,&temp[1]);
		new_vcc_adc[2] = temp[0] + (U16_T)(temp[1] << 8);
		if(new_vcc_adc[2] == 0xffff || new_vcc_adc[2] == 0)
		{		
			new_vcc_adc[2] = 1023;
			E2prom_Write_Byte(EEP_NEW_VCC_ADC3,new_vcc_adc[2]);
			E2prom_Write_Byte(EEP_NEW_VCC_ADC3 + 1,new_vcc_adc[2] >> 8);
		}
	}
#endif
	
#if ARM_MINI	
	E2prom_Read_Byte(EEP_PT1K_PARA_LO,&temp[0]);
	E2prom_Read_Byte(EEP_PT1K_PARA_HI,&temp[1]);
	PT1K_para = temp[0] + (U16_T)(temp[1] << 8);
	if(PT1K_para == 0xffff)
	{		
		PT1K_para = 10000;
		E2prom_Write_Byte(EEP_PT1K_PARA_LO,PT1K_para);
		E2prom_Write_Byte(EEP_PT1K_PARA_HI,PT1K_para >> 8);
	}
#endif	
	
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
	E2prom_Read_Byte(EEP_MAX_MASTER,&MAX_MASTER);
	if(MAX_MASTER == 0xff || MAX_MASTER <= 1)
	{
		MAX_MASTER = 254;
		E2prom_Write_Byte(EEP_MAX_MASTER,MAX_MASTER);
	}
	
	E2prom_Read_Byte(EEP_EX_MOUDLE_EN,&ex_moudle.enable);
	if((ex_moudle.enable >= 0x55) && (ex_moudle.enable <= 0x65))
	{		
		E2prom_Read_Byte(EEP_EX_MOUDLE_FLAG1,&temp[0]);
		E2prom_Read_Byte(EEP_EX_MOUDLE_FLAG2,&temp[1]);
		E2prom_Read_Byte(EEP_EX_MOUDLE_FLAG3,&temp[2]);
		E2prom_Read_Byte(EEP_EX_MOUDLE_FLAG4,&temp[3]);
		
		ex_moudle.flag = temp[0] + (U16_T)(temp[1] << 8) + ((U32_T)temp[2] << 16) + ((U32_T)temp[3] << 24);

		if(ex_moudle.flag == 0xffff)
		{
			ex_moudle.flag = 0;
			E2prom_Write_Byte(EEP_EX_MOUDLE_FLAG1,0);
			E2prom_Write_Byte(EEP_EX_MOUDLE_FLAG2,0);
			E2prom_Write_Byte(EEP_EX_MOUDLE_FLAG3,0);
			E2prom_Write_Byte(EEP_EX_MOUDLE_FLAG4,0);
			
		}
	}
    
#endif
	E2prom_Read_Byte(EEP_JASON,&webview_json_flash);
	
	E2prom_Read_Byte(EEP_VRESION_HI,&temp[0]);
	E2prom_Read_Byte(EEP_VERSION_LO,&temp[1]);

	if(temp[1] + (U16_T)(temp[0] * 100) != SW_REV)
	{
		E2prom_Write_Byte(EEP_VRESION_HI,SW_REV / 100);
		E2prom_Write_Byte(EEP_VERSION_LO,SW_REV % 100);
	}
#if ARM_MINI || ARM_TSTAT_WIFI
	E2prom_Read_Byte(EEP_DLS_START_MON,&Modbus.start_month);
	if(Modbus.start_month == 0xff || Modbus.start_month == 0x00)
	{
		Modbus.start_month = 3;
	}
	E2prom_Read_Byte(EEP_DLS_START_DAY,&Modbus.start_day);
	if(Modbus.start_day == 0xff || Modbus.start_day == 0x00)
	{
		Modbus.start_day = 14;
	}
	E2prom_Read_Byte(EEP_DLS_END_MON,&Modbus.end_month);
	if(Modbus.end_month == 0xff || Modbus.start_month == 0x00)
	{
		Modbus.end_month = 11;
	}
	E2prom_Read_Byte(EEP_DLS_END_DAY,&Modbus.end_day);
	if(Modbus.end_day == 0xff || Modbus.end_day == 0x00)
	{
		Modbus.end_day = 7;
	}
	Calculate_DSL_Time();
#endif
	
	E2prom_Read_Byte(EEP_ETR_REBOOT,&temp[0]);
	Test[49] = temp[0];
	
	E2prom_Read_Byte(EEP_TEST1,&temp[0]);
	Test[48] = temp[0];
	
	E2prom_Read_Byte(EEP_RAM_ERR,&temp[0]);
	Test[47] = temp[0];
}


void set_default_parameters(void)
{
	char loop;
	E2prom_Write_Byte(EEP_PORT_LOW,502);
	E2prom_Write_Byte(EEP_PORT_HIGH,502 >> 8);

	E2prom_Write_Byte(EEP_ADDRESS,1);
	E2prom_Write_Byte(EEP_EN_NODE_PLUG_N_PLAY,1);

	E2prom_Write_Byte(EEP_UART0_PARITY,0);
	E2prom_Write_Byte(EEP_UART2_PARITY,0);
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
	E2prom_Write_Byte(EEP_UART0_STOPBIT,0);
	E2prom_Write_Byte(EEP_UART2_STOPBIT,0);
#endif
	E2prom_Write_Byte(EEP_SUSPEND_MSTP,0);	
	E2prom_Write_Byte(EEP_USER_NAME,0);
	E2prom_Write_Byte(EEP_CUS_UNIT,0);
//	E2prom_Write_Byte(EEP_USB_MODE,0);

// clear SD_NUMBER
	for(loop = 0;loop < 12;loop++)
	{
		E2prom_Write_Byte(EEP_SD_BLOCK_HI1 + loop,0);
		E2prom_Write_Byte(EEP_SD_BLOCK_A1 + loop * 2,0);
		E2prom_Write_Byte(EEP_SD_BLOCK_A1 + loop * 2 + 1,0);
		E2prom_Write_Byte(EEP_SD_BLOCK_D1 + loop * 2,0);
		E2prom_Write_Byte(EEP_SD_BLOCK_D1 + loop * 2 + 1,0);
	}
	
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
	E2prom_Write_Byte(EEP_REFRESH_FLASH, 0 );	  // 5min
#endif
	
#if ARM_TSTAT_WIFI
	E2prom_Write_Byte(EEP_DISABLE_T10_DIS, 0 );	
#endif
	
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
	E2prom_Write_Byte(EEP_MAX_MASTER,254);
	E2prom_Write_Byte(EEP_DEAD_MASTER,0);
#endif	
	
	
#if (ASIX_MINI || ASIX_CM5)
	
	E2prom_Write_Byte(EEP_REFRESH_FLASH, 5 );	  // 5min
	
//	E2prom_Write_Byte(EEP_NO_USED80,++Test[41]);
	IntFlashErase(ERA_RUN,0x70000);	
	IntFlashWriteByte(0x4001,0);					
	AX11000_SoftReboot();  
//	Test[41] = 4;
#endif 
	
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
// erase all flash for user	
	Bacnet_Initial_Data();	
	
	__disable_irq();
	STMFLASH_Unlock();
					
	for(loop = 0;loop < 64;loop++)
	{
		STMFLASH_ErasePage(0x8060000 + 2048 * loop);	
	}
	
	STMFLASH_Lock();
	__enable_irq();

    for (loop = 0; loop < 26; loop++)
    {
        write_page_en[loop] = 1;
    }
    ChangeFlash = 3;

  flag_reboot = 1;
	
#endif

}



uint8_t far PDUBuffer[MAX_APDU];


//void Schedule_Init(void);

static void Init_Service_Handlers(
    void)
{
    //Device_Init(NULL);
    /* we need to handle who-is
       to support dynamic device binding to us */
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_WHO_IS, handler_who_is);
    /* handle i-am to support binding to other devices */
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_I_AM, handler_i_am_add);
    /* set the handler for all the services we don't implement
       It is required to send the proper reject message... */
    apdu_set_unrecognized_service_handler_handler
        (handler_unrecognized_service);
    /* we must implement read property - it's required! */
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROPERTY,
        handler_read_property);
		apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROP_MULTIPLE,
					handler_read_property_multiple);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_WRITE_PROPERTY,
        handler_write_property);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_WRITE_PROP_MULTIPLE,
        handler_write_property_multiple);
	
	// add more service about COV
#if COV
		apdu_set_confirmed_handler(SERVICE_CONFIRMED_SUBSCRIBE_COV,
        handler_cov_subscribe);
		apdu_set_confirmed_handler(SERVICE_CONFIRMED_COV_NOTIFICATION,
        handler_ccov_notification);
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_COV_NOTIFICATION,
        handler_ucov_notification);
#endif			
  
//		apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_RANGE,
//        handler_read_range);
//		Test[10] = 100;
}


void Inital_Bacnet_Server(void)
{
	if(((panelname[0] == 0) && (panelname[1] == 0) && (panelname[2] == 0))  || 
		((panelname[0] == 255) && (panelname[1] == 255) && (panelname[2] == 255)) )
	{
#if ARM_MINI || ASIX_MINI
		if((Modbus.mini_type == MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
			Set_Object_Name("T3-BB");
		else if((Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))
			Set_Object_Name("T3-LB");
		if((Modbus.mini_type == MINI_TINY) || (Modbus.mini_type == MINI_TINY_ARM)|| (Modbus.mini_type == MINI_TINY_ARM)) 
			Set_Object_Name("T3-TB");
		if(Modbus.mini_type == MINI_NANO) 
			Set_Object_Name("T3-NB");
#endif
		
#if ARM_CM5
		Set_Object_Name("BC Controller");
#endif
		
#if  ARM_TSTAT_WIFI
		if(Modbus.mini_type == MINI_TSTAT10) 
			Set_Object_Name("Tstat10");
		else if(Modbus.mini_type == MINI_T10P) 
			Set_Object_Name("T3-OEM");
		else if(Modbus.mini_type == MINI_T3OEM_12I)
			Set_Object_Name("T3-OEM-12I");
			
#endif

	}
	else
		Set_Object_Name(panelname);
	Device_Init();
	Init_Service_Handlers();
	Device_Set_Object_Instance_Number(Instance);  

	address_init();
#if !(ARM_TSTAT_WIFI )
	{
		U32_T bip_broadcast_addr;

		bip_broadcast_addr = ((U32_T)(Modbus.ip_addr[0] | (255 - Modbus.subnet[0])) << 24)
			| ((U32_T)(Modbus.ip_addr[1] | (255 - Modbus.subnet[1])) << 16)
			| ((U32_T)(Modbus.ip_addr[2] | (255 - Modbus.subnet[2])) << 8)
			| (U32_T)(Modbus.ip_addr[3] | (255 - Modbus.subnet[3]));
		bip_set_broadcast_addr(bip_broadcast_addr);
	}
#endif

#if BBMD_ENABLED
	if(bbmd_en == 1)
		bvlc_intial();
#endif
	
#if  BAC_COMMON   



#if BAC_SCHEDULE
	SCHEDULES = 8;
#endif
	
	
#if BAC_CALENDAR
	CALENDARS = 4;
#endif
	
#if BAC_TRENDLOG
	TRENDLOGS = 8;
	Trend_Log_Init();
#endif
	
#if BAC_TRENDLOG_MUL
	TRENDLOGS_MUL = 2;
	Trend_Log_Mul_Init();
#endif

#if BAC_PROPRIETARY

#if ARM_TSTAT_WIFI
		TemcoVars = 14;
#else	
	// add initial code
	TemcoVars = 5;
#endif

#endif

#if BAC_MSV
#if ARM_TSTAT_WIFI
		MSVS = 3;
#endif
#endif	

	Count_IN_Object_Number();
	Count_OUT_Object_Number();
	Count_VAR_Object_Number();
#endif	
}

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
//uint16_t count_send_whois = 0;
extern uint8_t count_hold_on_bip_to_mstp;
bool dcc_communication_initial_disabled(void);
int Send_private_scan(U8_T index);


uint8_t flag_suspend_mstp;
uint16_t count_suspend_mstp;
uint16_t count_start_task = 0;
void Switch_protocal_to_modbus(void);
uint8_t shutdown_Time;
bool Send_shutdown_Flag;
int count_update_mstp_db;
uint8_t upate_mstp_flag;
uint32_t shutdown_start_time;
uint32_t shutdown_send_time;
void Send_Shutdown(void);

uint8_t flag_start_scan_mstp;
uint8_t start_scan_mstp_count;
uint16_t Master_Scan_Mstp_Count;

void clear_count_whois(void)
{
	count_start_task = 1;
}

void set_mstp_master(void)
{
	
	Modbus.mstp_master = 0;
	Master_Scan_Mstp_Count = 0;
}

uint16_t count_show_mstp_err;
void Master_Node_task(void) reentrant
{
	portTickType xDelayPeriod  = ( portTickType ) 10 / portTICK_RATE_MS; // 1000
	uint16_t pdu_len = 0;  
	BACNET_ADDRESS far src; /* source address */
//	U8_T remote_bacnet_index;
	uint16_t Max_count_Mstp = 3000;
	U8_T i;
	int invoke;
	
	
	/* initialize datalink layer */	   	
	task_test.enable[7] = 1;
  dlmstp_init(NULL);
//	dcc_communication_initial_disabled();
	remote_bacnet_index = 0;
	pdu_len = 0;
	if(Modbus.com_config[2] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER)
	{
		Recievebuf_Initialize(2);
		Send_I_Am_Flag = 1;
		Send_Whois_Flag = 1;	
	}
	else if(Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[0] == BACNET_MASTER)
	{
		Recievebuf_Initialize(0);
		Send_I_Am_Flag = 1;
		Send_Whois_Flag = 1;	
	}
	Modbus.mstp_master = 1;
	Master_Scan_Mstp_Count = 0;
	
	count_start_task = 0;
	flag_receive_rmbp = 0;
	
	count_hold_on_bip_to_mstp = 0;
// UART2 have higher priority
	// check whether suspent mstp
#if MSTP_UPDATE	
	{
		uint8 temp;
		E2prom_Read_Byte(EEP_SUSPEND_MSTP,&temp);
		if(temp == 0xff)
		{
			E2prom_Write_Byte(EEP_SUSPEND_MSTP,0);
			flag_suspend_mstp = 0;
			count_suspend_mstp = 0;
		}
		else
		{
			if(temp & 0x80) 
			{
				flag_suspend_mstp = 1;
				count_suspend_mstp = (temp - 0x80 + 1) * 60;
				Switch_protocal_to_modbus();
			}
			else
			{
				flag_suspend_mstp = 0;	
				count_suspend_mstp = 0;
			}
		}
		shutdown_Time = 5;
		Send_shutdown_Flag = 0;
		count_update_mstp_db = -1;
		upate_mstp_flag = F_INITIAL;
		shutdown_send_time = 0;
	}
#endif

	flag_mstp_err[0] = flag_mstp_err[2] = 1;
	count_mstp_err[0] = count_mstp_err[2] = 0;
	count_show_mstp_err = 0;
	uart_serial_restart(0);
	for(;;)
	{
		if(flag_initial_uart == 1)
		{
			UART_Init(0);
			if(Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[0] == BACNET_MASTER)
			{				
				uart0_rece_count = 0;	
				count_mstp_err[0] = 0;
				count_show_mstp_err = 0;
				flag_mstp_err[0] = 1;
				Set_TXEN(0);// set mstp prot to RECEIVE
			}		
			if((Modbus.com_config[0] == MODBUS_SLAVE) || (Modbus.com_config[0] == 0))
				uart_serial_restart(0);
		
			flag_initial_uart = 0;
		}
		else if(flag_initial_uart == 2)
		{
			UART_Init(1);
			flag_initial_uart = 0;
		}
		else if(flag_initial_uart == 3)
		{
			UART_Init(2);
			if(Modbus.com_config[2] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER)
			{				
				uart2_rece_count = 0;	
				count_mstp_err[2] = 0;
				flag_mstp_err[2] = 1;	
				Set_TXEN(0);// set mstp prot to RECEIVE
			}		
			if((Modbus.com_config[2] == MODBUS_SLAVE) || (Modbus.com_config[2] == 0))
				uart_serial_restart(2);
			flag_initial_uart = 0;
		}	
		
		task_test.count[7]++; 
		current_task = 7;

		if(Master_Scan_Mstp_Count++ >= 12000)
		{  // did not find master 
			// current panel is master
			Modbus.mstp_master = 1;						
			Master_Scan_Mstp_Count = 0;	

		}
		
#if MSTP_UPDATE		
		// 10 minues later, resume mstp port
		if((run_time - shutdown_start_time > 600) && (shutdown_start_time == 1))
		{
			upate_mstp_flag = 0;
			
			flag_suspend_mstp = 0;
			count_suspend_mstp = 0;
			E2prom_Write_Byte(EEP_SUSPEND_MSTP,0);
			// resume mstp port
			resume_mstp_port();
		}		
		if(upate_mstp_flag != 0)
		{
			if((Modbus.com_config[0] != BACNET_MASTER) && (Modbus.com_config[0] != BACNET_SLAVE ) \
				&&(Modbus.com_config[2] != BACNET_MASTER) && (Modbus.com_config[2] != BACNET_SLAVE))
			{
				count_start_task++;
				delay_ms(5);
			}
			if(upate_mstp_flag == F_START_SHUTDOWN)
			{
				if(run_time - shutdown_start_time > 30)
				{
					upate_mstp_flag = F_TIMEOUT;
					
					flag_suspend_mstp = 0;
					count_suspend_mstp = 0;
					E2prom_Write_Byte(EEP_SUSPEND_MSTP,0);
					// resume mstp port
					resume_mstp_port();
				}
				if(Modbus.com_config[0] == BACNET_MASTER || Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[2] == BACNET_SLAVE)
				{
					if((count_update_mstp_db > 0) && (count_start_task % 300 == 0))
					{			
						//shutdown_Time = 5;
						Send_shutdown_Flag = 1;
						shutdown_send_time = run_time;
						
						count_update_mstp_db = -1;
					}
				}
			
				if(count_update_mstp_db == -1)
				{
					uint8_t check_time;
					if(Modbus.com_config[0] == BACNET_MASTER || Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[2] == BACNET_SLAVE)
						check_time = 5;
					else
						check_time = 10;
					if(run_time - shutdown_send_time > check_time)
					{							
						if(Modbus.com_config[2] == BACNET_MASTER)
						{
							Modbus.com_config[2] = MODBUS_MASTER;
							Setting_Info.reg.com_config[2] = MODBUS_MASTER;	
						}					
										
						
						if(Modbus.com_config[0] == BACNET_MASTER)
						{
							Modbus.com_config[0] = MODBUS_MASTER;
							Setting_Info.reg.com_config[0] = MODBUS_MASTER;
						}						
						Send_Shutdown();
						delay_ms(1000);
						Send_Shutdown();
						delay_ms(1000);
						Send_Shutdown();
						shutdown_send_time = 0;
						upate_mstp_flag = F_SUCCESS;
					}
				}	
			}				
		}
#endif

		if(Modbus.com_config[0] == BACNET_MASTER || Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[2] == BACNET_SLAVE)
		{
//			count_send_whois++;
			vTaskDelay( 5 / portTICK_RATE_MS);			
				
			if(count_start_task % 12000 == 0)	// 1 min
			{
				//if(((Modbus.mini_type >= MINI_BIG_ARM) && (Modbus.mini_type <= MINI_NANO)) || (Modbus.mini_type == MINI_TINY_11I))
				{
					//if(Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[0] == BACNET_MASTER)
					if(Modbus.mstp_master == 1 || flag_start_scan_mstp == 1)
					{
						if(upate_mstp_flag == 0)	
						{							
								Send_Whois_Flag = 1;
						}
						if((flag_start_scan_mstp++ > 2))
						{
							start_scan_mstp_count = 0;
							flag_start_scan_mstp = 0;
						}		
					}						
				}
				count_start_task = 0;
			}
			else
			{ 	 // whether exist remote mstp point
				//if(((Modbus.mini_type >= MINI_BIG_ARM) && (Modbus.mini_type <= MINI_NANO)) || (Modbus.mini_type == MINI_TINY_11I))
				{	
					if(Modbus.mstp_master == 1 || flag_start_scan_mstp == 1)//if(Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[0] == BACNET_MASTER)					
					{
						if((count_start_task % 200 == 0) && (upate_mstp_flag == 0)) // 1.5s
						{
							// check whether the device is online or offline
							if(flag_receive_rmbp == 1)
							{
								U8_T remote_panel_index;
								if(Get_rmp_index_by_panel(remote_points_list[remote_bacnet_index].point.panel,
								remote_points_list[remote_bacnet_index].point.sub_id, 
								&remote_panel_index,
								BAC_MSTP) != -1)
								{
									remote_panel_db[remote_panel_index].time_to_live = RMP_TIME_TO_LIVE;
								}
								remote_points_list[remote_bacnet_index].lose_count = 0;
								remote_points_list[remote_bacnet_index].decomisioned = 1;	
							}
							else
							{
								remote_points_list[remote_bacnet_index].lose_count++;
								if(remote_points_list[remote_bacnet_index].lose_count > 10)
								{
									remote_points_list[remote_bacnet_index].lose_count = 0;
									remote_points_list[remote_bacnet_index].decomisioned = 0;	
								}
							}					
							
						// read remote mstp points
							remote_bacnet_index = find_next_remote_bacnet_point(remote_bacnet_index);

							
							{  // read private modbus from Temco product
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
									static char j = 0;
									char count;
									if(j < remote_panel_num)
									{
										if(remote_panel_db[j].protocal == BAC_MSTP 
											&& remote_panel_db[j].sn == 0)
										{
											remote_panel_db[j].retry_reading_panel++;
											flag_receive_rmbp = 0;									
											invoke = Send_private_scan(j);
											remote_mstp_panel_index = j;
											while((flag_receive_rmbp == 0) && count++ < 20)
												delay_ms(200);	

											// add this condition to avoid reading remote points at same time, if that maybe cause conlict.
											remote_bacnet_index = 0xff;
										}
										if(remote_panel_db[j].retry_reading_panel > 5)
										{
											remote_panel_db[j].sn = remote_panel_db[j].device_id;
											remote_panel_db[j].retry_reading_panel = 0;
											remote_panel_db[j].product_model = 0;
										}								
									}
									j++;
									
									if(j > remote_panel_num) 
										j = 0;									
								
#endif
							}
								
							if(remote_bacnet_index != 0xff)
							{
								if(number_of_remote_points_bacnet > 0)
								{
									// read remote bacnet point
									//if(Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[0] == BACNET_MASTER)
									if(Modbus.mstp_master == 1 || flag_start_scan_mstp == 1)
									{
										flag_receive_rmbp = 0;
										invoke = GetRemotePoint(remote_points_list[remote_bacnet_index].tb.RP_bacnet.object,
													remote_points_list[remote_bacnet_index].tb.RP_bacnet.instance,
													panel_number,/*Modbus.network_ID[2],*/
													remote_points_list[remote_bacnet_index].tb.RP_bacnet.panel ,
													BAC_MSTP);
										// check whether the device is online or offline	
										
										if(invoke >= 0)
										{
											remote_points_list[remote_bacnet_index].invoked_id	= invoke;
										}
										else
										{
											remote_points_list[remote_bacnet_index].lose_count++;								
										}
									}
								}								
							}
						}
					}		
				}				
			}		
			
			count_start_task++;
			if(flag_suspend_mstp == 0)
			{			
				pdu_len = datalink_receive(&src, &PDUBuffer[0], sizeof(PDUBuffer), 0,BAC_MSTP);
				{ 								
					if(pdu_len) 
					{
						npdu_handler(&src, &PDUBuffer[0], pdu_len,BAC_MSTP);	
					} 					
				}			
			}
		}
		else
		{			
				if(upate_mstp_flag == 0)
					delay_ms(5000);
		}				
  }	
}

#endif

void TCP_IP_Init(void);
	
void store_buffer_to_SD(void);
//void RM_send_information(void);


U32_T count_sntp = 0;
U8_T flag_Update_Sntp;
U8_T Update_Sntp_Retry;
U8_T flag_Update_Dyndns;
U8_T Update_Dyndns_Retry;
void Get_Time_by_sec(u32 sec_time,UN_Time * rtc,uint8_t flag);
#define MAX_DDNS_RETRY_COUNT 5
void Common_task(void) reentrant
{
	static U8_T count = 0;
//	char text[20];		
	portTickType xDelayPeriod = ( portTickType ) 250 / portTICK_RATE_MS;
	U8_T i = 0;
	
	backup_sub_no = 0;
  backup_current_online_ctr = 0;
	backup_ether_rx_packet = 0;
	backup_ether_tx_packet = 0;
	count_refresh_all = 0;
	flag_Update_Sntp = 0;
	Update_Sntp_Retry = 0;
	count_sntp = 0;

//	portTickType xLastWakeTime = xTaskGetTickCount();
	task_test.enable[1] = 1;
#if (ASIX_MINI || ASIX_CM5)	
	count_write_E2 = 0;
#endif
	count_write_Flash = 0;
//	ChangeFlash = 0;
	
	for (;;)
	{	
		vTaskDelay(250 / portTICK_RATE_MS);
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
		if(task_test.count[0] == 0)  // if tcptask is not running, need watchdog in lower task
			IWDG_ReloadCounter(); 
#endif
		task_test.count[1]++;
		current_task = 1;
#if 0
#if STORE_TO_SD		

		store_buffer_to_SD();
#endif		

#endif	
	
		if(count % 4 == 0)  // 1 second
		{	

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)	//!(ARM_TSTAT_WIFI  )
			update_sntp();
#if (ARM_MINI || ASIX_MINI || ASIX_CM5)
			PIC_refresh();	
			
			if((Modbus.mini_type == MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
			{								
// Check LCD				
				Check_Lcd();				
			}
#endif
#endif	
			
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)	
			RTC_Get();
#endif
			
#if (ASIX_MINI || ASIX_CM5)	
			
#if (ARM_MINI || ASIX_MINI)
			if(Modbus.mini_type == MINI_VAV)
				LED_BEAT = ~LED_BEAT;	
#endif			
			
#endif
			
		
		}	
		count++; 	
	
		
#if (ARM_CM5 || ASIX_CM5)
		if(count % 4 == 0)  // 10 second
		{	
			Check_Lcd();				
		}
#endif

	}
}



#if (ASIX_MINI || ASIX_CM5)
void watchdog(void)
{
	/*software watchdog */
	#if AX_WATCHDOG_ENB
		TA = 0xAA;
		TA = 0x55;
		RWT=1;
	#endif
}



void SoftwareWatchdog_task(void) reentrant
{	
	portTickType xDelayPeriod = ( portTickType ) 100 / portTICK_RATE_MS;
	for(;;)
	{
		vTaskDelay(xDelayPeriod);
		/* clear watch dog */
		watchdog();
	}
}
#endif




U8_T flag_reset_default;   
U8_T flag_reboot;
U8_T ether_tx_inactive_count;
U8_T check_input_alarm_count;
void SET_VAV(U8_T level);
#if (ARM_MINI || ARM_CM5)
void tcpip_intial(void);
#endif
void Check_Program_Output_Pri_valid(void);
void Check_Send_bip(void);
void Check_Remote_Panel_Table(void);
void check_whether_suspend_mstp(void);
void Check_LCD_time_off(void);
void Check_Whether_TCP_STUCK(void);
void Check_TCP_UDP_Socket(uint16_t bip_port);
void tcpip_health_check(void);
void check_net_health(uint8_t interval);
uint8_t check_msv_data_len(uint8_t index);

// ???????
void check_override_timer_1s(void);
void check_transfer_bip_to_mstp(void);
void check_private_sending(void);
void Email_Task(void);
void ESP8266_Rst( void );

void Update_Array(void)
{
		u8 r;
	//arrays_data[0] = 1;// DAY_NIGHT
		r = arrays_data[0] / 1000;
		Modbus.icon_config &= 0xfc;
		if(r <= 3)
			Modbus.icon_config |= r;
				
		//arrays_data[1] = 2; // OCC_UNOCC
		r = arrays_data[1] / 1000;
		Modbus.icon_config &= 0xf3;
		if(r <= 3)
			Modbus.icon_config |= (r << 2);
				
		//arrays_data[2] = 1; // HEAT_COOL
		r = arrays_data[2] / 1000;
		Modbus.icon_config &= 0xcf;
		if(r <= 3)
			Modbus.icon_config |= (r << 4);
				
				
		//arrays_data[3] = 3; // FAN	
		r = arrays_data[3] / 1000;		
		Modbus.icon_config &= 0x3f;
		if(r <= 3)
			Modbus.icon_config |= (r << 6);
		
		E2prom_Write_Byte(EEP_T10_ICON_CONFIG,Modbus.icon_config);
}

void Test_Array(void)
{
	long *p;

	if(Modbus.mini_type == MINI_TSTAT10 || Modbus.mini_type == MINI_T10P)
	{
			memcpy(arrays[0].label,"ICON",9);
			arrays[0].length = 4;
			arrays_address[0] = &arrays_data[0];		
	
	}
	
}
#if COV
int send_cov_demo(void);
void handler_cov_task(uint8_t protocal);
#endif
void Monitor_Task_task(void) reentrant
{	
	portTickType xDelayPeriod = ( portTickType ) 1000 / portTICK_RATE_MS;
	U8_T loop;

	static U16_T check_sd = 0;
	static U8_T DYNDNS_TIMER = 0; // time  

	ether_tx_inactive_count = 0;
	flag_reset_default = 0;
	flag_reboot = 0;
#if (ASIX_MINI || ASIX_CM5)
	flag_udp_scan = 1;	
	flag_Updata_Clock = 1;	
	//Updata_Clock(0);
#endif
	flag_Update_Dyndns = 0;
	Update_Dyndns_Retry = 0;
	task_test.enable[14] = 1;
	flag_resume_rs485 = 0;
	resume_rs485_count = 0;
	check_sd = 0;
	Device_Set_Object_Instance_Number(Instance);
	count_reintial_tcpip = 600;
	Test_Array();
//	handler_cov_init();
	for(;;)
	{	
		
		vTaskDelay(1000 / portTICK_RATE_MS);
		task_test.count[14]++;	
		current_task = 14;
		Check_Program_Output_Pri_valid();	
		
#if SAVE_LOCAL_VAR_TO_E2
		write_local_val_to_E2();
#endif		

#if COV		
//		if(Test[0] >= 1000)
//		{
//			//handler_cov_init();
//			send_cov_demo();
//			Test[0] = 0;
//		}				
		handler_cov_task(BAC_IP_CLIENT);	
				
#endif
		
#if BBMD_ENABLED
		if(Test[30] == 100)
		{
		dlenv_maintenance_timer(1);}
#endif		
		

		Test[8] = ether_rx_packet;
		Test[9] = ether_tx_packet;
		if((ether_rx_packet > 999999) || (ether_tx_packet > 999999))
		{
			ether_rx_packet = 0;
			ether_tx_packet = 0;
		}
		Check_LCD_time_off();	
		check_mstp_traffic();
#if ARM_TSTAT_WIFI
		check_override_timer_1s();
		Check_identify_tstat10();
#endif

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
#if (ARM_MINI || ARM_CM5)		
		check_private_sending();
		Check_Whether_TCP_STUCK();		
		Check_TCP_UDP_Socket(Modbus.Bip_port);
		check_net_health(60);
		// for T3-XB
		check_modbus_slave();
		
#if REBOOT_PER_WEEK
		
		if((Rtc.Clk.week == 0) && (Rtc.Clk.hour == 2) && (Rtc.Clk.min == 0) && (run_time > 3600))
		{
			// if ethernet commuicnation is 
			//QuickSoftReset();			
		}
#endif
		check_transfer_bip_to_mstp();
#endif
		Check_whether_clear_conflict_id();
		
		
#endif
		
#if !(ARM_TSTAT_WIFI)	
#if SMTP
		Email_Task();
#endif
#endif

		
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)	
#if (ARM_MINI || ARM_CM5)		
		Check_Send_bip();
#endif		
		check_whether_suspend_mstp();
		
#if OUTPUT_DEATMASTER
		output_dead_master();
#endif
//		dcc_timer_seconds(1);
		if(count_hold_on_bip_to_mstp > 0)
			count_hold_on_bip_to_mstp--;
		Check_Remote_Panel_Table(); //  check remote_panel_db
		
#if (ARM_MINI || ASIX_MINI)				
		
#if 1
		if((Modbus.mini_type == MINI_BIG) ||(Modbus.mini_type == MINI_BIG_ARM)
			|| (Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM) 
			|| (Modbus.mini_type == MINI_NEW_TINY) || (Modbus.mini_type == MINI_TINY_ARM)	|| (Modbus.mini_type == MINI_TINY_ARM)
			|| (Modbus.mini_type == MINI_TINY)
			|| (Modbus.mini_type == MINI_NANO) 
			)
		{ // Only for LB, can not detect it 
			if((check_sd < 1200) && (SD_exist == 1))
			{	
				check_sd++;
				
				if(check_sd % 10 == 0)  //check SD per 5 second 
				{
					if((Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))
					{
						vTaskSuspend(xHandler_SPI);
						SPI1_Init(1);
					}
					check_SD_exist();

					if((Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))
					{
						SPI1_Init(0);
						vTaskResume(xHandler_SPI);
					}
				}
			}
		}
#endif
		
#endif
		
		
#endif		
		Check_Net_Point_Table();
		if(flag_reboot == 1)
		{		
			Store_Pulse_Counter(1);
			/* Never force OUT/IN/VAR write on reboot. After an erase-then-reset
			 * the table may be blank/0xFF or still in RAM-init zeros — writing
			 * that would permanently wipe flash. Only persist already-dirty pages. */
			Flash_Write_Mass();
#if (ASIX_MINI || ASIX_CM5)
			Flash_Write_Mass();
			IntFlashWriteByte(0x4001,0);
			AX11000_SoftReboot();
#else  // ARM
			SoftReset();
#endif			

		}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)		
		if(flag_Updata_Clock == 1)
		{
			Rtc_Set(Rtc.Clk.year,Rtc.Clk.mon,Rtc.Clk.day,Rtc.Clk.hour,Rtc.Clk.min,Rtc.Clk.sec,0);
			RTC_Get();
			flag_Updata_Clock = 0;
		}
#endif
		
#if (ASIX_MINI || ASIX_CM5)		
		if(flag_Updata_Clock == 1)
		{
			Updata_Clock(0);
			flag_Updata_Clock = 0;
		}
#endif
		
// check if time is expired
// if current panel is #1
#if TIME_SYNC	
#if (ASIX_MINI || ASIX_CM5)		
		check_time_sync();
#endif
#endif	

		
#if (ASIX_MINI || ASIX_CM5)
		if((Modbus.en_dyndns == 2) && (dyndns_provider == 3))
//			 // if cant connect temco_server, retry 3 time
//			// 0xc0a80359
		{
//			RM_Start();
		}
#endif
		
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
		if((Modbus.en_dyndns == 2) && (dyndns_provider == 3))
//			 // if cant connect temco_server, retry 3 time
//			// 0xc0a80359
		{			
//				RM_Start();
		}
#endif
	
// check count of read tstat name
		check_read_tstat_name();			

		
#if !(ARM_TSTAT_WIFI)		
		if(Modbus.en_dyndns == 2)
		{
			DYNDNS_TIMER++;
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
			do_dyndns();
#endif

			if(flag_Update_Dyndns == 0)
			{
				if(DynDNS_GetState() == DYNDNS_STATE_UPDATE_OK)
				{
					flag_Update_Dyndns = 1;
					
					memcpy(update_dyndns_time.all,Rtc.all,sizeof(UN_Time));
		
					Update_Dyndns_Retry = 0;
		
				}
				else
				{
					if(Update_Dyndns_Retry < MAX_DDNS_RETRY_COUNT)
					{
						if(DYNDNS_TIMER % 10 == 0)  // retry 1 time at a interval of 5 seconds
						{
							init_dyndns();
							Update_Dyndns_Retry++;
						}
					}
					else
					{
						Update_Dyndns_Retry = 0;
						flag_Update_Dyndns = 1;
					}
				}
			}
			
			if(DYNDNS_TIMER > dyndns_update_time * 60)  // 10 min
			{
				init_dyndns();
				DYNDNS_TIMER = 0;
				flag_Update_Dyndns = 0;
				Update_Dyndns_Retry = 0;
			}
		}
#endif
		
#if (ASIX_MINI || ASIX_CM5)	
		Check_whether_reiniTCP();

#if TIME_SYNC				
		flag_stop_timesync = 0;
#endif

		flag_udp_scan = 1;	
		
#endif	
		
// step3 ok	
		if(flag_resume_rs485 == 1)  // suspend rs485 task
		{
			resume_rs485_count++;
			if(resume_rs485_count > 5)
			{			
				vTaskResume(Handle_Scan);	
				flag_resume_rs485 = 2;  // resume rs485 task
				resume_rs485_count = 0;
			}
		}

  
// check input alarm	
#if ALARM_SYNC
		if(check_input_alarm_count < 60)  // 1min
		{
			check_input_alarm_count++;
		}
		else
		{
			check_input_alarm_count = 0;
			check_input_alarm();
		}
#endif		


// step2 ok		
		if(flag_reset_default == 1)
		{
			set_default_parameters();
			flag_reset_default = 0;
		}

		check_task();
		
		check_flash_changed();
	}
}


// implement it in Highest priority task
void check_task(void)// check task		
{
	uint8_t loop;

		for(loop = 0;loop < 15;loop++)	
		{				
			if(task_test.enable[loop] == 1)
			{
			  if(task_test.count[loop] != task_test.old_count[loop])
				{
					task_test.old_count[loop] = task_test.count[loop];
					task_test.inactive_count[loop] = 0;
				}
				else
					task_test.inactive_count[loop]++;
			}
#if !(ARM_TSTAT_WIFI)				
			if(task_test.inactive_count[0] > 20)	
			{ 					
				task_test.inactive_count[0] = 0;
				E2prom_Write_Byte(EEP_TEST1,200);	
				delay_ms(10);
				flag_reboot = 1;
			}	
#else
			if(task_test.inactive_count[1] > 20)	
			{ 	
				E2prom_Write_Byte(EEP_TEST1,Test[48]++);	
				delay_ms(10);
				task_test.inactive_count[1] = 0;				
				flag_reboot = 1;
			}	
#endif
		} 		
		
		
		//IWDG_ReloadCounter(); 
}

uint32_t net_health[4];
extern uint32_t wifi_rx;
void check_net_health(uint8_t interval)
{
	static uint8_t count = 0;
	static uint32_t backup_rx[4] = {0,0,0,0};
	
	if(interval <= 0)
		return ;
	
	if(count < interval)
		count++;
	else
	{
		count = 0;
		net_health[0] = (com_rx[2] - backup_rx[0]);
		net_health[1] = (com_rx[0] - backup_rx[1]);
		net_health[2] = (ether_rx_packet - backup_rx[2]);
		net_health[3] = (wifi_rx - backup_rx[3]);		
		
		backup_rx[0] = com_rx[2];
		backup_rx[1] = com_rx[0];
		backup_rx[2] = ether_rx_packet;
		backup_rx[3] = wifi_rx;
	}
}

void Read_ALL_Data(void);
u8 retry_tcpip_intial;
#if ARM_MINI 

extern U32_T last_initial_tcpip;

void Check_Whether_TCP_STUCK(void)
{
	static U8_T count = 0;
	static U8_T count2 = 0;
	static U32_T TX = 0;
	static U32_T RX = 0;

	/* NOTE:
	 * Old logic called tcpip_intial() every ~60s (count_reintial countdown with
	 * flag gate commented out), on 10s RX silence, and every hour. That tears
	 * down all Modbus TCP sessions and shows up as RST/reconnect storms.
	 * Reinit only when flag_reintial_tcpip is explicitly armed (ENC/hw/test),
	 * or when RX+TX both stay dead for a long time.
	 */
#if (ARM_MINI || ARM_CM5)
	tcpip_health_check();
#endif
	if(RX != ether_rx_packet)
	{
		RX = ether_rx_packet;
		retry_tcpip_intial = 0;
		count = 0;
		if(flag_reintial_tcpip == 0)
			count_reintial_tcpip = 600;
		// always rx++, but no IP
		// check whether it is a router
		if(Modbus.com_config[0] == BACNET_MASTER || Modbus.com_config[0] == BACNET_SLAVE || Modbus.com_config[2] == BACNET_MASTER || Modbus.com_config[2] == BACNET_SLAVE)
			flag_start_scan_mstp = 1;
		
	}	
	else
	{
		if(count < 255)
			count++;
		/* Quiet ethernet is normal. Suspect stuck after ~60s with prior traffic. */
		if(count >= 60)
		{
			if((ether_rx_packet > 0) && (retry_tcpip_intial < 3))
			{
				Test[17]++;
				flag_reintial_tcpip = 1;
				count_reintial_tcpip = 1;
				retry_tcpip_intial++;
			}
			count = 0;
		}
	}	
	
	if(TX != ether_tx_packet)
	{
		TX = ether_tx_packet;
		retry_tcpip_intial = 0;
		count2 = 0;
		if(flag_reintial_tcpip == 0)
			count_reintial_tcpip = 600;
	}
	else
	{
		if(count2 < 255)
			count2++;
		/* No TX for ~60s after we previously transmitted */
		if(count2 >= 60)
		{
			if((ether_tx_packet > 0) && (retry_tcpip_intial < 3))
			{
				Test[18]++;
				flag_reintial_tcpip = 1;
				count_reintial_tcpip = 1;
				retry_tcpip_intial++;
			}
			count2 = 0;
		}
	}	
	
	Test[43] = count_reintial_tcpip;
	if(flag_reintial_tcpip == 1)
	{	
		if(count_reintial_tcpip > 0)
			count_reintial_tcpip--;
		if(count_reintial_tcpip == 0)
		{	
#if (ARM_MINI || ARM_CM5)
			Test[16]++;
			tcpip_intial();
		
#elif (ASIX_MINI || ASIX_CM5)
			TCP_IP_Init();
#endif
			flag_reintial_tcpip = 0;
			count_reintial_tcpip = 600;
		}		

		if(retry_tcpip_intial >= 5)
		{
			retry_tcpip_intial = 0;
			//if(ether_rx_packet > 0) 
			//	QuickSoftReset();
		}
			
	}
	
}
#endif

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)

void refresh_led_switch_Task(void) reentrant;
void Scan_network_modbus_Task(void) reentrant;
void Scan_network_bacnet_Task(void) reentrant;
void Handler_COV_Task(void)	reentrant;


void watchdog_init(void)
{
	/* Enable write access to IWDG_PR and IWDG_RLR registers */ 
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	/* IWDG counter clock: 40KHz(LSI) / 4 = 10 KHz */ 
	IWDG_SetPrescaler(IWDG_Prescaler_128); 
	/* Set counter reload value to 10000 = 1s */ 
	IWDG_SetReload(4000); 
	IWDG_ReloadCounter(); // reload the value
	IWDG_Enable();  			//enable the watchdog

}

static void debug_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable/*GPIO_Remap_SWJ_JTAGDisable*/, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  						
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);										
	GPIO_ResetBits(GPIOA, GPIO_Pin_13 | GPIO_Pin_14);
}

void RESET_TOP_IO_config(void)
{
		    //GPIO????
  GPIO_InitTypeDef GPIO_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF , ENABLE);	//??USART1,GPIOA??

	//   PD6 PD8 PD9 PD10 PG6 PF11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			//??????
	GPIO_Init(GPIOF, &GPIO_InitStructure);			
}

#endif


void inputs_adc_init(void);
void Input_IO_Init(void);
void inpust_scan(void);



U8_T flag_reboot_ontime;
void main( void )
{
//	U8_T regisp;
	U8_T set_para;
	U16_T loop;
	U8_T flag_store;
	run_time = 0;
	
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)			

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x8008000);
	debug_config();	
	//ram_test = 0 ;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
 	delay_init(72);
	
	E2prom_Initial(); 	
	for(loop = 0;loop < 50;loop++)	Test[loop] = 0;	

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
	Output_IO_Init();//??
	uart1_init(115200);
// ????????????
#if REBOOT_PER_WEEK
	if(AT24CXX_ReadOneByte(EEP_APP2BOOT_TYPE) == 0x55)
	{
		U8_T temp[4];
		E2prom_Read_Byte(EEP_RUNNING_TIME1, &temp[0]);
		E2prom_Read_Byte(EEP_RUNNING_TIME2, &temp[1]);
		E2prom_Read_Byte(EEP_RUNNING_TIME3, &temp[2]);
		E2prom_Read_Byte(EEP_RUNNING_TIME4, &temp[3]);
		
		E2prom_Read_Byte(EEP_ISP_REV,&Modbus.IspVer);
		
		run_time_last = temp[0] + (U16_T)(temp[1] << 8) + ((U32_T)temp[2] << 16) + ((U32_T)temp[3] << 24);
//		flag_reboot_ontime = 1;
		if(Modbus.IspVer < 66)
			run_time_last += 10;
		else
			run_time_last += 3;
	}
	else
#endif
	{
		E2prom_Write_Byte(EEP_RUNNING_TIME1, 0);
		E2prom_Write_Byte(EEP_RUNNING_TIME2, 0);
		E2prom_Write_Byte(EEP_RUNNING_TIME3, 0);
		E2prom_Write_Byte(EEP_RUNNING_TIME4, 0);
		run_time_last = 0;
//		flag_reboot_ontime = 0;
	}
	
	AT24CXX_WriteOneByte(EEP_APP2BOOT_TYPE, 0);
	
	E2prom_Read_Byte(EEP_CPU_TYPE,&cpu_type);
	E2prom_Read_Byte(EEP_MINI_TYPE,&Modbus.mini_type);

#if (ARM_MINI || ASIX_MINI)
	if((Modbus.mini_type == MINI_NEW_TINY) || (Modbus.mini_type == MINI_TINY_ARM) || (Modbus.mini_type == MINI_TINY_ARM)|| (Modbus.mini_type == MINI_TINY_11I) ||
		(Modbus.mini_type == MINI_NANO))
	{	
		UART0_TXEN_TINY = 1;
		LED_IO_Init();
	}
	else
	{
		UART0_TXEN_BIG = 1;		
		RESET_TOP_IO_config();

	}
	if((Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))
	{
		TOP_CS = 1;
		SD_CS_SMALL = 0;
		
	}
//	printf("mini app\r\n");
#endif
#if ARM_UART_DEBUG
	UART0_TXEN_BIG = 1;
	UART2_TXEN_BIG = 1;
	//printf("tstat app\r\n");
#endif	
	
#endif
	

#if 0//ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("if mini type is %u E2 is ok. \r\n",Modbus.mini_type);
#endif

#if ARM_UART_DEBUG
	printf("intial RTC...\r\n");
#endif
	
	RTC_Init();
	
#if ARM_UART_DEBUG
	printf("intial RTC OK\r\n");
#endif
#endif

		
	
#if (ASIX_MINI || ASIX_CM5)
	AX11000_Init();
	ExecuteRuntimeFlag = 1;

#if AX_WATCHDOG_ENB
	AX11000_WatchDogSetting(0, 1, 0, WD_INTERVAL_67M);  /* time out, reset cpu */
	sTaskCreate(SoftwareWatchdog_task, (const signed portCHAR * const)"softwatch_task",portMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 11 , (xTaskHandle *)&xSoftWatchTask);
#endif
	
	DELAY_Init();
#endif


//	xLCDQueue = xQueueCreate( 5, sizeof(xLCDMessage));
	memset(&task_test,0,sizeof(STR_Task_Test));	
#if (ASIX_MINI || ASIX_CM5)
	IntFlashReadByte(0x6fff1,& set_para);
	if(set_para == 0xff)
	{ /* initial defautl parameters*/
//		Eeprom_Write_Cpu_Config();
		etr_reboot = 0;
		set_para = 0xaa;
		IntFlashWriteByte(0x6fff1,set_para);
	}	
	else
	{
		etr_reboot = 1;
	}
		
	E2prom_Initial(); 
	
#endif
	Bacnet_Initial_Data();
	Read_ALL_Data();  
#if !(ARM_TSTAT_WIFI)	
	if((Modbus.mini_type <= MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
	{
		Lcd_Initial();
		Display_Initial_Data();
		Key_Inital();
		Lcd_Show_String(0, 0, "step1: LCD & KEY ", NORMAL, 21,0x0000,0xffff);
		Lcd_Show_String(1, 0, "step2: E2 OK ", NORMAL, 21,0x0000,0xffff);		
		
	}
#endif
	
#if (ARM_MINI || ASIX_MINI)

	if((Modbus.mini_type == MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
	{
		RESET_TOP = 0;  // RESET c8051f023 
		DELAY_Us(500);
		RESET_TOP = 1; 	
	}
	else if((Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))
	{ // ARM REV dont have reset pin
		RESET_TOP = 0;  // RESET c8051f023 
		DELAY_Us(500);
		RESET_TOP = 1; 	
	}
	else if(Modbus.mini_type == MINI_TINY)
	{		
		if(Modbus.hardRev >= STM_TINY_REV)
		{			
			RESET_TOP = 0;  // RESET stm32
			DELAY_Us(500);
			RESET_TOP = 1; 	
		}
		else
		{  // old version, top board is sm5r16		
			RESET_TOP = 1;  // RESET sm5r16 
			DELAY_Us(500);
			RESET_TOP = 0;
		}
	}	
#endif	
	
	Comm_Tstat_Initial_Data();
	init_scan_db();
	Flash_Inital();	
#if (ASIX_MINI || ASIX_CM5)
	init_dyndns_data();
	IntFlashReadByte(0x7fff0,&flag_store);
//	Test[49] = flag_store;
	if(flag_store == 0x55)
	{		
		if((Modbus.mini_type <= MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
		Lcd_Show_String(2, 0, "step3: READ FLASH", NORMAL, 21,0x0000,0xffff);	
		Flash_Read_Mass();	
		Get_Tst_DB_From_Flash(); 
	}
	else
	{
		if((Modbus.mini_type <= MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
		Lcd_Show_String(2, 0, "step3: EMPTY FLASH", NORMAL, 21,0x0000,0xffff);	
	}
#endif

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)	
	Flash_Read_Mass();	
	Get_Tst_DB_From_Flash(); 
  TIM6_Int_Init(100, 719);
#endif
	
	SD_exist = 1;  // inexist
	
#if STORE_TO_SD	
	
//#if (ASIX_MINI || ASIX_CM5)
//	if((Modbus.mini_type == MINI_BIG) ||(Modbus.mini_type == MINI_BIG_ARM)
//	 || (Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM) 
//	|| (Modbus.mini_type == MINI_NEW_TINY) || (Modbus.mini_type == MINI_TINY_ARM)	
//	|| (Modbus.mini_type == MINI_TINY) 
//	)
		check_SD_exist();
//#endif
#if (ARM_MINI || ASIX_MINI)	
	if(SD_exist == 2)	
	{
		if((Modbus.mini_type <= MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
			Lcd_Show_String(3, 0, "step4: SD EXIST", NORMAL, 21,0x0000,0xffff);	
	}
	else
	{
		if((Modbus.mini_type <= MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
			Lcd_Show_String(3, 0, "step4: SD INEXIST", NORMAL, 21,0x0000,0xffff);	
	}
#endif
	
#endif
	if(Setting_Info.reg.webview_json_flash != 2)
		initial_graphic_point();
 	monitor_init();

#if (DEBUG_UART1)
	UART_Init(UART_SUB1);

	if(UART_SUB1 == 0)	UART0_TXEN_TINY = SEND;  // MINI
	if(UART_SUB1 == 2)	UART2_TXEN_BIG = SEND;  // MINI

	
	sprintf(debug_str,"intial\r\n");
	uart_send_string(debug_str,strlen(debug_str),UART_SUB1);
	
#endif	
	Initial_Panel_Info(); // read panel name, must read flash first
	Sync_Panel_Info();
	initSerial();
	current_online_ctr = 0;
#if (ARM_MINI || ASIX_MINI || ASIX_CM5)
	PIC_initial_data();
#endif

#if ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("APP intial ok\r\n");
#endif
#if (DEBUG_UART1)
	uart_init_send_com(UART_SUB1);	// for test		
	sprintf(debug_str," \r\n\ intial ok");
	uart_send_string(debug_str,strlen(debug_str),UART_SUB1);
#endif 
#if (ARM_MINI || ARM_TSTAT_WIFI)
#if ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("intial wifi\r\n");
#endif
	vStartWifiTasks(tskIDLE_PRIORITY + 10); 		
#endif

#if ARM_TSTAT_WIFI
	if(Modbus.mini_type == MINI_TSTAT10 || Modbus.mini_type == MINI_T10P)
	{
		vStartKeyTasks(tskIDLE_PRIORITY + 5);
		//vStartDisplayTasks(tskIDLE_PRIORITY + 4);
		LCD_Intial();
		vStartMenuTask(tskIDLE_PRIORITY + 4);
	}
#endif	
	sTaskCreate(Common_task,/* (const signed portCHAR * const)*/"Common_task",
		COMMON_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, (xTaskHandle *)&xHandleCommon);
#if ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("intial common\r\n");
#endif
	Inital_Bacnet_Server();
	vStartMainSerialTasks(tskIDLE_PRIORITY + 12);	 // main uart, rs485 or zigbee
#if !(ARM_TSTAT_WIFI)
	if((Modbus.mini_type <= MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM))
		Lcd_Show_String(4, 0, "step5: INITIAL OK", NORMAL, 21,0x0000,0xffff);	

	Display_IP();	
	sTaskCreate(TCPIP_Task, /*(const signed portCHAR * const)*/"TCPIP_task",
		TCPIP_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, (xTaskHandle *)&xHandleTcp); 

#if (ARM_MINI || ASIX_MINI)		
	/* slave select output enable, SPI master, SSO auto, SPI enable, SPI_STCFIE enable, baudrate, slave select */
		if((Modbus.mini_type == MINI_BIG) ||(Modbus.mini_type == MINI_BIG_ARM)
	|| (Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM) 
	|| (Modbus.mini_type == MINI_NEW_TINY) || (Modbus.mini_type == MINI_TINY_ARM)	|| (Modbus.mini_type == MINI_TINY_ARM)
	|| (Modbus.mini_type == MINI_TINY)) 
	{
#if ASIX_MINI
		SPI_Setup(SPI_SSO_ENB|SPI_MST_SEL|SPI_SS_AUTO|SPI_ENB, SPI_STCFIE, 10, SLAVE_SEL_1); // 25M
		vStartCommToTopTasks(tskIDLE_PRIORITY + 5);
#endif	
	}
#if ARM_MINI
		if((Modbus.mini_type == MINI_BIG) || (Modbus.mini_type == MINI_BIG_ARM)
			|| (Modbus.mini_type == MINI_SMALL) || (Modbus.mini_type == MINI_SMALL_ARM))
			vStartCommToTopTasks(tskIDLE_PRIORITY + 7);		
		
		else if((Modbus.mini_type == MINI_NEW_TINY) || (Modbus.mini_type == MINI_TINY_ARM) || (Modbus.mini_type == MINI_TINY_11I))
		{
			initial_HSP(); // dont have top board, still need intial HSP.
			sTaskCreate(refresh_led_switch_Task, "refresh_led_task", 500, NULL, tskIDLE_PRIORITY + 3, (xTaskHandle *)&xHandleLedRefresh);
		}		
#if 0//COV		
		sTaskCreate(Handler_COV_Task, (const signed portCHAR * const)"Handler_COV_Task", 2000, NULL, tskIDLE_PRIORITY + 1, (xTaskHandle *)&Handle_COV);
#endif
		
#if NETWORK_MODBUS_BAC
		sTaskCreate(Scan_network_bacnet_Task, (const signed portCHAR * const)"Scan_network_bacnet_Task", 800, NULL, tskIDLE_PRIORITY + 1, (xTaskHandle *)&Handle_Network_bacnet);
		sTaskCreate(Scan_network_modbus_Task, (const signed portCHAR * const)"Scan_network_modbus_Task", 256, NULL, tskIDLE_PRIORITY + 1, (xTaskHandle *)&Handle_Network_modbus);
#endif
		
#endif	

	
#endif
	

#if (ARM_CM5 || ASIX_CM5)		
#if ARM_CM5
		sTaskCreate(Scan_network_bacnet_Task, (const signed portCHAR * const)"Scan_network_bacnet_Task", 512, NULL, tskIDLE_PRIORITY + 1, (xTaskHandle *)&Handle_Scan_net);
#endif
		vStartKeyTasks(tskIDLE_PRIORITY + 5);
		sTaskCreate( Sampel_AI_Task, "SampleAItask", SampleAISTACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &Handle_SampleAI ); 
		sTaskCreate( Sampel_DI_Task, "SampleDItask", SampleDISTACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &Handle_SampleDI ); 
#endif
	
#endif
	
	vStartScanTask(tskIDLE_PRIORITY + 3);
#if MSTP
	sTaskCreate(Master_Node_task, (const signed portCHAR * const)"Master_Node_task", 
		BACnet_STACK_SIZE, NULL, tskIDLE_PRIORITY + 11, (xTaskHandle *)&xHandleMSTP);
#endif
	sTaskCreate(Bacnet_Control,/*(const signed portCHAR * const)*/"BAC_Control_task",
		Control_STACK_SIZE, NULL, tskIDLE_PRIORITY + 8,(xTaskHandle *)&xHandleBacnetControl);	

	sTaskCreate(Monitor_Task_task, /*(const signed portCHAR * const)*/"monitor_task",
		Monitor_STACK_SIZE, NULL, tskIDLE_PRIORITY + 13,(xTaskHandle *)&xHandleMornitor_task);
	
#if (ARM_MINI || ASIX_MINI || ARM_CM5 || ARM_TSTAT_WIFI)	
	vStartOutputTasks(tskIDLE_PRIORITY + 5);
	
#endif
	

//	if(Modbus.mini_type <= MINI_BIG)
//	{
//		sTaskCreate( vLCDTask, /*( signed portCHAR * )*/ "LCD", 500, NULL, tskIDLE_PRIORITY + 1, (xTaskHandle *)&xHandleLCD_task );
//	}
	/* Finally kick off the scheduler.  This function should never return. */

#if ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("intial end\r\n");
#endif
#if (ASIX_MINI || ASIX_CM5)
	vTaskStartScheduler( portUSE_PREEMPTION );
#else
	vTaskStartScheduler();
#endif

	/* Should never reach here now under control of the scheduler. */

}


/*-----------------------------------------------------------*/
#if !(ARM_TSTAT_WIFI)
U8_T Check_Lcd(void)
{
	char far text[21];
	if(count_refresh_all < 3600)  // 1hour
	{
		count_refresh_all++;
		if((backup_sub_no != sub_no) || (backup_current_online_ctr != current_online_ctr))
		{
			sprintf(text, "DB: %u ON: %u", (uint16)sub_no,(uint16)current_online_ctr);
			xMessage.x = 2;
			xMessage.y = 0;
			xMessage.str = text;
			xMessage.mode = NORMAL;
			xMessage.len = 21;
			xMessage.dcolor = 0;
			xMessage.bgcolor = 0xffff;
			Lcd_Show_String(xMessage.x, xMessage.y,xMessage.str, NORMAL, 21,xMessage.dcolor,xMessage .bgcolor);

			backup_sub_no = sub_no;
			backup_current_online_ctr = current_online_ctr;
			return 1;
		}
		if((backup_ether_rx_packet != ether_rx_packet) || (backup_ether_tx_packet != ether_tx_packet))
		{
			backup_ether_rx_packet = ether_rx_packet;
			backup_ether_tx_packet = ether_tx_packet;
			sprintf(text, "RX:%lu TX:%lu",ether_rx_packet,ether_tx_packet);
			xMessage.x = 3;
			xMessage.y = 0;
			xMessage.str = text;
			xMessage.mode = NORMAL;
			xMessage.len = 21;
			xMessage.dcolor = 0;
			xMessage.bgcolor = 0xffff;

			Lcd_Show_String(xMessage.x, xMessage.y,xMessage.str, NORMAL, 21,xMessage.dcolor,xMessage .bgcolor);

			//return 1;
		}	
		//else
		{
		// update time
			get_time_text();
			sprintf(text, "%s", time);
			xMessage.x = 4;
			xMessage.y = 0;
			xMessage.str = text;
			xMessage.mode = NORMAL;
			xMessage.len = 21;
			xMessage.dcolor = 0x0000;
			xMessage.bgcolor = 0xffff;	
		}
		Lcd_Show_String(xMessage.x, xMessage.y,xMessage.str, NORMAL, 21,xMessage.dcolor,xMessage .bgcolor);

	}
	else
	{
		Lcd_Initial();
		Display_IP();
		count_refresh_all = 0;
	}
	return 0;
}
#endif

#if 0
void vLCDTask( void ) reentrant
{
//	xLCDMessage xMessage1;
	uint16_t count;
	/* Initialise the LCD and display a startup message. */
//	prvConfigureLCD();
//	LCD_DrawMonoPict( ( unsigned portLONG * ) pcBitmap );

	task_test.enable[6] = 1;
	for( ;; )
	{
		/* Wait for a message to arrive that requires displaying. */
		task_test.count[6]++;
		count = 0;
//		while((cQueueReceive( xLCDQueue, &xMessage, portMAX_DELAY ) != pdPASS) && (count++ < 10000)) Test[30]++;
	 vTaskDelay( 500 / portTICK_RATE_MS);	
		/* Display the message.  Print each message to a different position. */
		//printf( ( portCHAR const * ) xMessage.pcMessage );
//		Lcd_Initial();
		Lcd_Show_String(xMessage.x, xMessage.y,xMessage.str, NORMAL, 21,xMessage.dcolor,xMessage .bgcolor);
	}
}
#endif

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)
void _ttywrch(int ch)
{
ch = ch;
}
#endif

