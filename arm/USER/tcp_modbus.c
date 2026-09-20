#include "main.h"
//#include "define.h"
//#include "tcpip.h"

#define UIP_HEAD 6
#define MB_EXC_SLAVE_BUSY			0x06
#define MB_EXC_GW_TARGET_FAILED		0x0B

extern uint8_t prog_loading;
extern uint8_t count_prg_load;
extern U8_T subcom_seq[3];
extern U8_T subcom_num;

u8 tcp_server_databuf[300];   	//??????	  
u8 tcp_server_sta;				//?????
 //[7]:0,???;1,????;
//[6]:0,???;1,???????
//[5]:0,???;1,???????
u8 update_firmware;

u8 tcp_server_sendbuf[500];
u16 tcp_server_sendlen;

/* Serialize TCP->RS485: only one forward in flight (RS485 is half-duplex). */
static u8 tcp_rs485_busy = 0;

void Set_transaction_ID(U8_T *str, U16_T id, U16_T num)
{
	str[0] = (U8_T)(id >> 8);		//transaction id
	str[1] = (U8_T)id;

	str[2] = 0;						//protocol id, modbus protocol = 0
	str[3] = 0;

	str[4] = (U8_T)(num >> 8);
	str[5] = (U8_T)num;
}

static void fill_modbus_tcp_exception(U8_T *req, U8_T exc)
{
	tcp_server_sendbuf[0] = req[0];
	tcp_server_sendbuf[1] = req[1];
	tcp_server_sendbuf[2] = 0;
	tcp_server_sendbuf[3] = 0;
	tcp_server_sendbuf[4] = 0;
	tcp_server_sendbuf[5] = 3;
	tcp_server_sendbuf[6] = req[6];
	tcp_server_sendbuf[7] = (U8_T)(req[7] | 0x80);
	tcp_server_sendbuf[8] = exc;
	tcp_server_sendlen = 9;
}

//????TCP ??????????
//?????UIP_APPCALL(tcp_demo_appcall)??,??Web Server???.
//?uip?????,UIP_APPCALL??????,??????(1200),??????????
//?? : ???TCP????????????????????????????????
void tcp_server_appcall(struct uip_conn * conn)
{
	u8 send_flag = 0;
	u8 pos;
 	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	
	if(uip_aborted()){tcp_server_aborted();	}	//????
 	if(uip_timedout()){tcp_server_timedout();}	//????   
	if(uip_closed()) {tcp_server_closed();}		//????	   
 	if(uip_connected()){tcp_server_connected();} 	//????	    
	if(uip_acked()) tcp_server_acked();			//????????? 
	if(uip_newdata())//???????????
	{
//		net_rx_count  = 2 ;
		memcpy(&tcp_server_databuf[0], uip_appdata,uip_len);		
		// check modbus data
		if( (tcp_server_databuf[0] == 0xee) && (tcp_server_databuf[1] == 0x10) &&
		(tcp_server_databuf[2] == 0x00) && (tcp_server_databuf[3] == 0x00) &&
		(tcp_server_databuf[4] == 0x00) && (tcp_server_databuf[5] == 0x00) &&
		(tcp_server_databuf[6] == 0x00) && (tcp_server_databuf[7] == 0x00) )
		{		
//			Udtcp_server_databuf(0);
			send_flag = 1;
			update_firmware = 1;

		}
		else if(tcp_server_databuf[6] == Modbus.address 
		|| ((tcp_server_databuf[6] == 255) && (tcp_server_databuf[7] != 0x19))
		)
		{
//			net_tx_count  = 2 ;
			send_flag = 1;
			tcp_server_sendlen = 0;
			responseCmd(1, tcp_server_databuf);
		}	
		else
		{
			// transfer data to sub ,TCP TO RS485
			U8_T header[6];	
			U8_T i;
			U16_T send_len;
			static uint8_t sub_index = 0;
			uint8_t port;
			
			
			if((tcp_server_databuf[UIP_HEAD] == 0x00) || 
			((tcp_server_databuf[UIP_HEAD + 1] != READ_VARIABLES) 
			&& (tcp_server_databuf[UIP_HEAD + 1] != WRITE_VARIABLES) 
			&& (tcp_server_databuf[UIP_HEAD + 1] != MULTIPLE_WRITE) 
			&& (tcp_server_databuf[UIP_HEAD + 1] != CHECKONLINE)
			&& (tcp_server_databuf[UIP_HEAD + 1] != READ_COIL)
			&& (tcp_server_databuf[UIP_HEAD + 1] != READ_DIS_INPUT)
			&& (tcp_server_databuf[UIP_HEAD + 1] != READ_INPUT)
			&& (tcp_server_databuf[UIP_HEAD + 1] != WRITE_COIL)
			&& (tcp_server_databuf[UIP_HEAD + 1] != WRITE_MULTI_COIL)
			&& (tcp_server_databuf[UIP_HEAD + 1] != CHECKONLINE_WIHTCOM)
			&& (tcp_server_databuf[UIP_HEAD + 1] != TEMCO_MODBUS)
			))
			{
				return;
			}
			if((tcp_server_databuf[UIP_HEAD + 1] == MULTIPLE_WRITE) && ((uip_len - UIP_HEAD) != (tcp_server_databuf[UIP_HEAD + 6] + 7)))
			{
				return;
			}

			/* Another TCP conn already waiting on RS485: reply busy, keep socket alive */
			if(tcp_rs485_busy)
			{
				fill_modbus_tcp_exception(tcp_server_databuf, MB_EXC_SLAVE_BUSY);
				send_flag = 1;
			}
			else if(subcom_num < 1)
			{
				return;
			}
			else
			{
				U8_T req_head[8];

				memcpy(req_head, tcp_server_databuf, 8);

				port = subcom_seq[sub_index];
				if(sub_index < subcom_num - 1)
				{
					sub_index++;	
				} 
				else
					sub_index = 0;
				
				Modbus.sub_port = port;		
			
//			if(Modbus.com_config[2] == MODBUS_MASTER)
//				Modbus.sub_port = 2;
//			else if(Modbus.com_config[0] == MODBUS_MASTER)
//				Modbus.sub_port = 0;
//			else if(Modbus.com_config[1] == MODBUS_MASTER)
//				Modbus.sub_port = 1;
//			else
//			{
//				return;
//			}
				for(i = 0;i <  sub_no ;i++)
				{
					if(tcp_server_databuf[UIP_HEAD] == uart2_sub_addr[i])
					{
						Modbus.sub_port = 2;
						continue;
					}
					else if(tcp_server_databuf[UIP_HEAD] == uart0_sub_addr[i])
					{
						Modbus.sub_port = 0;
						continue;
					}
					else if(tcp_server_databuf[UIP_HEAD] == uart1_sub_addr[i])
					{	
						Modbus.sub_port = 1;
						continue;
					}
				}		

				if(Modbus.mini_type == MINI_VAV)	
					Modbus.sub_port = 0;			
			
				if(flag_resume_rs485 == 0 || flag_resume_rs485 == 2)			
				{
					vTaskSuspend(Handle_Scan); 
				}		
				// dont check ram when loading prog file. 
				prog_loading = 1;
				count_prg_load = 0;						
				vTaskSuspend(xHandler_Output);	
				vTaskSuspend(xHandleCommon);		
//			vTaskSuspend(xHandleBacnetControl); 
				vTaskSuspend(xHandleMornitor_task);			
#if MSTP
				vTaskSuspend(xHandleMSTP);			
#endif 
			
#if ARM_MINI	
			
				if((Modbus.mini_type == MINI_BIG) ||	(Modbus.mini_type == MINI_BIG_ARM) 
					|| (Modbus.mini_type == MINI_SMALL)  || (Modbus.mini_type == MINI_SMALL_ARM) 
				|| (Modbus.mini_type == MINI_TINY)			
				)
					vTaskSuspend(xHandler_SPI);	
#endif

				task_test.inactive_count[0] = 0;

//			TcpSocket_ME = pMODBUSTCPConn->TcpSocket;
				send_flag = 1;
				tcp_server_sendlen = 0;
			
				if((tcp_server_databuf[UIP_HEAD + 1] == READ_DIS_INPUT) || (tcp_server_databuf[UIP_HEAD + 1] == READ_COIL))
					send_len = (tcp_server_databuf[UIP_HEAD + 5] + 7) / 8 + 3; // (buf[5] + 7) / 8 + 5;
				else if((tcp_server_databuf[UIP_HEAD + 1] == READ_VARIABLES) || (tcp_server_databuf[UIP_HEAD + 1] == READ_INPUT))
					send_len = tcp_server_databuf[UIP_HEAD + 5] * 2 + 3;
				else
					/* FC5/6/15/16 response: Unit+PDU = 6 (no RTU CRC on TCP) */
					send_len = 6;
			
				Set_transaction_ID(header, ((U16_T)req_head[0] << 8) | req_head[1], send_len);
				{
					/* Copy PDU: keep for retry; avoid depending on tcp_server_databuf across wait. */
					static U8_T tcp_rs485_pdu[256];
					U16_T pdu_len = uip_len - UIP_HEAD;

					if(pdu_len > sizeof(tcp_rs485_pdu))
						pdu_len = sizeof(tcp_rs485_pdu);
					memcpy(tcp_rs485_pdu, tcp_server_databuf + UIP_HEAD, pdu_len);

					tcp_rs485_busy = 1;
					Response_TCPIP_To_SUB(tcp_rs485_pdu, pdu_len, Modbus.sub_port, header);
					/* One retry on timeout/CRC — common with 3 concurrent Poll windows */
					if(tcp_server_sendlen == 0)
					{
						delay_ms(20);
						Response_TCPIP_To_SUB(tcp_rs485_pdu, pdu_len, Modbus.sub_port, header);
					}
					tcp_rs485_busy = 0;
					/* Still no RS485 reply: exception 0x0B (Modbus Poll text you see). */
					if(tcp_server_sendlen == 0)
						fill_modbus_tcp_exception(req_head, MB_EXC_GW_TARGET_FAILED);
				}
		
//			memcpy(tcp_server_sendbuf,tcp_server_databuf,2 * tcp_server_databuf[UIP_HEAD + 5] + 3);
//      tcp_server_sendlen = 2 * tcp_server_databuf[UIP_HEAD + 5] + 3;
				flag_resume_rs485 = 1;	// suspend rs485 task, resume it later, make the communication smoothly	
				resume_rs485_count = 0;
			
				vTaskResume(xHandler_Output); 
				vTaskResume(xHandleCommon);
//			vTaskResume(xHandleBacnetControl);
				vTaskResume(xHandleMornitor_task);
#if MSTP
				vTaskResume(xHandleMSTP);			
#endif 

#if ARM_MINI	

				if((Modbus.mini_type == MINI_BIG) ||	(Modbus.mini_type == MINI_BIG_ARM) 
					|| (Modbus.mini_type == MINI_SMALL)  || (Modbus.mini_type == MINI_SMALL_ARM) 
				|| (Modbus.mini_type == MINI_TINY)			
				)
				vTaskResume(xHandler_SPI);

#endif	
			}
		}
	}
	

//	/*else*/ if(tcp_server_sta & (1 << 5))			//???????
//	{
//		s->textptr = tcp_server_databuf;
//		s->textlen = strlen((const char*)tcp_server_databuf);
//		tcp_server_sta &= ~(1 << 5);			//????
//	}
	
	//???????????????????????,??uip???? 
	if(uip_rexmit() || uip_newdata() || uip_acked() || uip_connected() || uip_poll())
	{
		if(send_flag == 1)
		{
			s->textptr = tcp_server_sendbuf;
			s->textlen = tcp_server_sendlen;
			tcp_server_senddata();
//			if(tcp_server_sendlen > 400) {
//				memcpy(&Test[40],&tcp_server_sendbuf[74],20);
//			}
		}
		//if(tcp_server_sendlen > 0)	tcp_server_senddata();
	}
}

//????				    
void tcp_server_aborted(void)
{
	tcp_server_sta &= ~(1 << 7);				//??????
//	uip_log("tcp_server aborted!\r\n");			//??log
}

//????
void tcp_server_timedout(void)
{
	tcp_server_sta &= ~(1 << 7);				//??????
//	uip_log("tcp_server timeout!\r\n");			//??log
}

//????
void tcp_server_closed(void)
{
	tcp_server_sta &= ~(1 << 7);				//??????
//	uip_log("tcp_server closed!\r\n");			//??log
}

//????
void tcp_server_connected(void)
{								  
	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	//uip_conn??????"appstate"????????????????
	//????s??,????????
 	//?????????uip_conn????,?????uip??????
	//?uip.c ? ???????:
	//		struct uip_conn *uip_conn;
	//		struct uip_conn uip_conns[UIP_CONNS]; //UIP_CONNS??=10
	//???1??????,???????????
	//uip_conn????????,?????tcp?udp???
	tcp_server_sta |= 1 << 7;					//??????
//  	uip_log("tcp_server connected!\r\n");		//??log
	s->state = STATE_CMD; 						//????
	s->textlen = 0;
//	s->textptr = "Connect to STM32 Board Successfully!\r\n";
//	s->textlen = strlen((char *)s->textptr);
}

//?????????
void tcp_server_acked(void)
{						    	 
	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	s->textlen = 0;								//????
//	uip_log("tcp_server acked!\r\n");			//??????		 
}

//????????
void tcp_server_senddata(void)
{
	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	//s->textptr : ???????????
	//s->textlen :??????(????)		   
	if(s->textlen > 0)
		uip_send(s->textptr, s->textlen);//??TCP???	 
}

