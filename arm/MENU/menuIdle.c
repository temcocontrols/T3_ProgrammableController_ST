#include "main.h"
#include "define.h"
#include "LCD_TSTAT.h"
#include "menu.h"
#include "wifi.h"
#define	NODES_POLL_PERIOD	30

char UI_DIS_LINE1[4]; //对应之前 setpoint  fan 以及 sys
char UI_DIS_LINE2[4];
char UI_DIS_LINE3[4];
char UI_DIS_TOP[9];

static uint8 display_around_time_ctr = NODES_POLL_PERIOD;
static uint8 disp_index = 0;
static uint8 set_msv = 0;
static uint8 warming_state = TRUE;
static uint8 force_refresh = TRUE;
uint8 flag_left_key = 0;
uint8	count_left_key = 0;
uint8 flag_digital_top_area = 0;
uint8 digital_top_area_type = 0;
uint8 digital_top_area_num = 0;
uint8 digital_top_area_changed = 0;
void set_output_raw(uint8_t point,uint16_t value);
extern uint16_t count_suspend_mstp;
void MenuIdle_init(void)
{
	uint8 i,j;
	
	//LCDtest();
	ClearScreen(TSTAT8_BACK_COLOR);
	flag_digital_top_area = 0;
	digital_top_area_type = 0;
  digital_top_area_num = 0;
	digital_top_area_changed = 0;

	digital_top_area_type = Setting_Info.reg.display_lcd.lcd_mod_reg.npoint.point_type;
	digital_top_area_num = Setting_Info.reg.display_lcd.lcd_mod_reg.npoint.number - 1;	
						
	memset(UI_DIS_TOP,0,9);
	digital_top_area_changed = 0;
	
	disp_str(FORM15X30, SCH_XPOS,  0, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);					
	disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "            ",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
	disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
	disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT * 2 - 7, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);	
	
	if(digital_top_area_type == IN)
	{
		memcpy(UI_DIS_TOP, inputs[digital_top_area_num].label, 9);		
	}
	else if(digital_top_area_type == OUT)
		memcpy(UI_DIS_TOP, outputs[digital_top_area_num].label, 9);
	else if(digital_top_area_type == VAR)
		memcpy(UI_DIS_TOP, vars[digital_top_area_num].label, 9);		

	disp_null_icon(240, 36, 0, 0,TIME_POS,TSTAT8_CH_COLOR, TSTAT8_MENU_COLOR2);
	
  scroll = &scroll_ram[0][0];
//	fanspeedbuf = fan_speed_user;
	
	
	draw_tangle(102,105);
	draw_tangle(102,148);
	draw_tangle(102,191);

	memcpy(UI_DIS_LINE1, vars[0].label, 3);UI_DIS_LINE1[3] = 0;
	memcpy(UI_DIS_LINE2, vars[1].label, 3);UI_DIS_LINE2[3] = 0;
	memcpy(UI_DIS_LINE3, vars[2].label, 3);UI_DIS_LINE3[3] = 0;
	
//	disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, str,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
	disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, UI_DIS_LINE1,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
	disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, UI_DIS_LINE2,SCH_COLOR,TSTAT8_BACK_COLOR);
	disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, UI_DIS_LINE3,SCH_COLOR,TSTAT8_BACK_COLOR);

	//msv_data[MAX_MSV][STR_MSV_MULTIPLE_COUNT]
	for (i = 0;i < MAX_MSV;i++)
		for (j = 0; j < STR_MSV_MULTIPLE_COUNT;j++)
		{
			if(msv_data[i][j].status == 255)
			{
				msv_data[i][j].status = 0;
			}
		}
//#if ARM_UART_DEBUG
//	uart1_init(115200);
//	DEBUG_EN = 1;
//	printf("IDLE init \r\n");
//#endif	
}

 
void get_data_format(u8 loc,float num,char *s)
{
	u8 i,s_len,s_start,buf_start;
	
	if(loc == 0)
		sprintf(s,"%9.0f",num);
	else if(loc == 1)
		sprintf(s,"%9.1f",num);
	else if(loc == 2)
		sprintf(s,"%9.2f",num);
	else if(loc == 3)
		sprintf(s,"%9.3f",num);
	else if(loc == 4)
		sprintf(s,"%9.4f",num);
	else if(loc == 5)
		sprintf(s,"%9.5f",num);
	else if(loc == 6)
		sprintf(s,"%9.6f",num);
	else
		sprintf(s,"%f",num);
	
	for(i=0;i<9;i++)
	{
		if(s[i]!= 0x20) break;
	}
	s_len = 9 - i;   					//数据长度
	s_start = i;     					//数据起始位置
	buf_start = i - i / 2; 				//重新排置后的起始位置
	
	for(i=0;i<s_len;i++) 				//数据左移
	{
		s[buf_start + i] = s[s_start + i];
	}
	for(i=buf_start + s_len;i<9;i++ ) 	//补" "
	{
		s[i] = 0x20;
	} 
}
 
void MenuIdle_display(void)
{
   	static u8 count_tx = 0;
		static u8 count_rx = 0;
		
		if(memcmp(UI_DIS_LINE1,vars[0].label,3))
		{
			disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, "   ",SCH_COLOR,TSTAT8_BACK_COLOR);
			memset(UI_DIS_LINE1,'\0',4);
			memcpy(UI_DIS_LINE1, vars[0].label, 3);
		}
		if(memcmp(UI_DIS_LINE2,vars[1].label,3))
		{		
			disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, "   ",SCH_COLOR,TSTAT8_BACK_COLOR);
			memset(UI_DIS_LINE2,'\0',4);
			memcpy(UI_DIS_LINE2, vars[1].label, 3);
		}
		if(memcmp(UI_DIS_LINE3,vars[2].label,3))
		{
			disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, "   ",SCH_COLOR,TSTAT8_BACK_COLOR);
			memset(UI_DIS_LINE3,'\0',4);
			memcpy(UI_DIS_LINE3, vars[2].label, 3);
		}
		
    //display_input_value(inputs[0].value);
		//display_value(inputs[0].value);
		display_screen_value( 1); // 分别用var的值显示在原先的 set fan 以及sys 地方
		display_screen_value( 2);
		display_screen_value( 3);

		//display_SP(inputs[0].value / 1000);
		//display_fanspeed(outputs[0].value / 1000);
		//display_mode(vars[0].value / 1000);
		if(Modbus.disable_tstat10_display == 0)
		{
			display_scroll();			
			display_icon();
			display_fan();
		}		
		else if(Modbus.disable_tstat10_display == 1)
		{
			disp_str(FORM15X30, 0,TIME_POS,"            ",TSTAT8_CH_COLOR,TSTAT8_MENU_COLOR2); 
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, FIRST_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, SECOND_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, THIRD_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, FOURTH_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
		}
		else if(Modbus.disable_tstat10_display == 2)
		{
			display_scroll();		
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, FIRST_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, SECOND_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, THIRD_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
			disp_null_icon(ICON_XDOTS, ICON_YDOTS, 0, FOURTH_ICON_POS ,ICON_POS,TSTAT8_BACK_COLOR, TSTAT8_BACK_COLOR);
		}

//		if(Setting_Info.reg.display_lcd.lcddisplay[0] == 0)
//		{
//			if(Modbus.mini_type == MINI_T10P)
//			{
//				if((inputs[HI_COMMON_CHANNEL].digital_analog == 1) && inputs[HI_COMMON_CHANNEL].range == R10K_40_250DegF) //如果range选的是10K type2 F 就显示 F
//				{	
//					Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[HI_COMMON_CHANNEL].value / 100, TOP_AREA_DISP_UNIT_F);
//				}
//				else
//				{
//					Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[HI_COMMON_CHANNEL].value / 100, TOP_AREA_DISP_UNIT_C);
//				}
//			}
//			else
//			{
//				if((inputs[COMMON_CHANNEL].digital_analog == 1) && inputs[COMMON_CHANNEL].range == R10K_40_250DegF) //如果range选的是10K type2 F 就显示 F
//				{	
//					Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[COMMON_CHANNEL].value / 100, TOP_AREA_DISP_UNIT_F);
//				}
//				else
//				{
//					Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[COMMON_CHANNEL].value / 100, TOP_AREA_DISP_UNIT_C);
//				}
//			}
//		}
//		else
		{
			char type,num;
			//if(Setting_Info.reg.display_lcd.lcddisplay[0] == 1) // modbus
			{				
				if(digital_top_area_type != Setting_Info.reg.display_lcd.lcd_mod_reg.npoint.point_type)
				{
					digital_top_area_type = Setting_Info.reg.display_lcd.lcd_mod_reg.npoint.point_type;
					digital_top_area_changed = 1;
//					disp_str(FORM15X30, SCH_XPOS,  0, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);					
//					disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "            ",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
//					disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
//					disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT * 2 - 7, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
				}
				if(digital_top_area_num != Setting_Info.reg.display_lcd.lcd_mod_reg.npoint.number - 1)
				{
					digital_top_area_num = Setting_Info.reg.display_lcd.lcd_mod_reg.npoint.number - 1;
					digital_top_area_changed = 1;
//					disp_str(FORM15X30, SCH_XPOS,  0, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);					
//					disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "            ",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
//					disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
//					disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT * 2 - 7, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
				}
			
				type = digital_top_area_type;
				num = digital_top_area_num;
				
				if(digital_top_area_changed)
				{
					digital_top_area_changed = 0;
					disp_str(FORM15X30, SCH_XPOS,  0, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);					
					disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "            ",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
					disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
					disp_str(FORM15X30, SCH_XPOS,  CH_HEIGHT * 2 - 7, "              ",SCH_COLOR,TSTAT8_BACK_COLOR);
				}
				
				if(type == IN)
				{					
					if(inputs[num].digital_analog == 1)
					{
						flag_digital_top_area = 0;		
						if(inputs[num].range == R10K_40_250DegF)
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 100, TOP_AREA_DISP_UNIT_F);
						else if(inputs[num].range == R10K_40_120DegC)
						{
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 100, TOP_AREA_DISP_UNIT_C);
						}
						else if(inputs[num].range == 27)  // humidity
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 100, TOP_AREA_DISP_UNIT_RH);
						else 
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 1000, TOP_AREA_DISP_UNIT_NONE);
					
					}
					else
					{			
						flag_digital_top_area = 1;						
						memcpy(UI_DIS_TOP, inputs[digital_top_area_num].label, 9);
						disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
						
						
						if(inputs[num].control)
						{
							if(inputs[num].range == OFF_ON)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ON",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == CLOSED_OPEN)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OPEN",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == STOP_START)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "STRAT",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == DISABLED_ENABLED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ENABLED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == NORMAL_ALARM)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ALARM",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == UNOCCUPIED_OCCUPIED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OCC",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == LOW_HIGH)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "HIGH",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ON",SCH_COLOR,TSTAT8_BACK_COLOR);
						}
						else
						{
							if(inputs[num].range == OFF_ON)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OFF",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == CLOSED_OPEN)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "CLOSED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == STOP_START)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "STOP",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == DISABLED_ENABLED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "DISABLED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == NORMAL_ALARM)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "NORMAL",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == UNOCCUPIED_OCCUPIED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "UNOCC",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(inputs[num].range == LOW_HIGH)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE1_POS, "LOW",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OFF",SCH_COLOR,TSTAT8_BACK_COLOR);
						}
					
					}
				}
				else if(type == OUT)
				{					
					if(outputs[num].digital_analog == 1)
					{
						flag_digital_top_area = 0;	
						// tbd:
//						if(outputs[num].range == R10K_40_250DegF)
//							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 100, TOP_AREA_DISP_UNIT_F);
//						else if(inputs[num].range == R10K_40_120DegC)
//							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 100, TOP_AREA_DISP_UNIT_C);
//						else if(inputs[num].range == 27)  // humidity
//							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 100, TOP_AREA_DISP_UNIT_RH);
//						else 
//							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, inputs[num].value / 1000, TOP_AREA_DISP_UNIT_NONE);
					}
					else
					{
						flag_digital_top_area = 1;

						memcpy(UI_DIS_TOP, outputs[digital_top_area_num].label, 9);
						disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
						if(outputs[num].control)
						{
							if(outputs[num].range == OFF_ON)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ON",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == CLOSED_OPEN)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OPEN",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == STOP_START)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "START",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == DISABLED_ENABLED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ENABLED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == NORMAL_ALARM)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ALARM",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == UNOCCUPIED_OCCUPIED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OCC",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == LOW_HIGH)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "HIGH",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ON",SCH_COLOR,TSTAT8_BACK_COLOR);
						}
						else
						{
							if(outputs[num].range == OFF_ON)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OFF",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == CLOSED_OPEN)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "CLOSED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == STOP_START)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "STOP",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == DISABLED_ENABLED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "DISABLED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == NORMAL_ALARM)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "NORMAL",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == UNOCCUPIED_OCCUPIED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "UNOCC",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(outputs[num].range == LOW_HIGH)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE1_POS, "LOW",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OFF",SCH_COLOR,TSTAT8_BACK_COLOR);
						}
					
					}
				}
				else if(type == VAR)
				{
					if(vars[num].digital_analog == 1)
					{
						flag_digital_top_area = 0;	
						if(vars[num].range == degF) //如果range选的是10K type2 F 就显示 F
						{	
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 100, TOP_AREA_DISP_UNIT_F);
						}
						else	if(vars[num].range == degC) //如果range选的是10K type2 F 就显示 F
						{
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 100, TOP_AREA_DISP_UNIT_C);
						}
						else	if(vars[num].range == KPa) //如果range选的是10K type2 F 就显示 F
						{
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 1000, TOP_AREA_DISP_UNIT_kPa);
						}
						else	if(vars[num].range == Pa) //如果range选的是10K type2 F 就显示 F
						{
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 1000, TOP_AREA_DISP_UNIT_Pa);
						}
						else	if(vars[num].range == RH)
						{
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 1000, TOP_AREA_DISP_UNIT_RH);
						}
						else 	if(vars[num].range == ppm)
						{
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 1000, TOP_AREA_DISP_UNIT_PPM);
						}
						else
							Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].value / 1000, TOP_AREA_DISP_UNIT_NONE);
					}
					else
					{
						flag_digital_top_area = 1;						
						
						memcpy(UI_DIS_TOP, vars[digital_top_area_num].label, 9);
						disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
						
						if(vars[num].control)
						{
							if(vars[num].range == OFF_ON)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ON",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == CLOSED_OPEN)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OPEN",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == STOP_START)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "START",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == DISABLED_ENABLED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ENABLED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == NORMAL_ALARM)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ALARM",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == UNOCCUPIED_OCCUPIED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OCC",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == LOW_HIGH)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "HIGH",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "ON",SCH_COLOR,TSTAT8_BACK_COLOR);
						}
						else
						{
							if(vars[num].range == OFF_ON)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OFF",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == CLOSED_OPEN)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "CLOSED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == STOP_START)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "STOP",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == DISABLED_ENABLED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "DISABLED",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == NORMAL_ALARM)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "NORMAL",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == UNOCCUPIED_OCCUPIED)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "UNOCC",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else if(vars[num].range == LOW_HIGH)
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE1_POS, "LOW",SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
							else
								disp_str(FORM15X30, SCH_XPOS,  IDLE_LINE2_POS, "OFF",SCH_COLOR,TSTAT8_BACK_COLOR);
						}
					
					}
//					else
//					{						
//						Top_area_display(TOP_AREA_DISP_ITEM_TEMPERATURE, vars[num].control, TOP_AREA_DISP_UNIT_NONE);
//					}
//					else
//					{// uint is not 
//						// tbd: add more
//						
//					}
				}
				// ..... tbd: add more type
			}
// 			if(Setting_Info.reg.display_lcd.lcddisplay[0] == 1) // bacnet
//			{
//			}
			
		}
		
		if(count_left_key > 5) 
			disp_index = 0;
		else
			count_left_key++;

		if(disp_index == 1)
		{
			if(flag_digital_top_area == 1)
				disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, UI_DIS_LINE1,SCH_COLOR,TSTAT8_BACK_COLOR1);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, UI_DIS_LINE2,SCH_COLOR,TSTAT8_BACK_COLOR);
			disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, UI_DIS_LINE3,SCH_COLOR,TSTAT8_BACK_COLOR);
		}
		else if(disp_index == 2)
		{
			if(flag_digital_top_area == 1)
				disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, UI_DIS_LINE1,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, UI_DIS_LINE2,SCH_COLOR,TSTAT8_BACK_COLOR1);
			disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, UI_DIS_LINE3,SCH_COLOR,TSTAT8_BACK_COLOR);
		}
		else if(disp_index == 3)
		{
			if(flag_digital_top_area == 1)
				disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, UI_DIS_LINE1,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, UI_DIS_LINE2,SCH_COLOR,TSTAT8_BACK_COLOR);
			disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, UI_DIS_LINE3,SCH_COLOR,TSTAT8_BACK_COLOR1);
		}
		else if(disp_index == 4) // top area
		{
			if(flag_digital_top_area == 1)
				disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR1);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, UI_DIS_LINE1,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, UI_DIS_LINE2,SCH_COLOR,TSTAT8_BACK_COLOR);
			disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, UI_DIS_LINE3,SCH_COLOR,TSTAT8_BACK_COLOR);
		}
		else
		{
			if(flag_digital_top_area == 1)
				disp_str_16_24(FORM15X30, SCH_XPOS + 20,  IDLE_LINE1_POS, UI_DIS_TOP,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  SETPOINT_POS, UI_DIS_LINE1,SCH_COLOR,TSTAT8_BACK_COLOR);//TSTAT8_BACK_COLOR
			disp_str(FORM15X30, SCH_XPOS,  FAN_MODE_POS, UI_DIS_LINE2,SCH_COLOR,TSTAT8_BACK_COLOR);
			disp_str(FORM15X30, SCH_XPOS,  SYS_MODE_POS, UI_DIS_LINE3,SCH_COLOR,TSTAT8_BACK_COLOR);
		}

        //sprintf(test_char, "%d", SSID_Info.IP_Wifi_Status); //测试用，在屏幕左上角 显示 wifi状态的数值;
        //disp_str(FORM15X30, 0, 0, test_char, SCH_COLOR, TSTAT8_BACK_COLOR);

//        if (SSID_Info.IP_Wifi_Status == WIFI_NORMAL) 
//        {
//            disp_icon(26, 26, wificonnect, 210, 0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
//        }
//         //  
//        else if (SSID_Info.IP_Wifi_Status == WIFI_NO_WIFI || SSID_Info.IP_Wifi_Status == WIFI_NONE)
//        {
//            disp_null_icon(26, 26, 0, 210, 0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
//        }
//        else
//            disp_icon(26, 26, wifinocnnct, 210, 0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				
			if(SSID_Info.IP_Wifi_Status == WIFI_NORMAL)//在屏幕右上角显示wifi的状态
			{
				if(SSID_Info.rssi < 70)		
					disp_icon(26, 26, wifi_4, 210,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				else if(SSID_Info.rssi < 80)							
					disp_icon(26, 26, wifi_3, 210,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				else if(SSID_Info.rssi < 90)							
					disp_icon(26, 26, wifi_2, 210,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				else							
					disp_icon(26, 26, wifi_1, 210,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
			}
			else	if((SSID_Info.IP_Wifi_Status == WIFI_NO_CONNECT)
				|| (SSID_Info.IP_Wifi_Status == WIFI_SSID_FAIL))
					disp_icon(26, 26, wifi_0, 210,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				// if WIFI_NONE, do not show wifi flag
			else //if((SSID_Info.IP_Wifi_Status == WIFI_NO_WIFI)
				disp_icon(26, 26, wifi_none, 210,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
			
						
			// show TX,RX
			
			if(flagLED_uart0_tx > 0)
			{
				if(count_tx++ % 2 == 0)
					disp_icon(13, 26, cmnct_send, 	0,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				else
					disp_null_icon(13, 26, 0, 0,0,TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
			}
			else
			{
				count_tx = 0;
				disp_null_icon(13, 26, 0, 0,0,TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);//(26, 26, cmnct_icon, 	0,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
			}
			
			if(flagLED_uart0_rx > 0)
			{
				// if TX on, then RX off
				if(count_tx % 2 == 1)
					count_rx = 0;
				if(count_rx++ % 2 == 1)
					disp_icon(13, 26, cmnct_rcv, 	13,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
				else
					disp_null_icon(13, 26, 0, 13,0,TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
			}
			else
			{
				count_rx = 0;
				disp_null_icon(13, 26, 0, 13,0,TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);//(26, 26, cmnct_icon, 	0,	0, TSTAT8_CH_COLOR, TSTAT8_BACK_COLOR);
			}
			
			if(flagLED_uart0_tx > 0)
				flagLED_uart0_tx = 0;
			if(flagLED_uart0_rx > 0)
				flagLED_uart0_rx = 0;
				
}


extern uint8_t item_to_adjust;
uint8_t check_msv_data_len(uint8_t index)
{
	char j;
	char len;
	len = 0;
	
	for(j = 0; j < STR_MSV_MULTIPLE_COUNT;j++)
	{
		if(msv_data[index][j].status != 0)
		{
			len++;
		}
		else
		{
			return len;
		}
	}
	return len;
}


void MenuIdle_keycope(uint16 key_value)
{
    uint8 i;
    uint8 temp_value = 0;
	switch(key_value /*& KEY_SPEED_MASK*/)
	{
		case 0:
			break;
		case KEY_UP_MASK: 
			count_left_key = 0;
			if((disp_index >= 1) && (disp_index <= 3))
			{
				if ((vars[disp_index - 1].range >= 101) && (vars[disp_index - 1].range <= 103))  // 101 102 103 	MSV range
				{
					char len;
					len = check_msv_data_len(vars[disp_index - 1].range - 101);
					for (i = 0; i < len; i++)
					{
						if (vars[disp_index - 1].value / 1000 == msv_data[vars[disp_index - 1].range - 101][i].msv_value)
						{
							temp_value = i;
							break;
						}
					}

					for (i = temp_value; i < 7; i++)
					{
						if(strlen(msv_data[vars[disp_index - 1].range - 101][i + 1].msv_name) != 0
							&& msv_data[vars[disp_index - 1].range - 101][i + 1].msv_name[0] != 0xff)
						{
							vars[disp_index - 1].value = msv_data[vars[disp_index - 1].range - 101][i + 1].msv_value * 1000;
							break;
						}
					}
				}
				else
				{
					if(vars[disp_index - 1].digital_analog == 0)
					{
						if(vars[disp_index - 1].control == 0)
							vars[disp_index - 1].control = 1;
						else
							vars[disp_index - 1].control = 0;
					}
					else
					{
						if(vars[disp_index - 1].value < 999 * 1000)
								vars[disp_index - 1].value = vars[disp_index - 1].value + 1000;
							else
								vars[disp_index - 1].value = 0;
					}
				}
			}
			else // disp_index == 4
			{
				digital_top_area_changed = 1;
				if(digital_top_area_type == IN)
				{
					inputs[digital_top_area_num].control = ((inputs[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == VAR)
				{
					vars[digital_top_area_num].control = ((vars[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == OUT)
				{
					outputs[digital_top_area_num].control = ((outputs[digital_top_area_num].control) == 0) ? 1 : 0;
					if(outputs[digital_top_area_num].control) 					
						set_output_raw(digital_top_area_num,1000);
					else 
						set_output_raw(digital_top_area_num,0);	
				}
			}

			write_page_en[VAR] = 1;
			ChangeFlash = 1;
			break;
		case KEY_SPEED_10 | KEY_UP_MASK:	
			count_left_key = 0;
			if((disp_index >= 1) && (disp_index <= 3))
			{
				if ((vars[disp_index - 1].range >= 101) && (vars[disp_index - 1].range <= 103))  // 101 102 103 	MSV range
				{					
					char len;
					len = check_msv_data_len(vars[disp_index - 1].range - 101);
					for (i = 0; i < len; i++)
					{
						if (vars[disp_index - 1].value / 1000 == msv_data[vars[disp_index - 1].range - 101][i].msv_value)
						{
							temp_value = i;
							break;
						}
					}

					for (i = temp_value; i < 7; i++)
					{
						if(strlen(msv_data[vars[disp_index - 1].range - 101][i + 1].msv_name) != 0
							&& msv_data[vars[disp_index - 1].range - 101][i + 1].msv_name[0] != 0xff)
						{
							vars[disp_index - 1].value = msv_data[vars[disp_index - 1].range - 101][i + 1].msv_value * 1000;
							break;
						}
					}
				}
				else
				{
					if(vars[disp_index - 1].digital_analog == 0)
					{
						if(vars[disp_index - 1].control == 0)
							vars[disp_index - 1].control = 1;
						else
							vars[disp_index - 1].control = 0;
					}
					else
					{
					if(vars[disp_index - 1].value < 999 * 1000)
							vars[disp_index - 1].value = vars[disp_index - 1].value + 10000;
						else
							vars[disp_index - 1].value = 0;
					}
				}
			}
			else // disp_index == 4
			{
				digital_top_area_changed = 1;
				if(digital_top_area_type == IN)
				{
					inputs[digital_top_area_num].control = ((inputs[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == VAR)
				{
					vars[digital_top_area_num].control = ((vars[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == OUT)
				{
					outputs[digital_top_area_num].control = ((outputs[digital_top_area_num].control) == 0) ? 1 : 0;
					if(outputs[digital_top_area_num].control) 					
						set_output_raw(digital_top_area_num,1000);
					else 
						set_output_raw(digital_top_area_num,0);	
				}
			}

			write_page_en[VAR] = 1;
			ChangeFlash = 1;
			break;

		case KEY_DOWN_MASK:
			count_left_key = 0;			
			if((disp_index >= 1) && (disp_index <= 3))
			{
				if ((vars[disp_index - 1].range >= 101) && (vars[disp_index - 1].range <= 103))  // 101 102 103 	MSV range
				{
					//if(vars[disp_index - 1].range == 101)  //判断range 是不是多态，是的话 调整多态的值;
					{
						// check the lenght of msv_data
						char len;
						len = check_msv_data_len(vars[disp_index - 1].range - 101);
							for (i = 0; i < len; i++)
							{
									if (vars[disp_index - 1].value / 1000 == msv_data[vars[disp_index - 1].range - 101][i].msv_value)
									{
											temp_value = i;
											break;
									}
							}

							for (i = temp_value; i > 0; i--)
							{
									if (strlen(msv_data[vars[disp_index - 1].range - 101][i - 1].msv_name) != 0)
									{
											vars[disp_index - 1].value = msv_data[vars[disp_index - 1].range - 101][i - 1].msv_value * 1000;
											break;
									}
							}
					}
//					else
//					{
//						if(vars[disp_index - 1].value > 1000)
//							vars[disp_index - 1].value = vars[disp_index - 1].value - 1000;
//						else
//							vars[disp_index - 1].value = STR_MSV_MULTIPLE_COUNT * 1000;
//					}
				}
				else
				{
					if(vars[disp_index - 1].digital_analog == 0)
					{
						if(vars[disp_index - 1].control == 0)
							vars[disp_index - 1].control = 1;
						else
							vars[disp_index - 1].control = 0;
					}
					else
					{
//					if(vars[disp_index - 1].value > 1000)
							vars[disp_index - 1].value = vars[disp_index - 1].value - 1000;
//						else
//							vars[disp_index - 1].value = 99 * 1000;
					}
				}
			}
			else // disp_index == 4
			{
				digital_top_area_changed = 1;
				if(digital_top_area_type == IN)
				{
					inputs[digital_top_area_num].control = ((inputs[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == VAR)
				{
					vars[digital_top_area_num].control = ((vars[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == OUT)
				{
					outputs[digital_top_area_num].control = ((outputs[digital_top_area_num].control) == 0) ? 1 : 0;
					if(outputs[digital_top_area_num].control) 					
						set_output_raw(digital_top_area_num,1000);
					else 
						set_output_raw(digital_top_area_num,0);	
				}
			}
		
			write_page_en[VAR] = 1;
			ChangeFlash = 1;
			break;		
		case KEY_SPEED_10 | KEY_DOWN_MASK: 
			count_left_key = 0;			
			if((disp_index >= 1) && (disp_index <= 3))
			{
				if ((vars[disp_index - 1].range >= 101) && (vars[disp_index - 1].range <= 103))  // 101 102 103 	MSV range
				{
					char len;
					len = check_msv_data_len(vars[disp_index - 1].range - 101);
					for (i = 0; i < len; i++)
					{
							if (vars[disp_index - 1].value / 1000 == msv_data[vars[disp_index - 1].range - 101][i].msv_value)
							{
									temp_value = i;
									break;
							}
					}

					for (i = temp_value; i > 0; i--)
					{
							if (strlen(msv_data[vars[disp_index - 1].range - 101][i - 1].msv_name) != 0)
							{
									vars[disp_index - 1].value = msv_data[vars[disp_index - 1].range - 101][i - 1].msv_value * 1000;
									break;
							}
					}
				}
				else
				{
					if(vars[disp_index - 1].digital_analog == 0)
					{
						if(vars[disp_index - 1].control == 0)
							vars[disp_index - 1].control = 1;
						else
							vars[disp_index - 1].control = 0;
					}
					else
					{
						vars[disp_index - 1].value = vars[disp_index - 1].value - 10000;
					}
				}
			}
			else // disp_index == 4
			{
				digital_top_area_changed = 1;
				if(digital_top_area_type == IN)
				{
					inputs[digital_top_area_num].control = ((inputs[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == VAR)
				{
					vars[digital_top_area_num].control = ((vars[digital_top_area_num].control) == 0) ? 1 : 0;
				}
				else if(digital_top_area_type == OUT)
				{
					outputs[digital_top_area_num].control = ((outputs[digital_top_area_num].control) == 0) ? 1 : 0;
					if(outputs[digital_top_area_num].control) 					
						set_output_raw(digital_top_area_num,1000);
					else 
						set_output_raw(digital_top_area_num,0);	
				}
			}
		
			write_page_en[VAR] = 1;
			ChangeFlash = 1;
			break;
		
		case KEY_LEFT_MASK:
			// change SETP, FAN , SYS
			if(flag_digital_top_area == 1)
			{
				if(disp_index < 4) disp_index++;
				else 
					disp_index = 1;
			}
			else
			{
				if(disp_index < 3) disp_index++;
				else 
					disp_index = 1;
			}
			flag_left_key = 1;
			count_left_key = 0;
			break;
		case KEY_RIGHT_MASK:
			// go into main menu
			//vars[19].value += 1000;
			break;
		case KEY_LEFT_RIGHT_MASK:
			update_menu_state(MenuMain);
			break;
		default:
			break;
	}
}



