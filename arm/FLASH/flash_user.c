#include "product.h"
#include "flash_user.h"
#include "define.h"
#include "stmflash.h"
#include "user_data.h"
#include "scan.h"
#include "sntpc.h"
//#include "delay.h"
#include "wifi.h"
#include "24cxx.h" 
#include "main.h"
#include <string.h>

uint8_t write_page_en[26]  = {0} ;  //fandu ???? 25??26 ??????msv???
//uint8_t write_priotry_array;

static uint8_t tempbuf[20000] = {0};
/* Compare buffer: skip erase/write when flash page already matches RAM */
static uint8_t flash_page_cmp[2048];

STR_Flash_POS xdata Flash_Position[24];
STR_flag_flash 	far bac_flash;

// page 125 0x0803 e800  - 0x0803 efff    2K  OUT
// page 126 0x0803 f000  - 0x0803 f7ff    2K  IN
// page 127 0x0803 f800  - 0x0803 ffff    2K  VAR


// BASE_ADDR for IN,OUT,VAR...
#define FLASH_BASE_ADDR	0x8068000  // - 8078000 len 64k
// CODE ADDR for code
#define FLASH_CODE_ADDR	0x8060000  // - 8068000 code len 32k

// miscllion
#define FLASH_OTHER_ADDR 	0x8078000 // - other 2k
#define FLASH_OTHER_ADDR2 0x8078800 // - other 2k
#define FLASH_OTHER_ADDR3 0x8079000 // - other 2k  for output relinquish
#define FLASH_OTHER_ADDR4 0x8079800 // - other 2k
/* Spare pages after email (0x8079800+400). Shadow-write NEW data here first so
 * erase-then-power-loss on the live page can finish from backup. */
#define FLASH_SHADOW_DATA  0x807A000
#define FLASH_SHADOW_META  0x807A800
#define FLASH_SH_MAGIC     0xA55A
#define FLASH_SH_NEED      0x0001

// other PAGE1 2K

#define BASE_WIFI_SETTING			0x8078000  // length is 160
#define BASE_PANEL_NAME     	0x80780a0  // 20
#define BASE_DYNDNS_DONAME		0x80780c0  // 32
#define BASE_DYNDNS_USER			0x80780e0  // 32
#define BASE_DYNDNS_PASS			0x8078110  // 32
#define BASE_TST_NAME					0x8078130  // length is 1000H
#define BASE_MON_OPERATE_TIME 0x8078400 // length is 80, the lenght is at least 48
#define BASE_WEATHER  				0x8078450  // lenth is 0xc0, 192
#define BASE_SNTP_SERVER			0x8078510  // lenth is 0x20, 30
#define BASE_WEEKLY_ONOFF			0x8078530  // lenght is 0x240, 576
//#define BASE_EMAIL_SETTING		0x8078770  // length is 200
	


// other page2
#define BASE_MSV_DATA					0x8078800  	// lenght is  3x8x23  0x228, 552
#define BASE_OUT_RELINQUISH	  0x8078a28   // length is 4*MAX_OUTS  0x100  256
#define BASE_DIS_CONFIG			  0x8078b28  // length is 7
#define BASE_VENDOR_INFO			0x8078b30		// Length is 40
#define BASE_MSV_DATA2				0x8078b58		// length is 184
#define BASE_VAR_UNIT					0x8078c10   // length is 100 MAX_VAR_UNIT*VAR_UNIT_SIZE

// other page3
#define BASE_PRI_ARRAY				0x8079000  	// lenght is  2K

// other page4
#define BASE_EMAIL_SETTING		0x8079800  	// lenght is  400

void QuickSoftReset(void);
/* Label/desc all 0xFF — typical external-RAM / bus corruption pattern (shows as ÿ). */
static U8_T mem_is_all_ff(const void *p, U8_T len)
{
	const U8_T *b = (const U8_T *)p;
	U8_T j;
	for(j = 0; j < len; j++)
	{
		if(b[j] != 0xff)
			return 0;
	}
	return 1;
}

static U8_T mem_is_all_zero(const void *p, U16_T len)
{
	const U8_T *b = (const U8_T *)p;
	U16_T j;
	for(j = 0; j < len; j++)
	{
		if(b[j] != 0)
			return 0;
	}
	return 1;
}

/* RAM looks like ExtSRAM / bus garbage (0xFF). Empty unused slots are all 0 — that is normal. */
static U8_T out_ram_corrupt(const Str_out_point *o)
{
	if(((U8_T)o->auto_manual == 0xff) && ((U8_T)o->control == 0xff)
		&& ((U8_T)o->out_of_service == 0xff)
		&& ((U8_T)o->range == 0xff) && (o->pwm_period == 0xff))
		return 1;
	if(mem_is_all_ff(o->label, 9) && mem_is_all_ff(o->description, 19))
		return 1;
	return 0;
}

static U8_T in_ram_corrupt(const Str_in_point *in)
{
	if(((U8_T)in->filter == 0xff) && ((U8_T)in->control == 0xff)
		&& ((U8_T)in->auto_manual == 0xff) && ((U8_T)in->digital_analog == 0xff)
		&& ((U8_T)in->calibration_sign == 0xff) && (in->calibration_hi == 0xff)
		&& (in->calibration_lo == 0xff) && (in->range == 0xff))
		return 1;
	if(mem_is_all_ff(in->label, 9) && mem_is_all_ff(in->description, 21))
		return 1;
	return 0;
}

static U8_T var_ram_corrupt(const Str_variable_point *v)
{
	if((v->auto_manual == 0xff) && (v->digital_analog == 0xff)
		&& (v->control == 0xff) && (v->unused == 0xff)
		&& (v->range == 0xff))
		return 1;
	if(mem_is_all_ff(v->label, 9) && mem_is_all_ff(v->description, 21))
		return 1;
	return 0;
}

/* Flash slot is worth restoring from: not 0xFF garbage and not an empty unused (all-0) point.
 * Empty flash + 0xFF RAM used to false-trigger Check_Ram_Err and block VAR save. */
static U8_T out_flash_recoverable(const Str_out_point *fo)
{
	if(out_ram_corrupt(fo))
		return 0;
	if(mem_is_all_zero(fo, sizeof(*fo)))
		return 0;
	return 1;
}

static U8_T in_flash_recoverable(const Str_in_point *fi)
{
	if(in_ram_corrupt(fi))
		return 0;
	if(mem_is_all_zero(fi, sizeof(*fi)))
		return 0;
	return 1;
}

static U8_T var_flash_recoverable(const Str_variable_point *fv)
{
	if(var_ram_corrupt(fv))
		return 0;
	if(mem_is_all_zero(fv, sizeof(*fv)))
		return 0;
	return 1;
}

void restore_point_table_from_flash(U8_T table)
{
	U8_T pg;
	U32_T base_addr = Flash_Position[table].addr;
	U16_T len = Flash_Position[table].len;

	for(pg = 0; pg < len / 2048; pg++)
		STMFLASH_MUL_Read(FLASH_BASE_ADDR + base_addr + 2048 * pg,
			&tempbuf[2048 * pg], 2048);

	switch(table)
	{
	case OUT:
		memcpy(&outputs, &tempbuf, sizeof(Str_out_point) * MAX_OUTS);
		break;
	case IN:
		memcpy(&inputs, &tempbuf, sizeof(Str_in_point) * MAX_INS);
		break;
	case VAR:
		memcpy(&vars, &tempbuf, sizeof(Str_variable_point) * MAX_VARS);
		break;
	default:
		break;
	}
}

/*
 * RAM error = 0xFF garbage in RAM, and flash still has real (non-empty, non-FF) data.
 * Empty (all-0) unused points are normal. Does NOT clear write_page_en[] —
 * callers decide; clearing dirty here blocked intentional VAR saves.
 */
U8_T Check_Ram_Err(void)
{
	uint16_t hard_err;
	uint16_t i;
	uint16_t ret = 0;
	Str_out_point fo;
	Str_in_point fi;
	Str_variable_point fv;

	hard_err = 0;
	for(i = 0; i < MAX_OUTS; i++)
	{
		if(!out_ram_corrupt(&outputs[i]))
			continue;
		STMFLASH_MUL_Read(FLASH_BASE_ADDR + Flash_Position[OUT].addr
			+ (U32_T)i * sizeof(Str_out_point), (u8 *)&fo, sizeof(fo));
		if(!out_flash_recoverable(&fo))
			continue;
		hard_err++;
		if(hard_err > 2)
		{
			ret |= 0x01;
			break;
		}
	}

	hard_err = 0;
	for(i = 0; i < MAX_INS; i++)
	{
		if(!in_ram_corrupt(&inputs[i]))
			continue;
		STMFLASH_MUL_Read(FLASH_BASE_ADDR + Flash_Position[IN].addr
			+ (U32_T)i * sizeof(Str_in_point), (u8 *)&fi, sizeof(fi));
		if(!in_flash_recoverable(&fi))
			continue;
		hard_err++;
		if(hard_err > 2)
		{
			ret |= 0x02;
			break;
		}
	}

	hard_err = 0;
	for(i = 0; i < MAX_VARS; i++)
	{
		if(!var_ram_corrupt(&vars[i]))
			continue;
		STMFLASH_MUL_Read(FLASH_BASE_ADDR + Flash_Position[VAR].addr
			+ (U32_T)i * sizeof(Str_variable_point), (u8 *)&fv, sizeof(fv));
		if(!var_flash_recoverable(&fv))
			continue;
		hard_err++;
		if(hard_err > 2)
		{
			ret |= 0x04;
			break;
		}
	}
	return ret;
}


/* caclulate detailed position for every table */
void Flash_Inital(void)
{
	uint8_t loop;
	uint16_t baseAddr = 0;	 	
	uint16_t  len = 0;
	memset(write_page_en,0,26);
	for(loop = 0;loop < MAX_POINT_TYPE;loop++)
	{  		
		switch(loop)
		{	
	  case OUT:	
			baseAddr = 0;
			len = sizeof(Str_out_point) * MAX_OUTS;			
			break;
		case IN:
			baseAddr += len;
			len = sizeof(Str_in_point) * MAX_INS;
			break;
		case VAR:
			baseAddr += len;
			len = sizeof(Str_variable_point) * MAX_VARS;
			break;
		case CON:
			baseAddr += len;
			len = sizeof(Str_controller_point) * MAX_CONS;
			break;
		case WRT: 
			baseAddr += len;
			len = sizeof(Str_weekly_routine_point) * MAX_WR;
			break;
		case AR:
			baseAddr += len;
			len = sizeof(Str_annual_routine_point) * MAX_AR;
			break;
		case PRG:
			baseAddr += len;
			len = sizeof(Str_program_point) * MAX_PRGS;
			break;
	/*	case TBL:
			baseAddr += len;
			len = sizeof(Tbl_point) * MAX_TBLS * 16;  
			break;
		case TZ:
			baseAddr += len;
			len = sizeof(Str_totalizer_point) * MAX_TOTALIZERS;
			break;*/
		case AMON:
			baseAddr += len;
			len = sizeof(Str_monitor_point) * MAX_MONITORS;
			break; 
		case GRP:
			baseAddr += len;
			len = sizeof(Control_group_point) * MAX_GRPS;				
			break;
		case ALARMM:  
			baseAddr += len;
			len = sizeof(Alarm_point) * MAX_ALARMS;
			break;
		case ALARM_SET: 
			baseAddr += len;
			len = sizeof(Alarm_set_point) * MAX_ALARMS_SET;  
			break;
		case UNIT:
			baseAddr += len;
			len = sizeof(Units_element) * MAX_DIG_UNIT;
			break;
		case USER_NAME:
			baseAddr += len;
			len = sizeof(Password_point) * MAX_PASSW;
			break; 
		case WR_TIME:
			baseAddr += len; 
			len = sizeof(Wr_one_day) * MAX_WR * MAX_SCHEDULES_PER_WEEK;
			break;
		case AR_DATA:
			baseAddr += len; 
			len = sizeof(S8_T) * MAX_AR * AR_DATES_SIZE; 
			break;
		case TSTAT:
			baseAddr += len;
			len = sizeof(SCAN_DB) * 254/*SUB_NO*/;
			break;
		case GRP_POINT:			
			baseAddr += len;
			len = sizeof(Str_grp_element) * 240;
			break;
		case TBL:
			baseAddr += len;
			len = sizeof(Str_table_point) * MAX_TBLS ;  
			break;
#if SYNC_TSTAT
		case ID_ROUTION:			
			baseAddr += len;
			len = STORE_ID_LEN * 254;
			break;
#endif
		case ARRAY:
			baseAddr += len;
			len = sizeof(Str_array_point) * MAX_ARRAYS;
			break;
		default:
	//		len = 0;
			break; 
		}
		
		if(len % 2048 != 0) 
				len = (len / 2048 + 1) * 2048;	
	
		Flash_Position[loop].addr = baseAddr;
		Flash_Position[loop].len = len;	
		write_page_en[loop] = 0;
	}
	
	for(loop = 0;loop < MAX_PRGS;loop++)
		programs[loop].real_byte = 0;		
}

static void flash_feed_iwdg(void)
{
	IWDG_ReloadCounter();
}

static U8_T flash_page_matches(u32 addr, const u8 *src)
{
	U16_T i;
	const u16 *w = (const u16 *)src;
	for(i = 0; i < 1024; i++)
	{
		if(STMFLASH_ReadHalfWord(addr + (u32)i * 2) != w[i])
			return 0;
		if((i & 0x3f) == 0)
			flash_feed_iwdg();
	}
	return 1;
}

static void flash_program_page(u32 addr, u8 *src)
{
	U16_T i;
	u16 *w = (u16 *)src;
	/* Page must already be erased. Feed IWDG during long programs. */
	for(i = 0; i < 1024; i++)
	{
		STMFLASH_WriteHalfWord(addr + (u32)i * 2, w[i]);
		if((i & 0x3f) == 0)
			flash_feed_iwdg();
	}
}

static void flash_clear_shadow_need(void)
{
	/* NEED 0x0001 -> 0x0000 is 1-to-0, no erase. */
	if(STMFLASH_ReadHalfWord(FLASH_SHADOW_META + 2) == FLASH_SH_NEED)
		STMFLASH_WriteHalfWord(FLASH_SHADOW_META + 2, 0);
}

static void flash_mark_shadow_pending(u32 live_addr)
{
	u16 hw[4];
	hw[0] = FLASH_SH_MAGIC;
	hw[1] = FLASH_SH_NEED;
	hw[2] = (u16)(live_addr & 0xffff);
	hw[3] = (u16)(live_addr >> 16);
	flash_feed_iwdg();
	STMFLASH_ErasePage(FLASH_SHADOW_META);
	flash_feed_iwdg();
	STMFLASH_Write_NoCheck(FLASH_SHADOW_META, hw, 4);
}

/*
 * If a previous replace died after erasing the live page, shadow holds the
 * NEW image — copy it to live and clear NEED. Unlike the old "backup old
 * data" scheme, this finishes the save instead of rolling it back.
 */
static void flash_finish_pending_commit(void)
{
	u16 magic, need;
	u32 live_addr;
	static u8 shadow[2048];

	magic = STMFLASH_ReadHalfWord(FLASH_SHADOW_META);
	need = STMFLASH_ReadHalfWord(FLASH_SHADOW_META + 2);
	if(magic != FLASH_SH_MAGIC || need != FLASH_SH_NEED)
		return;

	live_addr = (u32)STMFLASH_ReadHalfWord(FLASH_SHADOW_META + 4)
		| ((u32)STMFLASH_ReadHalfWord(FLASH_SHADOW_META + 6) << 16);
	if(live_addr < FLASH_BASE_ADDR || live_addr > 0x807F800 || (live_addr & 0x7ff) != 0)
	{
		STMFLASH_Unlock();
		flash_clear_shadow_need();
		STMFLASH_Lock();
		return;
	}

	STMFLASH_MUL_Read(FLASH_SHADOW_DATA, shadow, 2048);
	if(flash_page_matches(live_addr, shadow))
	{
		STMFLASH_Unlock();
		flash_clear_shadow_need();
		STMFLASH_Lock();
		return;
	}

	flash_feed_iwdg();
	STMFLASH_Unlock();
	STMFLASH_ErasePage(live_addr);
	flash_feed_iwdg();
	flash_program_page(live_addr, shadow);
	if(flash_page_matches(live_addr, shadow))
		Test[17]++; /* completed interrupted commit from shadow */
	flash_clear_shadow_need();
	STMFLASH_Lock();
}

/*
 * Safe page replace (avoids erase-then-lose):
 *  1) Write NEW data to shadow page (live still intact)
 *  2) Mark NEED + live address in meta
 *  3) Erase+program live
 *  4) Clear NEED
 * Power loss after step 2/3 -> flash_finish_pending_commit() copies shadow to live.
 * No long IRQ-disable; feed IWDG throughout.
 */
static U8_T flash_replace_page(u32 page_addr, u8 *newdata)
{
	flash_feed_iwdg();
	STMFLASH_MUL_Read(page_addr, flash_page_cmp, 2048);
	if(memcmp(flash_page_cmp, newdata, 2048) == 0)
		return 1;

	/* 1) Shadow = new image first */
	flash_feed_iwdg();
	STMFLASH_Unlock();
	STMFLASH_ErasePage(FLASH_SHADOW_DATA);
	flash_feed_iwdg();
	flash_program_page(FLASH_SHADOW_DATA, newdata);
	if(!flash_page_matches(FLASH_SHADOW_DATA, newdata))
	{
		STMFLASH_Lock();
		return 0; /* live untouched */
	}

	/* 2) Commit marker — from here a reset will finish from shadow */
	flash_mark_shadow_pending(page_addr);

	/* 3) Update live */
	flash_feed_iwdg();
	STMFLASH_ErasePage(page_addr);
	flash_feed_iwdg();
	flash_program_page(page_addr, newdata);
	if(!flash_page_matches(page_addr, newdata))
	{
		/* Retry live from shadow */
		flash_feed_iwdg();
		STMFLASH_ErasePage(page_addr);
		flash_program_page(page_addr, newdata);
	}

	/* 4) Done */
	flash_clear_shadow_need();
	STMFLASH_Lock();
	flash_feed_iwdg();
	return flash_page_matches(page_addr, newdata) ? 1 : 0;
}

void Flash_Write_Mass(void)
{
	STR_flag_flash ptr_flash;
	uint32_t base_addr;

//	uint16_t	len = 0 ;
	uint16_t loop,i;	
	uint8_t	 page;

	flash_finish_pending_commit();

	/* Do NOT Check_Ram_Err / restore here. write_page_en[] means the host
	 * intentionally changed that table — always persist it. */

	for(loop = 0;loop < MAX_POINT_TYPE ;loop++)
	{
		if(write_page_en[loop] != 1)
			continue;

		ptr_flash.table = loop;	
		ptr_flash.len = Flash_Position[loop].len;
		base_addr = Flash_Position[loop].addr;
		/* Clear full allocated length so page padding is stable (0),
		 * otherwise leftover tempbuf bytes force unnecessary erase/write. */
		if(ptr_flash.len > 0 && ptr_flash.len <= sizeof(tempbuf))
			memset(tempbuf, 0, ptr_flash.len);
		switch(loop)
		{	
		case OUT: 
			memcpy(&tempbuf,&outputs,sizeof(Str_out_point) * MAX_OUTS);					
			break;
		case IN:
			memcpy(&tempbuf,&inputs,sizeof(Str_in_point) * MAX_INS);					
			break;
		case VAR:  
			memcpy(&tempbuf,&vars,sizeof(Str_variable_point) * MAX_VARS);					
			break;
		case CON:
			memcpy(&tempbuf,&controllers,sizeof(Str_controller_point) * MAX_CONS);					
			break;
		case WRT:  				
			memcpy(&tempbuf,&weekly_routines,sizeof(Str_weekly_routine_point) * MAX_WR);					
			break;
		case AR: 
			memcpy(&tempbuf,&annual_routines,sizeof(Str_annual_routine_point) * MAX_AR);					
			break;
		case PRG:  
			memcpy(&tempbuf,&programs,sizeof(Str_program_point) * MAX_PRGS);					
			break;
		case TBL:
			memcpy(&tempbuf,&custom_tab,sizeof(Str_table_point) * MAX_TBLS);					
			break;
	/*	case TZ:
			memcpy(&tempbuf,&totalizers,sizeof(Str_totalizer_point) * MAX_TOTALIZERS);					
			break;	*/
		case AMON:
			memcpy(&tempbuf,&monitors,sizeof(Str_monitor_point) * MAX_MONITORS);					
			break;	
	case GRP: 
			memcpy(&tempbuf,&control_groups,sizeof(Control_group_point) * MAX_GRPS);					
			break;
	case ARRAY:
			memcpy(&tempbuf,&arrays,sizeof(Str_array_point) * MAX_ARRAYS);					
			break; 
/*		case ALARMM: 
//				memcpy(&tempbuf,&alarms,sizeof(Alarm_point) * MAX_ALARMS);					
			break; 	*/		
		case ALARM_SET:  
			memcpy(&tempbuf,&alarms_set,sizeof(Alarm_set_point) * MAX_ALARMS_SET);					
			break; 
		case UNIT:
			memcpy(&tempbuf,&digi_units,sizeof(Units_element) * MAX_DIG_UNIT);					
			break; 
		case USER_NAME:
			memcpy(&tempbuf,&passwords,sizeof(Password_point) * MAX_PASSW);					
			break;		
		case WR_TIME: 
			memcpy(&tempbuf,&wr_times,sizeof(Wr_one_day) * 9 * MAX_WR);
			break;
		case AR_DATA:
			memcpy(&tempbuf,&ar_dates,46 * sizeof(S8_T) * MAX_AR);					
			break;
		case TSTAT:
			memcpy(&tempbuf,&scan_db,sizeof(SCAN_DB) * SUB_NO);					
			break;		
		case GRP_POINT:	
			//memcpy(&tempbuf,&group_data,sizeof(Str_grp_element) * 240);			
			memcpy(&tempbuf,&group_data_new,sizeof(Str_grp_element_new));	
			break;
#if SYNC_TSTAT
		case ID_ROUTION:
			for(i = 0;i < 254;i++)
			memcpy(&tempbuf[i * STORE_ID_LEN],&ID_Config[i], STORE_ID_LEN);		// store 15 bytes			
			break;
#endif
		default:	
			break;
	
		} 

		{
			uint32_t page_addr;

			if(loop == 15)  // store code
			{
				__disable_irq();
				STMFLASH_Unlock();
				Flash_Store_Code();
				STMFLASH_Lock();
				__enable_irq();
			}
			else 
			{
				for(page = 0;page < ptr_flash.len / 2048;page++)
				{
					page_addr = FLASH_BASE_ADDR + base_addr + 2048 * page;
					if(!flash_replace_page(page_addr, &tempbuf[2048 * page]))
						Test[41]++; /* page replace / verify failed */
				}				
			}
			
			write_page_en[loop] = 0 ;	
		}
	}	
	Flash_Write_Other();

	Flash_Write_Other_Page2();
	
}

void Flash_Read_Mass(void)
{
	STR_flag_flash ptr_flash;
	U16_T base_addr;
	U8_T loop,i;

	U8_T page;

	ptr_flash.index = 0;

	flash_finish_pending_commit();

	for(loop = 0;loop < MAX_POINT_TYPE;loop++)
	{
		ptr_flash.table = loop;
	
		ptr_flash.len = Flash_Position[loop].len;
		base_addr = Flash_Position[loop].addr;

	
		page = ptr_flash.len / 2048;
	
		for(page = 0;page < ptr_flash.len / 2048;page++)
		{
			if(FLASH_BASE_ADDR + base_addr + 2048 * page > 0x807F800)
			{
				break;
			}
			else
			{
				STMFLASH_MUL_Read(FLASH_BASE_ADDR + base_addr + 2048 * page,&tempbuf[page * 2048],2048);
			}
		}
		
		
		/* Skip only if entire table image is erased. First-10-byte check wrongly
		 * skipped restore when point0 was unused but later points had flash data. */
		{
			U16_T k, n;
			U8_T blank = 1;
			n = ptr_flash.len;
			if(n > sizeof(tempbuf))
				n = sizeof(tempbuf);
			for(k = 0; k < n; k++)
			{
				if(tempbuf[k] != 0xff)
				{
					blank = 0;
					break;
				}
			}
			if(blank)
				continue;
		}
 
		
		switch(loop)
		{
			case OUT: 
				memcpy(&outputs,&tempbuf,sizeof(Str_out_point) * MAX_OUTS);						
				break;
			case IN: 
				memcpy(&inputs,&tempbuf,sizeof(Str_in_point) * MAX_INS);					
				break;
			case VAR:
				memcpy(&vars,&tempbuf,sizeof(Str_variable_point) * MAX_VARS);					
				break;
			case CON:
				memcpy(&controllers,&tempbuf,sizeof(Str_controller_point) * MAX_CONS);					
				break;
			case WRT:  				
				memcpy(&weekly_routines,&tempbuf,sizeof(Str_weekly_routine_point) * MAX_WR);					
				break;
			case AR: 
				memcpy(&annual_routines,&tempbuf,sizeof(Str_annual_routine_point) * MAX_AR);					
				break;
			case PRG:  
				memcpy(&programs,&tempbuf,sizeof(Str_program_point) * MAX_PRGS);					
#if HANDLE_REBOOT_FLAG
                for ( i = 0; i < MAX_PRGS; i++)
                {
                    programs[i].on_off = 0;
                }
#endif
				break;
			case TBL:
				memcpy(&custom_tab,&tempbuf,sizeof(Str_table_point) * MAX_TBLS);					
				break;
		/*	case TZ:
				memcpy(&totalizers,&tempbuf,sizeof(Str_totalizer_point) * MAX_TOTALIZERS);					
				break;	*/
			case AMON:
				memcpy(&monitors,&tempbuf,sizeof(Str_monitor_point) * MAX_MONITORS);					
				break;	
		case GRP: 
				memcpy(&control_groups,&tempbuf,sizeof(Control_group_point) * MAX_GRPS);					
				break;
		case ARRAY:
				memcpy(&arrays,&tempbuf,sizeof(Str_array_point) * MAX_ARRAYS);					
				break; 
			case ALARMM: 
	//				memcpy(&alarms,&tempbuf,sizeof(Alarm_point) * MAX_ALARMS);					
				break; 			
			case ALARM_SET:  
				memcpy(&alarms_set,&tempbuf,sizeof(Alarm_set_point) * MAX_ALARMS_SET);					
				break; 
			case UNIT:
				memcpy(&digi_units,&tempbuf,sizeof(Units_element) * MAX_DIG_UNIT);					
				break; 
			case USER_NAME:
				memcpy(&passwords,&tempbuf,sizeof(Password_point) * MAX_PASSW);					
				break;		
			case WR_TIME: 
				memcpy(&wr_times,&tempbuf,sizeof(Wr_one_day) * 9 * MAX_WR);
			
				break;
			case AR_DATA:
				memcpy(&ar_dates,&tempbuf,46 * sizeof(S8_T) * MAX_AR);					
				break; 
			case TSTAT:
				memcpy(&scan_db,&tempbuf,sizeof(SCAN_DB) * SUB_NO);	
//				Get_Tst_DB_From_Flash(); 
				break;	   	
			case GRP_POINT:	
				//memcpy(&group_data,&tempbuf,sizeof(Str_grp_element) * 240);			
				memcpy(&group_data_new,&tempbuf,sizeof(Str_grp_element_new));			
				break;
#if SYNC_TSTAT
			case ID_ROUTION:	
				for(i = 0;i < MAX_ID;i++)
				{
					memcpy(&ID_Config[i],&tempbuf[i * STORE_ID_LEN],STORE_ID_LEN);	
					ID_Config_Sche[i] = ID_Config[i].Str.schedule;
				}
				break;
#endif
			default:
				break;

			}
	
	} 
	
	/* Do NOT zero/sanitize OUT/IN/VAR after Flash_Read_Mass.
	 * Check_Ram_Err means flash still has config ? memcpy above restored RAM.
	 * The old path cleared label/value to 0 and permanently lost data.
	 */

	for (i = 0; i < MAX_INS; i++)
	{
		if(inputs[i].range == 0)
			inputs[i].digital_analog = 1;
	}	
	for (i = 0; i < MAX_OUTS; i++)
	{
		if(outputs[i].range == 0)
			outputs[i].digital_analog = 1;
	}
	for (i = 0; i < MAX_VARS; i++)
	{
		if(vars[i].range == 0)
			vars[i].digital_analog = 1;
	}
	Flash_Read_Code();

	Flash_Read_Other();
//    Flash_Read_Other_Page2();
}

void Flash_Store_Code(void)
{
	U8_T i;
//	U16_T temp = 0;
//	U32_T base_addr = 0;
//	U16_T loop;
//  U8_T page;

	for(i = 0;i < MAX_PRGS;i++)
	{
		flash_feed_iwdg();
		STMFLASH_ErasePage(FLASH_CODE_ADDR + 2048 * i);	
		if(swap_word(programs[i].real_byte) > 0 && swap_word(programs[i].real_byte) <= CODE_ELEMENT * MAX_CODE)	
		{
			STMFLASH_WriteHalfWord(FLASH_CODE_ADDR + 2048 * i + 2000,swap_word(programs[i].real_byte));
			iap_write_appbin(FLASH_CODE_ADDR + 2048 * i,(uint8_t*)(&prg_code[i]), CODE_ELEMENT * MAX_CODE);
			
		}
		else
		{
			STMFLASH_WriteHalfWord(FLASH_CODE_ADDR + 2048 * i + 2000,0);
		}
		
	}

}


void Flash_Write_Other_Page2(void)
{
    if (write_page_en[25] == 1)
    {		
			__disable_irq();
			STMFLASH_Unlock();
			
				STMFLASH_ErasePage(FLASH_OTHER_ADDR2);
				iap_write_appbin(BASE_MSV_DATA, (u8 *)(msv_data), 3 * STR_MSV_MULTIPLE_COUNT * sizeof(multiple_struct));
				iap_write_appbin(BASE_MSV_DATA2, (u8 *)(&msv_data[3]), STR_MSV_MULTIPLE_COUNT * sizeof(multiple_struct));
				iap_write_appbin(BASE_OUT_RELINQUISH, (u8 *)(output_relinquish), 4 * MAX_OUTS);
				iap_write_appbin(BASE_VENDOR_INFO,(void *)(&bacnet_vendor_name),20);
				iap_write_appbin(BASE_VENDOR_INFO + 20,(void *)(&bacnet_vendor_product),20);
				iap_write_appbin(BASE_VAR_UNIT,(void *)(&var_unit),MAX_VAR_UNIT*VAR_UNIT_SIZE);
#if ARM_TSTAT_WIFI
			iap_write_appbin(BASE_DIS_CONFIG,Modbus.display_lcd.lcddisplay,sizeof(lcdconfig));
#endif				
			STMFLASH_Lock();			
			__enable_irq();
			write_page_en[25] = 0;
    }
}



void Flash_Write_Other(void)
{

 // name  20
	if(write_page_en[24] == 1)
	{
			__disable_irq();
			STMFLASH_Unlock();
			STMFLASH_ErasePage(FLASH_OTHER_ADDR);
			
		//	iap_write_appbin(BASE_SNTP_SERVER,(u8 *)(&sntp_server), 30);
			
			iap_write_appbin(BASE_PANEL_NAME,(u8 *)(&panelname),20);
#if ARM_MINI
			iap_write_appbin(BASE_DYNDNS_DONAME,dyndns_domain_name,MAX_DOMAIN_SIZE);

			iap_write_appbin(BASE_DYNDNS_USER,dyndns_username,MAX_USERNAME_SIZE);
			iap_write_appbin(BASE_DYNDNS_PASS,dyndns_password,MAX_PASSWORD_SIZE);

			// store name of tstat
			iap_write_appbin(BASE_TST_NAME,(u8 *)(&tstat_name),40/*MAX_ID*/ * 16);
			
			// store time of operating monitor 
			iap_write_appbin(BASE_MON_OPERATE_TIME,(u8 *)(&MISC_Info.reg.operate_time),MAX_MONITORS * 4);

			// store sntp
			iap_write_appbin(BASE_SNTP_SERVER,(u8 *)(&sntp_server), 30);
#endif		
			iap_write_appbin(BASE_WEEKLY_ONOFF,(u8 *)(&wr_time_on_off),576);
			
			//iap_write_appbin(BASE_EMAIL_SETTING,(u8 *)(&Email_Setting),sizeof(Str_Email_point));
#if (ARM_MINI || ARM_TSTAT_WIFI)			
			iap_write_appbin(BASE_WIFI_SETTING,(u8 *)(&SSID_Info),sizeof(STR_SSID));
#endif
			STMFLASH_Lock();			
			__enable_irq();	
		write_page_en[24] = 0;
	}

}


void Flash_Write_Output_PriArray(void)
{
   // if(write_priotry_array == 1)
    {		
			__disable_irq();
			STMFLASH_Unlock();
			
			STMFLASH_ErasePage(FLASH_OTHER_ADDR3);

			iap_write_appbin(BASE_PRI_ARRAY, (u8 *)(output_priority), 4 * 24 * 16);
							
			//write_priotry_array = 0;
			STMFLASH_Lock();
			__enable_irq();
    }
}

void Flash_Write_Email(void)
{

		__disable_irq();
		STMFLASH_Unlock();
		
		STMFLASH_ErasePage(FLASH_OTHER_ADDR4);
	
		iap_write_appbin(BASE_EMAIL_SETTING,(u8 *)(&Email_Setting),sizeof(Str_Email_point));
#if ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("write email\r\n");
	printf("server :%u %u %u %u\n",Email_Setting.reg.smtp_ip[0],Email_Setting.reg.smtp_ip[1],Email_Setting.reg.smtp_ip[2],Email_Setting.reg.smtp_ip[3]);	
	printf("smtp_domain :%s\n",Email_Setting.reg.smtp_domain);
	printf("user_name :%s\n",Email_Setting.reg.user_name);	
#endif				
		STMFLASH_Lock();
		__enable_irq();
    
}

void Flash_Read_Other(void)
{
	U16_T loop,loop1;
	U8_T temp[4];
//	U16_T loop1;
// gsm
#if 0//USB_HOST
	if(Modbus.usb_mode == 1)
	{
		apnlen = STMFLASH_ReadHalfWord(BASE_GSM_APN); 
		if(apnlen > MAX_GSM_APN)	apnlen = MAX_GSM_APN;
		
		STMFLASH_MUL_Read(BASE_GSM_APN,apnstr,MAX_GSM_APN);

		iplen = STMFLASH_ReadHalfWord(BASE_GSM_IP); 
		if(iplen > MAX_GSM_IP)	iplen = MAX_GSM_IP;
		STMFLASH_MUL_Read(BASE_GSM_IP,ipstr,MAX_GSM_IP);
	}
#endif
// name

	STMFLASH_MUL_Read(BASE_PANEL_NAME,(u8 *)(&panelname),20);
	if((panelname[0] == 0xff) && (panelname[1] == 0xff))
	{ // default, clear it to empty
		memset(panelname,0,20);
	}

#if ARM_MINI
	for(loop = 0;loop < MAX_DOMAIN_SIZE;loop++)
	{
		STMFLASH_MUL_Read(BASE_DYNDNS_DONAME ,dyndns_domain_name,MAX_DOMAIN_SIZE);
	}
	if((dyndns_domain_name[0] == 0xff) && (dyndns_domain_name[1] == 0xff))
	{ // default, clear it to empty
		memset(dyndns_domain_name,0,MAX_DOMAIN_SIZE);
	}
	
	for(loop = 0;loop < MAX_USERNAME_SIZE;loop++)
	{
		STMFLASH_MUL_Read(BASE_DYNDNS_USER,dyndns_username,MAX_USERNAME_SIZE);
	}
	if((dyndns_username[0] == 0xff) && (dyndns_username[1] == 0xff))
	{ // default, clear it to empty
		memset(dyndns_username,0,MAX_USERNAME_SIZE);
	}
	for(loop = 0;loop < MAX_PASSWORD_SIZE;loop++)
	{
		STMFLASH_MUL_Read(BASE_DYNDNS_PASS ,dyndns_password,MAX_PASSWORD_SIZE);
	}	
	if((dyndns_password[0] == 0xff) && (dyndns_password[1] == 0xff))
	{ // default, clear it to empty
		memset(dyndns_password,0,MAX_PASSWORD_SIZE);
	}
	
	// read name of tstat	
	STMFLASH_MUL_Read(BASE_TST_NAME,(u8 *)(&tstat_name),40 * 16);

	// read time of operating monitor  
	STMFLASH_MUL_Read(BASE_MON_OPERATE_TIME,(u8 *)(&MISC_Info.reg.operate_time), 4 * MAX_MONITORS);

// read sntp server
	STMFLASH_MUL_Read(BASE_SNTP_SERVER,(u8 *)(sntp_server), 30);	
#endif	
	STMFLASH_MUL_Read(BASE_WEEKLY_ONOFF,(u8 *)(wr_time_on_off), 576);

	STMFLASH_MUL_Read(BASE_MSV_DATA,(u8 *)(msv_data), 3 * STR_MSV_MULTIPLE_COUNT * sizeof(multiple_struct));
	STMFLASH_MUL_Read(BASE_MSV_DATA2,(u8 *)(&msv_data[3]), STR_MSV_MULTIPLE_COUNT * sizeof(multiple_struct));
	STMFLASH_MUL_Read(BASE_VAR_UNIT,(u8 *)(&var_unit), MAX_VAR_UNIT*VAR_UNIT_SIZE);
#if ARM_TSTAT_WIFI
	//TSTAT 10 ????? ??? ????TSTAT10 ???????????;
//	Test[11] = msv_data[1][0].msv_value;
//	Test[12] = msv_data[2][0].msv_value;
//	if(msv_data[0][0].msv_value == 0xffff)
//	{Test[15]++;
//		msv_data[0][0].msv_value = 0;
//    strcpy(msv_data[0][0].msv_name, "T1");
//    msv_data[0][1].msv_value = 1;
//    strcpy(msv_data[0][1].msv_name, "T2");
//    msv_data[0][2].msv_value = 2;
//    strcpy(msv_data[0][2].msv_name, "T3");
//		write_page_en[25] = 1;
//	}
	if(msv_data[1][0].msv_value == 0xffff)
	{
		msv_data[1][0].status = 1;
		msv_data[1][0].msv_value = 0;
    strcpy(msv_data[1][0].msv_name, "AUTO");
		msv_data[1][1].status = 1;
    msv_data[1][1].msv_value = 1;
    strcpy(msv_data[1][1].msv_name, "ON");
		msv_data[1][2].status = 1;
    msv_data[1][2].msv_value = 2;
    strcpy(msv_data[1][2].msv_name, "OFF");
		write_page_en[25] = 1;
	}
	if(msv_data[2][0].msv_value == 0xffff)
	{
		msv_data[2][0].status = 1;
    msv_data[2][0].msv_value = 0;
    strcpy(msv_data[2][0].msv_name, "AUTO");
		msv_data[2][1].status = 1;
    msv_data[2][1].msv_value = 1;
    strcpy(msv_data[2][1].msv_name, "COOL");
		msv_data[2][2].status = 1;
    msv_data[2][2].msv_value = 2;
    strcpy(msv_data[2][2].msv_name, "HEAT");
		write_page_en[25] = 1;
	}
	STMFLASH_MUL_Read(BASE_DIS_CONFIG, (u8 *)(Modbus.display_lcd.lcddisplay), sizeof(lcdconfig));

	if(Modbus.display_lcd.lcddisplay[0] == 0xff)
	{
		memset(Modbus.display_lcd.lcddisplay,0,sizeof(lcdconfig));
		Modbus.display_lcd.lcddisplay[0] = 1;
		Modbus.display_lcd.lcd_mod_reg.npoint.point_type = IN;
		if(Modbus.mini_type == MINI_T10P)
			Modbus.display_lcd.lcd_mod_reg.npoint.number = HI_COMMON_CHANNEL + 1;  // IN13 is internal termperature
		else
			Modbus.display_lcd.lcd_mod_reg.npoint.number = COMMON_CHANNEL + 1;  // IN9 is internal termperature
	}
#endif

	
	STMFLASH_MUL_Read(BASE_EMAIL_SETTING,(u8 *)(&Email_Setting),sizeof(Str_Email_point));
	
#if ARM_UART_DEBUG
	uart1_init(115200);
	DEBUG_EN = 1;
	printf("read email\r\n");
	printf("server :%u %u %u %u\n",Email_Setting.reg.smtp_ip[0],Email_Setting.reg.smtp_ip[1],Email_Setting.reg.smtp_ip[2],Email_Setting.reg.smtp_ip[3]);	
	printf("smtp_domain :%s\n",Email_Setting.reg.smtp_domain);
	printf("user_name :%s\n",Email_Setting.reg.user_name);	
#endif
#if (ARM_MINI || ARM_TSTAT_WIFI)	
	STMFLASH_MUL_Read(BASE_WIFI_SETTING,(u8 *)(&SSID_Info),sizeof(STR_SSID));

	if((SSID_Info.MANUEL_EN == 0xff) && (SSID_Info.IP_Auto_Manual == 0xff))
	{ // default, clear it to empty
		memset(&SSID_Info,0,sizeof(STR_SSID));
	}
#endif
	
	STMFLASH_MUL_Read(BASE_OUT_RELINQUISH, (u8 *)(output_relinquish), 4 * MAX_OUTS);
	
	for(loop = 0;loop < MAX_OUTS;loop++)
	{
		if((output_relinquish[loop] == 0xffff) || (output_relinquish[loop] == 0xffffffff) || (output_relinquish[loop] == 0))
		{
			output_relinquish[loop] = 0.0;
		}
	}
	
	STMFLASH_MUL_Read(BASE_PRI_ARRAY,(u8 *)(output_priority),4 * 24 * 16);
	
	// rev62.6 maybe write wrong priority array

	if((output_priority[0][0] == output_priority[0][1]) && (output_priority[0][2] == output_priority[0][3]))
	{
		if((output_priority[0][0] != 0xff)  &&(output_priority[0][2] != 0xff)
			&& ((output_priority[0][0] != 0xffff)  &&(output_priority[0][2] != 0xffff)))
			
		{
			// clear 
			__disable_irq();
		STMFLASH_Unlock();
		
		STMFLASH_ErasePage(FLASH_OTHER_ADDR3);	
		for(loop = 0;loop < 24;loop++)
		{	
			for (loop1 = 0;loop1 < 16; loop1++)
			{		
				output_priority[loop][loop1] = 0xffff;	
			}
		}		
		iap_write_appbin(BASE_PRI_ARRAY, (u8 *)(output_priority), 4 * 24 * 16);				
		//write_priotry_array = 0;
		STMFLASH_Lock();
		__enable_irq();
			
		}
	}

	for(loop = 0;loop < 24;loop++)
	{	
		for (loop1 = 0;loop1 < 16; loop1++)
		{			
			if(output_priority[loop][loop1] == 0xffff)
			{
				output_priority[loop][loop1] = 0xff;
			}
		}
	}
	
	STMFLASH_MUL_Read(BASE_VENDOR_INFO,(void *)(&bacnet_vendor_name),20);
	STMFLASH_MUL_Read(BASE_VENDOR_INFO + 20,(void *)(&bacnet_vendor_product),20);
	
}



void Flash_Read_Code(void)
{
	U8_T i;
//	U16_T loop;
	U16_T temp = 0;
//	U32_T base_addr = 0;
	Code_total_length = 0;
	for(i = 0;i < MAX_PRGS;i++)
	{	
		temp = STMFLASH_ReadHalfWord(FLASH_CODE_ADDR + 2048 * i + 2000);
		
		if(temp > CODE_ELEMENT * MAX_CODE)
			temp = 0;
		programs[i].real_byte = swap_word(temp); 
		if(swap_word(programs[i].real_byte) > 0 && swap_word(programs[i].real_byte) <= CODE_ELEMENT * MAX_CODE)	
		{
			STMFLASH_MUL_Read(FLASH_CODE_ADDR + 2048 * i,prg_code[i],CODE_ELEMENT * MAX_CODE);
		}
		else
		{
			memset(&prg_code[i] ,0, CODE_ELEMENT * MAX_CODE);
		}
		
	}	
}






