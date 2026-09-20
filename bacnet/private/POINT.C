#include "bacnet.h"
#include "point.h"
#include <string.h>
#include "user_data.h"
#include "COMMSUB.H"   // for add tstat table
#include "define.h"
#include "main.h"

#define DIG1 100

#if BAC_PRIVATE
extern void set_output_raw(uint8_t point,uint16_t value);
extern U8_T far output_pri_live[MAX_OUTS];
S16_T get_point_info_by_instacne(Point_Net * point);
uint16_t get_reg_from_list(uint8_t type,uint8_t index,uint8_t * len);
U16_T crc16(U8_T *p, U8_T length);
#if ARM_TSTAT_WIFI
extern uint8 digital_top_area_changed;
extern uint8 digital_top_area_num;
extern uint8 digital_top_area_type;
#endif

int write_NP_Bacnet_to_nodes(uint8_t object_type,uint32_t object_instance,uint8_t panel,uint8_t sub_id,float value);

#if SAVE_LOCAL_VAR_TO_E2
typedef struct 
{
	uint8_t point_type;
	uint8_t number;
	uint32_t value;
	uint8_t flag; // 1 - write, 0 - done
}STR_LOCAL_VAR;

#define MAX_LOCAL_VAR_TO_E2 50
STR_LOCAL_VAR Local_Var[MAX_LOCAL_VAR_TO_E2]; 
uint8_t max_local_var = 0;
void put_local_var_to_BUF(Point *point, uint32_t value)
{
    uint8_t i;
    /* search existing entry */
    for (i = 0; i < max_local_var && i < MAX_LOCAL_VAR_TO_E2; i++)
    {
        if ((Local_Var[i].point_type == point->point_type) && (Local_Var[i].number == point->number))
        {
            /* update existing */
            Local_Var[i].value = value;
						Local_Var[i].flag = 1;
            return;
        }
    }

    /* not found: add new if space available */
    if (max_local_var < MAX_LOCAL_VAR_TO_E2)
    {
        Local_Var[max_local_var].point_type = point->point_type;
        Local_Var[max_local_var].number = point->number;
        Local_Var[max_local_var].value = value;
				Local_Var[i].flag = 1;
        max_local_var++;
    }
    else
    {
        /* array full: choose policy ? here we ignore the new entry.
           Optionally overwrite oldest entry or handle error/logging. */
    }
}


void write_local_val_to_E2(void)
{
	static uint16_t count = 0;
	uint8_t i;
    /* search existing entry */
	Test[12]++;
  if (max_local_var == 0) return;  
	if(count++ % 600 == 0) // 10minute
	{
		for (i = 0; i < max_local_var && i < MAX_LOCAL_VAR_TO_E2; i++)
    {
        /* pack point_type and number into a 32-bit meta word:
           [31:8] unused, [7:0] number, [15:8] point_type (kept small and recoverable) */
			if(EEP_LOCAL_VAR_BASE + (i + 1) * sizeof(STR_LOCAL_VAR) < MAX_EEP_CONSTRANGE)
			{
				if(Local_Var[i].flag == 1)
				{Test[14]++;
					AT24CXX_WriteOneByte(EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR),Local_Var[i].point_type);
					AT24CXX_WriteOneByte(EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR) + 1,Local_Var[i].number);
					AT24CXX_WriteOneByte(EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR) + 2,(uint8_t)(Local_Var[i].value >> 24));
					AT24CXX_WriteOneByte(EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR) + 3,(uint8_t)(Local_Var[i].value >> 16));
					AT24CXX_WriteOneByte(EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR) + 4,(uint8_t)(Local_Var[i].value >> 8));
					AT24CXX_WriteOneByte(EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR) + 5,(uint8_t)(Local_Var[i].value));
					Local_Var[i].flag = 0;
				}
			}
			else
			{
				// error
			}
		}
		Test[13]++;
	}
}


void Check_lcoal_val(void)
{
    uint8_t i;
    uint8_t pt, num;
    uint8_t b0, b1, b2, b3;
    uint32_t value;

    max_local_var = 0; /* reset, will be filled from EEPROM */

    for (i = 0; i < MAX_LOCAL_VAR_TO_E2; i++)
    {
        uint32_t base = EEP_LOCAL_VAR_BASE + i * sizeof(STR_LOCAL_VAR);

        pt = AT24CXX_ReadOneByte(base + 0);
        num = AT24CXX_ReadOneByte(base + 1);

        /* treat empty slot when both bytes are 0xFF (default erased EEPROM) */
        if ((pt == 0xFF && num == 0xFF) || (pt == 0 && num == 0))
        {
            /* stop on first empty slot */
            break;
        }

				/* validate type and number:
           - type must be one of OUT, IN, VAR
           - number must be < 128
           Replace OUT/IN/VAR with your project's actual constants if different. */
        if (! (pt == (OUT + 1) || pt == (IN + 1) || pt == (VAR + 1)) )
        {
            continue; /* invalid type, skip */
        }
        if (num >= 128)
        {
            continue; /* invalid number, skip */
        }
				
        b0 = AT24CXX_ReadOneByte(base + 2);
        b1 = AT24CXX_ReadOneByte(base + 3);
        b2 = AT24CXX_ReadOneByte(base + 4);
        b3 = AT24CXX_ReadOneByte(base + 5);

        value = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | (uint32_t)b3;

        Local_Var[max_local_var].point_type = pt;
        Local_Var[max_local_var].number = num;
        Local_Var[max_local_var].value = value;

				if(pt == OUT + 1)
				{
					outputs[num].value = value;
				}
				if(pt == IN + 1)
				{
					inputs[num].value = value;					
				}
				if(pt == VAR + 1)
				{
					vars[num].value = value;
				}		
        max_local_var++;
    }
		
    /* clamp just in case */
    if (max_local_var > MAX_LOCAL_VAR_TO_E2) max_local_var = MAX_LOCAL_VAR_TO_E2;		
		
}

#endif

/*
 * ----------------------------------------------------------------------------
 * Function Name: rtrim
 * Purpose: retrim the text
 * Params:
 * Returns:
 * Note: this function is called in search_point funciton
 * ----------------------------------------------------------------------------
 */
S8_T *rtrim(S8_T *text)
{
	S16_T n,i;
	n = strlen(text);
	for (i=n-1;i>=0;i--)
			if (text[i]!=' ')
					break;
	text[i+1]='\0';
	return text;
}


#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
S16_T find_network_point( Point_Net *point )
{
	S16_T i;
	U8_T flag;
	NETWORK_POINTS *ptr;
	
	ptr = network_points_list;
	for( i = 0; i < MAXNETWORKPOINTS; i++, ptr++ )
	{			
		if(/*(point->network_number == ptr->point.network_number) &&*/ 
		(point->panel == ptr->point.panel) &&
		(point->point_type == ptr->point.point_type) &&
		(point->number == ptr->point.number) )
		{
			if(((ptr->point.sub_id == 0) && (point->sub_id == point->panel)) 
			|| ((ptr->point.sub_id == point->panel) && (point->sub_id == 0)) 
			||  (ptr->point.sub_id == point->sub_id))
			{
				break;
			}	
		}
		
		
	}	

	if( i < MAXNETWORKPOINTS )
	{		
	// add to scan tabel
		return i; // returns the index in the list
	} 
	else
	{

		return -1;
	}
}

/*
 * Fast lookup index for local points (point types OUT/IN/VAR).
 * Dimensions: [MAX_POINT_TYPE][MAX_VARS]. Value -1 means not present.
 */
static S16_T local_point_index[MAX_POINT_TYPE][MAX_VARS];

/* Return mapping for given point type index (pt = point_type-1) and number */
S16_T local_point_index_lookup(int pt, int num)
{
	if(pt < 0 || pt >= MAX_POINT_TYPE || num < 0 || num >= MAX_VARS) return -1;
	return local_point_index[pt][num];
}

/* Rebuild the lookup table from current local_points_list */
void rebuild_local_point_index(void)
{
	int i, ptn;
	/* initialize to -1 */
	for(ptn = 0; ptn < MAX_POINT_TYPE; ptn++)
	{
		for(i = 0; i < MAX_VARS; i++)
			local_point_index[ptn][i] = -1;
	}

	for(i = 0; i < MAXLOCALPOINTS; i++)
	{
		if(local_points_list[i].count)
		{
			ptn = local_points_list[i].point.point_type - 1;
			if(ptn >= 0 && ptn < MAX_POINT_TYPE)
			{
				int num = local_points_list[i].point.number;
				if(num >= 0 && num < MAX_VARS)
					local_point_index[ptn][num] = i;
			}
		}
	}
}

#endif

S16_T find_remote_point( Point_Net *point )
{
	S16_T i;
	U8_T flag;
	REMOTE_POINTS *ptr;

	ptr = remote_points_list;

//	if( point->network_number == 0xFF ||  point->network_number == 0)
//	{
//		point->network_number = Setting_Info.reg.network_number;//panel_net_info.network_number;
//		flag = 1;
//	}
//	else
//		flag = 0;

	for(i = 0; i < MAXREMOTEPOINTS; i++, ptr++)
	{
		if( ptr->count )
		{
			// change size to 4, dont compare network number
			if( !memcmp( (void*)point, (void*)&ptr->point, 4/*sizeof(Point_Net)*/ ) )
			{
				break;
			}
		}
	}
//	if( flag )
//		point->network_number = 0xFF;
	if( i < MAXREMOTEPOINTS )
	{
	// add to scan tabel
	
		return i; // returns the index in the list
	} 
	else
		return -1;
}

S16_T find_local_point( Point *point )
{
	S16_T i;
	POINTS_HEADER *ptr;

	/* fast-path: use small lookup table for common local point types (OUT/IN/VAR)
	   lazy-init the index map on first use to avoid extra startup changes */
	static int8_t local_index_inited = 0;
	extern void rebuild_local_point_index(void);

	if(!local_index_inited)
	{
		rebuild_local_point_index();
		local_index_inited = 1;
	}

	/* only use direct lookup for typical local point types (1..VAR+1) */
	if(point->point_type >= OUT + 1 && point->point_type <= VAR + 1)
	{
		int pt = point->point_type - 1;
		int num = point->number;
		if(num >= 0 && num < MAX_VARS)
		{
			extern S16_T local_point_index_lookup(int pt, int num);
			i = local_point_index_lookup(pt, num);
			if(i >= 0 && i < MAXLOCALPOINTS)
			{
				ptr = &local_points_list[i];
				if(ptr->count)
					return i;
			}
		}
		return -1;
	}

	/* fallback to original linear search for other point types */
	ptr = local_points_list;
	for(i = 0; i < MAXLOCALPOINTS; i++, ptr++)
	{
		if( !memcmp( (void*)point, (void*)&ptr->point, sizeof(Point) ) )
		{
			break;
		}
	}

	if( i < MAXLOCALPOINTS )
		return i;
	else
		return -1;
}

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
U8_T check_network_point_list(Point_Net *point,U8_T *index, U8_T protocal)
{
	U16_T i;
	NETWORK_POINTS *ptr;

	ptr = network_points_list;
	for(i = 0;i < MAXNETWORKPOINTS;i++,ptr++)
	{
		if((point->panel == ptr->point.panel) &&
		(point->sub_id == ptr->point.sub_id) &&
		//(point->point_type == ptr->point.point_type) &&
		(point->number == ptr->point.number) 		
		)
		{
			if(((point->point_type & 0x1f) == (ptr->point.point_type & 0x1f)) ||
				(((point->point_type & 0x1f) == VAR + 1) && ((ptr->point.point_type & 0x1f) == MB_REG + 1)) || 
				(((point->point_type & 0x1f) == MB_REG +1) && ((ptr->point.point_type & 0x1f)== VAR + 1)) || 
				(((point->point_type & 0x1f) == READ_DIS_INPUT +1) && ((ptr->point.point_type & 0x1f)== MB_DIS_REG + 1)) || 
				(((point->point_type & 0x1f) == READ_INPUT +1) && ((ptr->point.point_type & 0x1f)== MB_IN_REG + 1)) ||
				(((point->point_type & 0x1f) == READ_COIL +1) && ((ptr->point.point_type & 0x1f)== MB_COIL_REG + 1))
			)			
			{
			*index = i;
			return 1;
			}
		}
	}
	return 0;	
}
#endif

U8_T check_remote_point_list(Point_Net *point,U8_T *index, U8_T protocal)
{
	U16_T i;
	REMOTE_POINTS *ptr;

	ptr = remote_points_list;
	
	for(i = 0;i < MAXREMOTEPOINTS;i++,ptr++)
	{	
		if(ptr->point.panel == point->sub_id && ptr->point.sub_id == 0)
		{
			uint8_t len;	
			uint8_t object_type;
			uint16_t reg1,reg2;
			
			reg1 = get_reg_from_list(ptr->point.point_type,ptr->point.number,&len);
			reg2 = (U16_T)((point->point_type & 0xe0) << 3) + point->number
									+ (U16_T)((point->network_number & 0x1f) << 11);	
			if(reg1 == reg2)
			{
				*index = i;
				return 2;
			}			
		}
		
		
		if(/*(point->network_number == remote_points_list[i].point.network_number) &&*/ 
		(point->panel == ptr->point.panel) &&
		(point->sub_id == ptr->point.sub_id) &&
		//(point->point_type == ptr->point.point_type) &&
		(point->number == ptr->point.number) 		
		)
		{
			if(((point->point_type & 0x1f) == (ptr->point.point_type & 0x1f)) ||
				(((point->point_type & 0x1f) == VAR + 1) && ((ptr->point.point_type & 0x1f) == MB_REG + 1)) || 
				(((point->point_type & 0x1f) == MB_REG +1) && ((ptr->point.point_type & 0x1f)== VAR + 1))
			)			
			{
			*index = i;
			return 1;
			}
			
		}
	}
	return 0;	
}

// special -- 2: bacnet points, 0/1 - modbus points
// special -- 3: VAR OUT IN (mstp protocal)
void add_remote_point(U8_T id,U8_T point_type,U8_T high_5bit, U8_T number,S32_T val_ptr,U8_T specail,U8_T float_type)
{
	REMOTE_POINTS ptr;//	Point_Net *point; 
	U8_T index;
	U8_T type,number_high3bit;
	U8_T protocal;	
	U8_T ret;
	
	ptr.point.panel = panel_number;//Modbus.network_ID[2];
	ptr.point.sub_id = id;
	
	if(specail == 3)  // VAR OUT IN
	{
		protocal = 1;		
		type = point_type & 0x1f;
		ptr.point.point_type = type;		
	}
	else if(specail == 2)
	{
		protocal = 1;
		type = point_type & 0x1f;
		number_high3bit = point_type & 0xe0;
		if(type == OBJECT_ANALOG_VALUE || type == BAC_AV + 1)
		{		
			ptr.point.point_type = BAC_AV + 1 + number_high3bit;
		}
		else if(type == OBJECT_ANALOG_INPUT || type == BAC_AI + 1)
		{
			ptr.point.point_type = BAC_AI + 1 + number_high3bit;
		}
		else if(type == OBJECT_ANALOG_OUTPUT || type == BAC_AO + 1)
		{		
			ptr.point.point_type = BAC_AO + 1 + number_high3bit;
		}
		else if(type == OBJECT_BINARY_OUTPUT || type == BAC_BO + 1)
		{		
			ptr.point.point_type = BAC_BO + 1 + number_high3bit;
		}
		else if(type == OBJECT_BINARY_VALUE || type == BAC_BV + 1)
		{		
			ptr.point.point_type = BAC_BV + 1 + number_high3bit;
		}
		else if(type == OBJECT_BINARY_INPUT || type == BAC_BI + 1)
		{		
			ptr.point.point_type = BAC_BI + 1 + number_high3bit;
		}
		else if(type == OBJECT_MULTI_STATE_VALUE || type == BAC_MSV + 1)
		{		
			ptr.point.point_type = BAC_MSV + 1 + number_high3bit;
		}
	}
	else
	{
		protocal = 0;
		type = point_type & 0x1f;
		number_high3bit = point_type & 0xe0;				
		if(high_5bit > 0)
		{
			ptr.point.network_number = 0x80 + high_5bit;//Setting_Info.reg.network_number;
		}
		else
			ptr.point.network_number = 0; 
		if(type == READ_COIL)
			ptr.point.point_type = MB_COIL_REG + 1 + number_high3bit;
		else if(type == READ_DIS_INPUT)
			ptr.point.point_type = MB_DIS_REG + 1 + number_high3bit;
		else if(type == READ_INPUT)
		{
			ptr.point.point_type = MB_IN_REG + 1 + number_high3bit;
			if(float_type > 0)
			{
				ptr.point.point_type = ((MB_INPUT_FLOAT_ABCD + float_type) & 0x1f) + number_high3bit;
				ptr.point.network_number |= ((MB_INPUT_FLOAT_ABCD + float_type) & 0x60);
			}
		}
		else if(type == READ_VARIABLES || type == (OUT+1) || type == (IN+1) || type == (VAR+1))
		{
			if(specail == 1) 
			{
				ptr.point.point_type = MB_REG + 1 + number_high3bit;
			}
			else   // MB_REG & REG
			{// VAR IN OUT
				ptr.point.point_type = type/*VAR + 1*/ + number_high3bit;	
			}
			if(float_type > 0)
			{
				ptr.point.point_type = ((MB_HOLDING_FLOAT_ABCD + float_type) & 0x1f) + number_high3bit;
				ptr.point.network_number |= ((MB_HOLDING_FLOAT_ABCD + float_type) & 0x60);
			}
		}		
	}
	
	ptr.point.number = number;
	ptr.auto_manual = 0; 
	
	if(ret = check_remote_point_list(&ptr.point,&index,protocal))
	{		
		if(protocal == 0) // modbus
		{
			if(float_type == 0)
			{
				if(ret == 2)  // 3VARx 3INx 3OUTx
				{
					remote_points_list[index].point_value = val_ptr;
					// analog value must be raw value
					// digital value must be 1000x
					// should check digital or analog ?????					
					// digital on
					if(remote_points_list[index].digital_analog == 100)
					{
						if(val_ptr == 1) 
							remote_points_list[index].point_value = val_ptr * 1000;
					}
						
				}
				else
				{
					remote_points_list[index].point_value = val_ptr * 1000;
				}
				
			}
			else 
			{
				float f;
				Byte_to_Float(&f,val_ptr,float_type);
				remote_points_list[index].point_value = f * 1000;
			}
		}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
		else
		{ // mstp
			remote_points_list[index].point_value = val_ptr;
		}
#endif
		
		Last_Contact_Remote_points[index] = 0;
	}
}

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
// special -- 2: bacnet points, 0/1 - modbus points

void add_network_point(U8_T panel,U8_T id,U8_T point_type,U8_T number,S32_T val_ptr,U8_T specail,U8_T float_type)
{
	NETWORK_POINTS ptr;//	Point_Net *point; 
	U8_T index;
	U8_T protocal;
	U8_T type,number_high3bit;
	ptr.point.network_number = 0;//Setting_Info.reg.network_number;
	ptr.point.panel = panel;
  ptr.point.sub_id = id;
	ptr.point.point_type = point_type + 1;
//  ptr->auto_manual = auto_manual;
//	ptr->digital_analog = digital_analog;
//	type = point_type & 0x1f;
	ptr.point.number = number - 1;
	ptr.auto_manual = 0;  
	
	if(specail == 2)
	{
		protocal = 1;
		ptr.point.point_type = point_type + 1;
	}
	else
	{
		type = point_type & 0x1f;
		number_high3bit = point_type & 0xe0;		
		if(type == READ_COIL || type == MB_COIL_REG)
		{
			ptr.point.point_type = MB_COIL_REG + 1 + number_high3bit;
		}
		else if(type == READ_DIS_INPUT || type == MB_DIS_REG)
		{
			ptr.point.point_type = MB_DIS_REG + 1 + number_high3bit;
		}
		else if(type == READ_INPUT || type == MB_IN_REG)
		{
			ptr.point.point_type = MB_IN_REG + 1 + number_high3bit;
			//float_type = 1;			// read input float 32bit
			if(float_type > 0)
			{
				ptr.point.point_type = ((MB_INPUT_FLOAT_ABCD + float_type) & 0x1f) + number_high3bit;
				ptr.point.network_number |= ((MB_INPUT_FLOAT_ABCD + float_type) & 0x60);
			}		
		}
		else //if(type == READ_VARIABLES)
		{
			if(specail == 1) 
				ptr.point.point_type = MB_REG + 1 + number_high3bit;
			else   // MB_REG & REG
				ptr.point.point_type = VAR + 1 + number_high3bit;	
			
			if(float_type > 0)
			{
				ptr.point.point_type = ((MB_HOLDING_FLOAT_ABCD + float_type) & 0x1f) + number_high3bit;
				ptr.point.network_number |= ((MB_HOLDING_FLOAT_ABCD + float_type) & 0x60);
			}
		}
		protocal = 0;
	}
	// network panel
	if(check_network_point_list(&ptr.point,&index,protocal))
	{
#if NETWORK_MODBUS_BAC	
		if(protocal == 0) // modbus
		{
			if(float_type == 0)
			{
				network_points_list[index].point_value = val_ptr * 1000;
			}
			else 
			{
				float f;
				Byte_to_Float(&f,val_ptr,float_type);
				network_points_list[index].point_value = f*1000;
			}			
		}
		else
#endif
		{
			network_points_list[index].point_value = val_ptr;
		}	
		Last_Contact_Network_points[index] = 0;	
	}
}
#endif

S16_T insert_local_point( Point *point, S16_T index )
{
	S16_T i;
	POINTS_HEADER *ptr;
	U8_T point_type;	

	ptr = &local_points_list[0];
	if( index < 0 )
	{ /* index < 0 means that the index is unknown */		
		
		if( number_of_local_points >= MAXLOCALPOINTS ) 
			return -1;
		
		if( ( i = find_local_point( point ) ) >= 0 )
		{ /* the point is in the list */
			(ptr+i)->count++;
			return i;
		}
		else
		{
			for( i=0; i < MAXLOCALPOINTS; i++, ptr++ )
			{
				if( !ptr->count )
				{
						number_of_local_points++;						
						memcpy( &ptr->point, point, sizeof(Point_Net) );
						ptr->count = 1;

					return i;
				}
			}
			return -1;
		}
	}
	else
	{
		ptr += index;
		ptr->count++;
		return index;
	}
}


S16_T insert_remote_point( Point_Net *point, S16_T index )
{ 	
	S16_T i;
	REMOTE_POINTS *ptr;
	U8_T point_type;	

	ptr = &remote_points_list[0];

	if( index < 0 )
	{ /* index < 0 means that the index is unknown */		
		if(check_point_type(point) == 1)
		{
			if( number_of_remote_points_modbus >= MAXREMOTEPOINTS ) 
				return -1;
		}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
		else
		{
			if( number_of_remote_points_bacnet >= MAXREMOTEPOINTS ) 
				return -1;
		}
#endif
		if( ( i = find_remote_point( point ) ) >= 0 )
		{ /* the point is in the list */
			(ptr+i)->count++;
#if 0
			/* ensure lookup table points to this index */
			if(point->point_type >= OUT + 1 && point->point_type <= VAR + 1)
			{
				int pt = point->point_type - 1;
				int num = point->number;
				if(pt >= 0 && pt < MAX_POINT_TYPE && num >= 0 && num < MAX_VARS)
					local_point_index[pt][num] = i;
			}
#endif
			return i;
		}
		else
		{
			for( i=0; i < MAXREMOTEPOINTS; i++, ptr++ )
			{
				if( !ptr->count )
				{
					U8_T *addr;
#if 0
					/* update lookup table */
					if(point->point_type >= OUT + 1 && point->point_type <= VAR + 1)
					{
						int pt = point->point_type - 1;
						int num = point->number;
						if(pt >= 0 && pt < MAX_POINT_TYPE && num >= 0 && num < MAX_VARS)
							local_point_index[pt][num] = i;
					}
					return i;
#endif
					// check current id is modbus device or mstp device
					// add modbus command
					if(check_point_type(point) == 1)
					{
						point_type = (point->point_type & 0x1f) + (point->network_number & 0x60);
						if(point->sub_id != 0)
							remote_points_list[i].tb.RP_modbus.id = point->sub_id; 
						else
							remote_points_list[i].tb.RP_modbus.id = point->panel;
						
						if(point->network_number & 0x80)	// new way, modbus reg is 0-65535	
						{			
								remote_points_list[i].tb.RP_modbus.reg = (U16_T)((point->point_type & 0xe0) << 3) + point->number
									+ (U16_T)((point->network_number & 0x1f) << 11);	
						}							
						else // old way, modbus reg is 0-2047
						{// VAR1-128 OUT1-64 IN1-64
							if(point_type == VAR + 1 || point_type == OUT + 1 || point_type == IN + 1 )	
							{
								uint8_t len;
								remote_points_list[i].tb.RP_modbus.reg = 
								get_reg_from_list(point->point_type & 0x1f,point->number,&len);				

							}
							else
								remote_points_list[i].tb.RP_modbus.reg = (U16_T)((point->point_type & 0xe0) << 3) + point->number;
						}
						
						if(point_type == VAR + 1)						
							remote_points_list[i].tb.RP_modbus.func = READ_VARIABLES;
						else if(point_type == MB_REG + 1)
							remote_points_list[i].tb.RP_modbus.func = READ_VARIABLES + 0x80;						
						else if(point_type == MB_COIL_REG + 1)
							remote_points_list[i].tb.RP_modbus.func = READ_COIL;
						else if(point_type == MB_DIS_REG + 1)
							remote_points_list[i].tb.RP_modbus.func = READ_DIS_INPUT;
						else if(point_type == MB_IN_REG + 1)
							remote_points_list[i].tb.RP_modbus.func = READ_INPUT;	
						else if((point_type >= MB_HOLDING_FLOAT_ABCD + 1) &&( point_type <= MB_HOLDING_FLOAT_DCBA + 1))
						{
							remote_points_list[i].tb.RP_modbus.func = READ_VARIABLES + ((point_type - MB_HOLDING_FLOAT_ABCD) << 8);
						}
						else if((point_type >= MB_INPUT_FLOAT_ABCD + 1) &&( point_type <= MB_INPUT_FLOAT_DCBA + 1))
						{
							remote_points_list[i].tb.RP_modbus.func = READ_INPUT + ((point_type - MB_INPUT_FLOAT_ABCD) << 8);
						}
						else	// other things	
							remote_points_list[i].tb.RP_modbus.func = READ_VARIABLES;
						number_of_remote_points_modbus++;						
					}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
					else
					{
						point_type = (point->point_type & 0x1f) + (point->network_number & 0x60);
					// add AV,AI,AO,BO ... mstp command
						remote_points_list[i].tb.RP_bacnet.panel = point->sub_id;
						remote_points_list[i].tb.RP_bacnet.instance =  
							(U16_T)((point->point_type & 0xe0) << 3) + point->number;

						remote_points_list[i].tb.RP_bacnet.object = point_type ;						
						number_of_remote_points_bacnet++;
						
					}
#endif
					memcpy( &ptr->point, point, sizeof(Point_Net) );
					ptr->point_value = DEFUATL_REMOTE_NETWORK_VALUE;
					//if( point->network_number == 0x0FFFF)
					//ptr->point.network_number = Setting_Info.reg.network_number;
					ptr->count = 1;
//					ptr->change = 0;


					return i;
				}
			}
			return -1;
		}
	}
	else
	{
		ptr += index;
		ptr->count++;
		return index;
	}
}

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
S16_T insert_network_point( Point_Net *point, S16_T index )
{ 	
	S16_T i;
	NETWORK_POINTS *ptr;
	U8_T point_type;

	ptr = &network_points_list[0];	
	if( index < 0 )
	{ /* index < 0 means that the index is unknown */
		
		if( number_of_network_points_bacnet >= MAXNETWORKPOINTS ) 
			return -1;
		
		if( ( i = find_network_point( point ) ) >= 0 )
		{ /* the point is in the list */
			return i;
		}
		else
		{ 
			for( i = 0; i < MAXNETWORKPOINTS; i++, ptr++ )
			{	
				if( !ptr->count )		
				{		
#if NETWORK_MODBUS_BAC						
				// add modbus command
					if(check_point_type(point) == 1)
					{ // only support MB_COIL_REG to MB_REG
						point_type = (point->point_type & 0x1f) + (point->network_number & 0x60);
							network_points_list[i].point.panel = point->panel;
						
							if(point->network_number & 0x80)  // modbus range is 0-65535
									network_points_list[i].tb.NT_modbus.reg = (U16_T)((point->point_type & 0xe0) << 3) + point->number
									+ (U16_T)((point->network_number & 0x1f) << 11);		
							else	// modbus range is 0-2047
								network_points_list[i].tb.NT_modbus.reg = (U16_T)((point->point_type & 0xe0) << 3) + point->number;

							if(point->sub_id == 0)
								network_points_list[i].tb.NT_modbus.id = 255;//point->sub_id;
							else
								network_points_list[i].tb.NT_modbus.id = point->sub_id;
							
							if(point_type == VAR + 1)
								network_points_list[i].tb.NT_modbus.func = READ_VARIABLES;
							else if(point_type == MB_REG + 1)
								network_points_list[i].tb.NT_modbus.func = READ_VARIABLES + 0x80;						
							else if(point_type == MB_COIL_REG + 1)
								network_points_list[i].tb.NT_modbus.func = READ_COIL;
							else if(point_type == MB_DIS_REG + 1)
								network_points_list[i].tb.NT_modbus.func = READ_DIS_INPUT;
							else if(point_type == MB_IN_REG + 1)
								network_points_list[i].tb.NT_modbus.func = READ_INPUT;	
							else if((point_type >= MB_HOLDING_FLOAT_ABCD + 1) &&( point_type <= MB_HOLDING_FLOAT_DCBA + 1))
							{
								network_points_list[i].tb.NT_modbus.func = READ_VARIABLES	+ ((point_type - MB_HOLDING_FLOAT_ABCD) << 8);
							}		
							else if((point_type >= MB_INPUT_FLOAT_ABCD + 1) &&( point_type <= MB_INPUT_FLOAT_DCBA + 1))
							{
								network_points_list[i].tb.NT_modbus.func = READ_INPUT	+ ((point_type - MB_INPUT_FLOAT_ABCD) << 8);
							}							
							else	// other things	
								network_points_list[i].tb.NT_modbus.func = READ_VARIABLES;
							
							memcpy( &ptr->point, point, sizeof(Point_Net) );
					}
					else
					{
						memcpy( &ptr->point, point, sizeof(Point_Net) );						
						if(point->sub_id == 0 || point->panel == point->sub_id)
							ptr->point.sub_id = 0;					
						
					}
#endif
					
					//memcpy( &ptr->point, point, sizeof(Point_Net) );
					ptr->point_value = DEFUATL_REMOTE_NETWORK_VALUE;
					//if( point->network_number == 0x0FFFF )
					//ptr->point.network_number = Setting_Info.reg.network_number;
#if NETWORK_MODBUS_BAC					
					// add modbus command
					if(check_point_type(point) == 1)
					 // only support MB point
						number_of_network_points_modbus++;
					else
#endif
						number_of_network_points_bacnet++;
					
					ptr->count = 1;
					return i;
				}
			}
			return -1;
		}
	}
	else
	{
		ptr += index;
		ptr->count++;
		return index;
	}
}
#endif

POINTS_HEADER			far      points_header[MAXREMOTEPOINTS];

void Check_NPB_node_write_TTL(void);
// check if remote bacent points and network points online or not
void Check_bacnet_points_online(void)  
{
	
}

uint8_t find_next_remote_bacnet_point(uint8_t current_index) {
	uint8_t i;
	uint8_t total_points = number_of_remote_points_modbus + number_of_remote_points_bacnet;
	
	for (i = 1; i <= total_points; i++) {
        uint8_t index = (current_index + i) % total_points; 
        if (remote_points_list[index].instance != 0) {
            return index;  
        }
    }
    return 0xff;
}

uint8_t find_next_remote_modbus_point(uint8_t current_index) {
		uint8_t i;
		uint8_t total_points = number_of_remote_points_modbus + number_of_remote_points_bacnet;
    for (i = 1; i <= total_points; i++) {
        uint8_t index = (current_index + i) % total_points; 
        if (remote_points_list[index].instance == 0) {
            return index;  
        }
    }
    return 0xff;  // dont find modbsu point
}

uint8_t find_next_network_bacnet_point(uint8_t current_index) {
	uint8_t i;
    for (i = current_index + 1; i < number_of_network_points_modbus + number_of_network_points_bacnet; i++) {
        if (network_points_list[i].instance != 0) {
            return i; // find next poont
        }
    }
    return 0xff;  // dont find bacnet point
}

uint8_t find_next_network_modbus_point(uint8_t current_index) {
	uint8_t i;
    for (i = current_index + 1; i < number_of_network_points_modbus + number_of_network_points_bacnet; i++) {
        if (network_points_list[i].instance == 0) {
            return i; // find next poont
        }
    }
    return 0xff;  // dont find modbus point
}

void Update_RM_NT_points_table(void)
{
	
	U8_T i,j;

	j = 0;
	
	for(i = 0;i < number_of_remote_points_modbus;i++)
	{
		if(j < MAXREMOTEPOINTS - 1)
		{
			memcpy(&points_header[j],&remote_points_list[number_of_remote_points_bacnet + i],sizeof(POINTS_HEADER));
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
			points_header[j].point_value = my_honts_arm((remote_points_list[number_of_remote_points_bacnet + i].point_value));
			points_header[j].instance = my_honts_arm((float)remote_points_list[number_of_remote_points_bacnet + i].instance);
#endif
			points_header[j].time_to_live = Last_Contact_Remote_points[number_of_remote_points_bacnet + i];
			j++;
		}
	}
		
		
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
		for(i = 0;i < number_of_remote_points_bacnet;i++)
		{
			if(j < MAXREMOTEPOINTS - 1)
			{
				memcpy(&points_header[j],&remote_points_list[i],sizeof(POINTS_HEADER));

				points_header[j].point_value = my_honts_arm((float)remote_points_list[i].point_value );
				points_header[j].instance = my_honts_arm((float)remote_points_list[i].instance);
				points_header[j].time_to_live = Last_Contact_Remote_points[i];
				j++;
			}
		}
#if NETWORK_MODBUS_BAC	
		for(i = 0;i < number_of_network_points_modbus;i++)
		{
			if(j < MAXNETWORKPOINTS - 1)
			{				
				memcpy(&points_header[j],&network_points_list[number_of_network_points_bacnet + i],sizeof(POINTS_HEADER));

				points_header[j].point_value = my_honts_arm((float)network_points_list[number_of_network_points_bacnet + i].point_value);
				points_header[j].instance = my_honts_arm((float)network_points_list[number_of_network_points_bacnet + i].instance);
				points_header[j].time_to_live = Last_Contact_Network_points[number_of_network_points_bacnet + i];
				j++;
			}
		}
#endif		
		for(i = 0;i < number_of_network_points_bacnet;i++)
		{
			if(j < MAXNETWORKPOINTS - 1)
			{				
				memcpy(&points_header[j],&network_points_list[i],sizeof(POINTS_HEADER));
				// 11VAR1 11IN1 11OUT1 
				// show correct point_number in network table.
				if((points_header[j].point.point_type >= (OUT + 1)) && (points_header[j].point.point_type <= (VAR + 1)))
					points_header[j].point.number = network_points_list[i].point.number + 1;
				points_header[j].point_value = my_honts_arm((float)network_points_list[i].point_value);
				points_header[j].instance = my_honts_arm((float)network_points_list[i].instance);
				points_header[j].time_to_live = Last_Contact_Network_points[i];
				j++;
			}
		}
#endif
		
		for(i = 0;i < number_of_local_points;i++)
		{
			if(j < MAXLOCALPOINTS - 1)
			{
				memcpy(&points_header[j],&local_points_list[i],sizeof(POINTS_HEADER));
				if((points_header[j].point.point_type >= (OUT + 1)) && (points_header[j].point.point_type <= (VAR + 1)))
					points_header[j].point.number = local_points_list[i].point.number + 1;
				points_header[j].point_value = my_honts_arm(local_points_list[i].point_value);
				points_header[j].instance = my_honts_arm((float)local_points_list[i].instance);
				points_header[j].time_to_live = Last_Contact_Local_points[i];
				j++;
			}
		}
		
		
		memset(&points_header[j],0,(MAXREMOTEPOINTS - j) * sizeof(POINTS_HEADER));

}


void Check_Net_Point_Table(void)
{
	REMOTE_POINTS *ptr;
	NETWORK_POINTS *ptr1;
	POINTS_HEADER *ptr2;
	
	U8_T i;
	U8_T product;
	ptr = &remote_points_list[0];

	if(number_of_remote_points_modbus < MAXREMOTEPOINTS)
	{
		for(i = 0;i < number_of_remote_points_modbus;i++,ptr++)
		{
			// check whether remote point is T3 module
			// if it is T3, dont count live time
			if(ptr->point.panel)		
				product = Get_product_by_id(ptr->point.sub_id);
			
			if((product == PM_T34AO) || (product == PM_T3IOA) || (product == PM_T38I13O) || (product == PM_T332AI)
				|| (product == PM_T322AI) || (product == PM_T38AI8AO6DO) || (product == PM_T36CTA) || (product == PM_T3LC)
			|| (product == PM_T3PT12))
			{
				// do not count live time
			}
			else
			{
				// check whether device is on line				
				if(((current_online[remote_points_list[number_of_remote_points_bacnet + i].tb.RP_modbus.id / 8] & (1 << (remote_points_list[number_of_remote_points_bacnet + i].tb.RP_modbus.id % 8))))	
					|| (product == 0))  // customer device
				{		
					if(ptr->decomisioned != 0)					
						Last_Contact_Remote_points[number_of_remote_points_bacnet + i]++;		
					else
						Last_Contact_Remote_points[number_of_remote_points_bacnet + i] = 0;
					
					if(remote_points_list[number_of_remote_points_bacnet + i].time_to_live != PERMANENT_LIVE 
						&& remote_points_list[number_of_remote_points_bacnet + i].time_to_live > 0)
						remote_points_list[number_of_remote_points_bacnet + i].time_to_live--;					

					//if(ptr->decomisioned == 1)
					{
						if(ptr->time_to_live <= 0)
						{
							if(number_of_remote_points_modbus > 0)
							{
							// delete this point
							//memset(ptr,0,sizeof(REMOTE_POINTS));
								ptr->decomisioned = 0;
								memcpy(ptr,ptr+1,(number_of_remote_points_bacnet + number_of_remote_points_modbus - i - 1) * sizeof(REMOTE_POINTS));
								//memset(ptr + number_of_remote_points_modbus - i - 1,0,sizeof(REMOTE_POINTS));
								memset(&remote_points_list[number_of_remote_points_bacnet + number_of_remote_points_modbus - 1],0,sizeof(REMOTE_POINTS));
								number_of_remote_points_modbus--;
								
							}
						}
					}	
				}	
				else
				{
					ptr->time_to_live = 0;						
					if(number_of_remote_points_modbus > 0)
					{
					// delete this point
					//memset(ptr,0,sizeof(REMOTE_POINTS));
						ptr->decomisioned = 0;
						memcpy(ptr,ptr+1,(number_of_remote_points_bacnet + number_of_remote_points_modbus - i - 1) * sizeof(REMOTE_POINTS));
						//memset(ptr + number_of_remote_points_modbus - i - 1,0,sizeof(REMOTE_POINTS));
						memset(&remote_points_list[number_of_remote_points_bacnet + number_of_remote_points_modbus - 1],0,sizeof(REMOTE_POINTS));
						number_of_remote_points_modbus--;
					}
						
				}
			}
		}
	}
	/* else 
error
*/

#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
	ptr = &remote_points_list[0];
	
	for(i = 0;i < number_of_remote_points_bacnet;i++,ptr++)
	{
		if(remote_points_list[i].time_to_live != PERMANENT_LIVE
			&& remote_points_list[i].time_to_live > 0)
			remote_points_list[i].time_to_live--;
		Last_Contact_Remote_points[i]++;
		if(ptr->decomisioned == 1)
		{
			if(ptr->time_to_live <= 0)
			{
				if(number_of_remote_points_bacnet > 0)
				{
				// delete this point
				ptr->decomisioned = 0;		
				// move up, clear the last line				
				memcpy(ptr,ptr + 1,(number_of_remote_points_bacnet - i - 1) * sizeof(REMOTE_POINTS));
				//memset(ptr + number_of_remote_points_bacnet - i - 1,0,sizeof(REMOTE_POINTS));
				memset(&remote_points_list[number_of_remote_points_bacnet - 1],0,sizeof(REMOTE_POINTS));
				number_of_remote_points_bacnet--;
				}
				
			}
		}
	}
#if NETWORK_MODBUS_BAC	
	ptr1 = &network_points_list[0];
	

	for(i = 0;i < number_of_network_points_modbus;i++,ptr1++)
	{
		if(network_points_list[number_of_network_points_bacnet + i].time_to_live != PERMANENT_LIVE
			&& network_points_list[number_of_network_points_bacnet + i].time_to_live > 0)
			network_points_list[number_of_network_points_bacnet + i].time_to_live--;
		Last_Contact_Network_points[number_of_network_points_bacnet + i]++;
		//if(ptr1->decomisioned == 1)
		{
			if(ptr1->time_to_live <= 0)
			{
				// delete this point
				//memset(ptr1,0,sizeof(NETWORK_POINTS));
				ptr1->decomisioned = 0;
				// move up, clear the last line
				memcpy(ptr1,ptr1+1,(number_of_network_points_bacnet + number_of_network_points_modbus - i - 1) * sizeof(NETWORK_POINTS));
				//memset(ptr1 + number_of_network_points - i - 1,0,sizeof(NETWORK_POINTS));
				memset(&network_points_list[number_of_network_points_bacnet + number_of_network_points_modbus - 1],0,sizeof(NETWORK_POINTS));
				network_points_list[number_of_network_points_bacnet + number_of_network_points_modbus - 1].point_value = DEFUATL_REMOTE_NETWORK_VALUE;
				number_of_network_points_modbus--;
			}
		}
	}

	ptr1 = &network_points_list[0];
	
	for(i = 0;i < number_of_network_points_bacnet;i++,ptr1++)
	{
		if(network_points_list[i].time_to_live != PERMANENT_LIVE
			&& network_points_list[i].time_to_live > 0)
			network_points_list[i].time_to_live--;
		Last_Contact_Network_points[i]++;

		//if(ptr1->decomisioned == 1)
		{
			if(ptr1->time_to_live <= 0)
			{
				// delete this point
				//memset(ptr1,0,sizeof(NETWORK_POINTS));
				ptr1->decomisioned = 0;
				// move up, clear the last line
				memcpy(ptr1,ptr1+1,(number_of_network_points_bacnet - i - 1) * sizeof(NETWORK_POINTS));
				//memset(ptr1 + number_of_network_points - i - 1,0,sizeof(NETWORK_POINTS));
				memset(&network_points_list[number_of_network_points_bacnet - 1],0,sizeof(NETWORK_POINTS));
				network_points_list[number_of_network_points_bacnet - 1].point_value = DEFUATL_REMOTE_NETWORK_VALUE;
				number_of_network_points_bacnet--;
			}
		}
	}
	
	
	ptr2 = &local_points_list[0];	
	for(i = 0;i < number_of_local_points;i++,ptr2++)
	{
		if(local_points_list[i].time_to_live != PERMANENT_LIVE
			&& local_points_list[i].time_to_live > 0)
			local_points_list[i].time_to_live--;
		Last_Contact_Local_points[i]++;
		//if(ptr1->decomisioned == 1)
		{
			if(ptr2->time_to_live <= 0)
			{
				// delete this point
				ptr2->decomisioned = 0;
				// move up, clear the last line
				memcpy(ptr2,ptr2+1,(number_of_local_points - i - 1) * sizeof(POINTS_HEADER));
				memset(&local_points_list[number_of_local_points - 1],0,sizeof(POINTS_HEADER));
				local_points_list[number_of_local_points - 1].point_value = DEFUATL_REMOTE_NETWORK_VALUE;
				number_of_local_points--;
			}
		}
	}
	
	
	Check_NPB_node_write_TTL();
#endif
#endif	
	rebuild_local_point_index();
	Update_RM_NT_points_table();

}





S16_T get_point_value( Point *point, S32_T *val_ptr )
{
 	Str_points_ptr far sptr;
	S16_T index;	
	
	
	if(( OUTPUT <= point->point_type ) &&( point->point_type <= VARIABLE ))
	{
		if( ( index = find_local_point( point ) ) < 0 )
		{
			if( ( index = insert_local_point( point, -1 ) ) < 0 )
			{
				index = -1;
			}
		}
	}
	
	if( ( OUTPUT <= point->point_type ) &&( point->point_type <= ARRAYS ))
 	{
  	//if( point->number < table_bank[point->point_type-1] )
		/* Was MAX_VARS+12 — vars[] is only MAX_VARS; +12 read past end into adjacent RAM */
			if( point->number < table_bank[point->point_type-1] )
  		{
				switch( point->point_type-1 )
				{
				case OUT: 					
					sptr.pout = &outputs[point->number];
					if((!sptr.pout->digital_analog) && (sptr.pout->range != UNUSED)) /* DIGITAL */
					{ 											
					//	*val_ptr = swap_double(sptr.pout->control? 1000L:0L);
						if(( sptr.pout->range >= ON_OFF  && sptr.pout->range <= HIGH_LOW )
							||(sptr.pout->range >= custom_digital1 // customer digital unit
							&& sptr.pout->range <= custom_digital8
							&& digi_units[sptr.pout->range - custom_digital1].direct == 1))
						{// inverse
							*val_ptr = swap_double(sptr.pout->control ? 0L : 1000L);
						}
						else
							*val_ptr = swap_double(sptr.pout->control ? 1000L : 0L);	
					}
					else
					{
						*val_ptr = sptr.pout->value;
					}
					
					break;
				case IN:
					sptr.pin = &inputs[point->number];
				
					if( (!sptr.pin->digital_analog) && (sptr.pin->range != UNUSED))
					{	
						if(( sptr.pin->range >= ON_OFF  && sptr.pin->range <= HIGH_LOW )
					||(sptr.pin->range >= custom_digital1 // customer digital unit
					&& sptr.pin->range <= custom_digital8
					&& digi_units[sptr.pin->range - custom_digital1].direct == 1))
						{// inverse
							*val_ptr = swap_double(sptr.pin->control ? 0L : 1000L);
						}
						else
							*val_ptr = swap_double(sptr.pin->control ? 1000L : 0L);						
					}
					else
					{
						*val_ptr = sptr.pin->value;
					}
					break;
			 	case VAR:
					sptr.pvar = &vars[point->number];
					
					if( (!sptr.pvar->digital_analog) && (sptr.pvar->range != UNUSED))
					{		
						if(( sptr.pvar->range >= ON_OFF  && sptr.pvar->range <= HIGH_LOW )
							||(sptr.pvar->range >= custom_digital1 // customer digital unit
							&& sptr.pvar->range <= custom_digital8
							&& digi_units[sptr.pvar->range - custom_digital1].direct == 1))
						{// inverse
							*val_ptr = swap_double(sptr.pvar->control ? 0L : 1000L);
						}
						else
						{
							*val_ptr = swap_double(sptr.pvar->control ? 1000L : 0L);	
						}
					}
					else
					{
						*val_ptr = sptr.pvar->value;						
					}
					break;
			 	case CON:  
					*val_ptr = controllers[point->number].value;
					break;
				case WRT:
					*val_ptr = weekly_routines[point->number].value?1000L:0;
					break;
				case AR:
					*val_ptr = annual_routines[point->number].value?1000L:0;
					break;
				case PRG:
					*val_ptr = programs[point->number].on_off?1000L:0;
					break;
				case AMON:
					*val_ptr = monitors[point->number].status ? 1000L : 0;
					break;
				
				/*case TZ:  
					*val_ptr = totalizers[point->number].active?1000L:0;
					break; */
				
			}
			if(index != -1)
			{
				if(index < MAXLOCALPOINTS)
				{
				local_points_list[index].point_value = *val_ptr;
				local_points_list[index].point.panel = panel_number;
				local_points_list[index].point.sub_id = panel_number;
				local_points_list[index].point.point_type = point->point_type;
				local_points_list[index].point.number = point->number;
				local_points_list[index].time_to_live = 30;
				local_points_list[index].decomisioned = 1;
				local_points_list[index].point.network_number = 0;
				Last_Contact_Local_points[index] = 0;
				}
				
			}
			return point->number;
  		}
 	}
/*
  point->point_type = 0;
  point->number = 0;
*/
	*val_ptr = 0;
	return -1;
}





/*
 * ----------------------------------------------------------------------------
 * Function Name: put_point_value
 * Purpose: according to the type of point, write the value to the Str_points_ptr
 * Params:
 * Returns:
 * Note:  
 * ----------------------------------------------------------------------------
 */
void Update_Array(void);
S16_T put_point_value( Point *point, S32_T *val_ptr, S16_T aux, S16_T prog_op )
{
 	Str_points_ptr sptr;
	U32_T temp;
	S32_T old_value;
	S16_T index;	
	
	
	if(( OUTPUT <= point->point_type ) &&( point->point_type <= VARIABLE ))
	{
		if( ( index = find_local_point( point ) ) < 0 )
		{
			if( ( index = insert_local_point( point, -1 ) ) < 0 )
			{
				index = -1;
			}
		}
	}
/* write value to point	*/

	
	if( ( OUTPUT <= point->point_type ) &&	( point->point_type <= ARRAYS ) )
	{
		if( point->number < table_bank[point->point_type-1] )
		{
		 	switch( point->point_type-1 )
		 	{
			case OUT:
				sptr.pout = &outputs[point->number];
				output_pri_live[point->number] = 10;
				if( (sptr.pout->auto_manual == 0) && (prog_op == 0) || (sptr.pout->auto_manual == 1) && (prog_op == 1) )
				{	
					if( !sptr.pout->digital_analog ) 	// DIGITAL
					{							
						if(point->number < get_max_internal_output())
						{
								if(( outputs[point->number].range >= ON_OFF && outputs[point->number].range <= HIGH_LOW )
									||(outputs[point->number].range >= custom_digital1 // customer digital unit
									&& outputs[point->number].range <= custom_digital8
									&& digi_units[outputs[point->number].range - custom_digital1].direct == 1))
								{// inverse
									output_priority[point->number][9] = *val_ptr ? 0 : 1;		
#if ARM_TSTAT_WIFI									
									temp = Binary_Output_Present_Value(point->number) ? 0 : 1;
									if(temp != outputs[point->number].control)
										if((digital_top_area_type == OUT) && (point->number == digital_top_area_num))
											digital_top_area_changed = 1;
#endif
									outputs[point->number].control = Binary_Output_Present_Value(point->number) ? 0 : 1;	
								}
								else
								{							
									output_priority[point->number][9] = *val_ptr ? 1 : 0;	
#if ARM_TSTAT_WIFI									
									temp = Binary_Output_Present_Value(point->number) ? 1 : 0;
									if(temp != outputs[point->number].control)
										if((digital_top_area_type == OUT) && (point->number == digital_top_area_num))
											digital_top_area_changed = 1;
#endif
									outputs[point->number].control = Binary_Output_Present_Value(point->number) ? 1 : 0;	
														
								}	
						}
						else
						{
							old_value = outputs[point->number].control;
							
							if(( outputs[point->number].range >= ON_OFF && outputs[point->number].range <= HIGH_LOW )
								||(outputs[point->number].range >= custom_digital1 // customer digital unit
									&& outputs[point->number].range <= custom_digital8
									&& digi_units[outputs[point->number].range - custom_digital1].direct == 1))
							{ // inverse
#if ARM_TSTAT_WIFI									
									temp = *val_ptr ? 0 : 1;
									if(temp != outputs[point->number].control)
										if((digital_top_area_type == OUT) && (point->number == digital_top_area_num))
											digital_top_area_changed = 1;
#endif
								outputs[point->number].control = *val_ptr ? 0 : 1;	
								outputs[point->number].value = *val_ptr ? 0 : 1000;
							}
							else
							{	
#if ARM_TSTAT_WIFI									
									temp = *val_ptr ? 1 : 0;
									if(temp != outputs[point->number].control)
										if((digital_top_area_type == OUT) && (point->number == digital_top_area_num))
											digital_top_area_changed = 1;
#endif								
								outputs[point->number].control = *val_ptr ? 1 : 0;	
								outputs[point->number].value = *val_ptr ? 1000 : 0;
							}
						}
						
						if(outputs[point->number].control) 
							set_output_raw(point->number,1000);
						else 
							set_output_raw(point->number,0);
						
#if  T3_MAP
				if(point->number >= get_max_internal_output() && (point->number < base_out))
				{				
						if(old_value != outputs[point->number].control)
						{	
						vTaskSuspend(Handle_Scan);	// dont not read expansion io
#if (ARM_MINI || ASIX_MINI)
						vTaskSuspend(xHandler_Output);  // do not control local io
#endif						
						push_expansion_out_stack(&outputs[point->number],point->number,0);
							// resume output task
#if (ARM_MINI || ASIX_MINI)
						vTaskResume(xHandler_Output); 
#endif
						vTaskResume(Handle_Scan);		
						}
				}
#endif							
						
					}
					else
					{			
						if(point->number < get_max_internal_output())
							output_priority[point->number][9] = (float)(*val_ptr) / 1000;			
						// if external io
#if  T3_MAP
						if(point->number >= get_max_internal_output() && (point->number < base_out))
						{		
							old_value = outputs[point->number].value; 
							output_raw[point->number] = *val_ptr;
							// if current points is expansion IO
							if(old_value != *val_ptr)
							{
								vTaskSuspend(Handle_Scan);	// dont not read expansion io
#if (ARM_MINI || ASIX_MINI)
								vTaskSuspend(xHandler_Output);  // do not control local io
#endif
								push_expansion_out_stack(&outputs[point->number],point->number,0);
#if (ARM_MINI || ASIX_MINI)
									// resume output task
								vTaskResume(xHandler_Output); 
#endif
								vTaskResume(Handle_Scan); 							
							}
						}
							
#endif
						
#if SAVE_LOCAL_VAR_TO_E2
						// Save the local points whose values are changed in the program to EEPROM.
						if(point->number >= base_out)
						{
							old_value = outputs[point->number].value; 						
							if(old_value != *val_ptr)
							{
								put_local_var_to_BUF(point,*val_ptr);
							}
						}
#endif
					}
					check_output_priority_array(point->number,0);		
//					clear_dead_master();
					temp = *val_ptr;
#if ARM_TSTAT_WIFI
					if(temp != sptr.pout->value)
						if((digital_top_area_type == OUT) && (point->number == digital_top_area_num))
							digital_top_area_changed = 1;
#endif
					/* Mark dirty for timed save (ChangeFlash==2); do not force
					 * ChangeFlash=1 here — program may rewrite often. */
					if(sptr.pout->value != swap_double(temp))
						write_page_en[OUT] = 1;
					sptr.pout->value = swap_double(temp);
			
				}
				break;
		  	case IN:
				sptr.pin = &inputs[point->number];
				if( (sptr.pin->auto_manual == 0) && (prog_op == 0) || (sptr.pin->auto_manual == 1) && (prog_op == 1) )
				{
					if( !sptr.pin->digital_analog )
					{
						temp = *val_ptr ? 1 : 0;
						
#if ARM_TSTAT_WIFI
						if(sptr.pin->control != temp)
						{
							if((digital_top_area_type == IN) && (point->number == digital_top_area_num))
								digital_top_area_changed = 1;
						} 
#endif							
						
						sptr.pin->control = *val_ptr ? 1 : 0;
					}
				//	sptr.pin->value = convert_pointer_to_double(*val_ptr);
					temp = swap_double(*val_ptr);
#if ARM_TSTAT_WIFI
						if(sptr.pin->value != temp)
						{
							if((digital_top_area_type == IN) && (point->number == digital_top_area_num))
								digital_top_area_changed = 1;
						} 
#endif	
					
#if SAVE_LOCAL_VAR_TO_E2
					// Save the local points whose values are changed in the program to EEPROM.
					if(point->number >= base_in)
					{
						old_value = inputs[point->number].value;						
						if(old_value != *val_ptr)
						{
							put_local_var_to_BUF(point, *val_ptr);
						}
					}
#endif
					if(sptr.pin->value != temp)
						write_page_en[IN] = 1;
					sptr.pin->value = swap_double(*val_ptr);
				// add clear counter in program
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI)

					if((inputs[point->number].range == HI_spd_count) || (inputs[point->number].range == N0_2_32counts)
						|| (inputs[point->number].range == RPM)
					)
					{							
						if(swap_double(inputs[point->number].value) == 0) 
						{
							high_spd_counter[point->number] = 0; // clear high spd count	
							clear_high_spd[point->number] = 1;

						}											
					}	
					
#endif					
				}

				break;
		  	case VAR:
				sptr.pvar = &vars[point->number];
				if((sptr.pvar->auto_manual == 0) && (prog_op == 0) || (sptr.pvar->auto_manual == 1) && (prog_op == 1) )
				{	
							
					if(sptr.pvar->digital_analog == 0)
					{	
						temp = *val_ptr ? 1 : 0;
#if ARM_TSTAT_WIFI
					if(sptr.pvar->control != temp)
					{	
						if((digital_top_area_type == VAR) && (point->number == digital_top_area_num))
							digital_top_area_changed = 1;						
					}			
#endif	
						sptr.pvar->control = *val_ptr ? 1 : 0;
					} 					
					temp = swap_double(*val_ptr);
#if ARM_TSTAT_WIFI
					if(sptr.pvar->value != temp)
					{	
						if((digital_top_area_type == VAR) && (point->number == digital_top_area_num))
							digital_top_area_changed = 1;						
					}			
#endif	

#if SAVE_LOCAL_VAR_TO_E2					
					// Save the local points whose values are changed in the program to EEPROM.					
					{
						old_value = vars[point->number].value;
						
						if(old_value != *val_ptr)
						{
							put_local_var_to_BUF(point,*val_ptr);
						}
					}
#endif
					if(sptr.pvar->value != temp)
						write_page_en[VAR] = 1;
					sptr.pvar->value = temp;			
				}

				break;
		  	case CON:
				sptr.pcon = &controllers[point->number];
				if( (sptr.pcon->auto_manual == 0) && (prog_op == 0) || (sptr.pcon->auto_manual == 1) && (prog_op == 1) )
				{		
				//	sptr.pcon->value = *val_ptr;
					temp = *val_ptr ? 1 : 0;
#if (ASIX_MINI || ASIX_CM5)
					sptr.pcon->value = convert_pointer_to_double(temp); 
#endif
				}
				break;
		  	case WRT:
				sptr.pwr = &weekly_routines[point->number];
				if( (sptr.pwr->auto_manual == 0) && (prog_op == 0) || (sptr.pwr->auto_manual== 1) && (prog_op == 1) )		 
				{	
					temp = *val_ptr ? 1 : 0;
#if (ASIX_MINI || ASIX_CM5)
					sptr.pwr->value = convert_pointer_to_double(temp); 
#endif
				}
				break;
		  	case AR:
				sptr.panr = &annual_routines[point->number];
				if( !sptr.panr->auto_manual && !prog_op || sptr.panr->auto_manual && prog_op )		 
				{
				//	sptr.panr->value = *val_ptr ? 1 : 0;
					temp = *val_ptr ? 1 : 0;
#if (ASIX_MINI || ASIX_CM5)
					sptr.pwr->value = convert_pointer_to_double(temp); 
#endif
				}
				break;
		  	case PRG:
				sptr.pprg = &programs[point->number];
				if( (sptr.pprg->auto_manual == 0) && (prog_op == 0) || (sptr.pprg->auto_manual == 1) && (prog_op == 1) )
				{		 
				//	sptr.pprg->on_off = *val_ptr ? 1 : 0;
					temp = *val_ptr ? 1 : 0;
#if (ASIX_MINI || ASIX_CM5)
					sptr.pprg->on_off = convert_pointer_to_double(temp); 
#endif
				}
				break;
		 case ARRAY: 
				sptr.pary = &arrays[point->number];
				if( aux >= sptr.pary->length || aux < 0 )				aux = 0;		
				*(arrays_address[point->number] + aux) = *val_ptr;
		 
				Update_Array();
				break;
			  	
			case AMON:
				sptr.pmon = &monitors[point->number];
				sptr.pmon->status = *val_ptr ? 1 : 0;
				break;

		  /*case TOTAL:
				sptr.ptot = &totalizers[point->number];
				sptr.ptot->active = *val_ptr ? 1 : 0;
				break;*/
	
	/*			sptr.ptot->time_on = ptr->point_value; */
		 	}
	  	}
		if(index != -1)
		{
			if(index < MAXLOCALPOINTS)
			{
				local_points_list[index].point_value = *val_ptr;
				local_points_list[index].point.panel = panel_number;
				local_points_list[index].point.sub_id = panel_number;
				local_points_list[index].point.point_type = point->point_type;
				local_points_list[index].point.number = point->number;
				local_points_list[index].time_to_live = 30;
				local_points_list[index].decomisioned = 1;	
				local_points_list[index].point.network_number = 0;
				Last_Contact_Local_points[index] = 0;
			}
		}
		
		
		
		return point->number;
 	}
	
	point->point_type = 0;
	point->number = 0;
	*val_ptr = 0;
	return -1;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: put_net_point_value
 * Purpose: 
 * Params:
 * Returns:
 * Note:  write remote points and network points,put them into read and write buffer, wait for dealing
 * ----------------------------------------------------------------------------
 */
S16_T put_net_point_value( Point_Net *p, S32_T *val_ptr, S16_T aux, S16_T prog_op , U8_T mode )
{
	S16_T index;
//	uint32_t instance;
	Point_Net point;
	REMOTE_POINTS * ptr;
	NETWORK_POINTS * ptr1;
	U16_T reg = 0;
	S32_T value;
//	U8_T far temp[4];
	
	U8_T high_3bit;
	U8_T high_5bit;
	high_3bit = 0;
	high_5bit = 0;
	memcpy( &point, p, sizeof(Point_Net));
/*	point.network_number = Setting_Info.reg.network_number;
	
	if( point.network_number != Setting_Info.reg.network_number)
		return 0;*/  // change structure, network is used for instance
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )	
	/*instance = */get_point_info_by_instacne(&point);
#endif
	if(point.panel == 0)
	{
		return -1;
	}
	if(/*( point.network_number != Setting_Info.reg.network_number) ||*/( point.panel != panel_number)
		|| ((point.sub_id != panel_number) && (point.sub_id != 0)))	
	{
		if( (point.panel == panel_number) || // 1.2.MB_REGx
			(current_online[point.panel / 8] & (1 << (point.panel % 8))) // 2VARx  sub modbus device
			) 
		{		
			if( ( index = find_remote_point( &point ) ) < 0 )
			{
				if( ( index = insert_remote_point( &point, -1 ) ) < 0 )
				{
					return -1;
				}
			}
			// modbus remote point
			if(check_point_type(&point) == 1)
			{
				U8_T point_type;
				U8_T flag;
				ptr = (REMOTE_POINTS *)(&remote_points_list[0]) + index;
								
				point_type = (ptr->point.point_type & 0x1f) + (ptr->point.network_number & 0x60);
				
				high_3bit = ptr->point.point_type >> 5;		
				
				if(ptr->point.network_number & 0x80) 
						high_5bit = ptr->point.network_number & 0x1f;	
					else
						high_5bit = 0;					
					
				if((point_type == (VAR + 1)) \
				|| (point_type == (MB_IN_REG + 1))\
				|| (point_type == (MB_REG + 1)) \
				|| ((point_type >= MB_HOLDING_FLOAT_ABCD + 1) &&( point_type <= MB_HOLDING_FLOAT_DCBA + 1))\
				|| ((point_type >= MB_INPUT_FLOAT_ABCD + 1) &&( point_type <= MB_INPUT_FLOAT_DCBA + 1))\
				|| (point_type == (OUT + 1)) \
				|| (point_type == (IN + 1)) 
				)
				{		
					if(aux == 0)
					{	
						if(ptr->point_value != *val_ptr)
						{						
							if((point_type >= MB_HOLDING_FLOAT_ABCD + 1) &&( point_type <= MB_HOLDING_FLOAT_DCBA + 1))
							{						
								Float_to_Byte((float)(*val_ptr) / 1000,(U8_T *)&value,point_type - MB_HOLDING_FLOAT_ABCD);
								write_parameters_to_nodes(0x10,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,4);					
							}
							else if((point_type >= MB_INPUT_FLOAT_ABCD + 1) &&( point_type <= MB_INPUT_FLOAT_DCBA + 1))
							{				
								Float_to_Byte((float)(*val_ptr) / 1000,(U8_T *)&value,point_type - MB_INPUT_FLOAT_ABCD);
								write_parameters_to_nodes(0x10,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,4);					
							}
							else
							{								
								if(point.panel != panel_number) // 3VAR1 3IN1 3OUT1
								{ //  sub modbus device
									uint16_t reg;
									uint8_t len;
									uint16 val[2];
									if(ptr->digital_analog == 100) // digital on	
									{
										if(*val_ptr == 1000)									
											*val_ptr = 1;
									}
									val[0] = HTONS(*val_ptr >> 16);
									val[1] = HTONS(*val_ptr);																	
									reg = get_reg_from_list(point.point_type,point.number,&len);				
									write_parameters_to_nodes(0x10,point.panel,reg,(U16_T*)&val,4);					
								}
								else
								{// 1.3.MB_REG
									value = *val_ptr / 1000;
									write_parameters_to_nodes(0x06,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,1);					
								}
							}
						}
					}
					else
					{
						//value = crc16(val_ptr,aux);
						if(value != ptr->point_value)
							write_parameters_to_nodes(0x10,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)val_ptr,aux);	
						
					}				
					
				}
								
				if((point_type == (MB_COIL_REG + 1)) || (point_type == (MB_DIS_REG + 1)))
				{	
					if(aux == 0)
					{
						if(ptr->point_value != *val_ptr)
						{
							value = (float)*val_ptr / 1000; // write coil
							write_parameters_to_nodes(0x05,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,1);					
						}
					}
					else
					{
						if(value != ptr->point_value)
							write_parameters_to_nodes(0x0f,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)val_ptr,aux);	
   // 0F multi-write_coil
						
					}
				}
				
				if(aux == 1)
				{
					ptr->time_to_live = 60;
				}
				else
				{
					if(mode == 0)
						ptr->time_to_live = RM_TIME_TO_LIVE;
					else if(mode == 2)
						ptr->time_to_live = 60;
					else 
						ptr->time_to_live = PERMANENT_LIVE;
	//				ptr->decomisioned = 1;
				}
				
				ptr->instance = 0;
			}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
			else
			{  // write remote bacnet points
				ptr = (REMOTE_POINTS *)(&remote_points_list[0]) + index;				
				// write MSTP remote point
		//value  = ptr->point_value;//temp[0] + (U16_T)(temp[1] << 8) + ((U32_T)temp[2] << 16) + ((U32_T)temp[3] << 24);
				if(ptr->point.point_type == BAC_AV + 1)
				{				
					if(ptr->point_value != *val_ptr /*/ 1000*/)
					{
						WriteRemotePoint(OBJECT_ANALOG_VALUE,
									point.number,
									point.panel,
									point.sub_id ,
									(float)*val_ptr / 1000,
									BAC_MSTP);
						ptr->point_value = (*val_ptr);
					}					
				}
				else if(ptr->point.point_type == BAC_AI + 1)
				{
					if(ptr->point_value != *val_ptr /*/ 1000*/)
					{	
						//value = (float)*val_ptr / 1000;
						WriteRemotePoint(OBJECT_ANALOG_INPUT,
									point.number,
									point.panel,
									point.sub_id ,
									(float)*val_ptr / 1000,
									BAC_MSTP);	
						ptr->point_value = (*val_ptr);
					}
				}
				else if(((ptr->point.point_type == OUT + 1) && (point.number >= max_dos))
					|| (ptr->point.point_type == BAC_AO + 1))
				{		// ptransfer output 	
					if(ptr->point_value != *val_ptr /*/ 1000*/)
					{		
						if(ptr->point.point_type == BAC_AO + 1)
						{
							WriteRemotePoint(OBJECT_ANALOG_OUTPUT,
								point.number,
								point.panel,
								point.sub_id ,
								(float)*val_ptr / 1000,
								BAC_MSTP);
						}
						else
						{
							WriteRemotePoint(OBJECT_ANALOG_OUTPUT,
								point.number - max_dos,
								point.panel,
								point.sub_id ,
								(float)*val_ptr / 1000,
								BAC_MSTP);
						}
						ptr->point_value = (*val_ptr);
					}
				}
				else if(((ptr->point.point_type == OUT + 1) && (point.number < max_dos))
					|| (ptr->point.point_type == BAC_BO + 1))
				{		// ptransfer output 	
					if(ptr->point_value != *val_ptr)
					{		
						WriteRemotePoint(OBJECT_BINARY_OUTPUT,
									point.number,
									point.panel,
									point.sub_id ,
									(float)*val_ptr / 1000,
									BAC_MSTP);
						ptr->point_value = (*val_ptr);
					}
				
				}
				if(ptr->point.point_type == BAC_BV + 1)
				{	
					if(ptr->point_value != *val_ptr)
					{		
						WriteRemotePoint(OBJECT_BINARY_VALUE,
									point.number,
									point.panel,
									point.sub_id ,
									(*val_ptr ? 1 : 0),
									BAC_MSTP);
						ptr->point_value = *val_ptr ;
					}					
				}
				if(ptr->point.point_type == BAC_BI + 1)
				{				
					if(ptr->point_value != *val_ptr)
					{			

						WriteRemotePoint(OBJECT_BINARY_INPUT,
									point.number,
									point.panel,
									point.sub_id ,
									(*val_ptr ? 1 : 0),
									BAC_MSTP);
						ptr->point_value = (*val_ptr);
					}					
				}
				if(ptr->point.point_type == BAC_MSV + 1)
				{				
					if(ptr->point_value != *val_ptr)
					{			

						WriteRemotePoint(OBJECT_MULTI_STATE_VALUE,
									point.number,
									point.panel,
									point.sub_id ,
									(*val_ptr ? 1 : 0),
									BAC_MSTP);
						ptr->point_value = (*val_ptr);
					}					
				}
				// ...
			
				ptr->instance = Get_device_id_by_panel(point.panel,point.sub_id,BAC_MSTP);
			}
#endif
//			ptr->change = 1;
			if(mode == 0)
				ptr->time_to_live = RB_TIME_TO_LIVE;
			else
				ptr->time_to_live = PERMANENT_LIVE;		

		}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )

		else  // network points 
		{
// write network  points
			if( ( index = find_network_point( &point ) ) < 0 )
			{
				if( ( index = insert_network_point( &point, -1 ) ) < 0 ) return -1;
			}
#if  NETWORK_MODBUS_BAC				

			if(check_point_type(&point) == 1)
			{// network modbus point
				U8_T point_type;
				ptr1 = (NETWORK_POINTS *)(&network_points_list[0]) + index;
				point_type = (ptr1->point.point_type & 0x1f) + (ptr1->point.network_number & 0x60);
				high_3bit = ptr1->point.point_type >> 5;					
				
				if((point_type == (VAR + 1)) \
				|| (point_type == (MB_IN_REG + 1))\
				|| (point_type == (MB_REG + 1)) \
				|| (point_type == (MB_COIL_REG + 1)) \
				|| ((point_type >= MB_HOLDING_FLOAT_ABCD + 1) &&( point_type <= MB_HOLDING_FLOAT_DCBA + 1)) \
				|| ((point_type >= MB_INPUT_FLOAT_ABCD + 1) &&( point_type <= MB_INPUT_FLOAT_DCBA + 1))) 
				{					
					if(ptr1->point.network_number & 0x80) 
						high_5bit = ptr1->point.network_number & 0x1f;	
					else
						high_5bit = 0;		
					if(ptr1->point_value != *val_ptr)
					{		
						if((point_type >= MB_HOLDING_FLOAT_ABCD + 1) &&( point_type <= MB_HOLDING_FLOAT_DCBA + 1))
						{
							Float_to_Byte((float)(*val_ptr) / 1000,(U8_T *)&value,point_type - MB_HOLDING_FLOAT_ABCD);
							//network_points_list[index].invoked_id =
							write_NP_Modbus_to_nodes(ptr1->point.panel,0x10,ptr1->point.sub_id,ptr1->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,2);	
						}
						else if((point_type >= MB_INPUT_FLOAT_ABCD + 1) &&( point_type <= MB_INPUT_FLOAT_DCBA + 1))
						{
							Float_to_Byte((float)(*val_ptr) / 1000,(U8_T *)&value,point_type - MB_INPUT_FLOAT_ABCD);
							//network_points_list[index].invoked_id =
							write_NP_Modbus_to_nodes(ptr1->point.panel,0x10,ptr1->point.sub_id,ptr1->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,2);	
						}
						else	
						{	
							value = *val_ptr / 1000;
							if(point_type == (MB_COIL_REG + 1))  // WRITE COIL
							{
								//network_points_list[index].invoked_id =
									write_NP_Modbus_to_nodes(ptr1->point.panel,0x05,ptr1->point.sub_id,ptr1->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,1);					
							}
							else									
								//network_points_list[index].invoked_id =
									write_NP_Modbus_to_nodes(ptr1->point.panel,0x06,ptr1->point.sub_id,ptr1->point.number + 256 * high_3bit + 2048 * high_5bit,(U16_T*)&value,1);					
						}
							ptr1->point_value = *val_ptr;
					}

//				if(((ptr1->point.point_type & 0x1f) == (MB_COIL_REG + 1)) || ((ptr1->point.point_type & 0x1f) == (MB_DIS_REG + 1)))
//				{	
//					if(ptr1->point_value != *val_ptr)
//					{
//						value = (float)*val_ptr / 1000; // write coil
//						//write_parameters_to_nodes(0x05,remote_points_list[index].tb.RP_modbus.id,ptr->point.number + high_3bit * 256,(U16_T*)&value,1);					
//					}
//				}
				}
				if(mode == 0)
				{					
					ptr1->time_to_live = 30;//NP_TIME_TO_LIVE * number_of_network_points_modbus;
				}
				else if(mode == 2)
					ptr1->time_to_live = 60;
				else 
					ptr1->time_to_live = PERMANENT_LIVE;
				
				ptr1->instance = 0;
			}
			else

			{
				ptr1 = (NETWORK_POINTS *)(&network_points_list[0]) + index;
				if(point.sub_id == 0) // network point
					point.sub_id = point.panel;
				
				value = ptr1->point_value;

// write MSTP remote point
				if(ptr1->point.point_type == VAR + 1 
					|| ptr1->point.point_type == IN + 1 
					|| ptr1->point.point_type == OUT + 1)
				{
					if(value != *val_ptr)
					{		
						uint16_t reg;
						uint8_t len;
						uint32 deviceid = Get_device_id_by_panel(point.panel,point.sub_id,BAC_IP_CLIENT);
						if(deviceid != 0)
						{
							reg = get_reg_from_list(ptr1->point.point_type,point.number,&len);
							WritePrivateBacnetToModbusData(deviceid,reg,len,(float)*val_ptr);
							ptr1->point_value = *val_ptr;
						}
						
					}

				}
				else 	if(ptr1->point.point_type == BAC_AV + 1)
				{	
					if(value != *val_ptr)
					{		
						write_NP_Bacnet_to_nodes(OBJECT_ANALOG_VALUE,
									point.number/* + 1*/,
									point.panel,
									point.sub_id ,
									(float)*val_ptr / 1000);
						ptr1->point_value = *val_ptr;
						
					}
					
				}
				else if(ptr1->point.point_type == BAC_AI + 1)
				{
					
				}
				else if(((ptr1->point.point_type == OUT + 1) && (point.number >= max_dos))
					|| (ptr1->point.point_type == BAC_AO + 1))
				{		// ptransfer output 	
					if(value != *val_ptr)
					{		
						if(ptr1->point.point_type == BAC_AO + 1)
						{
							write_NP_Bacnet_to_nodes(	OBJECT_ANALOG_OUTPUT,
								point.number/* + 1*/,
								point.panel,
								point.sub_id ,
								(float)*val_ptr / 1000);					
						}
						else
						{
							write_NP_Bacnet_to_nodes(OBJECT_ANALOG_OUTPUT,
								point.number - max_dos /* + 1*/,
								point.panel,
								point.sub_id ,
								(float)*val_ptr / 1000);
						}
						ptr1->point_value = *val_ptr;
					}
				}
				else if(((ptr1->point.point_type == OUT + 1) && (point.number < max_dos))
					|| (ptr1->point.point_type == BAC_BO + 1))
				{		// ptransfer output 
					if(value != *val_ptr)
					{		
						write_NP_Bacnet_to_nodes(OBJECT_BINARY_OUTPUT,
									point.number/* + 1*/,
									point.panel,
									point.sub_id ,
									(float)*val_ptr / 1000);
						ptr1->point_value = *val_ptr;
					}
				}
				else if(ptr1->point.point_type == BAC_BV + 1)
				{		// ptransfer output 
					if(value != *val_ptr)
					{		
						write_NP_Bacnet_to_nodes(OBJECT_BINARY_VALUE,
									point.number/* + 1*/,
									point.panel,
									point.sub_id ,
									(float)*val_ptr / 1000);
						ptr1->point_value = *val_ptr;
					}
				}
				else if(ptr1->point.point_type == IN + 1)
				{  	// ptransfer input
					
				}
				
//				if(point.panel != 0)		
//				{					
					ptr1->instance = Get_device_id_by_panel(point.panel,point.sub_id,BAC_IP_CLIENT);
				
					if(mode == 0)
					{
						if(number_of_network_points_bacnet > 10)
							ptr1->time_to_live = 250;
						else
							ptr1->time_to_live = NP_TIME_TO_LIVE * number_of_network_points_bacnet;
					}
					else if(mode <= 2)
						ptr1->time_to_live = PERMANENT_LIVE;
					else
						ptr1->time_to_live = mode; // for cov->timeRemaining
				
//				}
//				else // device is not in database, still show it in network points table
//				{
//					ptr1->instance = instance;
//					ptr1->time_to_live = NP_TIME_TO_LIVE;
//				}			

			}		
#endif			
		}
#endif
	}
	else
	{
// local points
		if(prog_op == 1)
		{	
			put_point_value( (Point *)p, val_ptr, aux, prog_op );
		}		
	}

	return 1;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: get_net_point_value
 * Purpose: according to the network number of the networt point, get point value of the remote points,
 * Params: mode : 0 - not permanent, 1 - permanent
 * Returns:
 * Note: read remote points and network points
 * ----------------------------------------------------------------------------
 */
// mode -- 0 is RM_TIME_TO_LIVE, 1 is PERMANENT_LIVE
// flag -- 1 is grahpic, 0 is other, 2 -- new point structure

S16_T get_net_point_value( Point_Net *p, S32_T *val_ptr , U8_T mode,U8_T flag)
{
	S16_T index;
	Point_Net point;
	S32_T tempval = 0;
//	uint32_t instance;
	
	memcpy( &point, p, sizeof(Point_Net));
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )		
	/*instance = */get_point_info_by_instacne(&point);
#endif
	if(point.panel == 0)
	{
		return -1;
	}
	
	if(/*( point.network_number != Setting_Info.reg.network_number) ||*/( point.panel != panel_number)
		|| ((point.sub_id != panel_number) && (point.sub_id != 0)))	
	{
		if( (point.panel == panel_number) || // 1.2.MB_REGx
			(current_online[point.panel / 8] & (1 << (point.panel % 8))) // 2VARx  sub modbus device
			)  // remote points
		{
		 // remote points
			if( ( index = find_remote_point( &point ) ) < 0 )
			{				
				if( ( index = insert_remote_point( &point, -1 ) ) < 0 ) 
					return -1;
			}	
			if(index < MAXREMOTEPOINTS)
			{
				if(check_point_type(&point) == 1)
				{
					remote_points_list[index].instance = 0;
					if(remote_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{	
						*val_ptr = swap_double(remote_points_list[index].point_value);	

						if(mode == 0)
							remote_points_list[index].time_to_live = RM_TIME_TO_LIVE;
						else
							remote_points_list[index].time_to_live = PERMANENT_LIVE;
						
					}
					else
					{	
						if(mode == 0)
							remote_points_list[index].time_to_live = RM_TIME_TO_LIVE;
						else
							remote_points_list[index].time_to_live = PERMANENT_LIVE;
						
						*val_ptr = 0;
						return 0;
					}
				}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )
				else		// remote bacnet points
				{
					remote_points_list[index].instance = 
					Get_device_id_by_panel(remote_points_list[index].point.panel,remote_points_list[index].point.sub_id,BAC_MSTP);
					
					if(remote_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{	
						
						*val_ptr = remote_points_list[index].point_value;	
						
						if(mode == 0)
							remote_points_list[index].time_to_live = RB_TIME_TO_LIVE;
						else
							remote_points_list[index].time_to_live = PERMANENT_LIVE;
					}
					else
					{
						*val_ptr = 0;
						return 0;
					}
				}
#endif
			}
					
		}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )

		else  // network points
		{
			if(flag == 1)  // enter is graphic, ingore network bacnet points
				return -1;
			if( ( index = find_network_point( &point ) ) < 0 )
			{		
				if( ( index = insert_network_point( &point, -1 ) ) < 0 ) 
					return -1;
			}				
			if(index < MAXNETWORKPOINTS)
			{
#if NETWORK_MODBUS_BAC	
				if(check_point_type(&point) == 1)  // modbus points
				{
					network_points_list[index].instance = 0;				
					
					if(network_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{	
						*val_ptr = network_points_list[index].point_value;	
						
						if(mode == 0)
							network_points_list[index].time_to_live = 30;
						else
							network_points_list[index].time_to_live = PERMANENT_LIVE;
						
					}
					else
					{
						network_points_list[index].time_to_live = 250;//NP_TIME_TO_LIVE * number_of_network_points_modbus;
						*val_ptr = 0;
						return 0;
					}					
				}
				else
#endif
				{
//					if(network_points_list[index].point.panel != 0)
//					{
						network_points_list[index].instance = 
							Get_device_id_by_panel(network_points_list[index].point.panel,network_points_list[index].point.sub_id,BAC_IP_CLIENT);
						if(mode == 0)
						{
							if(number_of_network_points_bacnet > 10)
								network_points_list[index].time_to_live = 250;//NP_TIME_TO_LIVE * number_of_network_points_bacnet;
							else
								network_points_list[index].time_to_live = NP_TIME_TO_LIVE * number_of_network_points_bacnet;
						}
						else
							network_points_list[index].time_to_live = PERMANENT_LIVE;	
//					}
//					else
//					{
//						U8_T remote_i;
//						// if device is 0, need GET_PANEL_INFO to get panel first
//						// not in database, add it to network points table
//						network_points_list[index].instance = instance;
//						network_points_list[index].time_to_live = NP_TIME_TO_LIVE;
//						Test[27]++;
//						Test[28] = network_points_list[index].point.panel;
//						Test[29] = network_points_list[index].point.sub_id;
//						if(Get_remote_index_by_device_id(instance,&remote_i) != -1)
//						{Test[19]++;
//							network_points_list[index].point.panel = remote_panel_db[remote_i].panel;
//							network_points_list[index].point.sub_id = remote_panel_db[remote_i].panel;
//						}				
						
//					}
					
					
					
					
					if(network_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{		
						if(network_points_list[index].point_value == 0 )
						{
							return 0;
						}
						*val_ptr = network_points_list[index].point_value;	
						if(mode == 0)
							network_points_list[index].time_to_live = 250;//NP_TIME_TO_LIVE * number_of_network_points_bacnet;
						else
							network_points_list[index].time_to_live = PERMANENT_LIVE;
					
					}
					else
					{
						*val_ptr = 0;//DEFUATL_REMOTE_NETWORK_VALUE;
						return 0;
					}
				}					
			}				
		}

#endif
	}
	else 
	{ // local points
		get_point_value( (Point *)p, &tempval );
		*val_ptr = tempval;
	}	

	return 1;
}
#if 0
S16_T get_net_point_value_new( Point_Bacnet *p, S32_T *val_ptr , U8_T mode,U8_T flag)
{
	S16_T index;
	Point_Bacnet ptr_bacp;
	STR_REMOTE_PANEL_DB *ptr;
	S32_T tempval = 0;
	uint8_t i;
	uint32_t instance;
	uint8_t point_type;
	uint32_t obj_number;
	Point_Net point;

	memcpy( &ptr_bacp, p, sizeof(Point_Bacnet));
	instance = (((uint32_t)ptr_bacp.instance_high_byte) << 24) + (((uint16_t)ptr_bacp.instance_mid_byte) << 16) + ptr_bacp.instance_low_byte;
	point_type = ptr_bacp.point_type;
	obj_number = (((uint32_t)ptr_bacp.obj_number_high) << 24) + (((uint16_t)ptr_bacp.obj_number_mid) << 16) + ptr_bacp.obj_number_low;
	
	if(instance == 0)
			return 0;	
	
	ptr = remote_panel_db;
	for(i = 0;i < remote_panel_num;i++,ptr++)
	{
		if(ptr->device_id == instance)
		{
			point.panel = ptr->panel;
			point.sub_id = ptr->sub_id;
		}
	}	
	
	if(( point.panel != panel_number)
		|| ((point.sub_id != panel_number) && (point.sub_id != 0)))	
	{
		if( point.panel == panel_number)  // remote points
		{
		 // remote points
			if( ( index = find_remote_point( &point ) ) < 0 )
			{				
				if( ( index = insert_remote_point( &point, -1 ) ) < 0 ) 
					return -1;
			}	
			if(index < MAXREMOTEPOINTS)
			{
					remote_points_list[index].instance = 
					Get_device_id_by_panel(remote_points_list[index].point.panel,remote_points_list[index].point.sub_id,BAC_MSTP);
					if(remote_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{	
				
						*val_ptr = remote_points_list[index].point_value;	
						
						if(mode == 0)
							remote_points_list[index].time_to_live = RB_TIME_TO_LIVE;
						else
							remote_points_list[index].time_to_live = PERMANENT_LIVE;
//						remote_points_list[index].decomisioned = 1;
					}
					else
					{
						*val_ptr = 0;
						return 0;
					}

			}
					
		}
#if (ARM_MINI || ARM_CM5 || ARM_TSTAT_WIFI )

		else  // network points
		{
			if(flag == 1)  // enter is graphic, ingore network bacnet points
				return -1;
			if( ( index = find_network_point( &point ) ) < 0 )
			{		
				if( ( index = insert_network_point( &point, -1 ) ) < 0 ) 
					return -1;
			}							
			if(index < MAXNETWORKPOINTS)
			{
#if NETWORK_MODBUS_BAC	
				if(check_point_type(&point) == 1)  // modbus points
				{
					network_points_list[index].instance = 0;				
					
					if(network_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{	
						*val_ptr = network_points_list[index].point_value;	
						
						if(mode == 0)
							network_points_list[index].time_to_live = NP_TIME_TO_LIVE;
						else
							network_points_list[index].time_to_live = PERMANENT_LIVE;
						
					}
					else
					{
						network_points_list[index].time_to_live = NP_TIME_TO_LIVE;
						*val_ptr = 0;
						return 0;
					}					
				}
				else
#endif
				{
					//*val_ptr = network_points_list[index].point_value;	
					network_points_list[index].instance = 
					Get_device_id_by_panel(network_points_list[index].point.panel,network_points_list[index].point.sub_id,BAC_IP_CLIENT);
					
					if(mode == 0)
						network_points_list[index].time_to_live = NP_TIME_TO_LIVE;
					else
						network_points_list[index].time_to_live = PERMANENT_LIVE;	
					
			
					if(network_points_list[index].point_value != DEFUATL_REMOTE_NETWORK_VALUE)
					{		
						if(network_points_list[index].point_value == 0 )
						{
							return 0;
						}
						*val_ptr = network_points_list[index].point_value;	
						if(mode == 0)
							network_points_list[index].time_to_live = NP_TIME_TO_LIVE;
						else
							network_points_list[index].time_to_live = PERMANENT_LIVE;
						
					}
					else
					{
						*val_ptr = 0;//DEFUATL_REMOTE_NETWORK_VALUE;
						return 0;
					}
				}					
			}				
		}

#endif
	}
	else 
	{ // local points
		get_point_value( (Point *)p, &tempval );
		*val_ptr = tempval;
	}	

	return 1;
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: get_point_info
 * Purpose: 
 * Params:
 * Returns:
 * Note:   it is called back in sever.c
 * ----------------------------------------------------------------------------
 */
S16_T get_point_info( Point_info *ptr )
/* returns 1 if OK, 0 if point doesn't exist */
{
	Str_points_ptr sptr;
	S16_T absent = 1;
	bit tempbit;
	U8_T temp;
	ptr->decomisioned = 0;
	
	if( ptr->number < table_bank[ptr->point_type-1] )
	{
		ptr->point_value = 0;
		ptr->auto_manual = 0;
		ptr->digital_analog = 0;

		switch( ptr->point_type - 1)
		{
			case OUT:				
				sptr.pout = &outputs[ptr->number];
				ptr->auto_manual = sptr.pout->auto_manual;
				ptr->digital_analog = sptr.pout->digital_analog;
				ptr->decomisioned = sptr.pout->decom;				
			
				ptr->units = sptr.pout->range;
	
				if( !sptr.pout->digital_analog ) // DIGITAL
				{
					ptr->point_value = swap_double(sptr.pout->control? 1000L:0L);
					ptr->units -= DIG1;
				}
				else
				{
					ptr->point_value = sptr.pout->value;
				}     
				break;
			case IN:
				sptr.pin = &inputs[ptr->number];
			
				ptr->auto_manual = sptr.pin->auto_manual;
				ptr->digital_analog = sptr.pin->digital_analog;
				ptr->decomisioned = sptr.pin->decom;
				
				ptr->units = sptr.pin->range;
				if(!sptr.pin->digital_analog )
				{
					ptr->point_value = swap_double( sptr.pin->control ? 1000L : 0L);
					ptr->units -= DIG1;					
				}
				else
				{
					ptr->point_value = sptr.pin->value;					
				}
				break;
			case VAR:
				sptr.pvar = &vars[ptr->number];
				ptr->auto_manual = sptr.pvar->auto_manual;
				ptr->digital_analog = sptr.pvar->digital_analog;
			
				ptr->units = sptr.pvar->range;

				if( !sptr.pvar->digital_analog )
				{
					ptr->point_value = swap_double(sptr.pvar->control ? 1000L : 0L);
					ptr->units -= DIG1;
				}
				else
				{
					ptr->point_value = sptr.pvar->value;
				}
				break;
			case CON:
				sptr.pcon = &controllers[ptr->number];
				ptr->auto_manual = sptr.pcon->auto_manual;
				ptr->digital_analog = 1;
	
				ptr->units = procent;
				ptr->point_value = sptr.pcon->value;
				break;
/*
		case TOTAL:
				sptr.ptot = &totalizers[ptr->number];
				ptr->auto_manual = sptr.ptot->active;
				ptr->point_value = sptr.ptot->time_on;
				ptr->digital_analog = 1;
				ptr->units = time_unit;
				break;
*/
		case WRT:
				sptr.pwr = &weekly_routines[ptr->number];
				ptr->auto_manual = sptr.pwr->auto_manual;  
		
				ptr->point_value = sptr.pwr->value?1000L:0;
				ptr->units = 1;
				break;
		case AR:
				sptr.panr = &annual_routines[ptr->number];
				ptr->auto_manual = sptr.panr->auto_manual; 
				ptr->point_value = sptr.panr->value?1000L:0;
				ptr->units = 1;
				break;
		case PRG:
				sptr.pprg = &programs[ptr->number];
				ptr->auto_manual = sptr.pprg->auto_manual;
				ptr->point_value = sptr.pprg->on_off?1000L:0;
				ptr->units = 1;
				break;
		case AMON:
				sptr.pmon = &monitors[ptr->number];
				ptr->auto_manual = 0;
				ptr->point_value = sptr.pmon->status?1000L:0;
				ptr->digital_analog = 0;
				ptr->units = 1;
				break;
		case GRP: 
				sptr.pgrp = &control_groups[ptr->number];
				ptr->auto_manual = 0;
				ptr->point_value = 0;
				ptr->digital_analog = 0;
				ptr->units = 1;
				break;	
    	default:
				absent = 0;
//				ptr->point_value = 0;
//				ptr->auto_manual = 0;			
				ptr->digital_analog = 1;
				ptr->security = 0;
		}

  }
  else
  	absent = 0;
	return absent;
}



S16_T put_point_info( Point_info *info )
{
	Str_points_ptr sptr;
	S16_T i;
	U8_T num_point;
	Point_Net point;

	memcpy( &point, info, sizeof(Point_Net));
	if( point.network_number == 0xFFFF )
		point.network_number = panel_net_info.network_number;
	num_point = info->number;
	if( point.network_number != panel_net_info.network_number )
	{   /*the element is NOT on the LOCAL_PANEL*/
		i = find_remote_point( (Point_Net *)info );
		if( i < 0 ) return -1;
		memcpy( (S8_T*)&remote_points_list[i].point, info, sizeof( Point_info ) );
	}
	else
	{ /* the element is ON the LOCAL_PANEL */
	  	if( info->number < table_bank[info->point_type-1] )
    	{
			switch( point.point_type - 1)
			{
			 case OUT:
				sptr.pout = &outputs[num_point];
				sptr.pout->auto_manual = info->auto_manual;
				sptr.pout->digital_analog = info->digital_analog;
				sptr.pout->decom = info->decomisioned;
				if( !sptr.pout->digital_analog ) // DIGITAL
				{
					sptr.pout->control = info->point_value ? 1 : 0;
				}
				sptr.pout->value = info->point_value;
				break;
			 case IN:
				sptr.pin = &inputs[num_point];
				sptr.pin->auto_manual = info->auto_manual;
				sptr.pin->digital_analog = info->digital_analog;
				sptr.pin->decom = info->decomisioned;
				if( !sptr.pin->digital_analog )
				{
					sptr.pin->control = info->point_value ? 1 : 0;
				}
				sptr.pin->value = info->point_value;
				break;
			 case VAR:
				sptr.pvar = &vars[num_point];
				sptr.pvar->auto_manual = info->auto_manual;
				sptr.pvar->digital_analog = info->digital_analog;
				if( !sptr.pvar->digital_analog )
				{
					sptr.pvar->control = info->point_value ? 1 : 0;
				}
				sptr.pvar->value = info->point_value;
				break;
			 case CON:
				sptr.pcon = &controllers[num_point];
				sptr.pcon->auto_manual = info->auto_manual;
				sptr.pcon->value = info->point_value;
				break;
			 case WRT:
				sptr.pwr = &weekly_routines[num_point];
				sptr.pwr->auto_manual = info->auto_manual;
				sptr.pwr->value = info->point_value ? 1 : 0;
				break;
			 case AR:
				sptr.panr = &annual_routines[num_point];
				sptr.panr->auto_manual = info->auto_manual;
				sptr.panr->value = info->point_value ? 1 : 0;
				break;
			 case PRG:
				sptr.pprg = &programs[num_point];
				sptr.pprg->auto_manual = info->auto_manual;
				sptr.pprg->on_off = info->point_value ? 1 : 0;
				break;
			 case AMON:
				sptr.pmon = &monitors[num_point];
				sptr.pmon->status = info->point_value ? 1 : 0;
				break;
/*
			 case TOTAL:
				sptr.ptot = &totalizers[num_point];
				sptr.ptot->active = info->auto_manual;
				break;
*/
/*				sptr.ptot->time_on = ptr->point_value; */
			}
		}
	}
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: update_grp_element
 * Purpose: update the element of the group ponit
 * Params:
 * Returns:
 * Note:   it is called back in sever.c
 * ----------------------------------------------------------------------------
 */

S16_T update_grp_element( Str_grp_element *ptr )
{
	Str_points_ptr sptr;
	S16_T i;
	if(GetWordBit(ptr->flag5Int,grp_location,2)/*ptr->location*/ ) /* the element is NOT on the LOCAL_PANEL */
	{
		i = find_remote_point( &ptr->point );
		if( i < 0 ) return -1;
		sptr.prp = remote_points_list + i;
		SetByteBit(&ptr->flag1,GetByteBit(sptr.prp->flag1,REMOTE_auto_manual,1),grp_auto_manual,1);  //auto_manual
		SetByteBit(&ptr->flag1,GetByteBit(sptr.prp->flag1,REMOTE_digital_analog,grp_digital_analog),1,1);	// digital_analog
		SetByteBit(&ptr->flag1,GetByteBit(sptr.prp->flag1,REMOTE_decomisioned,1),grp_decomisioned,1);	// decomisioned
	//	ptr->auto_manual = sptr.prp->auto_manual;
	//	ptr->digital_analog = sptr.prp->digital_analog;
/*		ptr->description_label = sptr.prp->description_label;
		ptr->security = sptr.prp->security; */
	//	ptr->decomisioned = sptr.prp->decomisioned;
		ptr->units = sptr.prp->units;
		ptr->point_value = sptr.prp->point_value;
	}
	else   /* the element is on the LOCAL_PANEL */
	{
		if( !get_point_info( (Point_info*)ptr ) )
	    {
			ptr->units = 1;
			SetWordBit(&ptr->flag5Int,1,grp_absent,1);   // absent
		} 
	}
	return 1;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: move_groups
 * Purpose: 
 * Params:
 * Returns:
 * Note:   it is called back in sever.c
 * ----------------------------------------------------------------------------
 */
void move_groups( S8_T *dest, S8_T *source, S16_T length,S16_T no_elem, Str_grp_element *address )
{
  	U8_T direction, i;
	register Aux_group_point *ptr;

  	if( source > dest )
  	{
	  	direction = 0;
	    total_elements -= no_elem;
		group_data_length -= ( no_elem * sizeof( Str_grp_element ) );
  	}
	else
	{
		direction = 1;
		total_elements += no_elem;
		group_data_length += ( no_elem * sizeof( Str_grp_element ) );
	}
	memmove( dest, source, length );
  /* Update start addresses for the groups that were moved */
  	ptr = aux_groups;
	for( i=0; i<MAX_GRPS; i++, ptr++ )
		if( ptr->no_elements && ( address < ptr->address ) )
      		if( direction )
				ptr->address += no_elem;
      		else
				ptr->address -= no_elem;
}




/*
 * ----------------------------------------------------------------------------
 * Function Name: writepropertyvalue
 * Purpose: 
 * Params:
 * Returns:
 * Note:   it is called back in applayer.c(serverBACnet())
 * ----------------------------------------------------------------------------
 */
S16_T writepropertyvalue( BACnetObjectIdentifier *obj, S32_T lvalue )
{
 	Point point;

 	point.number       = obj->instance-1;
	point.point_type   = obj->object_type_low+(obj->object_type_hi<<2) - T3000_OBJECT_TYPE + 1;
 	put_point_value( &point, &lvalue, 0, OPERATOR );

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: writepropertyauto
 * Purpose: 
 * Params:
 * Returns:
 * Note:   it is called back in applayer.c(serverBACnet())
 * ----------------------------------------------------------------------------
 */
S16_T	writepropertyauto( BACnetObjectIdentifier *obj, S16_T auto_manual )
{
	Point point;
	point.number       = obj->instance-1;
	point.point_type   = obj->object_type_low+(obj->object_type_hi<<2) - T3000_OBJECT_TYPE + 1;

  	if( point.number < table_bank[point.point_type-1] )
  	{
		switch (point.point_type-1)
		{
			case OUT:
			/*	outputs[point.number].auto_manual = auto_manual;*/
				SetByteBit(&outputs[point.number].flag1,auto_manual,out_auto_manual,1);
				break;
		 	case IN:
			/*	inputs[point.number].auto_manual = auto_manual;*/
				SetByteBit(&inputs[point.number].flag1,auto_manual,in_auto_manual,1);
				break;
		 	case VAR:
			/*	vars[point.number].auto_manual = auto_manual;*/
				SetByteBit(&vars[point.number].flag,auto_manual,var_auto_manual,1);
				break;
	 		case WRT:
			/*	weekly_routines[point.number].auto_manual = auto_manual;*/
				SetByteBit(&weekly_routines[point.number].flag,auto_manual,weekly_auto_manual,1);
				break;
		 	case AR:
			/*	annual_routines[point.number].auto_manual = auto_manual;*/
				SetByteBit(&annual_routines[point.number].flag,auto_manual,annual_auto_manual,1);
        		misc_flags.check_ar=1;
				break;
	 		case CON:
			/*	controllers[point.number].auto_manual = auto_manual;*/
				SetByteBit(&controllers[point.number].flag,auto_manual,con_auto_manual,1);
      			break;
	 		case PRG:
			/*	programs[point.number].auto_manual = auto_manual;*/
				SetByteBit(&controllers[point.number].flag,auto_manual,prg_auto_manual,1);
      			break;
/*
		 	case TOTAL:
				totalizers[point.number].active = auto_manual;
  	    break;
*/
    	}
	}
	return 0;
}

#endif



void initial_graphic_point(void)
{	
	U8_T i;
	Point_Net point;  
	S32_T value;
	for(i = 0;i < MAX_ELEMENTS;i++)
	{  
		if(group_data_new.old_item[i].reg.label_status == 0) 
			break;
		point.number = group_data_new.old_item[i].reg.nPoint_number;
		point.point_type = group_data_new.old_item[i].reg.nPoint_type;
		point.panel = group_data_new.old_item[i].reg.nMain_Panel;
		point.sub_id = group_data_new.old_item[i].reg.nSub_Panel;
		point.network_number = 0;//Setting_Info.reg.network_number;	
		
		get_net_point_value(&point,&value,1,1);

	}

}

#endif