#include "convert_data.h"

uint32_t u32_ASCII_ToInt(uint8_t *data,int lenght)
{
			uint32_t	retvalue = 0;
			
			uint16_t temp_data = 0;
			uint16_t number = 0;
			uint16_t factor = 1;
	
			for(int i = 0; i < (lenght - 1); i++) 
			{
				factor = factor * 10;
			}
			for(int i = 0; i < lenght;i++)
			{
					temp_data = data[i] & 0x0F;
					retvalue += temp_data * factor;
					factor /= 10;
			}
			
			return retvalue;
}

