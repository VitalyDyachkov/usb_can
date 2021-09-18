#include "usb_parser.h"

Control_CMD_t ParsCMD(uint8_t* data_buff)
{
	Control_CMD_t ret_val = EMPTY;
	uint8_t dictionary_idx;
	char *ptr;
	uint8_t cmp_lng = 0;
	uint8_t n_word = 0;
		if(data_buff[0] == 'A')
		{
			if(data_buff[1] == 'T')
			{
					if(data_buff[2] == '+')
				{
					/*определим в пространстве каких символов стоит искать команду*/
					dictionary_idx = data_buff[3] - 0x41;
					/*произведем поиск команды в списке*/
									int i;
				int x_leng;
				
				for(i = 0;commands_listen[dictionary_idx][i] != 0;i++)
				{
					ptr = commands_listen[dictionary_idx][i];
					x_leng = strlen((const char*)ptr);
					/*если найдено слово проверим словарь на наличие схожих слов*/
						if(strncmp((const char*)data_buff,ptr,x_leng) == 0)
						{
							/*если первое совпадание в этой части словаря*/
							if(cmp_lng == 0)
							{
								cmp_lng = x_leng;
								n_word = i;
							}
							/*повторное совпадение сравним количество совпаших символов если их больше чем при первом совпадении то перезапишим переменные сс*/
							else
							{
								if(cmp_lng < x_leng)
								{
									cmp_lng = x_leng;
									n_word = i;
								}
							}
						}
					}
//				/*команда не найдена*/				
//				if(cmp_lng == 0)
//				{
//					/*проверим не достигли ли мы конца словаря*/
//					if(n_dictionary == 2)
//					{
//						return EMPTY;
//					}
//					else
//					{
//						n_dictionary++;
//						if(n_dictionary < 2)
//						{
//							result = Pars_CMD(cmd_buf,lng_cmd,n_dictionary);
//						}
//							return result;
//					}
//				}
				/*команда найдена*/
				if(cmp_lng != 0)//	else
				{
						ret_val = trans_table[dictionary_idx][n_word];
						return ret_val;
				}				
				}	
			}
		}
}
