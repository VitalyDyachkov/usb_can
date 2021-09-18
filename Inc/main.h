#ifndef MAIN_H
#define MAIN_H
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "usb_parser.h"
#include "convert_data.h"
#include <string.h> 

#define POSITION_MIN 38
#define POSITION_SEC 41
#define POSITION_mSEC 44
#define POSITION_uSEC 48
typedef enum 
{CAN_1 = 1,CAN_2,CAN_ALL}N_Can_t;

typedef struct{
	uint8_t baurate_ifc;
}CurSetCANTypeDef;

typedef struct {
	uint8_t source_msg;
	uint32_t id;
	char buff_can_msg [27];
}Packet ;
typedef struct 
	{
		uint16_t period;
		uint16_t number;
		uint16_t reload_period;
	}SendCAN;
static struct
	{
		uint8_t minute;
		uint8_t secund;
		uint16_t msec;
		uint16_t usec;
		bool not_empty_flag;
	}TimePack;
typedef struct {
	N_Can_t num_can;
	uint32_t id;
	char buff_can_msg [8];
	int dlc;
}Packet_to_CAN ;

typedef enum
{first_init,second_init}Init_t;
typedef enum 
{ 
		Kb_10,
		Kb_20,
		Kb_33_3,
		Kb_50,
		Kb_83_3,
		Kb_100,
		Kb_125,
		Kb_250,
		Kb_333_333,
		Kb_500,
		Kb_666_667,
		Kb_800,
		Mb_1
}S_Can_t;
void MX_CAN_ReInit(N_Can_t can_x,S_Can_t speed,CAN_HandleTypeDef *hcan_x,Init_t number_init,int *nmb_bps);
uint8_t Setting_CAN(void);
void Send_Msg(CAN_HandleTypeDef *hcan_x,bool *flag_bus_ok_can);
#endif /*MAIN_H*/

