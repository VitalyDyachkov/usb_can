#ifndef USB_PARSER_H
#define USB_PARSER_H

#include "main.h"

static char *commands_listen[][11]=
{
/*A*/{0},
/*B*/{0},
/*C*/{"AT+CANSPD:",0},
/*D*/{0},
/*E*/{"AT+EOF",0},
/*F*/{0},
/*G*/{0},
/*H*/{0},
/*I*/{0},
/*J*/{0},
/*K*/{0},
/*L*/{0},
/*M*/{0},
/*N*/{0},
/*O*/{0},
/*P*/{0},
/*Q*/{0},
/*R*/{0},
/*S*/{0},
/*T*/{0},
/*U*/{0},
/*V*/{0},
/*W*/{0},
/*X*/{0},
/*Y*/{0},
/*Z*/{0},
};

/*перечисление типов команд*/
typedef enum
{
	EMPTY = 0,
	CANSPD,
	EOFILE,
}Control_CMD_t;

/*таблица действия в соответсвии с которой принимается решение после получения команды*/
static Control_CMD_t trans_table[][11] = {
/*A*/{EMPTY},
/*B*/{EMPTY},
/*C*/{CANSPD,EMPTY},
/*D*/{EMPTY},
/*E*/{EOFILE,EMPTY},
/*F*/{EMPTY},
/*G*/{EMPTY},
/*H*/{EMPTY},
/*I*/{EMPTY},
/*J*/{EMPTY},
/*K*/{EMPTY},
/*L*/{EMPTY},
/*M*/{EMPTY},
/*N*/{EMPTY},
/*O*/{EMPTY},
/*P*/{EMPTY},
/*Q*/{EMPTY},
/*R*/{EMPTY},
/*S*/{EMPTY},
/*T*/{EMPTY},
/*U*/{EMPTY},
/*V*/{EMPTY},
/*W*/{EMPTY},
/*X*/{EMPTY},
/*Y*/{EMPTY},
/*Z*/{EMPTY}
};

Control_CMD_t ParsCMD(uint8_t* data_buff);

#endif
/*USB_PARSER_H*/

