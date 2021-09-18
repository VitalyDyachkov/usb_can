/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//CAN_HandleTypeDef hcan1;
//CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

//TIM_HandleTypeDef htim6;


/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

uint32_t error_msg_dbg = 0;
uint32_t error_time_dbg = 0;
SendCAN send_to_can_whith_period;

osThreadId defaultTaskHandle;
/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
RTC_HandleTypeDef hrtc;

Packet_to_CAN send_to_can_stct;	

osThreadId defaultTaskHandle;

bool flag_buss_can1_ok;
bool flag_buss_can2_ok;
bool flag_start_operation = false;
bool flag_stop_operation = false;
bool flag_on_marker = false;
bool flag_can_sender_1 = false;
bool flag_can_sender_2 = false;
bool flag_stop_stream_usb = false;
bool *flag_can_sender;

CanTxMsgTypeDef TxMsg;
CanTxMsgTypeDef TxMsgCan2;
CanRxMsgTypeDef RxMsg;
CanRxMsgTypeDef RxMsgCan2;

CurSetCANTypeDef CAN1_setings;
CurSetCANTypeDef CAN2_setings;

HAL_StatusTypeDef state_bus_can;
uint8_t buff[8] = {0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32};
uint8_t answer_sc_ok[10] = {"AT+SC_OK\r\n"};
uint8_t answer_sc_er[10] = {"AT+SC_ER\r\n"};
S_Can_t buff_baudrate[] = 
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
};
Control_CMD_t result_search;
uint8_t data_buff_to_usb[53];
uint8_t data_buff_input_usb[53];
uint8_t state_my_can;
uint32_t state_can_buss_errror;
int nmb_baud_CAN_1 = 0;
int nmb_baud_CAN_2 = 0;
uint8_t nmb_baud = 0;
uint32_t cnt_data_us = 0;
uint32_t cnt_us = 0;
uint32_t cnt_error =0;
extern uint32_t cnt_msecund;
extern uint32_t cnt_secund;
extern uint32_t cnt_minute;
extern RTC_TimeTypeDef Main_time;
extern char time_buff_to_usb[45];
/* USB переменные и функции*/
extern uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);
extern USBD_HandleTypeDef  *hUsbDevice_0;
extern uint8_t Temp_Copy_Data_USB[64];

xQueueHandle queue_can;
xQueueHandle queue_usb;
xQueueHandle queue_in_usb;
xQueueHandle queue_to_can;
xQueueHandle queue_from_timer;
xQueueHandle queue_periods;
char start_marker[53]={"2 STARTMRK FF-FF-FF-FF-FF-FF-FF-FF 0 000.00.000.000  "};
char stop_marker[53]= {"2 STOP_MRK 00-00-00-00-00-00-00-00 0 000.00.000.000  "};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
//static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t sendSemaphore = NULL;
/* USER CODE BEGIN PFP */
static void LoadTIMPeriod(uint16_t period);
static void MX_TIM4_Init(void);
/* Private function prototypes -----------------------------------------------*/
void Filter_CAN_Init(void);
void Capture_Time(char buff[],int idx);
//void MX_TIM1_TIM2_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*разбор пакета принятого в USB*/
void Task_Pars_USB (void *pvParametrs)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = 50;//pdFALSE;
	portBASE_TYPE retVal;
	//Packet_to_CAN send_to_can_stct;	
	
for(;;)
	{
		uint8_t shift = 28;
		retVal = xQueueReceive(queue_in_usb,&data_buff_input_usb, 0);
		if(retVal == pdPASS)
		{
			uint8_t can_speed;
			S_Can_t can_speed_t;
			/*если пришла команда конфигурации CANs*/
			if(data_buff_input_usb[0] == 'A')
			{
				result_search = ParsCMD(data_buff_input_usb);
				switch (result_search)
				{
					/*установка скорости CAN*/
								case CANSPD:
								{
									send_to_can_stct.num_can = (N_Can_t)(0x03 & data_buff_input_usb[10]);
									/*определим скорость для CAN*/
									can_speed = data_buff_input_usb[11];
									switch (can_speed)
									{
										case 'A':
										{
											can_speed_t = Kb_10;
										break;
										}
										case 'B':
										{
											can_speed_t = Kb_20;
										break;
										}
										case 'C':
										{
											can_speed_t = Kb_33_3;
										break;
										}
										case 'D':
										{
											can_speed_t = Kb_50;
										break;
										}
										case 'E':
										{
											can_speed_t = Kb_83_3;
										break;
										}
										case 'F':
										{
											can_speed_t = Kb_100;
										break;
										}
										case 'G':
										{
											can_speed_t = Kb_125;
										break;
										}
										case 'H':
										{
											can_speed_t = Kb_250;
										break;
										}
										case 'I':
										{
											can_speed_t = Kb_333_333;
										break;
										}
										case 'J':
										{
											can_speed_t = Kb_500;
										break;
										}
										case 'K':
										{
											can_speed_t = Kb_666_667;
										break;
										}
										case 'L':
										{
											can_speed_t = Kb_800;
										break;
										}
										case 'M':
										{
											can_speed_t = Mb_1;
										break;
										}
										default:
										{
											can_speed_t = Mb_1;
											break;
										}
									}
											if (send_to_can_stct.num_can == CAN_1 && CAN1_setings.baurate_ifc != can_speed)
											{
												CAN1_setings.baurate_ifc = can_speed;
												MX_CAN_ReInit(send_to_can_stct.num_can,can_speed_t,&hcan1,second_init,&nmb_baud_CAN_1);
											}
											else if(send_to_can_stct.num_can == CAN_2 && CAN2_setings.baurate_ifc != can_speed)
											{
												CAN2_setings.baurate_ifc = can_speed;
												MX_CAN_ReInit(send_to_can_stct.num_can,can_speed_t,&hcan2,second_init,&nmb_baud_CAN_2);
											}	
								/*ответим плодтверждением*/
											
								xQueueSend(queue_usb,answer_sc_ok,xHigherPriorityTaskWoken);
								}
								case EOFILE:
								{
									TimePack.not_empty_flag = false;
									break;
								}
					default :
					{
						break;
					}
				}
			}
			/*пришла строчка для отправки в CAN*/
			else
			{
					/*определим номер CAN*/
					send_to_can_stct.num_can = (N_Can_t)(0x03 & data_buff_input_usb[0]);
					/*определим длину пакета*/
					send_to_can_stct.dlc = 0x0F & data_buff_input_usb[35];
					/*определим скорость для CAN*/
					can_speed = data_buff_input_usb[37];
					
			switch (can_speed)
			{
							case 'A':
							{
								can_speed_t = Kb_10;
							break;
							}
							case 'B':
							{
								can_speed_t = Kb_20;
							break;
							}
							case 'C':
							{
								can_speed_t = Kb_33_3;
							break;
							}
							case 'D':
							{
								can_speed_t = Kb_50;
							break;
							}
							case 'E':
							{
								can_speed_t = Kb_83_3;
							break;
							}
							case 'F':
							{
								can_speed_t = Kb_100;
							break;
							}
							case 'G':
							{
								can_speed_t = Kb_125;
							break;
							}
							case 'H':
							{
								can_speed_t = Kb_250;
							break;
							}
							case 'I':
							{
								can_speed_t = Kb_333_333;
							break;
							}
							case 'J':
							{
								can_speed_t = Kb_500;
							break;
							}
							case 'K':
							{
								can_speed_t = Kb_666_667;
							break;
							}
							case 'L':
							{
								can_speed_t = Kb_800;
							break;
							}
							case 'M':
							{
								can_speed_t = Mb_1;
							break;
							}
							default:
							{
								can_speed_t = Mb_1;
							break;
							}
			}
					/*если один из CAN ранее не настроен то 
					произведем считывание параметров настройки из пакета данных*/
			if (send_to_can_stct.num_can == CAN_1 && CAN1_setings.baurate_ifc != can_speed)
			{
				CAN1_setings.baurate_ifc = can_speed;
				MX_CAN_ReInit(send_to_can_stct.num_can,can_speed_t,&hcan1,second_init,&nmb_baud_CAN_1);
			}
			else if(send_to_can_stct.num_can == CAN_2 && CAN2_setings.baurate_ifc != can_speed)
			{
				CAN2_setings.baurate_ifc = can_speed;
				MX_CAN_ReInit(send_to_can_stct.num_can,can_speed_t,&hcan2,second_init,&nmb_baud_CAN_2);
			}		
			
			uint8_t temp_data_to_can;
			int shift_data = 4;
			int idx = 0;
			uint8_t symbol_id;
			
			send_to_can_stct.id = 0;
			
			for(int i = 2;i<10;i++)
			{
				symbol_id = data_buff_input_usb[i];
				switch (symbol_id)
				{
					case 'A':
					{
						symbol_id = 0xA;
					break;
					}
					case 'B':
					{
						symbol_id = 0xB;
					break;
					}
					case 'C':
					{
						symbol_id = 0xC;
					break;
					}
					case 'D':
					{
						symbol_id = 0xD;
					break;
					}
					case 'E':
					{
						symbol_id = 0xE;
					break;
					}
					case 'F':
					{
						symbol_id = 0xF;
					break;
					}
					default:
					{
						symbol_id &= 0xF;
					break;
					
					}
				}
				send_to_can_stct.id |= symbol_id << shift;
				shift-=4;
			}
	memset(send_to_can_stct.buff_can_msg,0,8);
	for(int i = 11 ;data_buff_input_usb [i] != ' ';i++)
	{
		switch (data_buff_input_usb [i])
		{
			case 'A':
			{
				temp_data_to_can = 0xA;
				break;
			}
			case 'B':
			{
				temp_data_to_can = 0xB;
				break;
			}
			case 'C':
			{
				temp_data_to_can = 0xC;
				break;
			}
			case 'D':
			{
				temp_data_to_can = 0xD;
				break;
			}
			case 'E':
			{
				temp_data_to_can = 0xE;
				break;
			}
			case 'F':
			{
				temp_data_to_can = 0xF;
				break;
			}
			case '-':
			{
				temp_data_to_can = 0xFF;
				idx++;
				break;
			}
			default :
			{
				temp_data_to_can = 0xF & data_buff_input_usb [i];
				break;
			}
		}
		if(temp_data_to_can != 0xff)
		{
				send_to_can_stct.buff_can_msg[idx] |= temp_data_to_can << shift_data;
				if(shift_data == 4)
				{
					shift_data = 0;
				}
				else
				{
					shift_data = 4;
				}
		}
	}
	/*если пакет одиночный*/
				xQueueSend(queue_from_timer,&send_to_can_stct,portMAX_DELAY);

	/*занесем данные в очередь*/

//	if(xQueueSend(queue_to_can,&send_to_can_stct,xHigherPriorityTaskWoken) == pdFALSE)
//	{
//		cnt_error++;
//	}
	/*если поток USB приостановлен произведем его запуск*/
	if(flag_stop_stream_usb)
	{
		/*внесем временный буфер с данными в очередь*/
		if (xQueueSend(queue_in_usb,Temp_Copy_Data_USB,xHigherPriorityTaskWoken)!= pdFALSE)//Temp_Copy_Data_USB
		{
			/*сбросим флаг остановки потока данных из USB*/
			flag_stop_stream_usb = false;
			/*запустим прием данных по USB*/
			USBD_CDC_ReceivePacket(hUsbDevice_0);
		}
	}
}
		}
		osDelay(1);
	}

}
/*вставка метки оператора*/
void Operator_marker (void *pvParametrs)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	for(;;)
	{
	if(xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE)
	{
		if(flag_start_operation)
		{
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
		}
		else if(flag_stop_operation)
		{
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
			if(flag_on_marker)
			{
				Capture_Time(stop_marker,38);
				xQueueSendFromISR(queue_usb,stop_marker,&xHigherPriorityTaskWoken);
				flag_on_marker = false;
			}
			else
			{
				Capture_Time(start_marker,38);
				xQueueSendFromISR(queue_usb,start_marker,&xHigherPriorityTaskWoken);
				flag_on_marker = true;
			}
		}
			vTaskDelay(300);
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
    osDelay(1);
		
	}
}
/*отправка пакетов в USB*/
void Task_Send_USB (void *pvParametrs)
{
	portBASE_TYPE retVal;
	uint8_t data_to_usb[53];
	//MX_RTC_Init();
	for(;;)
	{
	  retVal = xQueueReceive(queue_usb,&data_to_usb, 50);
		if(retVal == pdPASS )
		{
			CDC_Transmit_FS(data_to_usb,sizeof(data_to_usb));
			memset(data_to_usb,0,53);
		}
		else
		{

		}

    osDelay(1);
		
	}
}
/*отправка пакетов в CAN*/
void Task_Send_CAN (void *pvParametrs)
{
	portBASE_TYPE retVal;
	Packet_to_CAN msg_to_can;
	CAN_HandleTypeDef * hcan_x;
	portBASE_TYPE xHigherPriorityTaskWoken;
	//CanTxMsgTypeDef TxMsg_to_CAN;
	//hcan1.pTxMsg = &TxMsg_to_CAN;
	for(;;)
	{
		//retVal = xQueueReceive(queue_to_can,&msg_to_can, 0);
				//xQueueReceiveFromISR(&send_to,&queue_from_timer,&xHigherPriorityTaskWoken);
		/*занесем данные в очередь*/
		//xQueueSendFromISR(queue_to_can,&send_to,&xHigherPriorityTaskWoken);
//		xSemaphoreTake( xSemaphore, portMAX_DELAY );
		
		retVal = xQueueReceive(queue_from_timer,&msg_to_can,xHigherPriorityTaskWoken);
		if(retVal == pdPASS )
		{
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
				TxMsg.DLC = msg_to_can.dlc;
				//TxMsg.DLC = send_to_can_stct.dlc;
				TxMsg.ExtId = msg_to_can.id;
				TxMsg.StdId = msg_to_can.id;
				TxMsg.IDE = CAN_ID_STD;
				//TxMsg.StdId = send_to_can_stct.id;
				//TxMsg.IDE = CAN_ID_STD;
				for(int i = 0;i<8;i++)
				{
					TxMsg.Data[i] = msg_to_can.buff_can_msg[i];
					//TxMsg.Data[i] = send_to_can_stct.buff_can_msg[i];
				}
				if(msg_to_can.num_can == CAN_1)
				//if(send_to_can_stct.num_can == CAN_1)
				{
					hcan_x = &hcan1;
				}
				else
				{
					hcan_x = &hcan2;
				}
				hcan_x->pTxMsg = &TxMsg;
				HAL_Delay(1);
				HAL_CAN_Transmit(hcan_x,1000);
		 }
		osDelay(1);
	}
	
}
/*разбор пакетов принятых по CAN*/
void Task_Pars_CAN (void *pvParametrs)
{
// /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	Packet data_to_usart;
	portBASE_TYPE retVal;
	uint8_t temp_data_h;
	uint8_t temp_data_l;
	uint8_t tempBuff_id     [8] ={0};
	uint8_t temp_buff_to_usb[52]={0};
  /* Infinite loop */
	HAL_Delay(1000);
  for(;;)
  {
	  retVal = xQueueReceive(queue_can, &data_to_usart, 0);
		if(retVal == pdPASS )
		{
			int idx = 0;
			temp_buff_to_usb [0] = data_to_usart.source_msg;
			temp_buff_to_usb [1] = 0x20;
			for(int i = 7 ; i >= 0; i-- )
			{
				/*переведем  ID принятого сообщения  в ASCII код*/
				tempBuff_id[i] = 0x0f&data_to_usart.id;
				if(tempBuff_id[i]>=0x0A)
				{
						switch(tempBuff_id[i])
						{
							case 0x0A:
							{
								tempBuff_id[i] = 'A';
								break;
							}
							case 0x0B:
							{
								tempBuff_id[i] = 'B';
								break;
							}
							case 0x0C:
							{
								tempBuff_id[i] = 'C';
								break;
							}
							case 0x0D:
							{
								tempBuff_id[i] = 'D';
								break;
							}
							case 0x0E:
							{
								tempBuff_id[i] = 'E';
								break;
							}
							
							case 0x0F:
							{
								tempBuff_id[i] = 'F';
								break;
							}
							default:
							{
								break;
							}
						}
				}
				else
				{
					tempBuff_id[i] += 0x30;
				}
				data_to_usart.id >>= 4;
			}
			for(int i = 2;i<10;i++)
			{
				temp_buff_to_usb [i] = tempBuff_id[i-2];
			}
			temp_buff_to_usb [10] = 0x20;
			idx = 11;
			uint8_t temp;
			/*переведем поле данных сообщения  в ASCII код*/
				for(int j = 0; j<8 ; j++ )
				{
					temp_data_h = 0xf0 & data_to_usart.buff_can_msg[j];
					temp_data_l = 0x0f & data_to_usart.buff_can_msg[j];
					temp = temp_data_h;
					temp >>= 4;
					for (int k = 0 ;k<2;k++)
					{
						if(temp>=0x0A)
						{
								switch(temp)
									{
										case 0x0A:
											{
												temp = 'A';
												break;
											}
										case 0x0B:
											{
												temp = 'B';
												break;
											}
										case 0x0C:
											{
												temp = 'C';
												break;
											}
										case 0x0D:
											{
												temp = 'D';
												break;
											}
										case 0x0E:
											{
												temp = 'E';
												break;
											}
										case 0x0F:
											{
												temp = 'F';
												break;
											}
									}
						}
						else
						{
							temp += 0x30;
						}
						temp_buff_to_usb [idx++] = temp;
						temp = temp_data_l;
			}
			/*если не все данные переведены в ASCII то вставим символ "дефис"*/
			if(idx != 34)
			{
				temp_buff_to_usb [idx++] = 0x2D;
			}
			else
			{
				for(int i = 8;i < 27;i++)
				{
					temp_buff_to_usb [idx++] = data_to_usart.buff_can_msg[i];
				}

			}
			}
			xQueueSend(queue_usb,temp_buff_to_usb,xHigherPriorityTaskWoken);
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
		}
		else
		{
				
		}

    osDelay(1);

  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
	//MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(5);

	Filter_CAN_Init();
	

	hcan1.pTxMsg = &TxMsg;
	
	hcan2.pTxMsg = &TxMsgCan2;
	
	hcan1.pRxMsg = &RxMsg;
	
	hcan2.pRxMsg = &RxMsgCan2;

	//здесь произвести автоонастройку CAN1 и CAN2
	state_my_can = Setting_CAN();
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	//xBinarySemaphore = xSemaphoreCreateBinary();
	//vSemaphoreCreateBinary( xSemaphore );
	vSemaphoreCreateBinary(xSemaphore);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	xTaskCreate( Task_Send_USB, (const char *) "Task_Send_USB", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	//xTaskCreate( Operator_marker, (const char *) "Operator_marker", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
	xTaskCreate( Task_Pars_CAN, (const char *) "Task_Pars_CAN", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( Task_Pars_USB, (const char *) "Task_Pars_USB", configMINIMAL_STACK_SIZE, NULL, 0, NULL );
	xTaskCreate( Task_Send_CAN, (const char *) "Task_Send_CAN", configMINIMAL_STACK_SIZE, NULL, 0, NULL );
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
		queue_can = xQueueCreate(16,sizeof(Packet));
		queue_usb = xQueueCreate(8,sizeof(data_buff_to_usb));
		queue_in_usb = xQueueCreate(8,sizeof(data_buff_input_usb));
		queue_to_can = xQueueCreate(8,sizeof(Packet_to_CAN));
		queue_from_timer = xQueueCreate(16,sizeof(Packet_to_CAN));
		queue_periods = xQueueCreate(1,sizeof(uint32_t));
  /* USER CODE END RTOS_QUEUES */
				if(state_my_can != 0)
				{
					HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
					HAL_CAN_Receive_IT(&hcan1,CAN_FIFO1);
					HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
				}
				//HAL_RTCEx_SetSecond_IT(&hrtc);
					MX_TIM3_Init();
				MX_TIM4_Init();
  //MX_RTC_Init();
  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;//8- 500 kBs//4 - 1MBs
  hcan1.Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_SILENT;//CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_2TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}

/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;//125 kBs
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_5TQ;
  hcan2.Init.BS2 = CAN_BS2_6TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan2);

}
/* I2C1 init function */
//void MX_I2C1_Init(void)
//{

//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 100000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//  HAL_I2C_Init(&hi2c1);

//}
/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  hrtc.DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  hrtc.DateToUpdate.Month = RTC_MONTH_JANUARY;
  hrtc.DateToUpdate.Date = 1;
  hrtc.DateToUpdate.Year = 0;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
	//HAL_TIM_Base_Start(&htim6);
  //HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);
	uint32_t temp_reg = 0;
	temp_reg = hrtc.Instance->CRL;
	while((temp_reg&RTC_FLAG_RSF) == 0);
	
	
	
	HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);
	HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BCD);
	
  
	
	//HAL_RTCEx_SetSecond_IT(&hrtc);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef OC_init;
	
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  HAL_TIM_Base_Init(&htim3);
	HAL_TIM_OC_Init(&htim3);
	
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
	//HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	
	
	//TIM_OCMODE_TIMING
	OC_init.OCMode = TIM_OCMODE_TIMING;
	OC_init.Pulse = 999;
	
	
	HAL_TIM_OC_ConfigChannel(&htim3,&OC_init,TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);

}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */
/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef OC_init;
	
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
 // HAL_TIM_Base_Init(&htim4);
	HAL_TIM_OC_Init(&htim4);
	
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
	//HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);
		
	//TIM_OCMODE_TIMING
	OC_init.OCMode = TIM_OCMODE_TIMING;
	OC_init.Pulse = 999;
	
	
	HAL_TIM_OC_ConfigChannel(&htim4,&OC_init,TIM_CHANNEL_1);
	//HAL_TIM_Base_Start(&htim4);
	
}
void LoadTIMPeriod(uint16_t period)
{
	TIM4->CCMR1 = period;
}
void Send_Msg(CAN_HandleTypeDef *hcan_x,bool *flag_bus_ok_can)
{
	state_bus_can = HAL_CAN_Receive(hcan_x,CAN_FIFO0,20);
	/*проверяем состояние шины CAN после отправки сообщения*/
	if (state_bus_can != HAL_TIMEOUT && state_bus_can != HAL_ERROR )
	{
		/*CAN настроен*/
		*flag_bus_ok_can = true;
	}
}
uint8_t Setting_CAN(void)
{
	uint8_t result = 0;
	uint8_t cnt_time_tuning = 0;
	 
	if (!flag_buss_can1_ok)
	{
		    MX_CAN_ReInit(CAN_1,buff_baudrate[nmb_baud++],&hcan1,first_init,&nmb_baud_CAN_1);		    
				do
				{
					state_can_buss_errror++;
					Send_Msg(&hcan1,&flag_buss_can1_ok);
					
					if (state_can_buss_errror == 15)
					{
						if(nmb_baud == 13)
						{
							nmb_baud = 0;
							cnt_time_tuning++;
						}	
						MX_CAN_ReInit(CAN_1,buff_baudrate[nmb_baud++],&hcan1,first_init,&nmb_baud_CAN_1);
						state_can_buss_errror = 0;
					}
				}
				//while(HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_ERROR || HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_TIMEOUT);
				while(!flag_buss_can1_ok && cnt_time_tuning != 3);//&& nmb_baud != 6);
				state_can_buss_errror = 0;
				cnt_time_tuning = 0;
				/*если шина настроена переведем ее в тихий режим*/
				if (flag_buss_can1_ok)
				{
					//MX_CAN_ReInit(CAN_1,buff_baudrate[--nmb_baud],&hcan1,second_init);
					//nmb_baud_CAN_1 = nmb_baud - 1;
					/*неободимо записать определенную скорость и то что настройка произведна,в память*/
					result = 0x01;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
				}
				else
				{
				/*произвести программный сброс*/
					//HAL_NVIC_SystemReset();
					MX_CAN_ReInit(CAN_1,buff_baudrate[0],&hcan1,first_init,&nmb_baud_CAN_1);
					nmb_baud_CAN_1 = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
					result = 0x04;
				}
	}
	if (!flag_buss_can2_ok)
	{
					nmb_baud = 0;
		      MX_CAN_ReInit(CAN_2,buff_baudrate[nmb_baud++],&hcan2,first_init,&nmb_baud_CAN_2);
					do
				{
					state_can_buss_errror++;
					Send_Msg(&hcan2,&flag_buss_can2_ok);
					if (state_can_buss_errror == 15)
					{
						if(nmb_baud == 13)
						{
							nmb_baud = 0;
							cnt_time_tuning++;
						}
						MX_CAN_ReInit(CAN_2,buff_baudrate[nmb_baud++],&hcan2,first_init,&nmb_baud_CAN_2);
						state_can_buss_errror = 0;
					}

				}
				//while(HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_ERROR || HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_TIMEOUT);
				while(!flag_buss_can2_ok && cnt_time_tuning != 3);// && nmb_baud != 6);
				state_can_buss_errror = 0;
				/*если шина настроена переведем ее в тихий режим*/
				if (flag_buss_can2_ok)
				{
					//MX_CAN_ReInit(CAN_2,buff_baudrate[--nmb_baud],&hcan2,second_init);
					//nmb_baud_CAN_2 = nmb_baud - 1;
					/*неободимо записать определенную скорость и то что настройка произведна,в память*/
					result |= 0x02;
					//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
				}
				else
				{
				/*произвести программный сброс*/
	//				HAL_NVIC_SystemReset();
					MX_CAN_ReInit(CAN_2,buff_baudrate[0],&hcan2,first_init,&nmb_baud_CAN_2);					
					nmb_baud_CAN_2 = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
				}
				
	}
return result;

}
void MX_CAN_ReInit(N_Can_t can_x,S_Can_t speed,CAN_HandleTypeDef *hcan_x,Init_t number_init,int *nmb_bps)
{
		switch (can_x)
		{
				case CAN_1:
				{
					hcan_x->Instance = CAN1;
					
					break;
				}
				case CAN_2:
				{
					hcan_x->Instance = CAN2;
					
					break;
				}
				default:
				{
					
				 break;
				}
		}
		HAL_CAN_DeInit(hcan_x);
		switch(speed)
		{
			case Kb_10:
			{
				hcan_x->Init.Prescaler = 96;//24 МГц частота CANa
				hcan_x->Init.SJW = CAN_SJW_1TQ;// max.CAN_SJW_4TQ
				hcan_x->Init.BS1 = CAN_BS1_16TQ;//  max.CAN_BS1_16TQ
				hcan_x->Init.BS2 = CAN_BS2_8TQ;//		max.CAN_BS2_8TQ;
				*nmb_bps = 0;
				break;
			}
			case Kb_20:
			{
				hcan_x->Init.Prescaler = 60;//24 МГц частота CANa
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_11TQ;
				hcan_x->Init.BS2 = CAN_BS2_8TQ;
				*nmb_bps = 1;
				break;
			}
			case Kb_33_3:
			{
				hcan_x->Init.Prescaler = 48;//24 МГц частота CANa
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_7TQ;
				hcan_x->Init.BS2 = CAN_BS2_7TQ;
				*nmb_bps = 2;
				break;
			}
			case Kb_50:
			{
				hcan_x->Init.Prescaler = 24;//24 МГц частота CANa
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_12TQ;
				hcan_x->Init.BS2 = CAN_BS2_7TQ;
				*nmb_bps = 3;
				break;
			}
				case Kb_83_3:
			{
				hcan_x->Init.Prescaler = 12;//24 МГц частота CANa
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_15TQ;
				hcan_x->Init.BS2 = CAN_BS2_8TQ;
				*nmb_bps = 4;
				break;
			}
			case Kb_100:
			{
				hcan_x->Init.Prescaler = 24;
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_4TQ;
				hcan_x->Init.BS2 = CAN_BS2_5TQ;
				*nmb_bps = 5;
				break;
			}
			case Kb_125:
			{
				hcan_x->Init.Prescaler = 24;
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_4TQ;
				hcan_x->Init.BS2 = CAN_BS2_3TQ;
				*nmb_bps = 6;
				break;
			}
			case Kb_250:
			{
				hcan_x->Init.Prescaler = 24;
			  hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_2TQ;
				hcan_x->Init.BS2 = CAN_BS2_1TQ;
				*nmb_bps = 7;
				break;
			}
				case Kb_333_333:
			{
				hcan_x->Init.Prescaler = 8;
			  hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_4TQ;
				hcan_x->Init.BS2 = CAN_BS2_4TQ;
				*nmb_bps = 8;
				break;
			}		
			case Kb_500:
			{
				hcan_x->Init.Prescaler = 6;
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_5TQ;
				hcan_x->Init.BS2 = CAN_BS2_2TQ;
				*nmb_bps = 9;
				break;
			}
			case Kb_666_667:
			{
				hcan_x->Init.Prescaler = 4;
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_4TQ;
				hcan_x->Init.BS2 = CAN_BS2_4TQ;
				*nmb_bps = 10;
				break;
			}
			case Kb_800:
			{
				hcan_x->Init.Prescaler = 3;
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_4TQ;
				hcan_x->Init.BS2 = CAN_BS2_5TQ;
				*nmb_bps = 11;
				break;
			}			
			case Mb_1:
			{
				hcan_x->Init.Prescaler = 4;
				hcan_x->Init.SJW = CAN_SJW_1TQ;
				hcan_x->Init.BS1 = CAN_BS1_2TQ;
				hcan_x->Init.BS2 = CAN_BS2_3TQ;
				*nmb_bps = 12;
				break;
			}
			default:
			{
				break;
			}
		}
	if(number_init == first_init)
	{
		hcan_x->Init.Mode = CAN_MODE_NORMAL;
		hcan_x->Init.TTCM = DISABLE;
		hcan_x->Init.ABOM = DISABLE;
		hcan_x->Init.AWUM = DISABLE;
		hcan_x->Init.NART = DISABLE;
		hcan_x->Init.RFLM = DISABLE;
		hcan_x->Init.TXFP = DISABLE;
		HAL_CAN_Init(hcan_x);
  }
	else
	{
		hcan_x->Init.Mode = CAN_MODE_NORMAL;
		hcan_x->Init.TTCM = DISABLE;
		hcan_x->Init.ABOM = DISABLE;
		hcan_x->Init.AWUM = DISABLE;
		hcan_x->Init.NART = DISABLE;
		hcan_x->Init.RFLM = DISABLE;
		hcan_x->Init.TXFP = DISABLE;
		HAL_CAN_Init(hcan_x);
		HAL_CAN_Receive_IT(hcan_x,CAN_FIFO0);
	}
}

void Filter_CAN_Init(void)
{
	CAN_FilterConfTypeDef Filter_config;

	Filter_config.BankNumber = 0;
	Filter_config.FilterActivation = ENABLE;
	Filter_config.FilterFIFOAssignment = 0;
	Filter_config.FilterIdHigh = 0x0000;
	Filter_config.FilterIdLow = 0x0000;
	Filter_config.FilterMaskIdHigh = 0x0000;
	Filter_config.FilterMaskIdLow = 0x0000;
	Filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
	Filter_config.FilterNumber = 0 ;
	Filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
	
	
	HAL_CAN_ConfigFilter(&hcan1, &Filter_config);	
   
	Filter_config.BankNumber = 14;
	
	HAL_CAN_ConfigFilter(&hcan2, &Filter_config);
	

}
void Capture_Time(char buff[],int idx)
{
	char temp_minute_buff[2];
	char temp_secund_buff[2];
	char temp_msecund_buff[3];
	char temp_us_buff [3] = {0};
//	uint16_t tmp_cnt = 100;
	int leng_time;
	char temp_bite_1;
	char temp_bite_2;
	
	
//do{
//cnt_us = htim3.Instance->CNT;
//	tmp_cnt--;
//}while(tmp_cnt != 0);
//cnt_us = __HAL_TIM_GetCounter(&htim3);
	cnt_us = TIM3->CNT;
leng_time = sprintf(temp_us_buff,"%d",cnt_us);
if(leng_time == 1)
{
	temp_bite_1 = temp_us_buff[0];
	temp_us_buff[0] = 0x30;
	temp_us_buff[1] = 0x30;
	temp_us_buff[2] = temp_bite_1;
}
else if(leng_time == 2)
{
	temp_bite_1 = temp_us_buff[0];
	temp_bite_2 = temp_us_buff[1];
	temp_us_buff[0] = 0x30;
	temp_us_buff[1] = temp_bite_1;
	temp_us_buff[2] = temp_bite_2;
}
leng_time = sprintf(temp_minute_buff,"%d",cnt_minute);
if(leng_time == 1)
{
	temp_bite_1 = temp_minute_buff[0];
	temp_minute_buff[0] = 0x30;
	temp_minute_buff[1] = temp_bite_1;
}
	buff[idx++] = temp_minute_buff[0];
	buff[idx++] = temp_minute_buff[1];
  buff[idx++] = 0x2E;
	leng_time = sprintf(temp_secund_buff,"%d",cnt_secund);
if(leng_time == 1)
{
	temp_bite_1 = temp_secund_buff[0];
	temp_secund_buff[0] = 0x30;
	temp_secund_buff[1] = temp_bite_1;
}
	buff[idx++] = temp_secund_buff[0];
	buff[idx++] = temp_secund_buff[1];
	buff[idx++] = 0x2E;
	leng_time = sprintf(temp_msecund_buff,"%d",cnt_msecund);
if(leng_time == 1)
{
	temp_bite_1 = temp_msecund_buff[0];
	temp_msecund_buff[0] = 0x30;
	temp_msecund_buff[1] = 0x30;
	temp_msecund_buff[2] = temp_bite_1;
}
else if(leng_time == 2)
{
	temp_bite_1 = temp_msecund_buff[0];
	temp_bite_2 = temp_msecund_buff[1];
	temp_msecund_buff[0] = 0x30;
	temp_msecund_buff[1] = temp_bite_1;
	temp_msecund_buff[2] = temp_bite_2;
}
	buff[idx++] = temp_msecund_buff[0];
	buff[idx++] = temp_msecund_buff[1];
	buff[idx++] = temp_msecund_buff[2];
	buff[idx++] = 0x2E;
	buff[idx++] = temp_us_buff[0];
	buff[idx++] = temp_us_buff[1];
	buff[idx++] = temp_us_buff[2];
	buff[idx++] = '\r';
	buff[idx++] = '\n';
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	Packet data_to_usart;
	portBASE_TYPE retVal;
//	int id_msg;
	uint8_t temp_data_h;
	uint8_t temp_data_l;
	uint8_t tempBuff_id     [8] ={0};
	uint8_t temp_buff_to_usb[52]={0};
  /* Infinite loop */
	//HAL_CAN_Transmit(&hcan2,1000);
	HAL_Delay(1000);
  for(;;)
  {
	  retVal = xQueueReceive(queue_can, &data_to_usart, 0);
		if(retVal == pdPASS )
		{
			int idx = 0;
			temp_buff_to_usb [0] = data_to_usart.source_msg;
			temp_buff_to_usb [1] = 0x20;
			for(int i = 7 ; i>=0;i-- )
			{
				tempBuff_id[i] = 0x0f&data_to_usart.id;
				if(tempBuff_id[i]>=0x0A)
			{
					switch(tempBuff_id[i])
					{
						case 0x0A:
						{
							tempBuff_id[i] = 'A';
							break;
						}
						case 0x0B:
						{
							tempBuff_id[i] = 'B';
							break;
						}
						case 0x0C:
						{
							tempBuff_id[i] = 'C';
							break;
						}
						case 0x0D:
						{
							tempBuff_id[i] = 'D';
							break;
						}
						case 0x0E:
						{
							tempBuff_id[i] = 'E';
							break;
						}
						
						case 0x0F:
						{
							tempBuff_id[i] = 'F';
							break;
						}
					}
				}
				else
				{
					tempBuff_id[i] += 0x30;
				}
				data_to_usart.id >>= 4;
			}
			for(int i = 2;i<10;i++)
			{
				temp_buff_to_usb [i] = tempBuff_id[i-2];
			}
			temp_buff_to_usb [10] = 0x20;
			idx = 11;
			uint8_t temp;
				for(int j = 0;j<8;j++)
				{
					temp_data_h = 0xf0 & data_to_usart.buff_can_msg[j];
					temp_data_l = 0x0f & data_to_usart.buff_can_msg[j];
					temp = temp_data_h;
					temp >>= 4;
					for (int k = 0 ;k<2;k++)
					{
						if(temp>=0x0A)
						{
								switch(temp)
									{
										case 0x0A:
											{
												temp = 'A';
												break;
											}
										case 0x0B:
											{
												temp = 'B';
												break;
											}
										case 0x0C:
											{
												temp = 'C';
												break;
											}
										case 0x0D:
											{
												temp = 'D';
												break;
											}
										case 0x0E:
											{
												temp = 'E';
												break;
											}
										case 0x0F:
											{
												temp = 'F';
												break;
											}
									}
						}
						else
						{
							temp += 0x30;
						}
						temp_buff_to_usb [idx++] = temp;
						temp = temp_data_l;
			}
			if(idx != 34)
			{
				temp_buff_to_usb [idx++] = 0x2D;
			}
			else
			{
				for(int i = 8 ;i<26;i++)
				{
					temp_buff_to_usb [idx++] = data_to_usart.buff_can_msg[i];
				}
			}
			}
			xQueueSend(queue_usb,temp_buff_to_usb,xHigherPriorityTaskWoken);
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
		}
		else
		{
				
		}

    osDelay(1);

  }
  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
