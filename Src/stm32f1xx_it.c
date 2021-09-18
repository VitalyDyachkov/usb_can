/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include <stdlib.h>

 extern xQueueHandle queue_from_timer;
 extern xQueueHandle queue_to_can;
 extern xQueueHandle queue_periods;
extern Packet_to_CAN send_to_can_stct;
extern SendCAN send_to_can_whith_period;
extern CanRxMsgTypeDef RxMsg;
extern CanRxMsgTypeDef RxMsgCan2;
extern  char buff_can_msg[27];
extern  uint8_t buff_can2_msg[27];
extern int nmb_baud_CAN_1;
extern int nmb_baud_CAN_2;
extern uint32_t cnt_data_us;
extern bool flag_start_operation;
extern bool flag_stop_operation;
char buff_secund[2] = {0x30,0x30};
char buff_minute[2]= {0x30,0x30};
char buff_hours[2]= {0x30,0x30};
char time_buff_to_usb[45]={"2 TIMESECD 0000001020000000000000000000000000"};
uint8_t can_spd_to_msg [] = {0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x21};
char symbol_can_speed[] = {'A','B','C','D','E','F','G','H','I','J','K','L','M'};
extern uint32_t id;
extern uint16_t dlc;
Packet can1_pack;
Packet can2_pack;
extern xQueueHandle queue_can;
extern xQueueHandle queue_usb;
extern SemaphoreHandle_t xSemaphore;
extern SemaphoreHandle_t sendSemaphore;
uint32_t cnt_msecund = 0;
uint32_t cnt_secund = 0;
uint32_t cnt_minute = 0;
uint32_t global_timer = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern void xPortSysTickHandler(void);
extern void Capture_Time(char buff[],int idx);
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
RTC_TimeTypeDef Main_time;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RTC global interrupt.
*/
void RTC_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_IRQn 0 */
//portBASE_TYPE xHigherPriorityTaskWoken;
	//запустим таймер миллисекунд
	//HAL_TIM_Base_Start(&htim6);
	//uint32_t secund;
	//secund = RTC_ReadTimeCounter(&hrtc);
	//HAL_RTC_GetTime(&hrtc,&Main_time,RTC_FORMAT_BIN);
  /* USER CODE END RTC_IRQn 0 */
  HAL_RTCEx_RTCIRQHandler(&hrtc);
	
	HAL_RTCEx_DeactivateSecond(&hrtc);
  /* USER CODE BEGIN RTC_IRQn 1 */
//	time_buff_to_usb[11] = 0x30+(Main_time.Hours/10);
//	time_buff_to_usb[12] = 0x30+(Main_time.Hours%10);
//	time_buff_to_usb[13] = 0x30+(Main_time.Minutes/10);
//	time_buff_to_usb[14] = 0x30+(Main_time.Minutes%10);
//	time_buff_to_usb[15] = 0x30+(Main_time.Seconds/10);
//	time_buff_to_usb[16] = 0x30+(Main_time.Seconds%10);
//	time_buff_to_usb[18] = can_spd_to_msg[nmb_baud_CAN_1];
//	time_buff_to_usb[20] = can_spd_to_msg[nmb_baud_CAN_2];
//	time_buff_to_usb[34] = '\r';
//	time_buff_to_usb[35] = '\n';
//	//}
//		xQueueSendFromISR(queue_usb,time_buff_to_usb,&xHigherPriorityTaskWoken);
  /* USER CODE END RTC_IRQn 1 */
}

/**
* @brief This function handles CAN1 RX0 interrupt.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
	/*занесем полученные данные в структуру*/
	can1_pack.source_msg = 0x31;
	can1_pack.id = RxMsg.StdId;
		if(RxMsg.IDE == 0x04)
	{
		can1_pack.id = RxMsg.ExtId;
	}
	else
	{
		can1_pack.id = RxMsg.StdId;
	}
	
	memset(can1_pack.buff_can_msg,0,8);
	
	for(int i = 0;i<RxMsg.DLC; i++)	
	{
		can1_pack.buff_can_msg[i] = RxMsg.Data[i];
	}
	Capture_Time(can1_pack.buff_can_msg,12);
	can1_pack.buff_can_msg[8] = 0x20;
	can1_pack.buff_can_msg[9] = 0x30 + RxMsg.DLC;
	can1_pack.buff_can_msg[10] = 0x20;
	can1_pack.buff_can_msg[11] = symbol_can_speed[nmb_baud_CAN_1];//can_spd_to_msg[nmb_baud_CAN_1];
	/*отправим структуру в очередь*/
	xQueueSendFromISR(queue_can,&can1_pack,&xHigherPriorityTaskWoken);
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles CAN2 RX0 interrupt.
*/
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */
	can2_pack.source_msg = 0x32;
	
	if(RxMsgCan2.IDE == 0x04)
	{
		can2_pack.id = RxMsgCan2.ExtId;
	}
	else
	{
		can2_pack.id = RxMsgCan2.StdId;
	}
	
	memset(can2_pack.buff_can_msg,0,8);
	
	for(int i = 0;i<RxMsgCan2.DLC; i++)	
	{
		can2_pack.buff_can_msg[i] = RxMsgCan2.Data[i];
	}
	Capture_Time(can2_pack.buff_can_msg,12);
	can2_pack.buff_can_msg[8] = 0x20;
	can2_pack.buff_can_msg[9] = 0x30 + RxMsgCan2.DLC;
	can2_pack.buff_can_msg[10] = 0x20;
	can2_pack.buff_can_msg[11] = symbol_can_speed[nmb_baud_CAN_2];//can_spd_to_msg[nmb_baud_CAN_2];
	/*отправим структуру в очередь*/
	xQueueSendFromISR(queue_can,&can2_pack,&xHigherPriorityTaskWoken);
	HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
* @brief This function handles USB OTG FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void EXTI3_IRQHandler(void)
{
	
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	if(flag_start_operation)
	{
		flag_stop_operation = true;
		flag_start_operation = false;
	}
	else
	{
		flag_start_operation = true;
		flag_stop_operation = false;
	}
	
	//cnt_data_ms = htim3.Instance->CNT;
	
	
	xSemaphoreGiveFromISR(xSemaphore,&xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken == pdTRUE)
	{
		 portYIELD_FROM_ISR(pdTRUE);
	}
}
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	cnt_msecund++;
	if(cnt_msecund == 1000)
	{
		cnt_msecund = 0;
		
		cnt_secund++;
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	
	//	uint8_t data_i2c = 0x45;
		
		//HAL_I2C_Master_Transmit(&hi2c1,0,&data_i2c,1,0);
		
		if(cnt_secund == 60)
		{
			cnt_secund = 0;
			cnt_minute++;
			if(cnt_minute == 60)
			{
				cnt_minute = 0;
			}
		}
	}
}
void TIM4_IRQHandler(void)
{
	BaseType_t xTaskWokenByReceive = pdFALSE;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	HAL_TIM_IRQHandler(&htim4);
	
	//uint32_t timer = 0 ;
	
	if(global_timer == 0)
	{
		xQueueReceiveFromISR(queue_periods,(void *)&global_timer,&xTaskWokenByReceive);
		
	}
	else
	{
		if(--global_timer == 0) xSemaphoreGiveFromISR(xSemaphore,&xHigherPriorityTaskWoken);
	}
	if(send_to_can_whith_period.period != 0)
	{
		send_to_can_whith_period.period--;
	}		
	if (send_to_can_whith_period.number != 0 && send_to_can_whith_period.period == 0)
	{
		send_to_can_whith_period.number--;
		send_to_can_whith_period.period = send_to_can_whith_period.reload_period;
		xSemaphoreGiveFromISR(xSemaphore,&xHigherPriorityTaskWoken);
	}
	if(xTaskWokenByReceive != pdFALSE)
	{
		taskYIELD();
	}
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef state_recv;
	uint8_t data_recv;
  state_recv =  HAL_I2C_Master_Receive_IT(&hi2c1,0,&data_recv,1);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
