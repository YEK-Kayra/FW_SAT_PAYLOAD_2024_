#include "SubSys_USART_ReceiveIT_CallBacks_Driver.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	/*! which UART interface receive data?
	 * UART2 is for Wireless communication
	 */
	if(huart->Instance == USART2){
		SubSys_WirelessCom_Telemetry_Receive_From_To(Sat_Carrier, Sat_Payload, &dev_WirelessComApp);
	}
	if(huart->Instance == USART1){
		GPS_UART_CallBack();
	}




}
