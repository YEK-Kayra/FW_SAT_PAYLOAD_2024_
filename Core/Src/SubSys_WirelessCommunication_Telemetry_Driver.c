
/******************************************************************************
         				#### WIRELESSCOM INCLUDES ####
******************************************************************************/
#include "SubSys_WirelessCommunication_Telemetry_Driver.h"
#include "stdio.h"
/******************************************************************************
         				#### WIRELESSCOM VARIABLES ####
******************************************************************************/
uint16_t Written_Bytes; /* is for save number of total converted buffer's characters*/

/******************************************************************************
         				#### WIRELESSCOM  FUNCTIONS ####
******************************************************************************/

/**
  * @brief Decimal, float and other formats are converted as character and save them into the TX buffer.
  * 		When TX buffer is fulfilled , it is sent by UART interface.
  * @note  Follow the transmitting rules, each if and else if has a why head for using
  * @param MissionUnit From_X, Packet type used for your specific purpose. Where do you  want to take it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param MissionUnit To_Y, Packet type used for your specific purpose. Where do you want to send it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
  * @retval NONE
  */
void SubSys_WirelessCom_Telemetry_Transfer_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp){

	/*! Use it when working on Sat_Carrier flight software*/
	if(From_X == Sat_Payload && To_Y == GroundStation){

		/*! Create message packet for Carrier for sending to the Payload*/
		SubSys_WirelessCom_Telemetry_Create_Packet_For(Sat_Payload, dev_WirelessComApp);

				/* 8 pairs of '<>' and y Byte data are x Byte as total budget*/
				Written_Bytes = sprintf(dev_WirelessComApp->Buffer.Temp,
									   "<%d><%d><%d>,<%d/%d/%d:%d/%d/%d>,<%.2f><%.2f><%.2f><%.2f><%.2f><%.2f><%.2f><%.2f><%.6f><%.6f><%.2f><%.2f><%.2f><%.2f><%c,%c,%c,%c><%.2f><%d>\n",
																		 dev_WirelessComApp->Variable.PAY_NumOfPacket,
																		 dev_WirelessComApp->Variable.PAY_SatelliteStatus,
																		 dev_WirelessComApp->Variable.PAY_SatelliteErrorCode,
																		 dev_WirelessComApp->Variable.PAY_date,
																		 dev_WirelessComApp->Variable.PAY_month,
																		 dev_WirelessComApp->Variable.PAY_year,
																		 dev_WirelessComApp->Variable.PAY_hour,
																		 dev_WirelessComApp->Variable.PAY_minute,
																		 dev_WirelessComApp->Variable.PAY_second,
																		 dev_WirelessComApp->Variable.PAY_Pressure,
																		 dev_WirelessComApp->Variable.PAY_Pressure2,
																		 dev_WirelessComApp->Variable.PAY_VertHeight,
																		 dev_WirelessComApp->Variable.PAY_VertHeight2,
																		 dev_WirelessComApp->Variable.PAY_PAY2CAR_DiffHeight,
																		 dev_WirelessComApp->Variable.PAY_VertSpeed,
																		 dev_WirelessComApp->Variable.PAY_Temperature,
																		 dev_WirelessComApp->Variable.PAY_BatteryVoltage,
																		 dev_WirelessComApp->Variable.PAY__GPS_Latitude,
																		 dev_WirelessComApp->Variable.PAY__GPS_Longitude,
																		 dev_WirelessComApp->Variable.PAY__GPS_Altitude,
																		 dev_WirelessComApp->Variable.PAY_Pitch,
																		 dev_WirelessComApp->Variable.PAY_Roll,
																		 dev_WirelessComApp->Variable.PAY_Yaw,
																		 dev_WirelessComApp->Variable.PAY_dataRHRH[0],
																		 dev_WirelessComApp->Variable.PAY_dataRHRH[1],
																		 dev_WirelessComApp->Variable.PAY_dataRHRH[2],
																		 dev_WirelessComApp->Variable.PAY_dataRHRH[3],
																		 dev_WirelessComApp->Variable.PAY_IOT_Temperature,
																		 dev_WirelessComApp->Variable.PAY_TeamNumber);



				for(int i = 0 ; i < Written_Bytes ; i++){

					dev_WirelessComApp->Buffer.Tx[i+3] = dev_WirelessComApp->Buffer.Temp[i];

				}




				HAL_UART_Transmit(dev_WirelessComApp->huartX, dev_WirelessComApp->Buffer.Tx , (Written_Bytes+3), 1000);
	}

}


/**
 * @brief  : Creates 3 types packet for Carrier, Payload and Ground Station
 * @note   : Where do you want to send packet, select that, For example you are a courier and
 * 			 carry the packet to the selected home(MissionUnite x)
 *
 * @param  : MissionUnit x, Packet type used for your specific purpose @arg 0 : Sat_Carrier
 * 																	   @arg 1 : Sat_Payload
 * 																	   @arg 2 : Ground_Sation
 * @param  : SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
 * @retval NONE
 */
void SubSys_WirelessCom_Telemetry_Create_Packet_For(MissionUnit x,SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp){

	switch(x){
		case Sat_Payload :  /*! This case create a packet for transmission from payload to the ground station */

			/*-------------TARGET DEVICE ADDRESS AND CHANNEL INFO----------------*/
			/*! Target device will be Satellite's Payload*/
			dev_WirelessComApp->Buffer.Tx[0] = dev_WirelessComApp->Target_ADDH;
			dev_WirelessComApp->Buffer.Tx[1] = dev_WirelessComApp->Target_ADDL;
			dev_WirelessComApp->Buffer.Tx[2] = dev_WirelessComApp->Target_Ch;

			/*-------------YOUR DEVICE VARIABLE THAT WILL BE SEND----------------*/ /*Note : Will be system variable opposite to variables*/
			/*From MS5611 & WirelessCommunication device*/
			dev_WirelessComApp->Variable.PAY_Pressure    = MS5611_Press;
			dev_WirelessComApp->Variable.PAY_Pressure2	 = CarrierPressure;
			dev_WirelessComApp->Variable.PAY_Temperature = MS5611_Temp;
			dev_WirelessComApp->Variable.PAY_VertHeight  = MS5611_Altitude;
			dev_WirelessComApp->Variable.PAY_VertHeight2 = CarrierVertHeight;
			dev_WirelessComApp->Variable.PAY_VertSpeed   = MS5611_VertSpeed;
			dev_WirelessComApp->Variable.PAY_PAY2CAR_DiffHeight = PAY2CAR_DiffHeight;

			/*From ADC*/
			dev_WirelessComApp->Variable.PAY_BatteryVoltage = BatteryVoltage;

			/*From L-86GPS*/
			dev_WirelessComApp->Variable.PAY__GPS_Latitude  = GPS_Latitude;
			dev_WirelessComApp->Variable.PAY__GPS_Longitude = GPS_Longitude;
			dev_WirelessComApp->Variable.PAY__GPS_Altitude  = GPS_Altitude;

			/*! Each time a packet is generated, the count will increase by 1*/
			dev_WirelessComApp->Variable.PAY_NumOfPacket    = NumberOfTelePacket;

			/*From FlightStatus driver*/
			dev_WirelessComApp->Variable.PAY_SatelliteStatus = SatelliteStatus;

			/*From ARAS driver*/
			dev_WirelessComApp->Variable.PAY_SatelliteErrorCode = SatelliteErrorCode;

			/*From RTC module*/
			dev_WirelessComApp->Variable.PAY_date	= date;
			dev_WirelessComApp->Variable.PAY_month	= month;
			dev_WirelessComApp->Variable.PAY_year	= year;
			dev_WirelessComApp->Variable.PAY_hour	= hour;
			dev_WirelessComApp->Variable.PAY_minute	= minute;
			dev_WirelessComApp->Variable.PAY_second = second;

			/*From IMU*/
			dev_WirelessComApp->Variable.PAY_Pitch	= euler_pitch;
			dev_WirelessComApp->Variable.PAY_Roll	= euler_roll;
			dev_WirelessComApp->Variable.PAY_Yaw	= euler_yaw;

			/*From GroundStation PC telecommand*/
			dev_WirelessComApp->Variable.PAY_dataRHRH[0] = command_RHRH[0];
			dev_WirelessComApp->Variable.PAY_dataRHRH[1] = command_RHRH[1];
			dev_WirelessComApp->Variable.PAY_dataRHRH[2] = command_RHRH[2];
			dev_WirelessComApp->Variable.PAY_dataRHRH[3] = command_RHRH[3];

			/*From Ground Station*/
			dev_WirelessComApp->Variable.PAY_IOT_Temperature = GroundStation_IOTTemparature;


			/*From main.c code block*/
			dev_WirelessComApp->Variable.PAY_TeamNumber = Race_TeamNo;

			break;

		   }

}

