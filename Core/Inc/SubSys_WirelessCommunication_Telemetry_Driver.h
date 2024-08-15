
#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_TELEMETRY_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_TELEMETRY_H


/******************************************************************************
         				#### WIRELESSCOM INCLUDES ####
******************************************************************************/
#include "main.h"

/******************************************************************************
         				#### WIRELESSCOM DEFINITIONS ####
******************************************************************************/

/*! 200 is the number of packets the LoRa module can send in a single transmission */
#define SizeOf_Wireless_TX_Buff_PAYLOAD 	200
#define SizeOf_Wireless_RX_Buff_CARRIER 	50
/******************************************************************************
         				#### WIRELESSCOM ENUMS ####
******************************************************************************/
typedef enum{

	Sat_Carrier = 1,
	Sat_Payload,
	GroundStation

}MissionUnit;


/******************************************************************************
         				#### WIRELESSCOM EXTERNS ####
******************************************************************************/
extern UART_HandleTypeDef huart2;

extern uint32_t NumberOfTelePacket;	 /*! The value is incremented by +1 at the end of each satellite operation period */

/**
 * 0: Ready for Launch (Before Rocket Ignition)
 * 1: Ascent
 * 2: Model Satellite Descent
 * 3: Separation
 * 4: Payload Descent
 * 5: Recovery (Payload Ground Contact)
 */
extern uint8_t SatelliteStatus;

/**
 * Releated byte will be ==> xxxx xxxx
 * index:
 * 	 	 0-> Satt. velocity is not range of 12-14 m/s.
 * 		 1-> Payload velocity is not range of 6-8 m/s.
 * 		 2-> Unable to get carrier pressure
 * 		 3-> Unable to get payload location
 * 		 4-> Autonomous separation did not occur
 */
extern uint8_t SatelliteErrorCode;

/**!
 * SubSys_Sensor_RTC_Driver variables
 */
extern uint8_t date ;
extern uint8_t month;
extern uint16_t year ;
extern uint8_t hour ;
extern uint8_t minute;
extern uint8_t second ;


extern float MS5611_Press;            /*! Pressure data variable 			*/
extern float MS5611_Temp;             /*! Temperature data variable 		*/
extern float MS5611_Altitude;         /*! Vertical Altitude data variable 	*/
extern float MS5611_VertSpeed;        /*! Vertical Speed data variable 		*/


extern float BatteryVoltage;		  /*! Voltage of the Satellite's battery  	  */

extern float GPS_Altitude;			  /*! Vertical distance info of satellite beetween  */
extern float GPS_Longitude;			  /*! Location info of satellite on the earth 	 	*/
extern float GPS_Latitude;			  /*! Location info of satellite on the earth 	 	*/

/**!
 * SubSys_Sensor_IMU_APP_Driver variables
 */
extern float euler_roll;
extern float euler_pitch;
extern float euler_yaw;

extern float PAY2CAR_DiffHeight;
extern float CarrierPressure;
extern float CarrierVertHeight;

extern float GroundStation_IOTTemparature;

extern char command_RHRH[4];			/*! Rakam Harf Rakam Harf receiving datas from ground station pc */
extern const uint32_t Race_TeamNo; 		/*! It is a fixed number provided by the competition organization.*/


/******************************************************************************
         				#### WIRELESSCOM STRUCTS ####
******************************************************************************/

typedef struct{

	/*####################### SATELLITE PAYLOAD UNIT VARIABLES #######################*/

	  	 /*-----------------------THERE ARE 59 BYTE FOR CARRIER PACKET--------------------------*/
			/*! Will be filled by MS5611 sensor*/
			float PAY_Pressure;			/* Unit : pascal 		(12Byte)e.g  => "101325.5938"   				*/
			float PAY_VertSpeed;		/* Unit : meter/second	(5Byte) e.g  => "1.0, 10.2, 158.7 "		  		*/
			float PAY_VertHeight;		/* Unit : meter 		(6Byte) e.g  => "223.4, 1023.7"		  			*/
			float PAY_Temperature;		/* Unit : C°,celcius 	(4Byte) e.g  => "32.7"		  					*/
			float PAY_Pressure2;		/* Unit : pascal 		(12Byte)e.g  => "101325.5938"   				*/
			float PAY_VertHeight2;		/* Unit : meter 		(6Byte) e.g  => "223.4, 1023.7"		  			*/
			/*! Will be filled by used ADC interface*/
			float PAY_BatteryVoltage;	/* Unit : Voltage		(4Byte) e.g => "8.25"							*/

			/*! Will be filled by L-86 GPS*/
			float PAY__GPS_Latitude;	/* Unit : degrees and minute(°)(')	(10Byte) e.g => "3150.7822N"  ,	NMEA data it means 31° 50.7822' minute 	'N' will be send separately		 */
			float PAY__GPS_Longitude;	/* Unit : degrees and minute(°)(') 	(10Byte) e.g => "11711.9278E" , NMEA data it means 117° 11.9278 minute	'E' will be send separately		 */
			float PAY__GPS_Altitude;	/* Unit : meter 					(6Byte) e.g =>  "1214.4"	  , NMEA data it means 1214.4 meter from sea. GPGGA is provides 			 */

			/*! Will be filled by IMU*/
			float PAY_Roll;			/*! IMU Roll Angle  */
			float PAY_Pitch;		/*! IMU Pitch Angle */
			float PAY_Yaw;			/*! IMU Yaw Angle   */

			/*! Will be filled by RTC sensor*/
			uint8_t  PAY_date ;
			uint8_t  PAY_month;
			uint16_t PAY_year ;
			uint8_t  PAY_hour ;
			uint8_t  PAY_minute;
			uint8_t  PAY_second ;

			uint32_t PAY_NumOfPacket;					/* Unit : unsigned int (4Byte) e.g => "337	  */

			uint8_t PAY_SatelliteErrorCode;
			uint8_t PAY_SatelliteStatus;

			float PAY_PAY2CAR_DiffHeight;

			float PAY_IOT_Temperature;
			uint32_t PAY_TeamNumber;

			char PAY_dataRHRH[4];						/* Unit : char(1Byte) e.g => '3','G','7','B' */

}SubSys_WirelessCom_VariableTypeDef;

typedef struct{
	uint8_t Tx[SizeOf_Wireless_TX_Buff_PAYLOAD];	 /*! Buffer for Datas that send to Lora 	 */
	char 	Temp[SizeOf_Wireless_TX_Buff_PAYLOAD];	 /*! Buffer for Datas that send to Lora		 */
	char 	Rx[SizeOf_Wireless_RX_Buff_CARRIER];	 /*! Buffer for Datas that receive from Lora */
}SubSys_WirelessCom_BufferTypeDef;

typedef struct{

	/*! Inner structs */
	SubSys_WirelessCom_BufferTypeDef Buffer;
	SubSys_WirelessCom_VariableTypeDef	Variable;

	/*! Used device DMA and Usart interface settings */
	UART_HandleTypeDef *huartX;
	DMA_HandleTypeDef *hdma_usartX_rx;
	DMA_HandleTypeDef *hdma_usartX_tx;

	/*! Target device Address and Channel settings */
	uint8_t Target_ADDH;
	uint8_t Target_ADDL;
	uint8_t Target_Ch;

}SubSys_WirelessCom_APP_HandleTypeDef;


/******************************************************************************
         			#### WIRELESSCOM PROTOTYPES OF FUNCTIONS ####
******************************************************************************/
/**
  * @brief Decimal, float and other formats are converted as character and save them into the TX buffer.
  * 		When TX buffer is fulfilled , it is sent by UART interface.
  * @note  Follow the transmitting rules, each if and else has a reason
  * @param MissionUnit From_X, Packet type used for your specific purpose. Where do you  want to take it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  *
  * @param MissionUnit To_Y, Packet type used for your specific purpose. Where do you want to send it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
  * @retval NONE
  */
void SubSys_WirelessCom_Telemetry_Transfer_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp);


/**
  * @brief
  * @note  Follow the transmitting rules, each if and else has a reason
  * @param MissionUnit From_X, Packet type used for your specific purpose. Where do you  want to take it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  *
  * @param MissionUnit To_Y, Packet type used for your specific purpose. Where do you want to send it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
  * @retval NONE
  */

void SubSys_WirelessCom_Telemetry_Receive_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp);


/**
 * @brief  : Creates 3 types packet for Carrier, Payload and Ground Station
 * @note   : Where do you want to send the packet, select that, For example you are a courier and
 * 			 carry the packet to the selected home(MissionUnite x)
 *
 * @param  : MissionUnit x, Packet type used for your specific purpose @arg 0 : Sat_Carrier
 * 																	   @arg 1 : Sat_Payload
 * 																	   @arg 2 : Ground_Sation
 *
 * @param  : SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
 * @retval NONE
 */
void SubSys_WirelessCom_Telemetry_Create_Packet_For(MissionUnit x,SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp);


#endif /* SAT_CARRIER_SUBSYS_DRIVERS_SUBSYS_INC_SUBSYS_WIRELESSCOMMUNICATION_TELEMETRY_DRIVER_H_ */
