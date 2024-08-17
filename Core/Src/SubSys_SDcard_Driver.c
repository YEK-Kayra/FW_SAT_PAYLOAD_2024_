/******************************************************************************
         			#### SD CARD OPERATIONS INCLUDES ####
******************************************************************************/
#include "SubSys_SDcard_Driver.h"
#include "stm32f4xx_hal.h"


/******************************************************************************
         			#### SD CARD OPERATIONS STRUCTURES ####
******************************************************************************/
/* LIBRARY VARIABLES OF "ff.h" AND "FatFs.h" */
FATFS FATFS_Ob; 	 /* File system object */
FIL FilePage;   	 /* SD card page that is stored in the directory */
FRESULT SD_result;   /* File function return code */

/* SD CARD TOP FRAME */
char* DataTopFrame = "PAY_PacketNo|PAY_SAT_Status|PAY_ErrorCode|PAY_Date&Time|PAY_Pressure|CAR_Pressure|PAY_Height|CAR_Height|PAY_DiffHeight|PAY_VertSpeed|PAY_Temparature|PAY_Voltage|PAY_gpsLatitude|PAY_gpsLongitude|PAY_gpsAltitude|PAY_Pitch|PAY_Roll|PAY_Yaw|PAY_RHRH|Station_IOTdata|TeamNo  ";

/* We create an object that keeps different satellite variable */
SD_Datas_HandleTypeDef SD_Data;

/*! We create a buffer that contains the satellite's carrier variables, and we fill it with variables from SD_Data objects */
char* SdDatasBuf[LineSize];

extern I2C_HandleTypeDef hi2c1;
extern float  MS5611_Press;		/*! Pressure data variable 			*/
extern float  MS5611_Temp;		/*! Temperature data variable 		*/
extern float  MS5611_Altitude;	/*! Vertical Altitude data variable */
extern float  MS5611_VertSpeed; /*! Vertical Speed data variable    */

extern float GPS_Altitude;				/*! Vertical distance info of satellite beetween */
extern float GPS_Longitude;				/*! Location info of satellite on the earth 	 */
extern float GPS_Latitude;				/*! Location info of satellite on the earth 	 */

extern float BatteryVoltage;

extern float euler_roll;
extern float euler_pitch;
extern float euler_yaw;

extern uint32_t NumberOfTelePacket;
extern uint8_t SatelliteStatus;
extern uint8_t SatelliteErrorCode;

extern uint8_t date ;
extern uint8_t month;
extern uint16_t year ;
extern uint8_t hour ;
extern uint8_t minute;
extern uint8_t second ;

extern uint32_t Race_TeamNo;

extern float CarrierPressure;
extern float CarrierVertHeight;

void SD_FillVariables(void){


		SD_Data.PAY_PacketNo 	= NumberOfTelePacket;
		SD_Data.PAY_SAT_Status  = SatelliteStatus;
		SD_Data.PAY_ErrorCode   = SatelliteErrorCode;

		SD_Data.PAY_DateTime[0] = date;
		SD_Data.PAY_DateTime[1] = month;
		SD_Data.PAY_DateTime[2] = year;
		SD_Data.PAY_DateTime[3] = hour;
		SD_Data.PAY_DateTime[4] = minute;
		SD_Data.PAY_DateTime[5] = second;

		SD_Data.PAY_Pressure 	= MS5611_Press;
		SD_Data.CAR_Pressure    = CarrierPressure;
		SD_Data.PAY_Height 		= MS5611_Altitude;
		SD_Data.CAR_Height		= CarrierVertHeight;
		SD_Data.PAY2CAR_DiffHeight = (SD_Data.PAY_Height-SD_Data.CAR_Height);

		SD_Data.PAY_VertSpeed 	= MS5611_VertSpeed;
		SD_Data.PAY_Temparature = MS5611_Temp;
		SD_Data.PAY_Voltage 	= BatteryVoltage;

		SD_Data.PAY_GPS_Latitude  = GPS_Latitude;
		SD_Data.PAY_GPS_Longitude = GPS_Longitude;
		SD_Data.PAY_GPS_Altitude  = GPS_Altitude;

		SD_Data.PAY_Pitch = euler_pitch;
		SD_Data.PAY_Roll  = euler_roll;
		SD_Data.PAY_Yaw   = euler_yaw;

		SD_Data.PAY_RHRH[0];
		SD_Data.PAY_RHRH[1];
		SD_Data.PAY_RHRH[2];
		SD_Data.PAY_RHRH[3];

		SD_Data.Station_IOTdata;
		SD_Data.TeamNo = Race_TeamNo;


    	sprintf(SdDatasBuf,"<%d>, <%d>, <%d>, <%d>, <%d>, <%d>, <%d>, <%d>, <%d>, <%.2f>, <%.2f>, <%.2f>, <%.2f>, <%.2f>, <%.2f>, <%.2f>, <%.2f>, <%.6f>,<%.6f>,<%.2f>, <%.2f>,<%.2f>,<%.2f>, <%c,%c,%c,%c>, <%.2f>, <%d>,\n",
																					SD_Data.PAY_PacketNo     ,SD_Data.PAY_SAT_Status   ,
																					SD_Data.PAY_ErrorCode  	 ,SD_Data.PAY_DateTime[0]  ,
																					SD_Data.PAY_DateTime[1]  ,SD_Data.PAY_DateTime[2]  ,
																					SD_Data.PAY_DateTime[3]  ,SD_Data.PAY_DateTime[4]  ,
																					SD_Data.PAY_DateTime[5]  ,SD_Data.PAY_Pressure	   ,
																					SD_Data.CAR_Pressure   	 ,SD_Data.PAY_Height	   ,
																					SD_Data.CAR_Height	 	 ,SD_Data.PAY2CAR_DiffHeight,
																					SD_Data.PAY_VertSpeed  	 ,SD_Data.PAY_Temparature   ,
																					SD_Data.PAY_Voltage	 	 ,SD_Data.PAY_GPS_Latitude  ,
																					SD_Data.PAY_GPS_Longitude,SD_Data.PAY_GPS_Altitude ,
																					SD_Data.PAY_Pitch		 ,SD_Data.PAY_Roll		   ,
																					SD_Data.PAY_Yaw			 ,SD_Data.PAY_RHRH[0]	   ,
																					SD_Data.PAY_RHRH[1]	 	 ,SD_Data.PAY_RHRH[2]	   ,
																					SD_Data.PAY_RHRH[3]	 	 ,SD_Data.Station_IOTdata  ,
																					SD_Data.TeamNo);

}


FRESULT SD_Mount (const TCHAR* SD_path, BYTE Mount_Op)
{

	SD_result = f_mount(&FATFS_Ob, SD_path, Mount_Op);

	if(SD_result != FR_OK){
		/**
		 * Buzzer will be activated like biiip biip bip
		 */
		//while(1);
	}
	else{

		return FR_OK;

	}
}


FRESULT SD_Create_Dir_File(const TCHAR* SD_Dir,const TCHAR* SD_FileName,char* SD_Buffer){

	SD_result = f_mkdir(SD_Dir);

	if((SD_result != FR_OK)&&(SD_result != FR_EXIST)){
		/**
		 * Buzzer will be activated like biiip biip bip
		 */
		while(1);
	}
	else{

		SD_result = f_open(&FilePage, SD_FileName, FA_CREATE_ALWAYS | FA_WRITE);
		SD_result = f_close(&FilePage);

		sprintf(SD_Buffer,"%s\n",DataTopFrame);
		SD_Write(SD_Buffer,"SAT_PAY/STM32.TXT");

		if(SD_result != FR_OK){
				/**
				 * Send to ground station error message
			     */
				while(1);
			}
		SD_result = f_close(&FilePage); //invalid object hatası verdi
		return FR_OK;
	}

}


FRESULT SD_Write(char* SD_Buffer,const TCHAR* SD_FileName){

	UINT written;

	SD_result = f_open(&FilePage, SD_FileName, FA_OPEN_APPEND | FA_WRITE);

	SD_result =  f_write(&FilePage,SD_Buffer,strlen(SD_Buffer),&written);

	SD_result = f_close(&FilePage);

	return FR_OK;
}
