
#include "SubSys_Payload_PeriodicReattempt.h"

void SubSys_Payload_MissionRetryLoop(){

	/*! At the beginning of each loop, the system retrieves the last recorded milliseconds */
	SystemTick = HAL_GetTick();


	/*! Get current Date & Time */
	DS1307_GetAllDatas();

	/*! It reads the TPGVH data and saves it into the variables created in the system
	* (T) = Temperature
	* (P) = Pressure
	* (G) = G force
	* (V) = Vertical Speed
	* (H) = Vertical Height
	**/
	MS5611_Read_ActVal(&MS5611);

	/*! It reads the battery voltage and stores it */
	ReadBatteryVoltage(&hadc1);

	/*! Get ARAS status data and save value into the "SatelliteErrorCode"*/
	ARAS_CheckSystem();

	/*! Get the Roll, Pitch, and Yaw values from the IMU sensor variables. */
	IMU_GetEulerVector();

	/*! GPS datas will be getting at the background */

	/*! The collected data is stored into variables that created for the SD card */
	SD_FillVariables();
	/*! The recorded variables are written to the SD card */
	SD_Write(SdDatasBuf, "SAT_PAY/STM32.TXT");

	/*! Transfer all necessary datas from Carrier to Payload of Satellite*/
	SubSys_WirelessCom_Telemetry_Transfer_From_To(Sat_Payload, GroundStation, &dev_WirelessComApp);

	NumberOfTelePacket++;

	/*! The system time is retrieved again and the loop waits until the elapsed time reaches 1000 milliseconds*/
	HAL_Delay(abs(1000 - (HAL_GetTick() - SystemTick)));

}

