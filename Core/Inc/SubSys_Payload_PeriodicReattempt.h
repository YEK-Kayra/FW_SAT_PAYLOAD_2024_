
#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_PERIODICREATTEMPT_H_
#define SAT_PAYLOAD_SUBSYS_DRIVERS_PERIODICREATTEMPT_H_


/******************************************************************************
         				#### INCLUDES ####
******************************************************************************/
#include "main.h"
#include "SubSys_Sensor_RTC_Driver.h"
#include "SubSys_Sensor_Battery_Driver.h"
#include "SubSys_SDcard_Driver.h"
#include "SubSys_Sensor_TPGVH_Driver.h"
#include "SubSys_WirelessCommunication_Setting_Driver.h"
#include "SubSys_WirelessCommunication_Telemetry_Driver.h"
#include "SubSys_ARAS.h"
#include "SubSys_Sensor_IMU_APP_Driver.h"



/******************************************************************************
         			#### EXTERN VARIABLE ####
******************************************************************************/
extern ADC_HandleTypeDef hadc1;									/*! Battery voltage measurement ADC interface objecct */
extern MS5611_HandleTypeDef MS5611;								/*! MS5611 object									  */
extern char SdDatasBuf[LineSize];								/*! We create a buffer that contains the satellite's variables, and we fill it with variables from SD_Data objects */
extern SubSys_WirelessCom_APP_HandleTypeDef dev_WirelessComApp; /*! Wireless Communication application object */
extern uint32_t SystemTick;   				 					/*! All system units will be work together at 1Hz*/
extern uint32_t NumberOfTelePacket;								/*! The value is incremented by +1 at the end of each satellite operation period */

/******************************************************************************
         			#### FUNCTIONS ####
******************************************************************************/
void SubSys_Payload_MissionRetryLoop();



#endif /* INC_SUBSYS_PAYLOAD_PERIODICREATTEMPT_H_ */
