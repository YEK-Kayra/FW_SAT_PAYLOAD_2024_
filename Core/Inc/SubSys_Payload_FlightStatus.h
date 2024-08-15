

#ifndef  SAT_PAYLOAD_SUBSYS_DRIVERS_FLIGHTSTATUS_H_
#define  SAT_PAYLOAD_SUBSYS_DRIVERS_FLIGHTSTATUS_H_

/******************************************************************************
         				#### INCLUDES ####
******************************************************************************/
#include "main.h"
#include "SubSys_AlertControl_Driver.h"
#include "SubSys_Actuator_Servo_Driver.h"
#include "SubSys_Payload_PeriodicReattempt.h"


/******************************************************************************
         				#### ENUMS ####
******************************************************************************/
typedef enum{
	ReadyForLaunch,
	Ascent,
	ModelSatelliteDescent,
	Separation,
	PayloadDescent,
	Recovery
}Satellite_Status;

typedef enum{

	Permission_NOT,
	Permission_OK

}Satellite_SeparationPermission;



/******************************************************************************
         				#### EXTERN VARIABLES ####
******************************************************************************/
/**
 * 0: Ready for Launch (Before Rocket Ignition)
 * 1: Ascent
 * 2: Model Satellite Descent
 * 3: Separation
 * 4: Payload Descent
 * 5: Recovery (Payload Ground Contact)
 */
extern uint8_t SatelliteStatus;

extern Actuator_Servo_HandleTypeDef dev_Servo_Separation;	/*! We will create object for the separation system*/
extern uint8_t AutonomoSeparationStatus; 					/*! if the value is 0 that means there is no separation permission else permission is OK*/

extern float  MS5611_VertSpeed;			 /*! Vertical Speed data variable    */
extern float  MS5611_Altitude;			 /*! Vertical Altitude data variable */

extern float euler_roll;				 /*! Gyro module roll angle */
extern float euler_pitch;				 /*! Gyro module pitch angle */
extern float euler_yaw;					 /*! Gyro module yaw angle */

extern TIM_HandleTypeDef htim3;			 /*! Passive Buzzer's timer PWM channel */


/******************************************************************************
         				#### FUNCTIONS ####
******************************************************************************/
void SubSys_SatelliteMission_Continue();


#endif /* INC_SUBSYS_PAYLOAD_FLIGHTSTATUS_H_ */
