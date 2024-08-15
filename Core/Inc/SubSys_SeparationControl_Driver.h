

#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_SEPARATION_CONTROL_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_SEPARATION_CONTROL_H
/******************************************************************************
         			#### INCLUDES ####
******************************************************************************/
#include "main.h"
#include "SubSys_Actuator_Servo_Driver.h"


/******************************************************************************
         			#### EXTERNAL VARIABLES ####
******************************************************************************/
extern Actuator_Servo_HandleTypeDef dev_Servo_Separation;


/******************************************************************************
         			#### FUNCTIONS ####
******************************************************************************/
void SubSys_SeparationMechanism_Lock_PayloadToCarrier();
void SubSys_SeparationMechanism_UnLock_PayloadFromCarrier();


#endif /* INC_SUBSYS_SEPARATIONCONTROL_DRIVER_H_ */
