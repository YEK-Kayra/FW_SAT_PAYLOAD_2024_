/*
 * SubSys_ColorFilterControl_Driver.h
 *
 *  Created on: Aug 5, 2024
 *      Author: yunus
 */

#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_COLORFILTER_CONTROL_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_COLORFILTER_CONTROL_H

/******************************************************************************
         			#### DEFINITIONS ####
******************************************************************************/
#define SAT_PAYLOAD_SUBSYS_DRIVERS_COLORFILTER_CONTROL_TEST_STATUS


/******************************************************************************
         			#### INCLUDES ####
******************************************************************************/
#include "main.h"
#include "SubSys_Actuator_Servo_Driver.h"


/******************************************************************************
         			#### ENUMS ####
******************************************************************************/
typedef enum{

	Filter_None,
	filter_Red,
	filter_Green,
	filter_Blue

}SubSys_ColorFilter_MagazineColor;


/******************************************************************************
         			#### FUNCTIONS ####
******************************************************************************/
void SubSys_ColorFilterMechanism_TurnTo(SubSys_ColorFilter_MagazineColor Color);

#endif /* INC_SUBSYS_COLORFILTERCONTROL_DRIVER_H_ */
