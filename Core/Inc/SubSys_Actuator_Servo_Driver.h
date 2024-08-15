
#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_H

#define SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_COLORFILTER_TEST_STATUS
#define SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_SEPARATION_TEST_STATUS

/******************************************************************************
         				#### INCLUDES ####
******************************************************************************/
#include "main.h"

typedef struct
{

	TIM_HandleTypeDef *htim_X;
	uint32_t tim_channel_in;

}Actuator_Servo_HandleTypeDef;


extern	Actuator_Servo_HandleTypeDef dev_Servo_Separation;
extern  Actuator_Servo_HandleTypeDef dev_Servo_ColorFilter;

void SubSys_Actuator_Servo_Init(Actuator_Servo_HandleTypeDef *dev);
void SubSys_Actuator_Servo_MoveTo(Actuator_Servo_HandleTypeDef *dev, uint16_t Angle);
void SubSys_Actuator_Servo_DeInit(Actuator_Servo_HandleTypeDef *dev);


#endif /* INC_SUBSYS_ACTUATOR_SERVO_DRIVER_H_ */
